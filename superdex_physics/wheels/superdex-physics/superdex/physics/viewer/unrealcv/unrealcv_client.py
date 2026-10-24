# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
High-level wrapper around the UnrealCV client for mochi integration.

This module provides a typed, high-level interface over the raw UnrealCV client,
with methods for object manipulation, camera control, and image capture.
"""

from __future__ import annotations

import json
import logging
import os
import queue
import socket
import threading
import time
from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from functools import cached_property
from io import BytesIO
from typing import cast, TYPE_CHECKING, TypeVar

import numpy as np
import numpy.typing as npt

from .unrealcv_image import npy_view

if TYPE_CHECKING:
    import unrealcv

logger = logging.getLogger(__name__)

_T = TypeVar("_T")

########################################################################################


def _fmt(value: float) -> str:
    """
    Format a float value for UnrealCV commands.

    Ensures values are always in decimal notation (never scientific notation)
    with sufficient precision for transform values.

    Args:
        value: The float value to format.

    Returns:
        String representation in decimal notation.
    """
    return f"{float(value):.10f}".rstrip("0").rstrip(".")


def _camera_id_list(camera_ids: Sequence[int]) -> str:
    return ",".join(str(camera_id) for camera_id in camera_ids)


@dataclass(frozen=True)
class LitMultiLayout:
    """Where a multi-view lit capture places each camera in its BGRA8 atlas."""

    width: int
    height: int
    ids: tuple[int, ...]
    tiles: tuple[tuple[int, int, int, int], ...]
    """One ``(x, y, width, height)`` pixel rectangle per camera, in ``ids`` order."""


########################################################################################


class _Call:
    """One call handed to a :class:`_CallWorker` and its outcome."""

    def __init__(self, fn: Callable[..., object], args: tuple[object, ...]) -> None:
        self.fn = fn
        self.args = args
        self.result: object = None
        self.error: Exception | None = None
        self.done = threading.Event()


class _CallWorker:
    """A daemon thread that runs submitted calls one at a time.

    Daemon, like the per-request threads it replaces, so a call stuck on a hung
    UE process never blocks interpreter exit.
    """

    def __init__(self) -> None:
        self._calls: queue.SimpleQueue[_Call] = queue.SimpleQueue()
        threading.Thread(target=self._run, name="unrealcv-call", daemon=True).start()

    def submit(self, fn: Callable[..., object], *args: object) -> _Call:
        call = _Call(fn, args)
        self._calls.put(call)
        return call

    def _run(self) -> None:
        while True:
            call = self._calls.get()
            try:
                call.result = call.fn(*call.args)
            except Exception as e:
                call.error = e
            call.done.set()


class UnrealCVClient:
    """
    High-level wrapper around the UnrealCV client for mochi integration.

    Provides typed methods for common operations like setting object transforms,
    controlling cameras, and capturing images. All UnrealCV communication is
    handled internally.
    """

    ####################################################################################
    # Members
    ####################################################################################

    _client: "unrealcv.Client"
    _host: str
    _port: int
    _timeout: float
    _request_timeout: float
    _connected: bool
    _supports_mesh_updates: bool | None
    _request_lock: threading.Lock

    ####################################################################################
    # Constructor
    ####################################################################################

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9000,
        timeout: float = 5.0,
        request_timeout: float | None = None,
        camera_mapping: dict[str, dict] | None = None,
    ):
        """
        Initialize the UnrealCV client wrapper.

        Args:
            host: Hostname of the UnrealCV server.
            port: Port of the UnrealCV server.
            timeout: Connection timeout in seconds.
            request_timeout: Per-request socket timeout in seconds applied
                after connect. ``None`` (default) reads the value from the
                ``UNREALCV_REQUEST_TIMEOUT`` env var, falling back to 60s.
                A finite timeout converts hung UE responses into a
                ``socket.timeout`` that the request methods catch and
                return as ``None`` — the offline-render loop then skips
                the affected H5 instead of blocking the whole pod.
            camera_mapping: Optional mapping from observation keys
                (e.g. ``"image.left"``) to camera config dicts containing
                ``ue_name``, ``image_width``, and ``image_height``.
                If provided, the mapping is resolved to camera indices
                automatically when :meth:`connect` succeeds.
        """
        import unrealcv

        self._host = host
        self._port = port
        self._timeout = timeout
        if request_timeout is None:
            request_timeout = float(os.environ.get("UNREALCV_REQUEST_TIMEOUT", "60.0"))
        self._request_timeout = request_timeout
        self._connected = False
        self._supports_mesh_updates = None
        self._request_lock = threading.Lock()
        self._call_worker: _CallWorker | None = None

        # Camera mapping state (resolved during connect)
        self._pending_camera_mapping: dict[str, dict] | None = camera_mapping
        self._camera_mapping: dict[str, dict] = {}
        self._obs_key_to_camera_id: dict[str, int] = {}
        self._camera_id_to_obs_key: dict[int, str] = {}
        self._cropped_camera_ids: set[int] = set()
        self._hidden_bone_camera_ids: set[int] = set()

        # vset commands held by defer_vsets() until one batch sends them all; None
        # when not deferring.
        self._deferred_vsets: list[str] | None = None
        self._reported_deferred_failures: set[str] = set()

        # Create the unrealcv client
        self._client = unrealcv.Client((host, port))

    ####################################################################################
    # Connection Management
    ####################################################################################

    def connect(self, retry_attempts: int = 3, resolve_cameras: bool = True) -> bool:
        """
        Establish connection to the UnrealCV server.

        Args:
            retry_attempts: Number of retry attempts if connection fails.
            resolve_cameras: If True, automatically resolve camera mapping after
                connection. If False, camera mapping can be resolved later via
                resolve_camera_mapping(). Default False to allow blueprint spawning
                before camera resolution.

        Returns:
            True if connection was successful, False otherwise.
        """
        for attempt in range(retry_attempts):
            try:
                self._client.connect(timeout=self._timeout)
                if self._client.isconnected():
                    self._connected = True
                    self._apply_request_timeout()
                    logger.info(
                        f"Connected to UnrealCV server at {self._host}:{self._port}"
                    )

                    # Only resolve cameras if explicitly requested (for backward compat)
                    if resolve_cameras and self._pending_camera_mapping:
                        self.set_camera_mapping(self._pending_camera_mapping)
                        self._pending_camera_mapping = None

                    return True
            except Exception as e:
                logger.warning(
                    f"Connection attempt {attempt + 1}/{retry_attempts} failed: {e}"
                )
                if attempt < retry_attempts - 1:
                    time.sleep(1.0)

        logger.error(
            f"Failed to connect to UnrealCV server at {self._host}:{self._port}"
        )
        return False

    def reconnect(self) -> bool:
        """Replace the connection with a fresh one to the same server.

        For a caller retrying from an episode or reset boundary after a request
        failed: the client never re-sends a request itself, since a request whose
        reply timed out may have run in UE (actor updates, a capture that advances
        temporal state) with only the reply lost. The old client's
        request/response counters can't be trusted again. Camera ids and the
        camera mapping are kept, since they index UE-side state that outlives
        the connection.
        """
        import unrealcv

        if self._connected:
            self._mark_disconnected()
        # A fresh connection reports each failing target again.
        self._reported_deferred_failures.clear()
        self._client = unrealcv.Client((self._host, self._port))
        return self.connect(resolve_cameras=False)

    def resolve_camera_mapping(self) -> None:
        """
        Resolve pending camera mapping after connection.

        This should be called after all actors/cameras have been spawned in UE.
        It queries the scene for camera names and maps them to indices.

        Can be called multiple times safely (idempotent). If no pending mapping
        exists, this is a no-op.

        Raises:
            ValueError: If any camera name in the mapping is not found in the scene.
        """
        if self._pending_camera_mapping:
            self.set_camera_mapping(self._pending_camera_mapping)
            self._pending_camera_mapping = None

    def _apply_request_timeout(self) -> None:
        # Set SO_RCVTIMEO on the unrealcv Client's socket so its background
        # recv() thread cannot block forever on a hung UE process. The
        # vendored Client (1.1.7) stores the socket on ``sock`` — there is no
        # ``message_client`` attribute, so the old lookup always missed and the
        # timeout was silently dropped. This bounds the reader thread;
        # ``_blocking_call`` bounds the caller, which otherwise waits on the
        # client's internal response queue with no timeout.
        if self._request_timeout <= 0:
            return
        sock = getattr(self._client, "sock", None)
        if sock is None:
            logger.warning(
                "UnrealCV socket not accessible; request timeout not applied"
            )
            return
        try:
            sock.settimeout(self._request_timeout)
        except OSError as e:
            logger.warning(f"Failed to set UnrealCV request timeout: {e}")

    def _blocking_call(self, fn: Callable[..., _T], *args: object) -> _T:
        """Run a blocking unrealcv client call bounded by ``_request_timeout``.

        The vendored unrealcv ``Client`` does the socket ``recv()`` on a
        background thread while ``request()`` / ``request_batch()`` block on an
        internal queue that has no timeout. A UE process that accepts the
        connection but never answers therefore wedges the caller until the pod
        watchdog SIGKILLs the job hours later. Running ``fn`` on a daemon thread
        and joining with the timeout bounds that wait. On timeout the connection
        is marked dead so the now-inconsistent client is never reused, and a
        ``TimeoutError`` is raised for the caller to turn into ``None``.

        ``_request_timeout <= 0`` disables the bound and calls ``fn`` directly.
        """
        if self._request_timeout <= 0:
            return fn(*args)

        # One long-lived worker per client: a thread per request created ~1000
        # threads/s on a training pod. Calls are already serialized by
        # _request_lock, so a single worker never queues behind another caller.
        if self._call_worker is None:
            self._call_worker = _CallWorker()
        call = self._call_worker.submit(fn, *args)

        if not call.done.wait(self._request_timeout):
            # The stuck call keeps this worker; the next request starts a new one.
            self._call_worker = None
            self._mark_disconnected()
            raise TimeoutError(
                f"UnrealCV request exceeded {self._request_timeout}s; "
                "connection marked dead"
            )
        if call.error is not None:
            raise call.error
        return cast(_T, call.result)

    def _mark_disconnected(self) -> None:
        """Tear down the connection after a request that timed out or failed.

        Flips ``is_connected()`` to False so later requests short-circuit
        instead of touching the client (whose request/response counters are now
        out of sync), and shuts down and closes the socket so the vendored
        background recv() thread unwinds at once.
        """
        self._connected = False
        sock = getattr(self._client, "sock", None)
        if sock is not None:
            # close() alone does not wake a thread blocked in recv() on this
            # socket, which would then outlive the request by up to its timeout.
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self._client.sock = None
        # The vendored client's receive thread may instead wait on its count queue,
        # which the socket shutdown does not wake; its quit signal (what
        # disconnect() sends) does.
        recv_num_q = getattr(self._client, "recv_num_q", None)
        if recv_num_q is not None:
            recv_num_q.put(None)

    def disconnect(self):
        """Close the connection to the UnrealCV server."""
        if self._connected:
            try:
                self._client.disconnect()
            except Exception as e:
                logger.warning(f"Error during disconnect: {e}")
            finally:
                self._connected = False
                logger.info("Disconnected from UnrealCV server")

    def is_connected(self) -> bool:
        """Check if the client is connected to the UnrealCV server."""
        return self._connected and self._client.isconnected()

    def _request(self, command: str, use_async: bool = False) -> str | None:
        """
        Send a request to the UnrealCV server.

        Args:
            command: The UnrealCV command to send.

        Returns:
            The response string, or None if the request failed.
        """
        self._flush_deferred_vsets()
        with self._request_lock:
            # Checked under the lock: a request that failed while this one waited
            # has retired the client.
            if not self.is_connected():
                logger.warning("Cannot send request: not connected to UnrealCV server")
                return None

            try:
                if use_async:
                    self._client.request_async(command)
                    return None

                return self._blocking_call(self._client.request, command)
            except Exception as e:
                logger.error(f"Request failed for command '{command}': {e}")
                self._mark_disconnected()
                return None

    @contextmanager
    def defer_vsets(self) -> Iterator[None]:
        """Send every ``vset`` batch made inside as one batch, when the block ends.

        A batch of only ``vset`` commands is held and answered ``ok`` at once; any
        other request first sends the held commands, so UE runs every command in
        the order it was made. The held batch's failures are logged when it is sent,
        once per command target per connection. For a reset's scene randomization, which makes many
        small batches, each otherwise a round trip to a renderer busy with other
        worlds.
        """
        if self._deferred_vsets is not None:
            yield
            return
        self._deferred_vsets = []
        try:
            yield
        finally:
            self._flush_deferred_vsets()
            self._deferred_vsets = None

    def _flush_deferred_vsets(self) -> None:
        if not self._deferred_vsets:
            return
        commands, self._deferred_vsets = self._deferred_vsets, []
        result = self._send_batch(commands)
        replies = list((result[0] if isinstance(result, tuple) else result) or [])
        # _send_batch returns one reply per command, in command order, on every path (failures are
        # None), so a short list can only be an unanswered tail; each missing reply counts as failed.
        replies += [None] * (len(commands) - len(replies))
        for command, reply in zip(commands, replies):
            if isinstance(reply, str) and reply.lower() == "ok":
                continue
            parts = command.split()
            target = parts[1] if len(parts) > 1 else command
            if target in self._reported_deferred_failures:
                continue
            # Once per target per connection (reconnect() clears it): a missing actor otherwise warns on every reset.
            self._reported_deferred_failures.add(target)
            logger.warning(
                "Deferred UnrealCV command failed (reported once per target): %s -> %s",
                command,
                reply,
            )

    def _request_batch(
        self, commands: list[str], use_async: bool = False, profile: bool = False
    ) -> list[str | None] | tuple[list[str | None], dict]:
        """Send ``commands`` in one batch (see :meth:`_send_batch`), or hold them
        inside :meth:`defer_vsets`."""
        if self._deferred_vsets is not None and not use_async:
            if commands and all(c.startswith("vset ") for c in commands):
                self._deferred_vsets.extend(commands)
                # Callers here never act on a vset reply; a request whose reply matters goes
                # through _request(), which sends the held commands first and returns UE's answer.
                held: list[str | None] = ["ok"] * len(commands)
                return (held, {}) if profile else held
            self._flush_deferred_vsets()
        return self._send_batch(commands, use_async=use_async, profile=profile)

    def _send_batch(  # noqa: C901
        self, commands: list[str], use_async: bool = False, profile: bool = False
    ) -> list[str | None] | tuple[list[str | None], dict]:
        """
        Send multiple requests to the UnrealCV server in a single batch.

        All commands are sent together as one list so the UnrealCV server can
        process them in a single Unreal Engine frame.

        Args:
            commands: List of UnrealCV commands to send.
            profile: If True, return a profiling dict alongside responses.

        Returns:
            If profile is False: list of response strings (or None for failures).
            If profile is True: tuple of (responses, profiling_dict) where
                profiling_dict contains detailed timing information.
        """
        if not commands:
            return ([], {}) if profile else []

        with self._request_lock:
            # Checked under the lock: a request that failed while this one waited
            # has retired the client.
            if not self.is_connected():
                logger.warning(
                    "Cannot send batch request: not connected to UnrealCV server"
                )
                empty: list[str | None] = [None] * len(commands)  # pyre-ignore[9]
                return (empty, {}) if profile else empty

            try:
                if use_async:
                    self._client.request_batch_async(commands)
                    empty: list[str | None] = [None] * len(commands)  # pyre-ignore[9]
                    return (empty, {}) if profile else empty

                if not profile:
                    return self._blocking_call(self._client.request_batch, commands)

                # ---- Profiled path ----
                profiling: dict = {}

                # Classify commands
                capture_cmd_indices: list[int] = []
                transform_cmd_indices: list[int] = []
                for i, c in enumerate(commands):
                    if c.startswith(("vget /camera/", "vget /cameras/lit_")):
                        capture_cmd_indices.append(i)
                    else:
                        transform_cmd_indices.append(i)
                profiling["num_commands"] = len(commands)
                profiling["num_transform_cmds"] = len(transform_cmd_indices)
                profiling["num_capture_cmds"] = len(capture_cmd_indices)

                # Try to split send vs receive using the internal MessageClient.
                # The unrealcv Client.request(list) does: send-all then receive-all.
                # If we can access the underlying message_client we replicate that
                # loop ourselves so we can timestamp each phase.
                client = self._client
                handler = getattr(client, "message_client", None)
                if (
                    handler is not None
                    and hasattr(handler, "send")
                    and hasattr(handler, "receive")
                ):
                    # --- send all commands in one burst ---
                    send_start = time.perf_counter()
                    for cmd in commands:
                        handler.send(cmd)
                    send_ms = (time.perf_counter() - send_start) * 1000
                    profiling["send_ms"] = send_ms

                    # --- receive responses one-by-one (order matches send) ---
                    recv_start = time.perf_counter()
                    responses: list[str | None] = []
                    per_cmd_recv_ms: list[float] = []
                    for _cmd in commands:
                        t0 = time.perf_counter()
                        resp = handler.receive()
                        per_cmd_recv_ms.append((time.perf_counter() - t0) * 1000)
                        responses.append(resp)
                    recv_ms = (time.perf_counter() - recv_start) * 1000
                    profiling["recv_ms"] = recv_ms
                    profiling["per_cmd_recv_ms"] = per_cmd_recv_ms

                    # Aggregate recv time by command type
                    profiling["transform_recv_ms"] = sum(
                        per_cmd_recv_ms[i] for i in transform_cmd_indices
                    )
                    profiling["capture_recv_ms"] = sum(
                        per_cmd_recv_ms[i] for i in capture_cmd_indices
                    )
                    profiling["batch_mode"] = "send_recv_split"

                else:
                    # Fallback: use the normal batched request (single call)
                    # and measure response sizes for classification.
                    batch_start = time.perf_counter()
                    responses = self._blocking_call(client.request_batch, commands)
                    batch_ms = (time.perf_counter() - batch_start) * 1000
                    profiling["batch_total_ms"] = batch_ms
                    profiling["batch_mode"] = "opaque_batch"

                    # Estimate per-response payload sizes
                    capture_bytes = 0
                    transform_bytes = 0
                    for i, resp in enumerate(responses):
                        sz = len(resp) if resp is not None else 0
                        if i in capture_cmd_indices:
                            capture_bytes += sz
                        else:
                            transform_bytes += sz
                    profiling["capture_response_bytes"] = capture_bytes
                    profiling["transform_response_bytes"] = transform_bytes

                return (responses, profiling)
            except Exception as e:
                logger.error(f"Batch request failed: {e}")
                # Part of the batch may have reached the server, whose replies would
                # then answer later commands, so the client is retired before the
                # lock lets a waiting request send.
                self._mark_disconnected()
                empty: list[str | None] = [None] * len(commands)  # pyre-ignore[9]
                return (empty, {}) if profile else empty

    ####################################################################################
    # Command Building (no network I/O — just returns command strings)
    ####################################################################################

    @staticmethod
    def build_object_transform_commands(
        transforms: list[
            tuple[str, npt.NDArray[np.floating], npt.NDArray[np.floating]]
        ],
    ) -> list[str]:
        """
        Build xform commands for multiple objects without sending.

        Args:
            transforms: List of (actor_name, position, rotation_quat) tuples.

        Returns:
            List of command strings.
        """
        commands = []
        for actor_name, position, rotation in transforms:
            x, y, z = position[0], position[1], position[2]
            qx, qy, qz, qw = rotation[0], rotation[1], rotation[2], rotation[3]
            commands.append(
                f"vset /object/{actor_name}/xform {_fmt(x)} {_fmt(y)} {_fmt(z)} {_fmt(qx)} {_fmt(qy)} {_fmt(qz)} {_fmt(qw)}"
            )
        return commands

    @staticmethod
    def build_poseable_xforms_command(
        actor_name: str,
        bone_transforms: list[
            tuple[str, npt.NDArray[np.floating], npt.NDArray[np.floating]]
        ],
        actor_root_position: npt.NDArray[np.floating] | None = None,
        actor_root_rotation: npt.NDArray[np.floating] | None = None,
    ) -> str | None:
        """
        Build a poseable_xforms command string without sending.

        Args:
            actor_name: The UE actor name.
            bone_transforms: List of (bone_name, position, rotation_quat) tuples.
            actor_root_position: Optional root position [x, y, z].
            actor_root_rotation: Optional root rotation quaternion [x, y, z, w].

        Returns:
            Command string, or None if no bone transforms.
        """
        if not bone_transforms:
            return None

        parts = []

        if actor_root_position is not None and actor_root_rotation is not None:
            root_str = ",".join(
                [
                    "1",
                    _fmt(actor_root_position[0]),
                    _fmt(actor_root_position[1]),
                    _fmt(actor_root_position[2]),
                    _fmt(actor_root_rotation[0]),
                    _fmt(actor_root_rotation[1]),
                    _fmt(actor_root_rotation[2]),
                    _fmt(actor_root_rotation[3]),
                ]
            )
        else:
            root_str = "0,0,0,0,0,0,0,1"
        parts.append(root_str)

        parts.append(str(len(bone_transforms)))

        for bone_name, position, rotation in bone_transforms:
            bone_str = ",".join(
                [
                    bone_name,
                    _fmt(position[0]),
                    _fmt(position[1]),
                    _fmt(position[2]),
                    _fmt(rotation[0]),
                    _fmt(rotation[1]),
                    _fmt(rotation[2]),
                    _fmt(rotation[3]),
                ]
            )
            parts.append(bone_str)

        xform_data = ";".join(parts)
        return f"vset /object/{actor_name}/poseable_xforms {xform_data}"

    @staticmethod
    def build_camera_location_command(
        camera_id: int, position: npt.NDArray[np.floating]
    ) -> str:
        """Build a camera location command string."""
        x, y, z = position[0], position[1], position[2]
        return f"vset /camera/{camera_id}/location {_fmt(x)} {_fmt(y)} {_fmt(z)}"

    @staticmethod
    def build_camera_rotation_command(
        camera_id: int, rotation: npt.NDArray[np.floating]
    ) -> str:
        """Build a camera rotation command string."""
        pitch, yaw, roll = rotation[0], rotation[1], rotation[2]
        return (
            f"vset /camera/{camera_id}/rotation {_fmt(pitch)} {_fmt(yaw)} {_fmt(roll)}"
        )

    @staticmethod
    def build_camera_local_location_command(
        camera_id: int, position: npt.NDArray[np.floating]
    ) -> str:
        """Build a camera local-location command string."""
        x, y, z = position[0], position[1], position[2]
        return f"vset /camera/{camera_id}/local_location {_fmt(x)} {_fmt(y)} {_fmt(z)}"

    @staticmethod
    def build_camera_local_rotation_command(
        camera_id: int, rotation: npt.NDArray[np.floating]
    ) -> str:
        """Build a camera local-rotation command string."""
        pitch, yaw, roll = rotation[0], rotation[1], rotation[2]
        return f"vset /camera/{camera_id}/local_rotation {_fmt(pitch)} {_fmt(yaw)} {_fmt(roll)}"

    @staticmethod
    def build_camera_filmback_command(
        camera_id: int, filmback: npt.NDArray[np.floating]
    ) -> str:
        """Build a physical filmback command string."""
        sensor_width, sensor_height = filmback[0], filmback[1]
        return f"vset /camera/{camera_id}/filmback {_fmt(sensor_width)} {_fmt(sensor_height)}"

    @staticmethod
    def build_camera_focal_length_command(camera_id: int, focal_length: float) -> str:
        """Build a focal-length command string."""
        return f"vset /camera/{camera_id}/focal_length {_fmt(focal_length)}"

    @staticmethod
    def build_camera_hidden_bones_command(
        camera_id: int, actor_name: str, bone_names: list[str]
    ) -> str:
        """Build a per-camera hidden-bones command string."""
        if not bone_names:
            raise ValueError(
                "bone_names must be non-empty; call clear_camera_hidden_bones() instead"
            )
        bones = ",".join(bone_names)
        return f"vset /camera/{camera_id}/hidden_bones {actor_name} {bones}"

    @staticmethod
    def build_clear_camera_hidden_bones_command(camera_id: int) -> str:
        """Build a command string that clears per-camera hidden bones."""
        return f"vset /camera/{camera_id}/clear_hidden_bones"

    @staticmethod
    def _response_ok(response: str | None) -> bool:
        return response is not None and response.lower() == "ok"

    @staticmethod
    def _parse_float_response(
        response: str | None, expected_count: int, description: str
    ) -> list[float] | None:
        if response is None or "error" in response.lower():
            return None
        try:
            parts = response.split()
            if len(parts) != expected_count:
                return None
            return [float(part) for part in parts]
        except ValueError as e:
            logger.error(f"Failed to parse {description} response '{response}': {e}")
            return None

    @staticmethod
    def build_capture_image_command(
        camera_id: int, mode: str = "lit", fmt: str = "npy"
    ) -> str:
        """Build an image capture command string."""
        return f"vget /camera/{camera_id}/{mode} {fmt}"

    @staticmethod
    def build_start_lit_async_command(camera_id: int) -> str:
        """Build a command to start an async lit capture (returns a key)."""
        return f"vget /camera/{camera_id}/lit_async"

    @staticmethod
    def build_get_lit_latest_command(camera_id: int, fmt: str = "npy") -> str:
        """Build a command to get the most recent async lit capture (no key needed)."""
        return f"vget /camera/{camera_id}/lit_get_latest {fmt}"

    @staticmethod
    def build_start_lit_async_multi_command(camera_ids: Sequence[int]) -> str:
        """Build a command that starts one multi-view async lit capture (replies "ok")."""
        return f"vget /cameras/lit_async_multi {_camera_id_list(camera_ids)}"

    @staticmethod
    def build_get_lit_latest_multi_command(
        camera_ids: Sequence[int], fmt: str = "npy"
    ) -> str:
        """Build a command to get the latest multi-view atlas (or "pending")."""
        return f"vget /cameras/lit_get_latest_multi {_camera_id_list(camera_ids)} {fmt}"

    @cached_property
    def _lit_layout_multi_cache(self) -> dict[tuple[int, ...], LitMultiLayout]:
        return {}

    def get_lit_layout_multi(self, camera_ids: Sequence[int]) -> LitMultiLayout:
        """Return the atlas layout of a multi-view lit capture of ``camera_ids``.

        UE derives the layout from the camera sizes, so it is cached per ordered
        id tuple until a camera size or crop setter invalidates it.

        Raises:
            ValueError: If ``camera_ids`` names a camera twice, since its tiles
                would collide in the per-camera frames.
            RuntimeError: If UE replies with an error or an unexpected layout.
        """
        ids = tuple(camera_ids)
        if len(set(ids)) != len(ids):
            raise ValueError(f"Multi-view capture repeats a camera: {list(ids)}")
        cached = self._lit_layout_multi_cache.get(ids)
        if cached is not None:
            return cached

        command = f"vget /cameras/lit_layout_multi {_camera_id_list(ids)}"
        response = self._request(command)
        try:
            parsed = json.loads(response or "")
            atlas_format = parsed["format"]
            layout = LitMultiLayout(
                width=int(parsed["width"]),
                height=int(parsed["height"]),
                ids=tuple(int(camera_id) for camera_id in parsed["ids"]),
                tiles=tuple(
                    (int(x), int(y), int(w), int(h)) for x, y, w, h in parsed["tiles"]
                ),
            )
        except (KeyError, TypeError, ValueError) as e:
            raise RuntimeError(f"UnrealCV '{command}' failed: {response!r}") from e
        tiles_in_atlas = all(
            x >= 0
            and y >= 0
            and w > 0
            and h > 0
            and x + w <= layout.width
            and y + h <= layout.height
            for x, y, w, h in layout.tiles
        )
        if (
            atlas_format != "BGRA8"
            or layout.ids != ids
            or len(layout.tiles) != len(ids)
            or not tiles_in_atlas
        ):
            raise RuntimeError(
                f"UnrealCV '{command}' returned an unexpected layout: {response!r}"
            )
        self._lit_layout_multi_cache[ids] = layout
        return layout

    def invalidate_lit_layout_multi(self) -> None:
        """Forget every cached multi-view layout so the next lookup queries UE."""
        self._lit_layout_multi_cache.clear()

    ####################################################################################
    # Object Manipulation
    ####################################################################################

    def get_object_location(self, actor_name: str) -> npt.NDArray[np.floating] | None:
        """
        Get the world location of an object.

        Args:
            actor_name: The name of the actor in UE.

        Returns:
            The position as [x, y, z] in UE coordinates, or None if failed.
        """
        response = self._request(f"vget /object/{actor_name}/location")
        if response is None:
            return None
        try:
            parts = response.split()
            return np.array([float(parts[0]), float(parts[1]), float(parts[2])])
        except (ValueError, IndexError) as e:
            logger.error(f"Failed to parse location response '{response}': {e}")
            return None

    def get_object_rotation(self, actor_name: str) -> npt.NDArray[np.floating] | None:
        """
        Get the rotation of an object.

        Args:
            actor_name: The name of the actor in UE.

        Returns:
            The rotation as [pitch, yaw, roll] in degrees, or None if failed.
        """
        response = self._request(f"vget /object/{actor_name}/rotation")
        if response is None:
            return None
        try:
            parts = response.split()
            return np.array([float(parts[0]), float(parts[1]), float(parts[2])])
        except (ValueError, IndexError) as e:
            logger.error(f"Failed to parse rotation response '{response}': {e}")
            return None

    def set_object_visibility(self, actor_name: str, visible: bool) -> bool:
        """
        Set the visibility of an object.

        Args:
            actor_name: The name of the actor in UE.
            visible: Whether the object should be visible.

        Returns:
            True if successful, False otherwise.
        """
        visibility_str = "show" if visible else "hide"
        response = self._request(f"vset /object/{actor_name}/{visibility_str}")
        return response is not None and response.lower() == "ok"

    def get_bone_names(self, actor_name: str) -> set[str] | None:
        """
        Get the set of bone names for a posable mesh actor.

        Queries the UE skeleton via ``vget /object/<actor>/bones``.

        Args:
            actor_name: The name of the actor with a posable mesh component.

        Returns:
            A set of bone name strings, or None if the query failed.
        """
        response = self._request(f"vget /object/{actor_name}/bones")
        if response is None:
            return None
        try:
            names = response.split()
            return set(names)
        except Exception as e:
            logger.error(f"Failed to parse bone names response '{response}': {e}")
            return None

    def object_exists(self, actor_name: str) -> bool:
        """
        Check if an object exists in the UE scene.

        Args:
            actor_name: The name of the actor to check.

        Returns:
            True if the object exists, False otherwise.
        """
        response = self._request(f"vget /object/{actor_name}/location")
        return response is not None and "error" not in response.lower()

    def set_object_hsva(
        self, actor_name: str, h: float, s: float, v: float, a: float
    ) -> bool:
        """
        Set custom HSVA color on an object.

        Calls the SetCustomHSVA function on the actor or its components if it exists.

        Args:
            actor_name: The name of the actor in UE.
            h: Hue component in degrees (0-360).
            s: Saturation component (0-1).
            v: Value component (0-1).
            a: Alpha component (0-1).

        Returns:
            True if successful, False otherwise (object not found or no SetCustomHSVA function).
        """
        response = self._request(
            f"vset /object/{actor_name}/hsva {_fmt(h)} {_fmt(s)} {_fmt(v)} {_fmt(a)}"
        )
        return response is not None and response.lower() == "ok"

    def set_object_rgba(
        self, actor_name: str, r: float, g: float, b: float, a: float
    ) -> bool:
        """
        Set custom RGBA color on an object.

        Calls the SetCustomRGBA function on the actor if it exists.

        Args:
            actor_name: The name of the actor in UE.
            r: Red component (0-1).
            g: Green component (0-1).
            b: Blue component (0-1).
            a: Alpha component (0-1).

        Returns:
            True if successful, False otherwise (object not found or no SetCustomRGBA function).
        """
        response = self._request(
            f"vset /object/{actor_name}/rgba {_fmt(r)} {_fmt(g)} {_fmt(b)} {_fmt(a)}"
        )
        return response is not None and response.lower() == "ok"

    def set_object_specular(self, actor_name: str, value: float) -> bool:
        """
        Set specular on an object.

        Args:
            actor_name: The name of the actor in UE.
            value: Specular value (0-1).

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(f"vset /object/{actor_name}/specular {_fmt(value)}")
        return response is not None and response.lower() == "ok"

    def set_object_metallic(self, actor_name: str, value: float) -> bool:
        """
        Set metallic on an object.

        Args:
            actor_name: The name of the actor in UE.
            value: Metallic value (0-1).

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(f"vset /object/{actor_name}/metallic {_fmt(value)}")
        return response is not None and response.lower() == "ok"

    def set_object_roughness(self, actor_name: str, value: float) -> bool:
        """
        Set roughness on an object.

        Args:
            actor_name: The name of the actor in UE.
            value: Roughness value (0-1).

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(f"vset /object/{actor_name}/roughness {_fmt(value)}")
        return response is not None and response.lower() == "ok"

    def set_object_light_intensity(self, actor_name: str, value: float) -> bool:
        """
        Set light intensity on an object.

        Args:
            actor_name: The name of the actor in UE.
            value: Light intensity value (0-160).

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/light_intensity {_fmt(value)}"
        )
        return response is not None and response.lower() == "ok"

    def set_object_source_radius(self, actor_name: str, value: float) -> bool:
        """
        Set light source radius on an object.

        Args:
            actor_name: The name of the actor in UE.
            value: Source radius value.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/source_radius {_fmt(value)}"
        )
        return response is not None and response.lower() == "ok"

    def set_object_texture_index(self, actor_name: str, value: float) -> bool:
        """
        Set texture index on an object.

        Sends a normalized 0.0-1.0 value. The C++ side maps this to a
        concrete texture index using the material's TextureCount parameter.

        Args:
            actor_name: The name of the actor in UE.
            value: Normalized texture selection (0.0-1.0).

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/texture_index {_fmt(value)}"
        )
        return response is not None and response.lower() == "ok"

    def set_skylight_cubemap(self, actor_name: str, cubemap_asset_path: str) -> bool:
        """
        Set the specified cubemap on an actor's SkyLight and recapture.

        Switches the SkyLight to Specified-Cubemap source type, loads the given
        ``UTextureCube`` asset, and triggers a sky recapture so the new HDRI
        drives the scene's image-based lighting and reflections.

        Args:
            actor_name: The name of the SkyLight actor in UE.
            cubemap_asset_path: Asset path of the cubemap, e.g.
                ``/Game/ControlPolicy/Environments/HDRI/TC_HDRI_01.TC_HDRI_01``.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/skylight_cubemap {cubemap_asset_path}"
        )
        return response is not None and response.lower() == "ok"

    def set_texture_cube_param(
        self, actor_name: str, param_name: str, cubemap_asset_path: str
    ) -> bool:
        """
        Set a TextureCube parameter on an actor's material(s).

        Used to drive a cubemap-sampling backdrop dome from the same
        ``UTextureCube`` asset that feeds the SkyLight, so the visible backdrop
        and the image-based lighting stay in sync.

        Args:
            actor_name: The name of the actor in UE.
            param_name: The TextureCube material parameter name (e.g. ``EnvCube``).
            cubemap_asset_path: Asset path of the cubemap, e.g.
                ``/Game/ControlPolicy/Environments/HDRI/TC_HDRI_01.TC_HDRI_01``.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/texture_cube_param {param_name} "
            f"{cubemap_asset_path}"
        )
        return response is not None and response.lower() == "ok"

    def set_skylight_angle(self, actor_name: str, degrees: float) -> bool:
        """
        Set the source cubemap angle (degrees) on an actor's SkyLight.

        Rotates the SkyLight's source cubemap and triggers a sky recapture so
        the IBL ambient/reflections rotate with the visible HDRI backdrop.

        Args:
            actor_name: The name of the SkyLight actor in UE.
            degrees: Source cubemap angle in degrees.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/skylight_angle {_fmt(degrees)}"
        )
        return response is not None and response.lower() == "ok"

    def set_cast_shadow(self, actor_name: str, enable: bool) -> bool:
        """
        Toggle shadow casting on an actor's primitive components.

        Args:
            actor_name: The name of the actor in UE.
            enable: True to cast shadows, False to disable shadow casting.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/cast_shadow {1 if enable else 0}"
        )
        return response is not None and response.lower() == "ok"

    def set_mpc_scalar(self, actor_name: str, param_name: str, value: float) -> bool:
        """
        Set a scalar parameter on a MaterialParameterCollection owned by an actor.

        Args:
            actor_name: The name of the actor that holds the MPC reference.
            param_name: The scalar parameter name in the MPC.
            value: The value to set.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/mpc_scalar {param_name} {_fmt(value)}"
        )
        return response is not None and response.lower() == "ok"

    def set_object_scale(self, actor_name: str, x: float, y: float, z: float) -> bool:
        """
        Set the scale of an object.

        Args:
            actor_name: The name of the actor in UE.
            x: Scale factor on X axis.
            y: Scale factor on Y axis.
            z: Scale factor on Z axis.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            f"vset /object/{actor_name}/scale {_fmt(x)} {_fmt(y)} {_fmt(z)}"
        )
        return response is not None and response.lower() == "ok"

    ####################################################################################
    # Camera Name Mapping
    ####################################################################################

    def set_camera_mapping(self, camera_mapping: dict[str, dict]) -> None:
        """
        Register a mapping from observation keys to camera configurations
        (ue_name, image_width, image_height) and resolve each name to a
        camera index by querying the scene.

        Must be called after :meth:`connect`.  After this call, name-based
        camera helpers (e.g. :meth:`set_mapped_camera_sizes`,
        :meth:`obs_key_to_camera_id`) become available.

        Args:
            camera_mapping: Mapping from observation key (e.g.
                ``"image.left"``) to camera config dict containing:
                - ``ue_name``: Unreal camera name (e.g. ``"CameraLeft"``)
                - ``image_width``: Width of the camera image
                - ``image_height``: Height of the camera image

        Raises:
            ValueError: If any camera name is not found in the scene.
        """
        self._camera_mapping = dict(camera_mapping)
        # Extract just the ue_name for name-to-index resolution
        camera_name_mapping = {
            obs_key: config["ue_name"] for obs_key, config in camera_mapping.items()
        }

        self._obs_key_to_camera_id = self._resolve_camera_name_to_index(
            camera_name_mapping
        )
        self._camera_id_to_obs_key = {
            cam_id: obs_key for obs_key, cam_id in self._obs_key_to_camera_id.items()
        }

    def obs_key_to_camera_id(self, obs_key: str) -> int | None:
        """Return the resolved camera index for an observation key, or *None*."""
        return self._obs_key_to_camera_id.get(obs_key)

    def obs_keys_to_camera_ids(self, obs_keys: list[str]) -> list[int]:
        """Return a de-duplicated, ordered list of camera indices for the
        given observation keys.  Unknown keys are silently skipped."""
        seen: set[int] = set()
        result: list[int] = []
        for key in obs_keys:
            cam_id = self._obs_key_to_camera_id.get(key)
            if cam_id is not None and cam_id not in seen:
                seen.add(cam_id)
                result.append(cam_id)
        return result

    def camera_id_to_obs_key(self, camera_id: int) -> str | None:
        """Return the observation key for a resolved camera index, or *None*."""
        return self._camera_id_to_obs_key.get(camera_id)

    def set_mapped_camera_sizes(self, camera_mapping: dict[str, dict]) -> None:
        """Set the capture resolution for each camera based on the camera mapping.

        Args:
            camera_mapping: Mapping from observation key to camera config dict
                containing 'ue_name', 'image_width', and 'image_height'.

        Requires :meth:`set_camera_mapping` to have been called first.
        """
        if not self._obs_key_to_camera_id:
            logger.warning("set_mapped_camera_sizes: no cameras in mapping")
            return

        for obs_key, config in camera_mapping.items():
            camera_id = self._obs_key_to_camera_id.get(obs_key)
            if camera_id is None:
                logger.warning(f"set_mapped_camera_sizes: unknown obs_key '{obs_key}'")
                continue

            width = config["image_width"]
            height = config["image_height"]
            success = self.set_camera_size(camera_id, width, height)
            if success:
                logger.info(
                    f"Set camera {camera_id} ({obs_key}) size to {width}x{height}"
                )
            else:
                logger.warning(
                    f"Failed to set camera {camera_id} ({obs_key}) size to {width}x{height}"
                )

    def set_mapped_camera_crops(self, camera_mapping: dict[str, dict]) -> None:
        """Apply normalized sub-frustums declared in a camera mapping.

        A camera previously cropped through this client is reset to full-frame
        when its mapping later omits ``crop``. Fresh crop-free mappings send no
        crop commands so they remain compatible with older UnrealCV builds.
        """
        for obs_key, config in camera_mapping.items():
            camera_id = self._obs_key_to_camera_id.get(obs_key)
            if camera_id is None:
                raise RuntimeError(
                    f"Camera crop mapping contains unknown observation key {obs_key!r}"
                )
            crop = config.get("crop")
            is_reset = crop is None
            if crop is None:
                if camera_id not in self._cropped_camera_ids:
                    continue
                crop = (0.0, 0.0, 1.0, 1.0)
            if len(crop) != 4:
                raise ValueError(
                    f"Camera crop for {obs_key!r} must have four values, got {crop!r}"
                )
            left, top, right, bottom = (float(value) for value in crop)
            if not (0.0 <= left < right <= 1.0 and 0.0 <= top < bottom <= 1.0):
                raise ValueError(f"Invalid camera crop for {obs_key!r}: {crop!r}")
            response = self._request_camera_crop(camera_id, left, top, right, bottom)
            if response is None or response.lower() != "ok":
                if is_reset:
                    raise RuntimeError(
                        "UnrealCV rejected the full-frame crop reset for "
                        f"{obs_key!r} (camera {camera_id}): {response!r}"
                    )
                raise RuntimeError(
                    "UnrealCV rejected the camera crop command for "
                    f"{obs_key!r} (camera {camera_id}): {response!r}; verify the "
                    "camera has a valid perspective sensor and the deployed "
                    "UnrealCV server supports sub-frustum cropping"
                )
            logger.info("Set camera %d (%s) crop to %s", camera_id, obs_key, crop)

    ####################################################################################
    # Camera Control
    ####################################################################################

    def get_camera_names(self) -> list[str]:
        """
        Query the list of camera names from the Unreal Engine scene.

        Sends the ``vget /cameras`` command, which returns a space-separated
        string of camera (sensor) names.

        Returns:
            List of camera name strings. Empty list if the query fails.
        """
        response = self._request("vget /cameras")
        if response is None:
            logger.warning("Failed to query camera names from UnrealCV server")
            return []
        return response.split()

    def _resolve_camera_name_to_index(
        self, camera_name_mapping: dict[str, str]
    ) -> dict[str, int]:
        """
        Resolve a mapping of observation keys to Unreal camera names into a
        mapping of observation keys to camera indices.

        Queries the scene's camera list via ``vget /cameras`` and builds an
        index lookup from the returned order.

        Args:
            camera_name_mapping: Mapping from observation key (e.g.
                ``"image.left"``) to Unreal camera name (e.g.
                ``"CameraLeft"``).

        Returns:
            Mapping from observation key to camera index.

        Raises:
            ValueError: If any camera name in the mapping is not found in the
                scene's camera list.
        """
        camera_names = self.get_camera_names()
        if not camera_names:
            raise ValueError(
                "Failed to query camera names from UnrealCV server. "
                "Make sure the Unreal Engine scene has cameras configured."
            )

        name_to_index: dict[str, int] = {
            name: idx for idx, name in enumerate(camera_names)
        }

        resolved: dict[str, int] = {}
        for obs_key, ue_camera_name in camera_name_mapping.items():
            if ue_camera_name not in name_to_index:
                raise ValueError(
                    f"Camera '{ue_camera_name}' (for observation key "
                    f"'{obs_key}') not found in the scene. "
                    f"Available cameras: {camera_names}"
                )
            resolved[obs_key] = name_to_index[ue_camera_name]

        logger.info(
            f"Resolved camera name mapping: {camera_name_mapping} -> {resolved}"
        )
        return resolved

    def set_camera_size(self, camera_id: int, width: int, height: int) -> bool:
        """
        Set the image capture size for a camera.

        Args:
            camera_id: The camera ID.
            width: The image width in pixels.
            height: The image height in pixels.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(f"vset /camera/{camera_id}/size {width} {height}")
        self.invalidate_lit_layout_multi()
        return response is not None and response.lower() == "ok"

    def set_camera_crop(
        self,
        camera_id: int,
        left: float,
        top: float,
        right: float,
        bottom: float,
    ) -> bool:
        """Set a normalized ``(left, top, right, bottom)`` sub-frustum."""
        response = self._request_camera_crop(camera_id, left, top, right, bottom)
        return response is not None and response.lower() == "ok"

    def _request_camera_crop(
        self,
        camera_id: int,
        left: float,
        top: float,
        right: float,
        bottom: float,
    ) -> str | None:
        response = self._request(
            f"vset /camera/{camera_id}/crop {_fmt(left)} {_fmt(top)} "
            f"{_fmt(right)} {_fmt(bottom)}"
        )
        self.invalidate_lit_layout_multi()
        success = response is not None and response.lower() == "ok"
        if success:
            if (left, top, right, bottom) == (0.0, 0.0, 1.0, 1.0):
                self._cropped_camera_ids.discard(camera_id)
            else:
                self._cropped_camera_ids.add(camera_id)
        return response

    def set_all_camera_sizes(
        self, camera_ids: list[int], width: int, height: int
    ) -> list[bool]:
        """
        Set the image capture size for multiple cameras.

        Args:
            camera_ids: List of camera IDs to configure.
            width: The image width in pixels.
            height: The image height in pixels.

        Returns:
            List of booleans indicating success for each camera.
        """
        if not camera_ids:
            return []

        commands = [
            f"vset /camera/{camera_id}/size {width} {height}"
            for camera_id in camera_ids
        ]
        result = self._request_batch(commands)
        self.invalidate_lit_layout_multi()
        # When profile=False (default), _request_batch returns list[str | None]
        responses: list[str | None] = result if isinstance(result, list) else result[0]

        return [
            resp is not None and isinstance(resp, str) and resp.lower() == "ok"
            for resp in responses
        ]

    def set_camera_location(
        self, camera_id: int, position: npt.NDArray[np.floating]
    ) -> bool:
        """
        Set the camera position.

        Args:
            camera_id: The camera ID (typically 0).
            position: The position as [x, y, z] in UE coordinates (centimeters).

        Returns:
            True if successful, False otherwise.
        """
        x, y, z = position[0], position[1], position[2]
        response = self._request(
            f"vset /camera/{camera_id}/location {_fmt(x)} {_fmt(y)} {_fmt(z)}"
        )
        return response is not None and response.lower() == "ok"

    def set_camera_rotation(
        self, camera_id: int, rotation: npt.NDArray[np.floating]
    ) -> bool:
        """
        Set the camera rotation.

        Args:
            camera_id: The camera ID (typically 0).
            rotation: The rotation as [pitch, yaw, roll] in degrees.

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(
            self.build_camera_rotation_command(camera_id, rotation)
        )
        return self._response_ok(response)

    def set_camera_local_location(
        self, camera_id: int, position: npt.NDArray[np.floating]
    ) -> bool:
        """Set the camera-local position in Unreal centimeters."""
        response = self._request(
            self.build_camera_local_location_command(camera_id, position)
        )
        return self._response_ok(response)

    def set_camera_local_rotation(
        self, camera_id: int, rotation: npt.NDArray[np.floating]
    ) -> bool:
        """Set the camera-local rotation as Unreal pitch/yaw/roll degrees."""
        response = self._request(
            self.build_camera_local_rotation_command(camera_id, rotation)
        )
        return self._response_ok(response)

    def set_camera_filmback(
        self, camera_id: int, filmback: npt.NDArray[np.floating]
    ) -> bool:
        """Set the physical camera filmback as [sensor_width, sensor_height] mm."""
        response = self._request(
            self.build_camera_filmback_command(camera_id, filmback)
        )
        return self._response_ok(response)

    def set_camera_hidden_bones(
        self, camera_id: int, actor_name: str, bone_names: list[str]
    ) -> bool:
        """Hide skeletal-mesh bones from THIS camera's renders only.

        Args:
            camera_id: UnrealCV camera index.
            actor_name: UE actor name owning the skeletal/poseable mesh (e.g. the robot).
            bone_names: Bone names to hide (e.g. ["d405_camera_mount"] for the
                T3 wrist view).
        """
        response = self._request(
            self.build_camera_hidden_bones_command(camera_id, actor_name, bone_names)
        )
        success = self._response_ok(response)
        if success:
            self._hidden_bone_camera_ids.add(camera_id)
        return success

    def clear_camera_hidden_bones(self, camera_id: int) -> bool:
        """Clear any per-camera hidden bones for this camera."""
        response = self._request(
            self.build_clear_camera_hidden_bones_command(camera_id)
        )
        success = self._response_ok(response)
        if success:
            self._hidden_bone_camera_ids.discard(camera_id)
        return success

    def set_camera_focal_length(self, camera_id: int, focal_length: float) -> bool:
        """Set the physical camera focal length in millimeters."""
        response = self._request(
            self.build_camera_focal_length_command(camera_id, focal_length)
        )
        return self._response_ok(response)

    def apply_camera_calibration(
        self,
        camera_id: int,
        local_location: npt.NDArray[np.floating] | None = None,
        local_rotation: npt.NDArray[np.floating] | None = None,
        filmback: npt.NDArray[np.floating] | None = None,
        focal_length: float | None = None,
    ) -> list[bool]:
        """Apply optional per-camera calibration in a deterministic batch."""
        commands: list[str] = []
        if local_location is not None:
            commands.append(
                self.build_camera_local_location_command(camera_id, local_location)
            )
        if local_rotation is not None:
            commands.append(
                self.build_camera_local_rotation_command(camera_id, local_rotation)
            )
        if filmback is not None:
            commands.append(self.build_camera_filmback_command(camera_id, filmback))
        if focal_length is not None:
            commands.append(
                self.build_camera_focal_length_command(camera_id, focal_length)
            )
        if not commands:
            return []

        result = self._request_batch(commands)
        responses: list[str | None] = result if isinstance(result, list) else result[0]
        return [self._response_ok(response) for response in responses]

    def get_camera_location(self, camera_id: int) -> npt.NDArray[np.floating] | None:
        """
        Get the camera position.

        Args:
            camera_id: The camera ID (typically 0).

        Returns:
            The position as [x, y, z] in UE coordinates, or None if failed.
        """
        response = self._request(f"vget /camera/{camera_id}/location")
        if response is None:
            return None
        try:
            parts = response.split()
            return np.array([float(parts[0]), float(parts[1]), float(parts[2])])
        except (ValueError, IndexError) as e:
            logger.error(f"Failed to parse camera location response '{response}': {e}")
            return None

    def get_camera_rotation(self, camera_id: int) -> npt.NDArray[np.floating] | None:
        """
        Get the camera rotation.

        Args:
            camera_id: The camera ID (typically 0).

        Returns:
            The rotation as [pitch, yaw, roll] in degrees, or None if failed.
        """
        response = self._request(f"vget /camera/{camera_id}/rotation")
        if response is None:
            return None
        try:
            parts = response.split()
            return np.array([float(parts[0]), float(parts[1]), float(parts[2])])
        except (ValueError, IndexError) as e:
            logger.error(f"Failed to parse camera rotation response '{response}': {e}")
            return None

    def get_camera_local_location(
        self, camera_id: int
    ) -> npt.NDArray[np.floating] | None:
        """Get the camera-local position in Unreal centimeters."""
        values = self._parse_float_response(
            self._request(f"vget /camera/{camera_id}/local_location"),
            3,
            "camera local location",
        )
        return None if values is None else np.array(values)

    def get_camera_local_rotation(
        self, camera_id: int
    ) -> npt.NDArray[np.floating] | None:
        """Get the camera-local rotation as Unreal pitch/yaw/roll degrees."""
        values = self._parse_float_response(
            self._request(f"vget /camera/{camera_id}/local_rotation"),
            3,
            "camera local rotation",
        )
        return None if values is None else np.array(values)

    def get_camera_filmback(self, camera_id: int) -> npt.NDArray[np.floating] | None:
        """Get the physical camera filmback as [sensor_width, sensor_height] mm."""
        values = self._parse_float_response(
            self._request(f"vget /camera/{camera_id}/filmback"),
            2,
            "camera filmback",
        )
        return None if values is None else np.array(values)

    def get_camera_focal_length(self, camera_id: int) -> float | None:
        """Get the physical camera focal length in millimeters."""
        values = self._parse_float_response(
            self._request(f"vget /camera/{camera_id}/focal_length"),
            1,
            "camera focal length",
        )
        return None if values is None else values[0]

    ####################################################################################
    # Image Capture
    ####################################################################################

    def capture_image(
        self, camera_id: int, mode: str = "lit", fmt: str = "npy"
    ) -> npt.NDArray[np.uint8] | None:
        """
        Capture an image from the specified camera.

        Args:
            camera_id: The camera ID (typically 0).
            mode: The capture mode. Common options:
                - "lit": Normal rendered view
                - "depth": Depth buffer
                - "normal": Surface normals
                - "object_mask": Object segmentation mask
            fmt: The capture format ("npy" or "png").

        Returns:
            RGB image as numpy array of shape (H, W, 3), or None if failed.
        """
        response = self._request(f"vget /camera/{camera_id}/{mode} {fmt}")
        if response is None:
            return None

        try:
            if fmt == "png":
                from PIL import Image

                img = Image.open(BytesIO(response)).convert("RGB")
                return np.array(img)
            else:
                # Response is raw npy binary data (BGRA uint8)
                arr = npy_view(response)
                if arr.ndim == 3 and arr.shape[2] == 4:
                    # BGRA -> RGB
                    return arr[:, :, 2::-1].copy()
                elif arr.ndim == 3 and arr.shape[2] == 3:
                    # BGR -> RGB
                    return arr[:, :, ::-1].copy()
                return arr.copy()
        except Exception as e:
            logger.error(f"Failed to decode captured image: {e}")
            return None

    def capture_image_to_file(
        self, camera_id: int, filepath: str, mode: str = "lit"
    ) -> bool:
        """
        Capture an image and save it to a file.

        Args:
            camera_id: The camera ID (typically 0).
            filepath: Path where to save the image.
            mode: The capture mode (see capture_image).

        Returns:
            True if successful, False otherwise.
        """
        response = self._request(f"vget /camera/{camera_id}/{mode} {filepath}")
        return response is not None and response == filepath

    ####################################################################################
    # Scene Reset
    ####################################################################################

    def trigger_scene_reset(self, seed: int | None = None) -> bool:
        """
        Trigger a scene reset event in Unreal.

        This broadcasts the JitterMaterial event to all actors that implement
        the JitterMaterial function (UFUNCTION). Use this when resetting the
        environment to randomize materials or other visual properties.

        Args:
            seed: Non-negative seed for the JitterMaterial draws. Renderers
                started with ``-ControlPolicyDeterministicDR`` use it; others
                ignore it. Builds without the seeded command reject it, so pass
                it only to renderers that list the ``seeded_scene_reset``
                capability.

        Returns:
            True if successful, False otherwise.
        """
        command = "vset /action/scene/reset"
        if seed is not None:
            command += f" {seed}"
        try:
            self._request(command, use_async=True)
            return True
        except Exception as e:
            logger.error(f"Failed to trigger scene reset: {e}")
            return False
