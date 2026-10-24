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

"""MochiRenderer-based viewer for SuperDex Lab gym environments.

This module provides a client-side viewer for the standalone ``mochi_viewer`` TCP
server. It implements only a subset of the Polyscope ``Viewer`` interface.
"""

from __future__ import annotations

import atexit
import logging
import math
import struct
from collections.abc import Callable
from pathlib import Path

import numpy as np
import numpy.typing as npt
import superdex.physics as sdp
from scipy.spatial.transform import Rotation
from superdex.physics import Actor, ActorHandle, ActorType, Scene
from superdex.physics.utils.coordinate_systems import (
    COORDINATE_SYSTEMS,
    CoordinateSystem,
    CoordinateTransform,
    DEFAULT_COORDINATE_SYSTEM,
)
from superdex.physics.viewer.utils.aabb import AABB

from .actor_palette import actor_color, srgb_to_linear
from .camera_state import CameraState
from .grid_mesh import build_grid_meshes, GridMesh
from .ground_plane import (
    is_plane_actor,
    plane_center_and_axes,
    PLANE_COLOR_1,
    PLANE_COLOR_2,
    PLANE_PERIOD,
)
from .mochi_renderer_client import CommandEntry, MochiRendererClient
from .mochi_renderer_viewer_cfg import MochiRendererViewerCfg
from .viewer_app_process import find_viewer_app_executable, ViewerAppProcess

logger = logging.getLogger(__name__)

########################################################################################

RenderFrame = npt.NDArray[np.uint8]
"""A render frame, consists of an RGBA image."""

_GRID_OBJECT_PREFIX = "__grid__"
"""Reserved server-object namespace for grids, so a grid named "Floor" cannot
collide with an actor of the same name."""

_PLANE_OBJECT_PREFIX = "__plane__"
"""Reserved server-object namespace for auto-generated ground planes."""

_DEFAULT_VERTICAL_FOV_DEG = 45.0
"""Vertical field of view assumed when a camera does not configure one. The server
offers no readback, so framing has to assume its default."""

_MIN_FOV_DEG = 5.0
_MAX_FOV_DEG = 120.0
"""Bounds for interactive FOV zoom."""

_MIN_SCENE_EXTENT = 1e-3
"""Floor on each AABB axis when computing a framing distance, so a planar scene
does not collapse the enclosing radius."""

_MAX_FRAME_DEFER_RENDERS = 120
"""Renders a deferred framing request waits for actors before giving up."""

_DEFAULT_LOOK_FROM: npt.NDArray = np.ones(3)
"""Eye of the client's default camera pose, in the server's coordinates, used when the
capture camera configures none. It looks at the origin. The server's initial camera
poses differ, so the viewer sends this pose with the first render."""

_DEFAULT_LOOK_DIR: npt.NDArray = -_DEFAULT_LOOK_FROM / math.sqrt(3.0)
"""Look direction of the default camera pose, in the server's coordinates, and the
fallback when the cached pose has no direction."""

_DEFAULT_UP: npt.NDArray = np.array([0.0, 1.0, 0.0])
"""Up of the default camera pose, in the server's coordinates, matching the server's Y-up
scene (render space, ground plane and window manipulator)."""

_CAMERA_FORWARD: npt.NDArray = np.array([0.0, 0.0, -1.0])
"""View direction of a camera in its own frame: cameras look along their -Z axis."""

_RENDER_COORDINATE_SYSTEM = DEFAULT_COORDINATE_SYSTEM
"""Convention the server renders in: its render space, ground and window camera are
Y-up."""

_LOCAL_BOUNDS_ACTOR_TYPES = (ActorType.RIGID, ActorType.SOFT, ActorType.SHELL)
"""Actor types whose surface positions fully determine their bounds in the local
frame, so world bounds follow from the root transform. Articulated and rod actors
move their joints without changing the root transform, so theirs must be queried."""


def _local_aabb_from_positions(positions: npt.NDArray) -> AABB | None:
    """Bounds of a flat array of local-frame vertex positions, or None if empty.

    Reduces in the buffer's own dtype: the positions are float32 and re-fetched
    every frame for deformables, so widening them first would copy megabytes."""
    points = np.asarray(positions).reshape(-1, 3)
    if not len(points):
        return None
    return AABB(
        points.min(axis=0).astype(np.float64),
        points.max(axis=0).astype(np.float64),
    )


def _vertical_fov_deg(horizontal_fov_deg: float, aspect: float) -> float:
    """Convert a horizontal field of view to the vertical one the server expects."""
    hfov_rad = math.radians(horizontal_fov_deg)
    return math.degrees(2.0 * math.atan(math.tan(hfov_rad / 2.0) / aspect))


def _root_transform_matrix(actor: Actor) -> npt.NDArray:
    """World-from-local 4x4 transform of an actor's root, or identity if it has none."""
    transform = np.eye(4)
    if not actor.has_root_transform():
        return transform
    xform = actor.get_root_transform()
    transform[:3, :3] = Rotation.from_quat(
        np.asarray(xform.rotation, dtype=np.float64)
    ).as_matrix()
    transform[:3, 3] = np.asarray(xform.translation, dtype=np.float64)
    return transform


def _queried_world_aabb(actor: Actor) -> AABB | None:
    """World AABB straight from the physics engine, for actors whose cached local
    bounds would go stale (articulated, rod) or were never fetched (glTF overrides)."""
    try:
        aabb = actor.get_aabb_world()
        return AABB(
            min=np.array([aabb.min[0], aabb.min[1], aabb.min[2]], dtype=np.float64),
            max=np.array([aabb.max[0], aabb.max[1], aabb.max[2]], dtype=np.float64),
        )
    except Exception:
        return None


class MochiRendererViewer:
    """Client-side viewer for the standalone ``mochi_viewer`` TCP server.

    This class implements a subset of the Polyscope ``Viewer`` interface. It connects
    to a running TCP server and synchronizes Mochi simulation state (actor meshes,
    transforms, camera) with the renderer.
    """

    ####################################################################################
    # Members
    ####################################################################################

    _cfg: MochiRendererViewerCfg
    _coordinate_system: CoordinateSystem
    _coordinate_transform: CoordinateTransform
    _render_from_source: npt.NDArray | None
    _default_look_dir: npt.NDArray
    _client: MochiRendererClient
    _scene: Scene | None
    _paused: bool
    _actor_handles: dict[ActorHandle, _ActorInfo]
    _grid_objects: dict[str, list[str]]
    _plane_objects: dict[ActorHandle, list[str]]
    _hidden: set[ActorHandle]
    _scene_aabb: AABB | None
    _camera: CameraState
    _vertical_fov_deg: float
    _initial_vertical_fov_deg: float
    _capture_size: tuple[int, int]
    _background_color: tuple[float, float, float] | None
    _initial_look_dir: npt.NDArray | None
    _app_process: ViewerAppProcess | None

    ####################################################################################
    # Constructor
    ####################################################################################

    def __init__(self, cfg: MochiRendererViewerCfg) -> None:
        """
        Initialize the MochiRenderer viewer and connect to the server.

        Args:
            cfg: Configuration for the viewer.

        Raises:
            ValueError: If the coordinate system is invalid, or mirrors the scene while
                glTF assets are configured.
            RuntimeError: If connection to the mochi_renderer server fails.
        """
        self._cfg = cfg
        self._coordinate_system = self._resolve_coordinate_system(cfg.coordinate_system)
        self._coordinate_transform = CoordinateTransform(
            self._coordinate_system, _RENDER_COORDINATE_SYSTEM
        )
        render_from_source = self._coordinate_transform.source_to_target[:3, :3]
        # None marks the identity, so per-frame vertex buffers skip the conversion.
        self._render_from_source = (
            None
            if np.array_equal(render_from_source, np.eye(3))
            else render_from_source.astype(np.float64)
        )
        self._default_look_dir = self._from_render(_DEFAULT_LOOK_DIR)
        if self._coordinate_transform.encodes_reflection and (
            cfg.actor_gltfs or cfg.environment_gltf
        ):
            raise ValueError(
                f"glTF assets cannot be mirrored into the renderer's coordinate system, "
                f"so they require a right-handed one, not {self._coordinate_system}."
            )
        self._scene = None
        # Bumped on every set_scene so observers can detect a scene swap without relying
        # on object identity, whose address CPython may reuse for the replacement.
        self._scene_generation = 0
        self._paused = False
        self._actor_handles = {}
        self._grid_objects = {}
        self._plane_objects = {}
        self._hidden = set()
        self._scene_aabb = None
        self._camera = CameraState()
        # The server has no camera readback, so track the capture camera's size and FOV
        # here, seeded from its config.
        capture = (cfg.cameras or {}).get(cfg.camera_name)
        self._capture_size = (
            cfg.size if capture is None else (capture.width, capture.height)
        )
        self._initial_vertical_fov_deg = (
            _DEFAULT_VERTICAL_FOV_DEG
            if capture is None or capture.horizontal_fov_deg is None
            else _vertical_fov_deg(
                capture.horizontal_fov_deg, capture.width / capture.height
            )
        )
        self._vertical_fov_deg = self._initial_vertical_fov_deg
        self._background_color = None
        self._app_process = None

        port = cfg.port
        if cfg.auto_launch:
            self._app_process = self._launch_viewer_app(cfg)
            port = self._app_process.port
            atexit.register(self._terminate_app_process)

        # Create and connect the TCP client
        logger.info(f"Connecting to mochi_renderer at {cfg.host}:{port}...")
        self._client = MochiRendererClient(cfg.host, port)
        if not self._client.connect(cfg.connection_timeout, cfg.retry_attempts):
            self._terminate_app_process()
            raise RuntimeError(
                f"Failed to connect to mochi_renderer at {cfg.host}:{port}. "
                "Make sure the mochi_renderer server is running, or set "
                "auto_launch=True to start one automatically."
            )
        logger.info("Connected to mochi_renderer.")

        self._create_cameras(cfg)
        self._apply_camera_settings(cfg)
        self._seed_cached_camera_pose(cfg)
        # Set by the first rendered frame, once the environment has set its camera.
        self._initial_look_dir = None
        self._load_environment_ibl(cfg)

        # NOTE: environment glTF is loaded in set_scene(), not here.
        # Loading it here would cause a redundant load since set_scene()
        # destroys and re-creates it anyway.

    @staticmethod
    def _resolve_coordinate_system(
        coordinate_system: CoordinateSystem | str | None,
    ) -> CoordinateSystem:
        """Resolve the configured coordinate system, preset name or None (default)."""
        if coordinate_system is None:
            return DEFAULT_COORDINATE_SYSTEM
        if isinstance(coordinate_system, str):
            if coordinate_system not in COORDINATE_SYSTEMS:
                available_presets = ", ".join(f'"{cs}"' for cs in COORDINATE_SYSTEMS)
                raise ValueError(
                    f'Invalid coordinate system "{coordinate_system}". '
                    "You must provide a valid coordinate system object, or a "
                    "valid coordinate system preset, or None to use the default "
                    f"coordinate system. Available presets are: {available_presets}."
                )
            return COORDINATE_SYSTEMS[coordinate_system]
        return coordinate_system

    def get_coordinate_system(self) -> CoordinateSystem:
        """Returns the coordinate system the scene is expressed in. It can only be set at
        initialization time, through the config."""
        return self._coordinate_system

    def _to_render(self, vectors: npt.ArrayLike) -> npt.NDArray:
        """Map points or directions, a single triple or a flat/stacked buffer of them,
        from the scene's coordinate system to the server's."""
        vectors = np.asarray(vectors)
        if self._render_from_source is None:
            return vectors
        mapped = vectors.reshape(-1, 3) @ self._render_from_source.T
        return mapped.astype(vectors.dtype, copy=False).reshape(vectors.shape)

    def _from_render(self, vector: npt.NDArray) -> npt.NDArray:
        """Map a point or direction from the server's coordinate system to the scene's,
        as a new array."""
        if self._render_from_source is None:
            return vector.copy()
        return vector @ self._render_from_source

    def _indices_to_render(self, indices: npt.NDArray) -> npt.NDArray:
        """Triangle indices for the server, with the winding reversed when the conversion
        mirrors the scene, so faces keep pointing outwards."""
        if not self._coordinate_transform.encodes_reflection:
            return indices
        return indices.reshape(-1, 3)[:, ::-1]

    def _create_cameras(self, cfg: MochiRendererViewerCfg) -> None:
        """Create the capture camera and any cameras named in the config.

        The default camera is only created explicitly when it is not already part of
        cfg.cameras, since the server rejects duplicate names.
        """
        commands: list[CommandEntry] = []
        cameras = cfg.cameras
        if cameras is None or cfg.camera_name not in cameras:
            w, h = cfg.size
            commands.append(
                CommandEntry(text=f"vset /camera/{cfg.camera_name}/create {w} {h}")
            )
            # Pin the default camera's FOV to the value framing assumes, so it is known
            # and can be adjusted for interactive zoom.
            commands.append(
                CommandEntry(
                    text=f"vset /camera/{cfg.camera_name}/fov {self._vertical_fov_deg}"
                )
            )

        if cameras is not None:
            for cam_name, cam_cfg in cameras.items():
                commands.append(
                    CommandEntry(
                        text=f"vset /camera/{cam_name}/create {cam_cfg.width} {cam_cfg.height}"
                    )
                )

        self._client.request_batch(commands)

    def _apply_camera_settings(self, cfg: MochiRendererViewerCfg) -> None:
        """Send the initial pose and field of view of each configured camera."""
        if cfg.cameras is None:
            return

        commands: list[CommandEntry] = []
        for cam_name, cam_cfg in cfg.cameras.items():
            # A full transform is preferred over a look-at when both are given.
            if cam_cfg.position is not None and cam_cfg.rotation is not None:
                commands.append(
                    self._camera_transform_command(
                        cam_name, cam_cfg.position, cam_cfg.rotation
                    )
                )
            elif cam_cfg.look_from is not None and cam_cfg.look_at is not None:
                commands.append(
                    self._lookat_command(cam_name, cam_cfg.look_from, cam_cfg.look_at)
                )

            if cam_cfg.horizontal_fov_deg is not None:
                vfov_deg = _vertical_fov_deg(
                    cam_cfg.horizontal_fov_deg, cam_cfg.width / cam_cfg.height
                )
                commands.append(
                    CommandEntry(text=f"vset /camera/{cam_name}/fov {vfov_deg}")
                )

        if commands:
            self._client.request_batch(commands)

    def _seed_cached_camera_pose(self, cfg: MochiRendererViewerCfg) -> None:
        """Seed the cached pose from the capture camera's configuration.

        Framing, the follow camera and the trackball derive the camera from this cache,
        because the server offers no camera readback. Without a configured pose, the
        default pose is cached and sent with the first render.
        """
        camera = (cfg.cameras or {}).get(cfg.camera_name)
        if (
            camera is not None
            and camera.position is not None
            and camera.rotation is not None
        ):
            # A full transform wins, as in _apply_camera_settings.
            position = np.asarray(camera.position, dtype=np.float64)
            forward = Rotation.from_quat(camera.rotation).apply(_CAMERA_FORWARD)
            self._camera.look_from = position
            self._camera.look_at = position + forward
        elif (
            camera is not None
            and camera.look_from is not None
            and camera.look_at is not None
        ):
            self._camera.look_from = np.asarray(camera.look_from, dtype=np.float64)
            self._camera.look_at = np.asarray(camera.look_at, dtype=np.float64)
        else:
            self._camera.up = self._from_render(_DEFAULT_UP)
            self._set_camera_pose(self._from_render(_DEFAULT_LOOK_FROM), np.zeros(3))

    def _load_environment_ibl(self, cfg: MochiRendererViewerCfg) -> None:
        """Load the image-based lighting environment, if one is configured."""
        if not cfg.environment_ibl:
            logger.debug("No IBL configured (environment_ibl is None)")
            if cfg.background_color is not None:
                self.set_background_color(cfg.background_color)
            return

        logger.info(f"Loading IBL: {cfg.environment_ibl}")
        self._client.request_batch(
            [
                CommandEntry(text=f"vset /scene/ibl {cfg.environment_ibl}"),
                CommandEntry(
                    text=f"vset /scene/skybox_visible {'1' if cfg.skybox_visible else '0'}"
                ),
            ]
        )

    ####################################################################################
    # Server process management
    ####################################################################################

    @staticmethod
    def _launch_viewer_app(cfg: MochiRendererViewerCfg) -> ViewerAppProcess:
        """Spawn a mochi_viewer_app server for this viewer to drive.

        Raises:
            RuntimeError: If the executable cannot be found or fails to start.
        """
        if cfg.viewer_app_path is not None:
            executable = Path(cfg.viewer_app_path)
            if not executable.is_file():
                raise RuntimeError(
                    f"viewer_app_path does not point to a file: {executable}"
                )
        else:
            found = find_viewer_app_executable()
            if found is None:
                raise RuntimeError(
                    "Could not find the mochi_viewer_app executable. Build it, or "
                    "set viewer_app_path / the MOCHI_VIEWER_APP_PATH environment "
                    "variable."
                )
            executable = found

        windowed = cfg.windowed if cfg.windowed is not None else not cfg.offscreen
        return ViewerAppProcess(
            executable=executable,
            size=cfg.size,
            windowed=windowed,
            launch_timeout=cfg.launch_timeout,
        )

    def _terminate_app_process(self) -> None:
        """Stop the spawned server, if this viewer started one."""
        if self._app_process is None:
            return
        self._app_process.terminate()
        self._app_process = None
        # The hook holds a strong reference to this viewer; drop it.
        atexit.unregister(self._terminate_app_process)

    ####################################################################################
    # Viewer management methods
    ####################################################################################

    def render(
        self,
        camera_names: list[str] | None = None,
        out: RenderFrame | dict[str, RenderFrame] | None = None,
    ) -> RenderFrame | dict[str, RenderFrame] | None:
        """
        Render the current frame.

        Updates all actor transforms / mesh data and captures images.

        Args:
            camera_names: If provided, capture from all listed cameras and
                return a dict mapping camera name to RGBA image. If None,
                capture from the default camera only.
            out: Optional storage to receive the images without allocating: one
                array when camera_names is None, or a dict keyed by camera name.
                Each must be a writable, C-contiguous uint8 array with room for the
                12-byte image header plus the pixels, and not a view of another
                array. A returned image is a view of the storage it was received
                into, which is its ``base``: your array when it is large enough, or a
                new one otherwise. Reuse it with ``frame = viewer.render(out=storage)``
                followed by ``storage = frame.base``. The caller owns the storage;
                the next render into it overwrites earlier views, so each camera
                needs its own storage.

        Returns:
            When camera_names is None: RGBA image as a numpy uint8 array with
                shape (H, W, 4), or None if no scene is set, the capture failed,
                or the viewer is not offscreen (the server presents to its own
                window, so no readback is performed).
            When camera_names is provided: dict mapping camera name to RGBA
                image array. Cameras that fail to capture are omitted.

        Raises:
            ValueError: If *out* is a dict without *camera_names* or a single array
                with them, or a storage array in it is not a writable, C-contiguous
                uint8 array, or is a view of another array. Checked before the
                viewer changes any state.
        """
        if self._scene is None:
            return {} if camera_names is not None else None

        if out is not None:
            if isinstance(out, dict) != (camera_names is not None):
                raise ValueError(
                    "out must be a dict keyed by camera name when camera_names is "
                    "given, and a single array otherwise"
                )
            for storage in out.values() if isinstance(out, dict) else [out]:
                # NumPy gives views of a view the viewed array as their base, so
                # frame.base would not be this storage.
                if (
                    not isinstance(storage, np.ndarray)
                    or storage.dtype != np.uint8
                    or memoryview(storage).readonly
                    or not storage.flags.c_contiguous
                    or isinstance(storage.base, np.ndarray)
                ):
                    raise ValueError(
                        "Frame storage must be a writable, C-contiguous uint8 array "
                        "that is not a view of another array"
                    )

        commands: list[CommandEntry] = []
        self._sync_actors(self._scene, commands)
        # Camera commands must precede the captures: _decode_capture indexes images
        # off the tail of the response list.
        commands.extend(self._build_camera_commands())
        if (
            self._initial_look_dir is None
            and not self._camera.frame_camera_on_next_update
        ):
            # The first view the user sees, so "reset camera" can return to it.
            self._initial_look_dir = self.get_camera_look_dir()

        if camera_names is None and not self._cfg.offscreen:
            # The window presents itself, so a readback would only buy a blocking
            # flush per step.
            if commands:
                self._client.request_batch(commands)
            return None

        response_buffers: list[memoryview | None] = [None] * len(commands)
        if camera_names is not None:
            for cam_name in camera_names:
                storage = None if out is None else out.get(cam_name)
                response_buffers.append(
                    None if storage is None else memoryview(storage)
                )
                commands.append(CommandEntry(text=f"vget /camera/{cam_name}/lit"))
        else:
            response_buffers.append(None if out is None else memoryview(out))
            commands.append(
                CommandEntry(text=f"vget /camera/{self._cfg.camera_name}/lit")
            )

        responses = self._client.request_batch(
            commands, response_buffers=response_buffers
        )

        if not responses:
            return {} if camera_names is not None else None

        return self._decode_capture(responses, camera_names)

    def _sync_actors(self, scene: Scene, commands: list[CommandEntry]) -> None:
        """Detect added/removed/updated actors and append commands."""
        current_handles: dict[ActorHandle, Actor] = {}
        current_planes: dict[ActorHandle, Actor] = {}
        live_handles: set[ActorHandle] = set()

        def _gather(actor: Actor) -> None:
            live_handles.add(actor.get_handle())
            if not actor.get_surface_mesh().is_empty():
                current_handles[actor.get_handle()] = actor
            elif is_plane_actor(actor):
                current_planes[actor.get_handle()] = actor

        scene.for_each_actor(_gather)
        # Forget departed actors, so an actor that later reuses a handle starts visible.
        self._hidden &= live_handles

        self._sync_planes(current_planes, commands)

        current_set = set(current_handles.keys())
        known_set = set(self._actor_handles.keys())

        for handle in known_set - current_set:
            info = self._actor_handles.pop(handle)
            commands.append(CommandEntry(text=f"vset /object/{info.name}/destroy"))

        for handle in current_set - known_set:
            commands.extend(self._create_actor_mesh(current_handles[handle]))

        for handle, info in self._actor_handles.items():
            actor = current_handles.get(handle)
            if actor is not None:
                commands.extend(self._update_actor(actor, info))

        self._update_scene_aabb()

    def _sync_planes(
        self, current: dict[ActorHandle, Actor], commands: list[CommandEntry]
    ) -> None:
        """Create grids for new ground planes and drop grids for departed ones.

        Planes are static, so their grids are built once and never updated.
        """
        for handle in set(self._plane_objects) - set(current):
            commands.extend(self._destroy_plane_commands(handle))

        for handle in set(current) - set(self._plane_objects):
            commands.extend(self._create_plane_grid(handle, current[handle]))

    def _create_plane_grid(
        self, handle: ActorHandle, actor: Actor
    ) -> list[CommandEntry]:
        """Build the checker grid standing in for a plane collision shape."""
        pose = plane_center_and_axes(actor)
        if pose is None:
            return []
        center, axes = pose

        meshes = build_grid_meshes(
            size=np.inf,
            center=center,
            period=PLANE_PERIOD,
            axes=axes,
            style="checker",
            color_1=PLANE_COLOR_1,
            color_2=PLANE_COLOR_2,
            # The inferred normal's sign is irrelevant when both sides are drawn.
            double_sided=True,
        )

        commands: list[CommandEntry] = []
        object_names: list[str] = []
        unique_name = f"{actor.get_name()}_h{handle.value}"
        for index, mesh in enumerate(meshes):
            object_name = f"{_PLANE_OBJECT_PREFIX}/{unique_name}/{index}"
            object_names.append(object_name)
            commands.extend(self._upload_grid_mesh(object_name, mesh))

        self._plane_objects[handle] = object_names
        # Preserve a hidden state if this plane was toggled off before being recreated.
        if handle in self._hidden:
            commands.extend(
                CommandEntry(text=f"vset /object/{name}/hide") for name in object_names
            )
        return commands

    def _destroy_plane_commands(self, handle: ActorHandle) -> list[CommandEntry]:
        """Destroy commands for a previously uploaded ground plane, if any."""
        return [
            CommandEntry(text=f"vset /object/{object_name}/destroy")
            for object_name in self._plane_objects.pop(handle, [])
        ]

    def _decode_capture(
        self,
        responses: list,
        camera_names: list[str] | None,
    ) -> RenderFrame | dict[str, RenderFrame] | None:
        """Decode image responses from a render batch."""
        if camera_names is not None:
            num_cameras = len(camera_names)
            image_responses = responses[-num_cameras:]
            result: dict[str, RenderFrame] = {}
            for cam_name, resp in zip(camera_names, image_responses):
                if resp.type == 1:
                    frame = self._decode_image_response(resp.data)
                    if frame is not None:
                        result[cam_name] = frame
            return result

        image_resp = responses[-1]
        if image_resp.type != 1:
            return None
        return self._decode_image_response(image_resp.data)

    def close(self) -> None:
        """
        Close the viewer and disconnect from the server.

        Calling close on an already closed viewer has no effect.
        """
        if not self._client.is_connected():
            self._terminate_app_process()
            return

        # Destroy all known actors and environment glTF
        commands: list[CommandEntry] = []
        for info in self._actor_handles.values():
            commands.append(CommandEntry(text=f"vset /object/{info.name}/destroy"))
        for name in list(self._grid_objects):
            commands.extend(self._destroy_grid_commands(name))
        for handle in list(self._plane_objects):
            commands.extend(self._destroy_plane_commands(handle))
        if self._cfg.environment_gltf:
            commands.append(CommandEntry(text="vset /object/__environment__/destroy"))
        if commands:
            try:
                self._client.request_batch(commands)
            except Exception:
                pass

        self._actor_handles.clear()
        self._client.disconnect()
        self._terminate_app_process()
        logger.info("MochiRenderer viewer closed.")

    def set_paused(self, paused: bool) -> None:
        """Sets the paused state of the viewer."""
        self._paused = paused

    def is_paused(self) -> bool:
        """Returns the paused state of the viewer."""
        return self._paused

    def set_camera_view(
        self,
        look_from: list[float] | npt.NDArray | None = None,
        look_at: list[float] | npt.NDArray | None = None,
        up_dir: list[float] | npt.NDArray | None = None,
        **kwargs,
    ) -> None:
        """Set the camera position and target via a look-at command.

        Args:
            look_from: World-space eye position.
            look_at: World-space point to look at.
            up_dir: Up direction fixing the camera's roll. Sticky: it is reused by
                framing and the follow camera until changed. None leaves it as is; if
                none was ever set, the coordinate system's up is used, or its right
                when looking along up.
        """
        if up_dir is not None:
            self._camera.up = np.asarray(up_dir, dtype=np.float64)
        if look_from is None or look_at is None:
            return
        self._set_camera_pose(
            np.asarray(look_from, dtype=np.float64),
            np.asarray(look_at, dtype=np.float64),
        )
        self._client.request_batch(self._camera_pose_commands())
        self._camera.dirty = False

    def set_background_color(self, color: tuple[float, float, float]) -> None:
        """Set the color rendered behind the scene.

        Args:
            color: Red, green and blue in [0, 1], in sRGB space.

        Note:
            This replaces any IBL environment, since the skybox carries both.
        """
        r, g, b = srgb_to_linear(color)
        self._client.request(f"vset /scene/background {r:.6f} {g:.6f} {b:.6f}")
        self._background_color = (color[0], color[1], color[2])

    def get_background_color(self) -> tuple[float, float, float] | None:
        """Return the background color last set, in sRGB, or None if none was set (the
        IBL or the server's default is shown)."""
        return self._background_color

    ####################################################################################
    # Camera framing and follow camera
    ####################################################################################

    def get_camera_look_dir(self) -> npt.NDArray:
        """Returns the normalized direction the camera is pointing in.

        The server has no camera readback, so this derives from the locally cached
        pose, falling back to the default pose's direction if it has none.
        """
        state = self._camera
        if state.look_from is None or state.look_at is None:
            return self._default_look_dir.copy()
        look_dir = state.look_at - state.look_from
        norm = float(np.linalg.norm(look_dir))
        if norm == 0.0:
            return self._default_look_dir.copy()
        return look_dir / norm

    def get_camera_pose(
        self,
    ) -> tuple[npt.NDArray, npt.NDArray, npt.NDArray | None] | None:
        """Return the current ``(look_from, look_at, up)`` pose, or None if unset.

        The server has no camera readback, so this returns copies of the locally cached
        pose. ``up`` is None if no roll has been fixed yet.
        """
        state = self._camera
        if state.look_from is None or state.look_at is None:
            return None
        up = None if state.up is None else state.up.copy()
        return state.look_from.copy(), state.look_at.copy(), up

    def get_vertical_fov(self) -> float:
        """Return the observation camera's vertical field of view, in degrees."""
        return self._vertical_fov_deg

    def set_vertical_fov(self, degrees: float) -> None:
        """Set the observation camera's vertical field of view (clamped), in degrees.

        The server's look-at command controls only the view direction (not the eye
        distance), so interactive zoom is done by narrowing/widening the FOV.
        """
        self._vertical_fov_deg = float(np.clip(degrees, _MIN_FOV_DEG, _MAX_FOV_DEG))
        self._client.request(
            f"vset /camera/{self._cfg.camera_name}/fov {self._vertical_fov_deg}"
        )

    def set_capture_size(self, width: int, height: int) -> None:
        """Resize the observation (capture) camera; the next render returns this size.

        Also updates the cached capture size so scene framing keeps the correct aspect
        ratio. Used by the interactive window to make the render resolution follow the
        frame area (the part of the window the docked panels leave free).
        """
        width = max(1, int(width))
        height = max(1, int(height))
        self._capture_size = (width, height)
        self._client.request(
            f"vset /camera/{self._cfg.camera_name}/size {width} {height}"
        )

    def reset_camera(self) -> None:
        """Reset zoom (FOV) and orientation to their initial values and reframe.

        Restores the configured field of view (or the default one) and reframes from
        the look direction of the first rendered frame, so both the zoom and the
        orientation return to their initial state.
        """
        self.set_vertical_fov(self._initial_vertical_fov_deg)
        self.frame_scene(look_dir=self._initial_look_dir)

    def frame_scene(
        self,
        look_dir: npt.NDArray | None = None,
        fly_to: bool = False,
    ) -> None:
        """Center the camera on the scene's AABB.

        Environments call this before any actor exists, so framing is deferred to the
        first render that has a bounded scene.

        Args:
            look_dir: Direction to look along. Defaults to the current direction.
            fly_to: Ignored; the server has no camera animation.
        """
        if self._scene_aabb is None:
            self._camera.frame_camera_on_next_update = True
            self._camera.frame_camera_direction = look_dir
            self._camera.frame_camera_attempts_left = _MAX_FRAME_DEFER_RENDERS
            return

        if look_dir is None:
            look_dir = self.get_camera_look_dir()
        self.frame_scene_from_direction(
            look_dir, self._compute_frame_distance(self._scene_aabb), fly_to
        )

    def frame_scene_from_direction(
        self,
        look_dir: npt.NDArray,
        distance: float,
        fly_to: bool = False,
    ) -> None:
        """Center the camera on the scene's AABB from a given direction and distance.

        Args:
            look_dir: Direction to look along.
            distance: Distance from the scene center.
            fly_to: Ignored; the server has no camera animation.
        """
        if self._scene_aabb is None:
            return

        look_at = self._scene_aabb.center
        self._set_camera_pose(
            look_at - distance * np.asarray(look_dir, dtype=np.float64), look_at
        )

        # Snap the follow camera so it resumes from the framed pose.
        self._camera.smoothed_target_position = look_at
        self._camera.distance = distance
        self._camera.smoothed_distance = distance

    def is_follow_camera_enabled(self) -> bool:
        """Returns whether the follow camera is enabled."""
        return self._camera.use_follow_camera

    def set_enable_follow_camera(self, enabled: bool) -> None:
        """Enable the follow camera, which keeps the camera centered on the actors."""
        self._camera.use_follow_camera = enabled

    def set_follow_camera_smoothness(self, smoothing: float) -> None:
        """Set the follow camera smoothing factor, clamped to [0, 1]."""
        self._camera.smoothing = float(np.clip(smoothing, 0.0, 1.0))

    def get_follow_camera_smoothness(self) -> float:
        """Returns the follow camera smoothing factor."""
        return self._camera.smoothing

    def set_compute_automatic_distance(self, enabled: bool) -> None:
        """Set whether the follow camera reframes the scene each update."""
        self._camera.automatic_distance = enabled

    def is_automatic_distance_enabled(self) -> bool:
        """Returns whether automatic follow distance is enabled."""
        return self._camera.automatic_distance

    def snap_follow_camera(self) -> None:
        """Snap the follow camera to the current scene bounds without smoothing."""
        if not self._camera.use_follow_camera:
            return
        if self._scene_aabb is None or self._camera.distance <= 0:
            return
        self.frame_scene_from_direction(
            self.get_camera_look_dir(), self._camera.distance
        )

    def _set_camera_pose(self, look_from: npt.NDArray, look_at: npt.NDArray) -> None:
        """Record a new camera pose to be sent with the next batch."""
        self._camera.look_from = np.asarray(look_from, dtype=np.float64)
        self._camera.look_at = np.asarray(look_at, dtype=np.float64)
        self._camera.dirty = True

    def _camera_pose_commands(self) -> list[CommandEntry]:
        """Look-at commands for the cached pose, or an empty list if unset."""
        state = self._camera
        if state.look_from is None or state.look_at is None:
            return []

        # A single observation-camera pose. In windowed mode the server also steers the
        # interactive window from this command (see the presented-camera-moved hook), so
        # no separate presentation command is needed.
        return [
            self._lookat_command(
                self._cfg.camera_name, state.look_from, state.look_at, state.up
            )
        ]

    def _lookat_command(
        self,
        camera_name: str,
        look_from: npt.ArrayLike,
        look_at: npt.ArrayLike,
        up: npt.ArrayLike | None = None,
    ) -> CommandEntry:
        """Look-at command in the server's coordinates. Without an up direction, the
        coordinate system's up is used, or its right when looking along up."""
        look_from = np.asarray(look_from, dtype=np.float64)
        look_at = np.asarray(look_at, dtype=np.float64)
        if up is None:
            up = self._coordinate_system.up.to_vector().astype(np.float64)
            look_dir = look_at - look_from
            if abs(np.dot(up, look_dir)) > 0.99 * np.linalg.norm(look_dir):
                up = self._coordinate_system.right.to_vector().astype(np.float64)
        f = self._to_render(look_from)
        t = self._to_render(look_at)
        u = self._to_render(np.asarray(up, dtype=np.float64))
        return CommandEntry(
            text=(
                f"vset /camera/{camera_name}/lookat "
                f"{f[0]} {f[1]} {f[2]} {t[0]} {t[1]} {t[2]} {u[0]} {u[1]} {u[2]}"
            )
        )

    def _camera_transform_command(
        self,
        camera_name: str,
        position: npt.ArrayLike,
        rotation: npt.ArrayLike,
    ) -> CommandEntry:
        """Transform command for a camera pose in the server's coordinates."""
        p = self._to_render(np.asarray(position, dtype=np.float64))
        r = np.asarray(rotation, dtype=np.float64)
        if self._render_from_source is not None:
            # The camera frame stays right-handed, so a mirroring conversion keeps the
            # view and up axes and flips the camera's right axis instead.
            handedness = np.diag([np.linalg.det(self._render_from_source), 1.0, 1.0])
            r = Rotation.from_matrix(
                self._render_from_source
                @ Rotation.from_quat(r).as_matrix()
                @ handedness
            ).as_quat()
        return CommandEntry(
            text=(
                f"vset /camera/{camera_name}/transform "
                f"{p[0]} {p[1]} {p[2]} {r[0]} {r[1]} {r[2]} {r[3]}"
            )
        )

    def _build_camera_commands(self) -> list[CommandEntry]:
        """Resolve pending framing and follow-camera updates into commands.

        Returns commands rather than issuing them so they can join the render batch
        ahead of the image captures.
        """
        state = self._camera

        if state.frame_camera_on_next_update:
            if self._scene_aabb is not None:
                state.frame_camera_on_next_update = False
                self.frame_scene(state.frame_camera_direction)
                state.frame_camera_direction = None
            else:
                state.frame_camera_attempts_left -= 1
                if state.frame_camera_attempts_left <= 0:
                    logger.warning(
                        "Giving up on deferred frame_scene: the scene has no actors "
                        "with usable bounds."
                    )
                    state.frame_camera_on_next_update = False
                    state.frame_camera_direction = None

        if state.use_follow_camera:
            self._update_follow_camera()

        if not state.dirty:
            return []
        state.dirty = False
        return self._camera_pose_commands()

    def _update_follow_camera(self) -> None:
        """Move the camera towards the scene center, with smoothing."""
        aabb = self._scene_aabb
        if aabb is None:
            return

        state = self._camera

        def smooth(current: npt.NDArray | float, target: npt.NDArray | float):
            return state.smoothing * current + (1 - state.smoothing) * target

        center = aabb.center
        if state.smoothed_target_position is None:
            state.smoothed_target_position = center
        else:
            state.smoothed_target_position = smooth(
                state.smoothed_target_position, center
            )

        # Only smooth the distance when we control it; otherwise smoothing would
        # fight any distance the user has dialed in.
        if state.automatic_distance:
            state.distance = self._compute_frame_distance(aabb)
            state.smoothed_distance = smooth(state.smoothed_distance, state.distance)
        else:
            if state.distance <= 0:
                state.distance = self._initial_follow_distance(aabb)
            state.smoothed_distance = state.distance

        look_at = state.smoothed_target_position
        self._set_camera_pose(
            look_at - state.smoothed_distance * self.get_camera_look_dir(), look_at
        )

    def _initial_follow_distance(self, aabb: AABB) -> float:
        """Distance to use when the follow camera is enabled before any framing.

        Keeps the eye where the user put it if there is a pose to measure, so
        enabling the follow camera does not also move the camera."""
        state = self._camera
        if state.look_from is not None and state.look_at is not None:
            distance = float(np.linalg.norm(state.look_from - state.look_at))
            if distance > 0:
                return distance
        return self._compute_frame_distance(aabb)

    def _compute_frame_distance(self, aabb: AABB) -> float:
        """Distance at which the given AABB fills the capture camera's frustum."""
        extents = np.maximum(aabb.extents, _MIN_SCENE_EXTENT)
        radius = float(np.linalg.norm(extents)) / 2
        half_fov, aspect = self._capture_camera_frustum()
        max_half_fov = max(half_fov, half_fov / aspect)
        return 1.5 * radius / math.tan(max_half_fov)

    def _capture_camera_frustum(self) -> tuple[float, float]:
        """Vertical half field of view in radians, and aspect ratio, of the capture
        camera, tracked locally since the server has no readback. Uses the initial FOV,
        not the zoomed one, so framing does not undo interactive zoom."""
        width, height = self._capture_size
        return math.radians(self._initial_vertical_fov_deg) / 2, width / height

    def _update_scene_aabb(self) -> None:
        """Recompute the scene AABB from the tracked actors.

        Grids are deliberately excluded: they are tracked separately from
        ``_actor_handles`` so an "infinite" floor cannot dominate the framing.
        """
        aabbs = [
            info.world_aabb
            for info in self._actor_handles.values()
            if info.world_aabb is not None
        ]
        self._scene_aabb = AABB.from_aabbs(aabbs) if aabbs else None

    def add_grid(
        self,
        name: str,
        size: float = np.inf,
        center: npt.NDArray | None = None,
        period: float = 1,
        axes: str = "xz",
        style: str = "checker",
        color_1: tuple[float, float, float] = (0.9, 0.9, 0.9),
        color_2: tuple[float, float, float] = (1, 1, 1),
        double_sided: bool = False,
    ) -> None:
        """Add a reference grid, replacing any existing grid with the same name.

        The server has no grid primitive, so the grid is tessellated locally and
        uploaded as two single-colored meshes.

        Args:
            name: Unique name of the grid.
            size: Side length of the grid. Infinite grids are clamped to a finite
                extent.
            center: World-space center of the grid plane. Defaults to the origin.
            period: Spacing between grid lines.
            axes: Plane the grid lies in: "xy", "xz" or "yz".
            style: "checker" for alternating cells, "grid" for lines on a backdrop.
            color_1: Checker cells of even parity, or the grid lines.
            color_2: Checker cells of odd parity, or the backdrop.
            double_sided: Whether the grid is visible from both sides.
        """
        meshes = build_grid_meshes(
            size=size,
            center=np.zeros(3) if center is None else np.asarray(center),
            period=period,
            axes=axes,
            style=style,
            color_1=color_1,
            color_2=color_2,
            double_sided=double_sided,
        )

        commands: list[CommandEntry] = []
        commands.extend(self._destroy_grid_commands(name))

        object_names: list[str] = []
        for index, mesh in enumerate(meshes):
            object_name = f"{_GRID_OBJECT_PREFIX}/{name}/{index}"
            object_names.append(object_name)
            commands.extend(self._upload_grid_mesh(object_name, mesh))

        self._grid_objects[name] = object_names
        self._client.request_batch(commands)

    def _upload_grid_mesh(self, object_name: str, mesh: GridMesh) -> list[CommandEntry]:
        """Mesh-create and show commands for one tessellated grid piece."""
        r, g, b = srgb_to_linear(mesh.color)
        return [
            CommandEntry(
                text=(
                    f"vset /object/{object_name}/mesh "
                    f"{len(mesh.positions)} {mesh.indices.size} 0 "
                    f"{r:.6f} {g:.6f} {b:.6f}"
                ),
                binary_data=(
                    self._to_render(mesh.positions).tobytes()
                    + self._to_render(mesh.normals).tobytes()
                    + self._indices_to_render(mesh.indices).tobytes()
                ),
            ),
            CommandEntry(text=f"vset /object/{object_name}/show"),
        ]

    def _destroy_grid_commands(self, name: str) -> list[CommandEntry]:
        """Destroy commands for a previously uploaded grid, if any."""
        object_names = self._grid_objects.pop(name, [])
        return [
            CommandEntry(text=f"vset /object/{object_name}/destroy")
            for object_name in object_names
        ]

    def add_ui_tab(self, name: str, builder: Callable[[], None]) -> None:
        """No-op ΓÇö the mochi_renderer server has no UI subsystem."""
        pass

    def consume_reset_request(self) -> bool:
        """Return False -- this viewer has no UI, so it can never request a reset."""
        return False

    def user_requested_close(self) -> bool:
        """Return False -- this viewer has no window the user can close."""
        return False

    def create_camera(self, name: str, width: int, height: int) -> None:
        """Create a named camera on the renderer server.

        Args:
            name: Camera name.
            width: Image width in pixels.
            height: Image height in pixels.
        """
        self._client.request(f"vset /camera/{name}/create {width} {height}")

    def set_camera_transform(
        self,
        name: str,
        position: list[float] | npt.NDArray,
        rotation: list[float] | npt.NDArray,
    ) -> None:
        """Set a named camera's pose via position and orientation.

        Args:
            name: Camera name.
            position: Camera position (x, y, z).
            rotation: Camera orientation as an XYZW quaternion.
        """
        self._client.request(
            self._camera_transform_command(name, position, rotation).text
        )

    def set_camera_lookat(
        self,
        name: str,
        look_from: list[float] | npt.NDArray,
        look_at: list[float] | npt.NDArray,
    ) -> None:
        """Set a named camera's position and target.

        Args:
            name: Camera name.
            look_from: Eye position (x, y, z).
            look_at: Target position (x, y, z).
        """
        self._client.request(self._lookat_command(name, look_from, look_at).text)

    ####################################################################################
    # Scene management
    ####################################################################################

    def get_scene(self) -> Scene | None:
        """Returns the mochi scene associated with the viewer."""
        return self._scene

    def get_scene_generation(self) -> int:
        """Returns a counter incremented on every :meth:`set_scene` call.

        Lets callers detect that the scene was replaced. Prefer this over ``id(scene)``,
        which can repeat when a collected scene's address is reused by its replacement.
        """
        return self._scene_generation

    def set_scene(self, scene: Scene | None) -> None:
        """
        Set the mochi scene to render.

        Iterates all actors in the scene, extracts meshes, and sends
        mesh creation commands to the renderer. Also loads the
        environment glTF if configured.

        Args:
            scene: The mochi scene to render, or None to clear.
        """
        # Visibility overrides are keyed by the outgoing scene's handles; drop them.
        self._hidden.clear()

        # Clean up previous scene actors and environment
        cleanup_commands: list[CommandEntry] = []
        if self._actor_handles:
            for info in self._actor_handles.values():
                cleanup_commands.append(
                    CommandEntry(text=f"vset /object/{info.name}/destroy")
                )
            self._actor_handles.clear()
        # Ground planes belong to the outgoing scene's actors.
        for handle in list(self._plane_objects):
            cleanup_commands.extend(self._destroy_plane_commands(handle))
        if self._cfg.environment_gltf:
            logger.info("[env_gltf] destroying environment gltf")
            cleanup_commands.append(
                CommandEntry(text="vset /object/__environment__/destroy")
            )
        if cleanup_commands:
            try:
                self._client.request_batch(cleanup_commands)
            except Exception:
                pass

        self._scene = scene
        self._scene_generation += 1

        if scene is None:
            self._scene_aabb = None
            return

        # Iterate actors and create meshes
        commands: list[CommandEntry] = []

        # Load environment glTF if configured
        commands.extend(self._environment_gltf_commands())

        def _create(actor: Actor) -> None:
            if not actor.get_surface_mesh().is_empty():
                cmds = self._create_actor_mesh(actor)
                commands.extend(cmds)
            elif is_plane_actor(actor):
                commands.extend(self._create_plane_grid(actor.get_handle(), actor))

        scene.for_each_actor(_create)
        self._update_scene_aabb()

        if commands:
            self._client.request_batch(commands)

    def _environment_gltf_commands(self) -> list[CommandEntry]:
        """Commands loading the environment glTF, if configured, rotated into the
        server's coordinate system."""
        if not self._cfg.environment_gltf:
            return []
        commands = [
            CommandEntry(
                text=f"vset /object/__environment__/gltf {self._cfg.environment_gltf}"
            )
        ]
        if self._render_from_source is not None:
            x, y, z, w = Rotation.from_matrix(self._render_from_source).as_quat()
            commands.append(
                CommandEntry(
                    text=f"vset /object/__environment__/xform 0 0 0 {x} {y} {z} {w}"
                )
            )
        return commands

    def set_actor_visible(self, handle: ActorHandle, visible: bool) -> None:
        """Show or hide an actor's rendered object(s).

        Keyed by ``ActorHandle`` because actor names are not unique. A ground-plane actor
        maps to several generated grid objects; all of them are toggled together. This is a
        no-op for a handle with no render object (e.g. an articulated root that has no
        surface mesh of its own).
        """
        if visible:
            self._hidden.discard(handle)
        else:
            self._hidden.add(handle)
        names = self._object_names_for_handle(handle)
        if not names:
            return
        action = "show" if visible else "hide"
        self._client.request_batch(
            [CommandEntry(text=f"vset /object/{name}/{action}") for name in names]
        )

    def is_actor_visible(self, handle: ActorHandle) -> bool:
        """Return whether the actor is currently shown (actors are shown by default)."""
        return handle not in self._hidden

    def _object_names_for_handle(self, handle: ActorHandle) -> list[str]:
        """Server object name(s) backing an actor handle, or empty if it has none."""
        info = self._actor_handles.get(handle)
        if info is not None:
            return [info.name]
        return list(self._plane_objects.get(handle, []))

    ####################################################################################
    # Actor helpers
    ####################################################################################

    def _create_actor_mesh(self, actor: Actor) -> list[CommandEntry]:
        """
        Extract mesh data from an actor and build mesh creation commands.

        If the actor has a glTF mapping in ``cfg.actor_gltfs``, a glTF load
        command is sent instead of the raw physics mesh.

        Returns:
            List of CommandEntry objects for mesh creation + initial transform.
        """
        name = actor.get_name()
        actor_type = actor.get_type()

        # Check if this actor has a glTF override
        gltf_path = self._cfg.actor_gltfs.get(name) if self._cfg.actor_gltfs else None

        commands: list[CommandEntry] = []

        if gltf_path is not None:
            # Use glTF mesh instead of physics mesh
            commands.append(CommandEntry(text=f"vset /object/{name}/gltf {gltf_path}"))

            # Send initial transform
            xform_cmd = self._build_xform_command(actor, name)
            if xform_cmd is not None:
                commands.append(xform_cmd)

            # Track this actor
            info = _ActorInfo(name=name, actor_type=actor_type, num_verts=0)
            self._actor_handles[actor.get_handle()] = info
            self._refresh_actor_aabb(actor, info)
        else:
            # Use physics mesh (original path)
            is_dynamic = actor_type in (
                ActorType.SOFT,
                ActorType.SHELL,
            )

            # Get mesh data
            actor.register_query_and_compute(sdp.QueryType.SURFACE_NODE_POSITIONS)
            positions = np.asarray(
                actor.get_surface_mesh_node_positions_local(), dtype=np.float32
            ).ravel()

            indices = np.asarray(
                actor.get_surface_mesh().connectivity, dtype=np.int32
            ).ravel()

            actor.register_query_and_compute(sdp.QueryType.SURFACE_NODE_NORMALS)
            normals = np.asarray(
                actor.get_surface_mesh_node_normals_local(), dtype=np.float32
            ).ravel()

            num_verts = len(positions) // 3
            num_indices = len(indices)

            # Build binary payload: [positions] [normals] [indices]
            binary_data = (
                self._to_render(positions).tobytes()
                + self._to_render(normals).tobytes()
                + self._indices_to_render(indices).tobytes()
            )

            # Color actors so the links of a robot stay distinguishable; the server
            # otherwise renders every mesh in the same grey.
            r, g, b = srgb_to_linear(actor_color(actor.get_handle().value))

            commands.append(
                CommandEntry(
                    text=(
                        f"vset /object/{name}/mesh "
                        f"{num_verts} {num_indices} {int(is_dynamic)} "
                        f"{r:.6f} {g:.6f} {b:.6f}"
                    ),
                    binary_data=binary_data,
                )
            )

            # Send initial transform
            xform_cmd = self._build_xform_command(actor, name)
            if xform_cmd is not None:
                commands.append(xform_cmd)

            # Show the object
            commands.append(CommandEntry(text=f"vset /object/{name}/show"))

            # Track this actor
            info = _ActorInfo(name=name, actor_type=actor_type, num_verts=num_verts)
            if actor_type in _LOCAL_BOUNDS_ACTOR_TYPES:
                info.local_aabb = _local_aabb_from_positions(positions)
            self._actor_handles[actor.get_handle()] = info
            self._refresh_actor_aabb(actor, info)

        # Preserve a hidden state if this actor was toggled off before being recreated.
        if actor.get_handle() in self._hidden:
            commands.append(CommandEntry(text=f"vset /object/{name}/hide"))

        return commands

    def _update_actor(self, actor: Actor, info: _ActorInfo) -> list[CommandEntry]:
        """
        Build update commands for an existing actor.

        For rigid/articulated actors: sends transform update.
        For soft FEM actors: sends updated mesh vertex data + transform.
        """
        commands: list[CommandEntry] = []

        if info.actor_type in (ActorType.SOFT, ActorType.SHELL):
            # Re-extract vertex positions and normals
            actor.register_query_and_compute(sdp.QueryType.SURFACE_NODE_POSITIONS)
            positions = np.asarray(
                actor.get_surface_mesh_node_positions_local(), dtype=np.float32
            ).ravel()

            actor.register_query_and_compute(sdp.QueryType.SURFACE_NODE_NORMALS)
            normals = np.asarray(
                actor.get_surface_mesh_node_normals_local(), dtype=np.float32
            ).ravel()

            num_verts = len(positions) // 3
            binary_data = (
                self._to_render(positions).tobytes()
                + self._to_render(normals).tobytes()
            )
            info.local_aabb = _local_aabb_from_positions(positions)

            commands.append(
                CommandEntry(
                    text=f"vset /object/{info.name}/update_mesh {num_verts}",
                    binary_data=binary_data,
                )
            )

        # Always update transform
        xform_cmd = self._build_xform_command(actor, info.name)
        if xform_cmd is not None:
            commands.append(xform_cmd)

        self._refresh_actor_aabb(actor, info)

        return commands

    @staticmethod
    def _refresh_actor_aabb(actor: Actor, info: _ActorInfo) -> None:
        """Recompute an actor's world bounds, from its cached local bounds when
        it has them and from the engine otherwise."""
        if info.local_aabb is None:
            info.world_aabb = _queried_world_aabb(actor)
            return
        world_aabb = AABB.empty()
        world_aabb.compute_from_transformed_aabb(
            info.local_aabb, _root_transform_matrix(actor)
        )
        info.world_aabb = world_aabb

    def _build_xform_command(self, actor: Actor, name: str) -> CommandEntry | None:
        """Build a transform command for an actor, or None if it has no transform.

        The server handles Y-up → Z-up correction for glTF objects.
        """
        if not actor.has_root_transform():
            return None

        xform = actor.get_root_transform()
        pos = self._to_render(np.asarray(xform.translation, dtype=np.float64))
        quat = np.asarray(xform.rotation, dtype=np.float64)
        if self._render_from_source is not None:
            if name in (self._cfg.actor_gltfs or {}):
                # The asset's own frame is not converted, so rotate it into the server's.
                quat = (
                    Rotation.from_matrix(self._render_from_source)
                    * Rotation.from_quat(quat)
                ).as_quat()
            else:
                # Physics meshes are uploaded converted, so conjugate the rotation.
                quat = self._coordinate_transform.rotation_to_target(quat)

        return CommandEntry(
            text=(
                f"vset /object/{name}/xform "
                f"{pos[0]} {pos[1]} {pos[2]} "
                f"{quat[0]} {quat[1]} {quat[2]} {quat[3]}"
            )
        )

    @staticmethod
    def _decode_image_response(data: bytes | memoryview) -> RenderFrame | None:
        """
        Decode an image response from the server.

        Image format: [uint32 width] [uint32 height] [uint32 channels] [RGBA pixels]

        The frame is a view of the storage the response was received into, which
        becomes its ``base``, so it can be passed back to :meth:`render` as ``out``.
        """
        if len(data) < 12:
            return None

        width, height, channels = struct.unpack_from("<III", data, 0)
        size = len(data) - 12
        expected_size = width * height * channels
        if size < expected_size:
            logger.warning(
                f"Image response truncated: expected {expected_size} bytes, got {size}"
            )
            return None

        # The client receives each response at the start of its storage.
        storage = data.obj if isinstance(data, memoryview) else data
        if not isinstance(storage, np.ndarray):
            storage = np.frombuffer(storage, dtype=np.uint8)
        pixels = storage.reshape(-1)[12 : 12 + expected_size]
        return pixels.reshape((height, width, channels))

    ####################################################################################
    # Context manager support
    ####################################################################################

    def __enter__(self) -> MochiRendererViewer:
        """Support with-statement for the viewer."""
        return self

    def __exit__(self, *ignored) -> bool:
        """Close the viewer at the end of the with-statement."""
        self.close()
        return False  # Propagate exceptions


########################################################################################
# Internal helper
########################################################################################


class _ActorInfo:
    """Bookkeeping for a tracked actor."""

    __slots__ = ("name", "actor_type", "num_verts", "local_aabb", "world_aabb")

    def __init__(self, name: str, actor_type: ActorType, num_verts: int) -> None:
        self.name = name
        self.actor_type = actor_type
        self.num_verts = num_verts
        # Bounds in the actor's local frame, when they can be cached.
        self.local_aabb: AABB | None = None
        # Bounds in world space, refreshed every render.
        self.world_aabb: AABB | None = None
