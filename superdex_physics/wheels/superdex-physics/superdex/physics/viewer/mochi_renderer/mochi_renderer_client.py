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
TCP client for communicating with the mochi_renderer server.

Implements the binary wire protocol for sending batched commands and
receiving text/image responses over a persistent TCP connection.
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass
from typing import BinaryIO

logger = logging.getLogger(__name__)

########################################################################################


@dataclass
class CommandEntry:
    """A single command in a batch request."""

    text: str
    binary_data: bytes = b""


@dataclass
class ResponseEntry:
    """A single response from a batch request.

    ``data`` is a view of the destination given for the command when the response
    fit in it, or of a new buffer otherwise.
    """

    type: int  # 0 = text, 1 = image
    data: bytes | memoryview = b""


########################################################################################


class MochiRendererClient:
    """
    TCP client for the mochi_renderer server.

    Implements the binary wire protocol:
      Request:  [uint32 payload_len] [payload]
        payload: [uint32 num_commands]
                 per command: [uint32 text_len] [text] [uint32 data_len] [data]
      Response: [uint32 payload_len] [payload]
        payload: [uint32 num_responses]
                 per response: [uint8 type] [uint32 data_len] [data]

    Thread-safe: all socket operations are serialized through a lock.
    """

    def __init__(self, host: str = "localhost", port: int = 9000) -> None:
        self._host = host
        self._port = port
        self._socket: socket.socket | None = None
        self._reader: BinaryIO | None = None
        self._lock = threading.Lock()

    # ──────────────────────────────────────────────────────────────────────────
    # Connection management
    # ──────────────────────────────────────────────────────────────────────────

    def connect(self, timeout: float = 5.0, retry_attempts: int = 3) -> bool:
        """
        Connect to the mochi_renderer server.

        Args:
            timeout: Socket timeout in seconds for the connection attempt.
            retry_attempts: Number of times to retry on failure.

        Returns:
            True if the connection was established, False otherwise.
        """
        if self._socket is not None:
            self.disconnect()

        for attempt in range(retry_attempts):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(timeout)
                sock.connect((self._host, self._port))
                # After connecting, use blocking mode with no timeout for I/O
                sock.settimeout(None)
                with self._lock:
                    self._socket = sock
                logger.info(f"Connected to mochi_renderer at {self._host}:{self._port}")
                return True
            except OSError as e:
                logger.warning(
                    f"Connection attempt {attempt + 1}/{retry_attempts} failed: {e}"
                )
                try:
                    sock.close()
                except Exception:
                    pass
                if attempt < retry_attempts - 1:
                    backoff = min(2**attempt, 8)
                    logger.info(f"Retrying in {backoff}s...")
                    time.sleep(backoff)
        logger.error(
            f"Failed to connect to mochi_renderer at "
            f"{self._host}:{self._port} after {retry_attempts} attempts"
        )
        return False

    def disconnect(self) -> None:
        """Disconnect from the server. Safe to call if already disconnected."""
        with self._lock:
            if self._socket is not None:
                self._close_locked()
                logger.info("Disconnected from mochi_renderer")

    def is_connected(self) -> bool:
        """Return whether the client currently holds an open connection."""
        with self._lock:
            return self._socket is not None

    # ──────────────────────────────────────────────────────────────────────────
    # Public request API
    # ──────────────────────────────────────────────────────────────────────────

    def request(self, command: str, binary_data: bytes | None = None) -> bytes:
        """
        Send a single command and return the raw response data.

        This is a convenience wrapper around :meth:`request_batch` for the
        common single-command case.

        Args:
            command: The command text (e.g. ``"vset /object/box/show"``).
            binary_data: Optional binary payload attached to the command.

        Returns:
            The ``data`` field of the first :class:`ResponseEntry`.

        Raises:
            ConnectionError: If not connected or the server closes the
                connection unexpectedly.
            RuntimeError: If the server returns an unexpected number of
                responses.
        """
        entry = CommandEntry(text=command, binary_data=binary_data or b"")
        responses = self.request_batch([entry])
        if not responses:
            raise RuntimeError("Server returned zero responses for a single command")
        return bytes(responses[0].data)

    def request_batch(
        self,
        commands: list[CommandEntry],
        response_buffers: Sequence[memoryview | None] | None = None,
    ) -> list[ResponseEntry]:
        """
        Send a batch of commands and return all responses.

        Args:
            commands: List of :class:`CommandEntry` objects to send.
            response_buffers: Optional destination for each command's response, in
                command order; None leaves a response to a new buffer. A response
                that fits in its destination is received directly into it, and its
                ``data`` is then a view of the destination's first bytes.

        Returns:
            List of :class:`ResponseEntry` objects, one per server response.

        Raises:
            ConnectionError: If not connected, the connection is lost, or the response
                framing is inconsistent. The client then disconnects, since the
                stream may be left mid-frame.
            ValueError: If *response_buffers*
                or a destination is read-only or not C-contiguous. Checked before
                anything is sent.
        """
        destinations: list[memoryview | None] = [None] * len(commands)
        if response_buffers is not None:
            if len(response_buffers) != len(commands):
                raise ValueError(
                    f"Got {len(response_buffers)} response buffers for "
                    f"{len(commands)} commands"
                )
            for index, view in enumerate(response_buffers):
                if view is None:
                    continue
                if view.readonly or not view.c_contiguous:
                    raise ValueError(
                        "Response buffers must be writable and C-contiguous"
                    )
                destinations[index] = view.cast("B")
        with self._lock:
            sock = self._socket
            if sock is None:
                raise ConnectionError("Not connected to mochi_renderer")
            if self._reader is None:
                self._reader = sock.makefile("rb")
            payload = self._encode_request(commands)
            try:
                self._send_framed(sock, payload)
                return self._read_response(self._reader, destinations)
            except OSError:
                # The stream may be mid-frame: never reuse it.
                self._close_locked()
                raise

    def _close_locked(self) -> None:
        """Close the reader and then the socket; the caller holds the lock."""
        for stream in (self._reader, self._socket):
            if stream is not None:
                try:
                    stream.close()
                except Exception:
                    pass
        self._reader = None
        self._socket = None

    # ──────────────────────────────────────────────────────────────────────────
    # Wire protocol encoding / decoding
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _encode_request(commands: list[CommandEntry]) -> bytes:
        """
        Encode a list of commands into the request payload.

        Layout:
            [uint32 num_commands]
            per command:
                [uint32 text_length] [UTF-8 text]
                [uint32 data_length] [binary data]
        """
        parts: list[bytes] = []
        parts.append(struct.pack("<I", len(commands)))
        for cmd in commands:
            text_bytes = cmd.text.encode("utf-8")
            parts.append(struct.pack("<I", len(text_bytes)))
            parts.append(text_bytes)
            parts.append(struct.pack("<I", len(cmd.binary_data)))
            if cmd.binary_data:
                parts.append(cmd.binary_data)
        return b"".join(parts)

    @staticmethod
    def _read_response(
        reader: BinaryIO, destinations: Sequence[memoryview | None]
    ) -> list[ResponseEntry]:
        """
        Read one framed response, receiving each response into its destination.

        Layout:
            [uint32 payload_length]   (frame header)
            [uint32 num_responses]    (payload starts here)
            per response:
                [uint8 type]
                [uint32 data_length] [data]
        """
        # Check every length against what is left of the frame before reading, so a
        # corrupt one cannot read into the next frame or allocate an outsized buffer.
        (payload_length,) = struct.unpack(
            "<I", MochiRendererClient._read_exact(reader, 4)
        )
        if payload_length < 4:
            raise MochiRendererClient._framing_error(payload_length)
        (num_responses,) = struct.unpack(
            "<I", MochiRendererClient._read_exact(reader, 4)
        )
        remaining = payload_length - 4
        entries: list[ResponseEntry] = []
        for index in range(num_responses):
            if remaining < 5:
                raise MochiRendererClient._framing_error(payload_length)
            resp_type, data_len = struct.unpack(
                "<BI", MochiRendererClient._read_exact(reader, 5)
            )
            remaining -= 5
            if data_len > remaining:
                raise MochiRendererClient._framing_error(payload_length)
            remaining -= data_len
            destination = destinations[index] if index < len(destinations) else None
            data = MochiRendererClient._read_into(reader, data_len, destination)
            entries.append(ResponseEntry(type=resp_type, data=data))
        if remaining != 0:
            raise MochiRendererClient._framing_error(payload_length)
        return entries

    @staticmethod
    def _framing_error(payload_length: int) -> ConnectionError:
        return ConnectionError(
            f"mochi_renderer response length {payload_length} does not match its "
            f"contents"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Low-level framed I/O
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _send_framed(sock: socket.socket, payload: bytes) -> None:
        """Send a length-prefixed payload: [uint32_le length] [bytes]."""
        header = struct.pack("<I", len(payload))
        sock.sendall(header + payload)

    @staticmethod
    def _read_into(
        reader: BinaryIO, size: int, destination: memoryview | None
    ) -> memoryview:
        """Read *size* bytes into *destination* if they fit, else into a new buffer."""
        if destination is not None and len(destination) >= size:
            data = destination[:size]
        else:
            data = memoryview(bytearray(size))
        if reader.readinto(data) != size:
            raise ConnectionError("Connection closed while reading from mochi_renderer")
        return data

    @staticmethod
    def _read_exact(reader: BinaryIO, num_bytes: int) -> bytes:
        """Read exactly *num_bytes*, raising on premature EOF."""
        data = reader.read(num_bytes)
        if len(data) != num_bytes:
            raise ConnectionError("Connection closed while reading from mochi_renderer")
        return data
