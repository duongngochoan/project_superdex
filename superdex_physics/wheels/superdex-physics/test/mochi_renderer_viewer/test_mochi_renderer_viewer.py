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
Tests for the MochiRendererClient wire protocol encoding and decoding.

Uses mock sockets to test the binary protocol without a running server.
"""

from __future__ import annotations

import io
import socket
import struct
import unittest
from unittest.mock import MagicMock, patch

from superdex.physics.viewer.mochi_renderer.mochi_renderer_client import (
    CommandEntry,
    MochiRendererClient,
    ResponseEntry,
)

########################################################################################


def _framed_response(*responses: tuple[int, bytes]) -> bytes:
    """Frame (type, data) responses the way MochiViewerServer sends them."""
    body = struct.pack("<I", len(responses))
    for resp_type, data in responses:
        body += struct.pack("<BI", resp_type, len(data)) + data
    return struct.pack("<I", len(body)) + body


def _read_response(
    frame: bytes, destinations: list[memoryview | None] | None = None
) -> list[ResponseEntry]:
    return MochiRendererClient._read_response(io.BytesIO(frame), destinations or [])


########################################################################################
# Dataclass tests
########################################################################################


class TestCommandEntry:
    """Tests for the CommandEntry dataclass."""

    def test_default_binary_data(self) -> None:
        entry = CommandEntry(text="vset /object/box/show")
        assert entry.text == "vset /object/box/show"
        assert entry.binary_data == b""

    def test_with_binary_data(self) -> None:
        data = b"\x01\x02\x03\x04"
        entry = CommandEntry(text="vset /object/box/mesh 1 3 0", binary_data=data)
        assert entry.text == "vset /object/box/mesh 1 3 0"
        assert entry.binary_data == data


class TestResponseEntry:
    """Tests for the ResponseEntry dataclass."""

    def test_text_response(self) -> None:
        entry = ResponseEntry(type=0, data=b"ok")
        assert entry.type == 0
        assert entry.data == b"ok"

    def test_image_response(self) -> None:
        pixel_data = b"\xff" * 16
        entry = ResponseEntry(type=1, data=pixel_data)
        assert entry.type == 1
        assert entry.data == pixel_data

    def test_default_data(self) -> None:
        entry = ResponseEntry(type=0)
        assert entry.data == b""


########################################################################################
# Wire protocol encoding tests
########################################################################################


class TestEncodeRequest:
    """Tests for MochiRendererClient._encode_request."""

    def test_single_command_no_binary(self) -> None:
        commands = [CommandEntry(text="vset /object/box/show")]
        payload = MochiRendererClient._encode_request(commands)

        offset = 0
        # num_commands
        (num_cmds,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        assert num_cmds == 1

        # text_length + text
        text_bytes = b"vset /object/box/show"
        (text_len,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        assert text_len == len(text_bytes)
        assert payload[offset : offset + text_len] == text_bytes
        offset += text_len

        # data_length (should be 0)
        (data_len,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        assert data_len == 0

        assert offset == len(payload)

    def test_single_command_with_binary(self) -> None:
        binary = struct.pack("<3f", 1.0, 2.0, 3.0)
        commands = [
            CommandEntry(text="vset /object/box/mesh 1 3 0", binary_data=binary)
        ]
        payload = MochiRendererClient._encode_request(commands)

        offset = 0
        (num_cmds,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        assert num_cmds == 1

        # text
        (text_len,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        text = payload[offset : offset + text_len]
        offset += text_len
        assert text == b"vset /object/box/mesh 1 3 0"

        # binary data
        (data_len,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        assert data_len == len(binary)
        assert payload[offset : offset + data_len] == binary
        offset += data_len

        assert offset == len(payload)

    def test_multiple_commands(self) -> None:
        commands = [
            CommandEntry(text="vset /object/a/show"),
            CommandEntry(text="vset /object/b/hide"),
            CommandEntry(text="vget /camera/default/lit"),
        ]
        payload = MochiRendererClient._encode_request(commands)

        offset = 0
        (num_cmds,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        assert num_cmds == 3

        # Parse each command
        for cmd in commands:
            text_bytes = cmd.text.encode("utf-8")
            (text_len,) = struct.unpack_from("<I", payload, offset)
            offset += 4
            assert text_len == len(text_bytes)
            assert payload[offset : offset + text_len] == text_bytes
            offset += text_len

            (data_len,) = struct.unpack_from("<I", payload, offset)
            offset += 4
            assert data_len == 0

        assert offset == len(payload)

    def test_empty_command_list(self) -> None:
        payload = MochiRendererClient._encode_request([])
        assert payload == struct.pack("<I", 0)

    def test_unicode_command_text(self) -> None:
        commands = [CommandEntry(text="vset /object/cafe\u0301/show")]
        payload = MochiRendererClient._encode_request(commands)

        offset = 0
        (num_cmds,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        assert num_cmds == 1

        (text_len,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        text = payload[offset : offset + text_len].decode("utf-8")
        assert text == "vset /object/cafe\u0301/show"


########################################################################################
# Wire protocol decoding tests
########################################################################################


class TestReadResponse(unittest.TestCase):
    """Tests for MochiRendererClient._read_response."""

    def test_responses_without_destinations_get_new_buffers(self) -> None:
        entries = _read_response(_framed_response((0, b"ok"), (1, b"\x01\x02\x03")))

        self.assertEqual([0, 1], [entry.type for entry in entries])
        self.assertEqual([b"ok", b"\x01\x02\x03"], [entry.data for entry in entries])
        self.assertFalse(any(entry.data.readonly for entry in entries))

    def test_empty_response_list(self) -> None:
        self.assertEqual([], _read_response(_framed_response()))

    def test_response_is_received_into_a_large_enough_destination(self) -> None:
        destination = bytearray(8)

        (entry,) = _read_response(
            _framed_response((1, b"pixels")), [memoryview(destination)]
        )

        self.assertEqual(b"pixels", entry.data)
        self.assertIs(destination, entry.data.obj)
        self.assertEqual(b"pixels\x00\x00", destination)

    def test_too_small_destination_is_left_untouched(self) -> None:
        destination = bytearray(5)

        (entry,) = _read_response(
            _framed_response((1, b"pixels")), [memoryview(destination)]
        )

        self.assertEqual(b"pixels", entry.data)
        self.assertIsNot(destination, entry.data.obj)
        self.assertEqual(bytearray(5), destination)

    def test_destinations_are_matched_by_response_index(self) -> None:
        first, second = bytearray(16), bytearray(16)

        entries = _read_response(
            _framed_response((0, b"error: camera not found: a"[:16]), (1, b"pixels")),
            [memoryview(first), memoryview(second)],
        )

        self.assertIs(first, entries[0].data.obj)
        self.assertIs(second, entries[1].data.obj)
        self.assertEqual(b"pixels", entries[1].data)

    def test_length_that_disagrees_with_the_contents_raises(self) -> None:
        frame = bytearray(_framed_response((0, b"ok")))
        frame[0] += 1

        with self.assertRaises(ConnectionError):
            _read_response(bytes(frame))

    def test_truncated_response_raises(self) -> None:
        with self.assertRaises(ConnectionError):
            _read_response(_framed_response((1, b"pixels"))[:-1])

    def test_payload_too_short_for_its_count_raises(self) -> None:
        with self.assertRaises(ConnectionError):
            _read_response(struct.pack("<II", 2, 0))

    def test_inner_length_past_the_frame_raises_without_reading_on(self) -> None:
        # The frame claims 1 response of 100 bytes but holds only 2; a later frame
        # follows.
        body = struct.pack("<IBI", 1, 0, 100) + b"ok"
        later = _framed_response((0, b"next"))
        reader = io.BytesIO(struct.pack("<I", len(body)) + body + later)

        with self.assertRaises(ConnectionError):
            MochiRendererClient._read_response(reader, [])

        self.assertEqual(later, reader.read()[-len(later) :])

    def test_response_header_past_the_frame_raises(self) -> None:
        body = struct.pack("<I", 1)  # A response is announced but the frame ends.
        with self.assertRaises(ConnectionError):
            _read_response(struct.pack("<I", len(body)) + body + bytes(5))


########################################################################################
# Roundtrip tests
########################################################################################


class TestEncodeDecodeRoundtrip:
    """Test that encode and decode are consistent."""

    def test_encode_produces_valid_framing(self) -> None:
        """Verify the encoded payload can be wrapped in a frame header."""
        commands = [
            CommandEntry(text="vset /object/box/show"),
            CommandEntry(
                text="vset /object/box/mesh 4 6 0",
                binary_data=b"\x00" * 48,
            ),
        ]
        payload = MochiRendererClient._encode_request(commands)

        # Frame it
        frame = struct.pack("<I", len(payload)) + payload

        # Read back the frame
        (frame_len,) = struct.unpack_from("<I", frame, 0)
        assert frame_len == len(payload)
        assert frame[4:] == payload


########################################################################################
# Connection / mock socket tests
########################################################################################


class TestConnection:
    """Tests for connect / disconnect using mock sockets."""

    @patch("superdex.physics.viewer.mochi_renderer.mochi_renderer_client.socket.socket")
    def test_connect_success(self, mock_socket_class: MagicMock) -> None:
        mock_sock = MagicMock()
        mock_socket_class.return_value = mock_sock

        client = MochiRendererClient("localhost", 9000)
        result = client.connect(timeout=1.0, retry_attempts=1)

        assert result is True
        assert client.is_connected()
        mock_sock.settimeout.assert_any_call(1.0)
        mock_sock.connect.assert_called_once_with(("localhost", 9000))

    @patch("superdex.physics.viewer.mochi_renderer.mochi_renderer_client.socket.socket")
    def test_connect_failure(self, mock_socket_class: MagicMock) -> None:
        mock_sock = MagicMock()
        mock_socket_class.return_value = mock_sock
        mock_sock.connect.side_effect = ConnectionRefusedError("refused")

        client = MochiRendererClient("localhost", 9000)
        result = client.connect(timeout=1.0, retry_attempts=2)

        assert result is False
        assert not client.is_connected()
        assert mock_sock.connect.call_count == 2

    def test_disconnect_when_not_connected(self) -> None:
        client = MochiRendererClient()
        # Should not raise
        client.disconnect()
        assert not client.is_connected()

    @patch("superdex.physics.viewer.mochi_renderer.mochi_renderer_client.socket.socket")
    def test_disconnect_after_connect(self, mock_socket_class: MagicMock) -> None:
        mock_sock = MagicMock()
        mock_socket_class.return_value = mock_sock

        client = MochiRendererClient()
        client.connect(timeout=1.0, retry_attempts=1)
        assert client.is_connected()

        client.disconnect()
        assert not client.is_connected()
        mock_sock.close.assert_called_once()


########################################################################################
# request / request_batch over a socket pair
########################################################################################


class TestRequestBatch(unittest.TestCase):
    """Tests for request / request_batch over a connected socket pair."""

    def _connected_client(
        self, response: bytes
    ) -> tuple[MochiRendererClient, socket.socket]:
        """A client whose server end has already queued *response*."""
        client_sock, server_sock = socket.socketpair()
        self.addCleanup(client_sock.close)
        self.addCleanup(server_sock.close)
        server_sock.sendall(response)
        client = MochiRendererClient()
        client._socket = client_sock
        return client, server_sock

    def _assert_nothing_sent(self, server_sock: socket.socket) -> None:
        server_sock.setblocking(False)
        with self.assertRaises(BlockingIOError):
            server_sock.recv(1)

    def test_request_batch_not_connected(self) -> None:
        client = MochiRendererClient()
        with self.assertRaises(ConnectionError):
            client.request_batch([CommandEntry(text="vget /objects")])

    def test_request_batch_sends_the_commands_and_reads_the_responses(self) -> None:
        client, server_sock = self._connected_client(
            _framed_response((0, b"ok"), (1, b"pixels"))
        )
        commands = [
            CommandEntry(text="vset /object/box/show"),
            CommandEntry(text="vget /camera/default/lit"),
        ]

        entries = client.request_batch(commands)

        payload = MochiRendererClient._encode_request(commands)
        expected_request = struct.pack("<I", len(payload)) + payload
        self.assertEqual(expected_request, server_sock.recv(len(expected_request)))
        self.assertEqual([b"ok", b"pixels"], [entry.data for entry in entries])

    def test_request_batch_receives_into_the_given_buffers(self) -> None:
        client, _ = self._connected_client(_framed_response((0, b"ok"), (1, b"pixels")))
        destination = bytearray(8)

        entries = client.request_batch(
            [CommandEntry(text="vset /a/show"), CommandEntry(text="vget /b/lit")],
            response_buffers=[None, memoryview(destination)],
        )

        self.assertIs(destination, entries[1].data.obj)
        self.assertEqual(b"pixels", entries[1].data)

    def test_one_buffer_per_command_is_required(self) -> None:
        client, server_sock = self._connected_client(b"")

        with self.assertRaises(ValueError):
            client.request_batch(
                [CommandEntry(text="vset /a/show"), CommandEntry(text="vget /b/lit")],
                response_buffers=[memoryview(bytearray(8))],
            )

        self._assert_nothing_sent(server_sock)

    def test_unusable_buffer_is_rejected_before_sending(self) -> None:
        buffers = {
            "read-only": memoryview(bytes(8)),
            "not contiguous": memoryview(bytearray(16))[::2],
        }
        for reason, buffer in buffers.items():
            with self.subTest(reason):
                client, server_sock = self._connected_client(b"")

                with self.assertRaises(ValueError):
                    client.request_batch(
                        [CommandEntry(text="vget /b/lit")], response_buffers=[buffer]
                    )

                self._assert_nothing_sent(server_sock)

    def test_request_returns_the_first_response_data_as_bytes(self) -> None:
        client, _ = self._connected_client(_framed_response((0, b"ok")))

        data = client.request("vset /object/box/show")

        self.assertIs(bytes, type(data))
        self.assertEqual(b"ok", data)

    def test_consecutive_requests_share_the_connection(self) -> None:
        client, _ = self._connected_client(
            _framed_response((0, b"first")) + _framed_response((0, b"second"))
        )

        self.assertEqual(b"first", client.request("vget /a"))
        self.assertEqual(b"second", client.request("vget /b"))
        self.assertTrue(client.is_connected())

    def test_a_framing_error_disconnects(self) -> None:
        body = struct.pack("<IBI", 1, 0, 100) + b"ok"
        client, _ = self._connected_client(struct.pack("<I", len(body)) + body)

        with self.assertRaises(ConnectionError):
            client.request("vget /a")

        self.assertFalse(client.is_connected())

    def test_send_framed(self) -> None:
        mock_sock = MagicMock()
        payload = b"test_payload"
        MochiRendererClient._send_framed(mock_sock, payload)

        expected = struct.pack("<I", len(payload)) + payload
        mock_sock.sendall.assert_called_once_with(expected)
