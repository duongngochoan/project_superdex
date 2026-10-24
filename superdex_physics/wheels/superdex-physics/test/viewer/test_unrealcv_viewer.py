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

from __future__ import annotations

import io
import unittest
from types import SimpleNamespace

import numpy as np
from superdex.physics.viewer.unrealcv.unrealcv_client import LitMultiLayout
from superdex.physics.viewer.unrealcv.unrealcv_viewer import UnrealCVViewer
from superdex.physics.viewer.unrealcv.unrealcv_viewer_cfg import CaptureFormat

_LAYOUT = LitMultiLayout(
    width=24,
    height=19,
    ids=(2, 0),
    tiles=((0, 0, 16, 12), (18, 3, 5, 16)),
)


def _npy(array: np.ndarray) -> bytes:
    buffer = io.BytesIO()
    np.save(buffer, array)
    return buffer.getvalue()


def _camera_images() -> dict[int, np.ndarray]:
    rng = np.random.default_rng(0)
    return {
        camera_id: rng.integers(0, 256, (h, w, 4), dtype=np.uint8)
        for camera_id, (_, _, w, h) in zip(_LAYOUT.ids, _LAYOUT.tiles)
    }


def _atlas(images: dict[int, np.ndarray]) -> np.ndarray:
    atlas = np.full((_LAYOUT.height, _LAYOUT.width, 4), 123, dtype=np.uint8)
    for camera_id, (x, y, w, h) in zip(_LAYOUT.ids, _LAYOUT.tiles):
        atlas[y : y + h, x : x + w] = images[camera_id]
    return atlas


class TestUnrealCVViewerMultiViewDecode(unittest.TestCase):
    def test_tiles_decode_like_separate_per_camera_captures(self) -> None:
        images = _camera_images()
        payload = _npy(_atlas(images))
        per_camera_viewer = SimpleNamespace(
            _state=SimpleNamespace(capture_format=CaptureFormat.NPY)
        )

        frames = UnrealCVViewer._decode_multiview_response(payload, _LAYOUT)

        self.assertEqual(list(frames), [2, 0])
        for camera_id, image in images.items():
            with self.subTest(camera_id=camera_id):
                frame = frames[camera_id]
                expected = UnrealCVViewer._decode_capture_response(
                    per_camera_viewer, _npy(image)
                )
                np.testing.assert_array_equal(frame, expected)
                self.assertEqual(
                    (frame.dtype, frame.shape, frame.strides),
                    (expected.dtype, expected.shape, expected.strides),
                )
                self.assertTrue(frame.flags.c_contiguous)
                self.assertFalse(
                    np.shares_memory(frame, np.frombuffer(payload, np.uint8))
                )

    def test_text_reply_raises_with_the_camera_ids(self) -> None:
        with self.assertRaisesRegex(RuntimeError, r"\[2, 0\].*readback failed"):
            UnrealCVViewer._decode_multiview_response("error readback failed", _LAYOUT)

    def test_atlas_that_does_not_match_the_layout_raises(self) -> None:
        atlas = _atlas(_camera_images())
        for case, mismatched in (
            ("short", atlas[:-1]),
            ("three channels", atlas[..., :3]),
        ):
            with (
                self.subTest(case=case),
                self.assertRaisesRegex(
                    RuntimeError, "does not match the 24x19 BGRA8 layout"
                ),
            ):
                UnrealCVViewer._decode_multiview_response(_npy(mismatched), _LAYOUT)
