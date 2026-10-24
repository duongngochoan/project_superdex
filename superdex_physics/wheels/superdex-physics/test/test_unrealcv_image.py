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

import numpy as np

from .module_stand_ins import load_test_module, PHYSICS_ROOT

unrealcv_image = load_test_module(
    "unrealcv_image_test_target",
    PHYSICS_ROOT / "viewer" / "unrealcv" / "unrealcv_image.py",
)


class BgrToRgbTest(unittest.TestCase):
    def test_returns_exact_owning_rgb(self) -> None:
        for height in (0, 2):
            for channels in (3, 4):
                for dtype in (np.uint8, np.uint16, np.float32):
                    with self.subTest(height=height, channels=channels, dtype=dtype):
                        source = np.arange(height * 5 * channels, dtype=dtype).reshape(
                            height, 5, channels
                        )
                        expected = source[:, :, 2::-1].copy()

                        converted = unrealcv_image.bgr_to_rgb(source)

                        np.testing.assert_array_equal(converted, expected)
                        self.assertEqual(converted.dtype, source.dtype)
                        self.assertTrue(converted.flags.c_contiguous)
                        self.assertTrue(converted.flags.owndata)
                        self.assertTrue(converted.flags["WRITEABLE"])


class NpyViewTest(unittest.TestCase):
    def test_matches_np_load_without_copying(self) -> None:
        sources = {
            "bgra frame": np.arange(2 * 3 * 4, dtype=np.uint8).reshape(2, 3, 4),
            "float16 depth": np.linspace(0, 1, 12, dtype=np.float16).reshape(3, 4),
            "fortran order": np.asfortranarray(np.arange(6.0).reshape(2, 3)),
            "empty": np.zeros((0, 4), dtype=np.uint8),
        }
        for name, source in sources.items():
            for version in ((1, 0), (2, 0)):
                with self.subTest(name=name, version=version):
                    stream = io.BytesIO()
                    np.lib.format.write_array(stream, source, version=version)
                    payload = stream.getvalue()

                    view = unrealcv_image.npy_view(payload)

                    np.testing.assert_array_equal(view, np.load(io.BytesIO(payload)))
                    self.assertEqual(view.dtype, source.dtype)
                    self.assertFalse(view.flags["WRITEABLE"])

    def test_rejects_unsupported_format_versions(self) -> None:
        stream = io.BytesIO()
        np.lib.format.write_array(stream, np.zeros(3), version=(3, 0))

        with self.assertRaisesRegex(ValueError, "format version"):
            unrealcv_image.npy_view(stream.getvalue())
