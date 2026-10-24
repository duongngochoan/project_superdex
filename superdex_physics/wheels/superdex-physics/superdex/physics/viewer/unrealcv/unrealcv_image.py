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

import math
from io import BytesIO

import cv2
import numpy as np


def npy_view(payload: bytes) -> np.ndarray:
    """Read-only array over an in-memory ``.npy`` payload, without copying it.

    ``np.load`` on a byte stream copies the array data in 256 KiB chunks while
    holding the GIL, which serializes camera decoding across environment threads.
    """
    stream = BytesIO(payload)
    version = np.lib.format.read_magic(stream)
    read_header = {
        (1, 0): np.lib.format.read_array_header_1_0,
        (2, 0): np.lib.format.read_array_header_2_0,
    }.get(version)
    if read_header is None:
        raise ValueError(f"Unsupported .npy format version {version}")
    shape, fortran_order, dtype = read_header(stream)
    return np.frombuffer(
        payload, dtype=dtype, count=math.prod(shape), offset=stream.tell()
    ).reshape(shape, order="F" if fortran_order else "C")


def bgr_to_rgb(image: np.ndarray) -> np.ndarray:
    """Return an owning, contiguous RGB copy of a three- or four-channel image."""
    if image.dtype == np.uint8 and image.size > 0:
        conversion = cv2.COLOR_BGRA2RGB if image.shape[2] == 4 else cv2.COLOR_BGR2RGB
        return cv2.cvtColor(image, conversion)
    return image[:, :, 2::-1].copy()
