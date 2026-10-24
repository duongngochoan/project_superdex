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
Camera bookkeeping for the MochiRenderer viewer.

The renderer server offers no camera readback, so the viewer owns the authoritative
pose and derives the look direction from it. This is a private copy of the Polyscope
``CameraState``; the two viewers share no state and Polyscope is on its way out.
"""

from __future__ import annotations

import dataclasses

import numpy.typing as npt

########################################################################################


@dataclasses.dataclass
class CameraState:
    """Camera pose, framing request, and follow-camera settings."""

    look_from: npt.NDArray | None = None
    """Cached eye position. None until a pose has been set."""
    look_at: npt.NDArray | None = None
    """Cached target position. None until a pose has been set."""
    up: npt.NDArray | None = None
    """Up direction fixing the camera's roll. None leaves the server default."""
    dirty: bool = False
    """Whether the cached pose still has to be sent to the server."""

    use_follow_camera: bool = False
    """If True, the camera follows the scene actors."""
    automatic_distance: bool = False
    """If True, the follow distance is recomputed to frame the scene each frame."""
    smoothing: float = 0.4
    """Follow camera smoothing factor, in [0, 1]."""

    frame_camera_on_next_update: bool = False
    """If True, framing was requested before a scene existed and is still pending."""
    frame_camera_direction: npt.NDArray | None = None
    """Look direction to use when the pending framing resolves."""
    frame_camera_attempts_left: int = 0
    """Renders left before a pending framing request is abandoned."""

    smoothed_target_position: npt.NDArray | None = None
    """Smoothed follow-camera target."""
    distance: float = 0.0
    """Last view distance to the actors."""
    smoothed_distance: float = 0.0
    """Smoothed view distance to the actors."""
