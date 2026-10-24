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
Deterministic per-actor colors for the MochiRenderer viewer.

The Filament server renders every mesh in one hardcoded grey unless the mesh command
carries a color, which would leave the links of a robot indistinguishable. The palette
here mirrors the one the samples app and the debugger already draw with
(``ViewportScene::GetRotatingColor`` in mochi_samples/app/viewport_scene.cpp and
``GetRotatingColor`` in mochi_debugger/app/viewport/render_scene.cpp) so all three
Filament viewports agree.

Colors are keyed by actor handle: stable for a given scene, but they shift when actors
are added or removed. That matches the other two viewports.
"""

from __future__ import annotations

import colorsys

########################################################################################

_HUE_STEP_DEG = 137.5
"""Golden-angle hue step, so consecutive actors land far apart on the color wheel."""

_SATURATION = 0.7
"""HSL saturation shared by every actor color."""

_LIGHTNESS = 0.6
"""HSL lightness shared by every actor color."""

_SRGB_LINEAR_CUTOFF = 0.04045
"""Below this the sRGB transfer function is linear rather than a power curve."""


def actor_color(handle_value: int) -> tuple[float, float, float]:
    """Return the sRGB color for an actor, keyed by its handle.

    Args:
        handle_value: Value of the actor's handle. Negative values are fine; Python's
            modulo keeps the hue in range.

    Returns:
        Red, green and blue in [0, 1], in sRGB space.
    """
    hue = (int(handle_value) * _HUE_STEP_DEG % 360.0) / 360.0
    return colorsys.hls_to_rgb(hue, _LIGHTNESS, _SATURATION)


def srgb_to_linear(color: tuple[float, float, float]) -> tuple[float, float, float]:
    """Convert an sRGB color to the linear space Filament's ``baseColor`` expects.

    Filament shades in linear light, so passing sRGB values straight through would make
    mid-tones roughly four times too bright and flatten the contrast between hues.

    Args:
        color: Red, green and blue in [0, 1], in sRGB space.

    Returns:
        The same color in linear space.
    """
    red, green, blue = color
    return (
        _channel_to_linear(red),
        _channel_to_linear(green),
        _channel_to_linear(blue),
    )


def _channel_to_linear(channel: float) -> float:
    """Apply the sRGB electro-optical transfer function to a single channel."""
    if channel <= _SRGB_LINEAR_CUTOFF:
        return channel / 12.92
    return ((channel + 0.055) / 1.055) ** 2.4
