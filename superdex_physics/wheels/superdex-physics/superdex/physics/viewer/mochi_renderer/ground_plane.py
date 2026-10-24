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
Ground planes for the MochiRenderer viewer.

Plane collision shapes carry no surface mesh, so the viewer has nothing to upload for
them and a scene with a ground plane would render with no floor. Polyscope draws them
with ``StaticPlaneRenderer``; this reproduces that so both backends show the same floor.

The pose is inferred from the actor's world AABB, which is how Polyscope does it: a plane
is unbounded in the two directions spanning it, so the single axis with a finite bound is
its normal, and that bound is its offset along the normal. This only ever describes an
axis-aligned plane, which is the same limitation Polyscope has.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt
from superdex.physics import Actor

########################################################################################

PLANE_COLOR_1 = (0.85, 0.85, 0.85)
"""Darker checker square, matching Polyscope's ``StaticPlaneRenderer``."""

PLANE_COLOR_2 = (0.95, 0.95, 0.95)
"""Lighter checker square, matching Polyscope's ``StaticPlaneRenderer``."""

PLANE_PERIOD = 1.0
"""Checker cell size in meters, matching Polyscope's ``StaticPlaneRenderer``."""

_NORMAL_AXIS_TO_PLANE = {0: "yz", 1: "xz", 2: "xy"}
"""Grid plane spanned by the two axes that are not the normal."""


def is_plane_actor(actor: Actor) -> bool:
    """Return whether an actor is a static plane collision shape.

    Planes are the only shapes with an unbounded AABB; spheres, boxes and capsules are
    all finite. Mirrors ``Viewer._is_plane_actor``.
    """
    try:
        if not actor.get_surface_mesh().is_empty() or not actor.is_static():
            return False
        aabb = actor.get_aabb_world()
        return not all(
            np.isfinite(aabb.min[axis]) and np.isfinite(aabb.max[axis])
            for axis in range(3)
        )
    except Exception:
        return False


def plane_center_and_axes(actor: Actor) -> tuple[npt.NDArray, str] | None:
    """Infer a plane actor's center and the grid plane it lies in.

    Returns None when the bounds are unreadable or describe no usable plane.
    """
    try:
        aabb = actor.get_aabb_world()
    except Exception:
        return None

    for axis in range(3):
        low = float(aabb.min[axis])
        high = float(aabb.max[axis])
        # The bounded side is the surface; the solid extends away from it.
        if np.isfinite(low) or np.isfinite(high):
            center = np.zeros(3, dtype=np.float64)
            center[axis] = low if np.isfinite(low) else high
            return center, _NORMAL_AXIS_TO_PLANE[axis]
    return None
