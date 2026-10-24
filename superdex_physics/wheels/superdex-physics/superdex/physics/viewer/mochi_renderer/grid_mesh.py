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
Client-side geometry for reference grids.

The mochi_renderer server has no grid primitive and no UV-parameterization
shader, so the grid Polyscope draws with a single quad has to be tessellated
into real triangles here and uploaded as ordinary meshes.
"""

from __future__ import annotations

import dataclasses
import logging

import numpy as np
import numpy.typing as npt

logger: logging.Logger = logging.getLogger(__name__)

########################################################################################

INFINITE_SIZE = 1e3
"""Extent used to approximate an infinite grid, matching ``GridRenderer``."""

MAX_CELLS_PER_AXIS = 128
"""Upper bound on the tessellation. Polyscope draws its grid in a shader and can
afford an arbitrarily fine pattern; real triangles cannot, so an infinite grid is
shrunk to this many cells and a finite grid has its period coarsened."""

LINE_WIDTH_FRACTION = 0.02
"""Width of a grid line as a fraction of the period, for the "grid" style."""

_PLANE_AXES: dict[str, tuple[int, int, int]] = {
    "xy": (0, 1, 2),
    "xz": (2, 0, 1),
    "yz": (1, 2, 0),
}
"""Maps a grid plane to its (u axis, v axis, normal axis) indices. Each triple
is right-handed, so the triangle winding below agrees with the emitted normal
and the mesh survives the server's back-face culling."""

_LINE_DEPTH_OFFSET_FRACTION = 1e-3
"""Offset of the grid lines above the backdrop, as a fraction of the cell size.
Coplanar geometry would z-fight; the server exposes no polygon-offset control."""


@dataclasses.dataclass
class GridMesh:
    """One uploadable piece of a grid: a triangle soup with a single color."""

    positions: npt.NDArray[np.float32]
    """Vertex positions, shape (N, 3)."""
    normals: npt.NDArray[np.float32]
    """Vertex normals, shape (N, 3)."""
    indices: npt.NDArray[np.int32]
    """Triangle indices, shape (M, 3)."""
    color: tuple[float, float, float]
    """Base color of the piece."""


########################################################################################


def build_grid_meshes(
    size: float,
    center: npt.NDArray[np.floating],
    period: float,
    axes: str,
    style: str,
    color_1: tuple[float, float, float],
    color_2: tuple[float, float, float],
    double_sided: bool,
) -> list[GridMesh]:
    """Tessellate a reference grid into two single-colored meshes.

    Args:
        size: Side length of the grid, or ``inf`` for an "infinite" grid.
        center: World-space center of the grid plane.
        period: Spacing between grid lines.
        axes: Plane the grid lies in: "xy", "xz" or "yz".
        style: "checker" for alternating cells, "grid" for lines on a backdrop.
        color_1: Checker cells of even parity, or the grid lines.
        color_2: Checker cells of odd parity, or the backdrop.
        double_sided: Whether the grid is visible from both sides.

    Returns:
        Two meshes, the first colored ``color_1`` and the second ``color_2``.

    Raises:
        ValueError: If ``axes`` or ``style`` is unknown, or a dimension is not
            positive.
    """
    if axes not in _PLANE_AXES:
        raise ValueError(f"Invalid axes: {axes}. Expected one of {list(_PLANE_AXES)}.")
    if style not in ("checker", "grid"):
        raise ValueError(f"Invalid style: {style}. Expected 'checker' or 'grid'.")
    if period <= 0:
        raise ValueError("Period must be positive.")
    if size <= 0:
        raise ValueError("Size must be positive.")

    extent, cell_size, cells = _resolve_tessellation(size, period)
    plane = _PLANE_AXES[axes]
    edges = -0.5 * extent + cell_size * np.arange(cells + 1, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64).reshape(3)

    if style == "checker":
        quads_1, quads_2 = _checker_quads(edges, cells)
        offset_1 = 0.0
    else:
        quads_1, quads_2 = _line_quads(edges, extent, cell_size)
        offset_1 = _LINE_DEPTH_OFFSET_FRACTION * cell_size

    # A single-cell checker has no odd-parity cells, and the server rejects an
    # empty mesh.
    return [
        _mesh_from_quads(quads, plane, center, color, double_sided, offset)
        for quads, color, offset in (
            (quads_1, color_1, offset_1),
            (quads_2, color_2, 0.0),
        )
        if len(quads)
    ]


def _resolve_tessellation(size: float, period: float) -> tuple[float, float, int]:
    """Pick an extent, cell size and cell count that stay within the vertex budget.

    An infinite grid is shrunk so its cells keep the requested period; a finite
    grid keeps its requested extent and has its period coarsened instead.
    """
    if not np.isfinite(size):
        extent = min(INFINITE_SIZE, period * MAX_CELLS_PER_AXIS)
        cells = max(1, int(round(extent / period)))
        return extent, extent / cells, cells

    cells = max(1, int(round(size / period)))
    if cells > MAX_CELLS_PER_AXIS:
        logger.warning(
            f"Grid of size {size} with period {period} needs {cells} cells per axis; "
            f"coarsening to {MAX_CELLS_PER_AXIS}."
        )
        cells = MAX_CELLS_PER_AXIS
    return size, size / cells, cells


def _checker_quads(
    edges: npt.NDArray[np.floating], cells: int
) -> tuple[npt.NDArray[np.floating], npt.NDArray[np.floating]]:
    """Split the cell grid into its two checker parities.

    Returns:
        Two (M, 4) arrays of ``(u0, u1, v0, v1)`` cell bounds.
    """
    i, j = np.meshgrid(np.arange(cells), np.arange(cells), indexing="ij")
    i = i.ravel()
    j = j.ravel()
    quads = np.stack([edges[i], edges[i + 1], edges[j], edges[j + 1]], axis=1)
    even = ((i + j) % 2) == 0
    return quads[even], quads[~even]


def _line_quads(
    edges: npt.NDArray[np.floating], extent: float, cell_size: float
) -> tuple[npt.NDArray[np.floating], npt.NDArray[np.floating]]:
    """Build thin strips along both axes plus a full-extent backdrop.

    Returns:
        The line quads and the single backdrop quad, as ``(u0, u1, v0, v1)``.
    """
    half_width = 0.5 * LINE_WIDTH_FRACTION * cell_size
    low = -0.5 * extent
    high = 0.5 * extent

    along_v = np.stack(
        [
            edges - half_width,
            edges + half_width,
            np.full_like(edges, low),
            np.full_like(edges, high),
        ],
        axis=1,
    )
    along_u = np.stack(
        [
            np.full_like(edges, low),
            np.full_like(edges, high),
            edges - half_width,
            edges + half_width,
        ],
        axis=1,
    )
    backdrop = np.array([[low, high, low, high]], dtype=np.float64)
    return np.concatenate([along_v, along_u]), backdrop


def _mesh_from_quads(
    quads: npt.NDArray[np.floating],
    plane: tuple[int, int, int],
    center: npt.NDArray[np.floating],
    color: tuple[float, float, float],
    double_sided: bool,
    normal_offset: float,
) -> GridMesh:
    """Turn ``(u0, u1, v0, v1)`` bounds into a triangulated, centered mesh."""
    u_axis, v_axis, normal_axis = plane
    count = len(quads)

    corners = np.zeros((count, 4, 3), dtype=np.float64)
    corners[:, [0, 3], u_axis] = quads[:, 0, None]
    corners[:, [1, 2], u_axis] = quads[:, 1, None]
    corners[:, [0, 1], v_axis] = quads[:, 2, None]
    corners[:, [2, 3], v_axis] = quads[:, 3, None]
    corners[:, :, normal_axis] = normal_offset
    corners += center

    normal = np.zeros(3, dtype=np.float64)
    normal[normal_axis] = 1.0

    # Counter-clockwise when viewed from the +normal side.
    base = 4 * np.arange(count, dtype=np.int32).reshape(-1, 1, 1)
    front = base + np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)

    positions = corners.reshape(-1, 3)
    normals = np.broadcast_to(normal, positions.shape)
    indices = front.reshape(-1, 3)

    if double_sided:
        # The server has no back-face policy, so the reverse side is real geometry.
        back = indices[:, ::-1] + len(positions)
        positions = np.concatenate([positions, positions])
        normals = np.concatenate([normals, -normals])
        indices = np.concatenate([indices, back])

    return GridMesh(
        positions=np.ascontiguousarray(positions, dtype=np.float32),
        normals=np.ascontiguousarray(normals, dtype=np.float32),
        indices=np.ascontiguousarray(indices, dtype=np.int32),
        color=color,
    )
