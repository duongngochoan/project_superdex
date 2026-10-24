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
Backend-neutral plotting data and grouping logic.

These types carry no dependency on any UI backend (Polyscope or imgui_bundle), so both
the legacy Polyscope viewer and the client-side Filament window share one definition of
what a plot is and how plots are grouped and validated. Only the thin per-backend "draw"
layer (which issues ImPlot calls) differs between viewers.
"""

from __future__ import annotations

import dataclasses
import logging

import numpy.typing as npt

logger: logging.Logger = logging.getLogger(__name__)

########################################################################################


@dataclasses.dataclass
class PlotAxisInfo:
    """Structure holding the axis information of a plot."""

    limit: tuple[float, float] | None = None
    """Range of data to show."""

    name: str | None = None
    """Name and unit of the data along this axis."""


@dataclasses.dataclass
class PlotState:
    """Structure holding the state of a plot."""

    name: str | None = None
    """Name of the plot."""

    legend: str | None = None
    """If this is a grouped plot, this is the legend for this plot."""

    x: npt.NDArray[float] | None = None
    """Data for x-axis."""

    y: npt.NDArray[float] | None = None
    """Data for y-axis."""

    lower: npt.NDArray[float] | None = None
    """Data for y-axis shaded plot lower bound."""

    upper: npt.NDArray[float] | None = None
    """Data for y-axis shaded plot upper bound."""

    shaded_legend_name: str | None = None
    """Custom name for the shaded plot."""

    x_axis_info: PlotAxisInfo | None = None
    """Information of the x-axis data."""

    y_axis_info: PlotAxisInfo | None = None
    """Information of the y-axis data."""


########################################################################################


@dataclasses.dataclass
class PlotGroup:
    """A set of plots sharing one name, drawn together in a single plot.

    Holds the backend-neutral validation and axis-info bookkeeping; the ImPlot draw
    calls live in each backend's plot panel.
    """

    name: str | None = None
    """Name shared by every plot in this group."""

    plots: list[PlotState] | None = None
    """Plots belonging to this group."""

    x_axis_info: PlotAxisInfo | None = None
    """Information about the x-axis of all plots in this group."""

    y_axis_info: PlotAxisInfo | None = None
    """Information about the y-axis of all plots in this group."""

    def add_plot(self, plot: PlotState) -> None:  # noqa: C901
        """Validate and append a plot to this group.

        Rejects mismatched names, missing or mismatched data, and invalid axis limits,
        logging the reason. Only the first member that supplies axis info owns it.
        """
        if plot.name != self.name:
            logger.error("Plot name does not match group name")
            return
        if plot.x is None or plot.y is None:
            logger.warning("Plot has no data")
            return
        if plot.x.shape != plot.y.shape:
            logger.error("Plot x and y data have different sizes")
            return
        if plot.lower is not None and plot.upper is not None:
            if plot.lower.shape != plot.x.shape:
                logger.error("Plot x and lower bound data have different sizes")
                return
            if plot.upper.shape != plot.x.shape:
                logger.error("Plot x and upper bound data have different sizes")
                return

        if self.plots is None:
            self.plots = []
        self.plots.append(plot)

        if self.x_axis_info is None:
            self.x_axis_info = plot.x_axis_info
            self._sanitize_limit(self.x_axis_info, "x-axis")
        elif plot.x_axis_info is not None:
            logger.warning("Only one plot can have x-axis info")

        if self.y_axis_info is None:
            self.y_axis_info = plot.y_axis_info
            self._sanitize_limit(self.y_axis_info, "y-axis")
        elif plot.y_axis_info is not None:
            logger.warning("Only one plot can have y-axis info")

    @staticmethod
    def _sanitize_limit(axis_info: PlotAxisInfo | None, axis_name: str) -> None:
        """Drop an axis limit whose low bound is not below its high bound."""
        if axis_info is None or axis_info.limit is None:
            return
        low, high = axis_info.limit
        if low >= high:
            logger.warning(f"Invalid {axis_name} limit")
            axis_info.limit = None


def group_plots(plots: list[PlotState]) -> dict[str, PlotGroup]:
    """Group a flat list of plots by name, applying per-group validation.

    Plots with no name are skipped. Returns an insertion-ordered mapping from group
    name to the assembled :class:`PlotGroup`.
    """
    groups: dict[str, PlotGroup] = {}
    for plot in plots:
        name = plot.name
        if name is None:
            logger.warning("Plot has no name; skipping")
            continue
        if name not in groups:
            groups[name] = PlotGroup(name=name)
        groups[name].add_plot(plot)
    return groups
