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

import logging
from typing import Dict

from superdex.physics.viewer.backend import (
    polyscope_imgui as psim,
    polyscope_implot as psimp,
)
from superdex.physics.viewer.plotting import group_plots, PlotGroup
from superdex.physics.viewer.viewer_state import ViewerState

logger = logging.getLogger(__name__)

_implot_warning_logged = False


def build_plot_panel(state: ViewerState):
    """
    Build and render the plot window UI.

    This window displays user created runtime plot information. We currently
    only support grouped 2D line plots

    Args:
        state: The viewer state containing logging configuration and handler.
    """

    global _implot_warning_logged
    if psimp is None:
        if not _implot_warning_logged:
            logger.warning("polyscope_implot is not available; plot panel is disabled")
            _implot_warning_logged = True
        if psim is not None:
            psim.TextDisabled("PLOTTING NOT SUPPORTED (implot unavailable)")
        return

    """ Opacity for uncertainty """
    # kAlpha = 0.25

    if state.plots is None:
        psim.TextDisabled("NO PLOT SPECIFIED")
        return

    """ Group plots by names """
    plot_groups: Dict[str, PlotGroup] = group_plots(state.plots)

    """ Create a plot for each group """
    for plot_group in plot_groups.values():
        psimp.BeginPlot(
            "Untitled Plot Group" if plot_group.name is None else plot_group.name
        )

        """ Create axes """
        psimp.SetupAxis(
            psimp.ImAxis_X1,
            "x-axis"
            if plot_group.x_axis_info is None or plot_group.x_axis_info.name is None
            else plot_group.x_axis_info.name,
        )
        psimp.SetupAxis(
            psimp.ImAxis_Y1,
            "y-axis"
            if plot_group.y_axis_info is None or plot_group.y_axis_info.name is None
            else plot_group.y_axis_info.name,
        )
        if (
            plot_group.x_axis_info is not None
            and plot_group.x_axis_info.limit is not None
        ):
            info = plot_group.x_axis_info
            assert info is not None
            limit = info.limit
            assert limit is not None
            psimp.SetupAxisLimits(
                psimp.ImAxis_X1,
                limit[0],
                limit[1],
                psimp.ImPlotCond_Always,
            )
        if (
            plot_group.y_axis_info is not None
            and plot_group.y_axis_info.limit is not None
        ):
            info = plot_group.y_axis_info
            assert info is not None
            limit = info.limit
            assert limit is not None
            psimp.SetupAxisLimits(
                psimp.ImAxis_Y1,
                limit[0],
                limit[1],
                psimp.ImPlotCond_Always,
            )

        """ Create plots """
        plots = plot_group.plots
        assert plots is not None
        for plot in plots:
            name = "Untitled Plot" if plot.legend is None else plot.legend
            psimp.PlotLine(name, plot.x, plot.y)
            if plot.lower is not None and plot.upper is not None:
                # psimp.PushStyleVar(psimp.ImPlotStyleVar_FillAlpha, kAlpha)
                psimp.PlotShaded(
                    name + "-Uncertainty"
                    if plot.shaded_legend_name is None
                    else plot.shaded_legend_name,
                    plot.x,
                    plot.lower,
                    plot.upper,
                )
                # psimp.PopStyleVar()
        psimp.EndPlot()
