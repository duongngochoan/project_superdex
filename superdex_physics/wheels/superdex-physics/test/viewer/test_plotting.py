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

"""Tests for the backend-neutral plot grouping/validation logic."""

from __future__ import annotations

import unittest

import numpy as np
from superdex.physics.viewer.plotting import (
    group_plots,
    PlotAxisInfo,
    PlotGroup,
    PlotState,
)


def _plot(name: str, legend: str, n: int = 3, **kwargs) -> PlotState:
    return PlotState(
        name=name,
        legend=legend,
        x=np.arange(n, dtype=float),
        y=np.arange(n, dtype=float),
        **kwargs,
    )


class GroupPlotsTest(unittest.TestCase):
    def test_groups_share_a_name(self) -> None:
        groups = group_plots([_plot("A", "a1"), _plot("A", "a2"), _plot("B", "b1")])
        self.assertEqual({"A", "B"}, set(groups))
        assert groups["A"].plots is not None
        self.assertEqual(2, len(groups["A"].plots))
        assert groups["B"].plots is not None
        self.assertEqual(1, len(groups["B"].plots))

    def test_unnamed_plot_is_skipped(self) -> None:
        groups = group_plots([PlotState(name=None, x=np.zeros(2), y=np.zeros(2))])
        self.assertEqual({}, groups)


class PlotGroupTest(unittest.TestCase):
    def test_rejects_mismatched_name(self) -> None:
        group = PlotGroup(name="A")
        group.add_plot(_plot("B", "b"))
        self.assertIsNone(group.plots)

    def test_rejects_missing_data(self) -> None:
        group = PlotGroup(name="A")
        group.add_plot(PlotState(name="A", legend="a", x=None, y=None))
        self.assertIsNone(group.plots)

    def test_rejects_shape_mismatch(self) -> None:
        group = PlotGroup(name="A")
        group.add_plot(PlotState(name="A", legend="a", x=np.zeros(3), y=np.zeros(2)))
        self.assertIsNone(group.plots)

    def test_rejects_uncertainty_shape_mismatch(self) -> None:
        group = PlotGroup(name="A")
        group.add_plot(_plot("A", "a", lower=np.zeros(2), upper=np.zeros(2)))
        self.assertIsNone(group.plots)

    def test_first_axis_info_wins(self) -> None:
        group = PlotGroup(name="A")
        group.add_plot(_plot("A", "a", x_axis_info=PlotAxisInfo(name="time")))
        group.add_plot(_plot("A", "b", x_axis_info=PlotAxisInfo(name="ignored")))
        assert group.x_axis_info is not None
        self.assertEqual("time", group.x_axis_info.name)

    def test_invalid_limit_is_dropped(self) -> None:
        group = PlotGroup(name="A")
        group.add_plot(_plot("A", "a", x_axis_info=PlotAxisInfo(limit=(1.0, 0.0))))
        assert group.x_axis_info is not None
        self.assertIsNone(group.x_axis_info.limit)
