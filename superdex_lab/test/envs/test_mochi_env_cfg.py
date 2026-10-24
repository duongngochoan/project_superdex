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
Tests for MochiEnvCfg viewer configuration.

These only construct configs, never environments, so no renderer is started.
"""

from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np
from superdex.lab.gym.envs.mochi_env import _format_info_value, MochiEnvCfg
from superdex.physics.viewer.mochi_renderer import MochiRendererViewerCfg


def _cfg(**overrides) -> MochiEnvCfg:
    return MochiEnvCfg(control_frequency=50, simulation_frequency=100, **overrides)


class MochiEnvCfgTest(unittest.TestCase):
    def test_mochi_viewer_needs_no_explicit_cfg(self) -> None:
        """The viewer must be usable without naming an explicit config."""
        self.assertIsNone(_cfg().mochi_renderer_cfg)

    def test_viewer_cfg_is_accepted(self) -> None:
        cfg = _cfg(mochi_renderer_cfg=MochiRendererViewerCfg(port=1234))
        self.assertIsNotNone(cfg.mochi_renderer_cfg)
        self.assertEqual(1234, cfg.mochi_renderer_cfg.port)

    def test_rejects_an_unknown_render_mode(self) -> None:
        with self.assertRaisesRegex(ValueError, "Invalid render_mode"):
            _cfg(render_mode="hologram")


class FormatInfoValueTest(unittest.TestCase):
    def test_json_values_are_formatted_as_json(self) -> None:
        self.assertEqual("[1.0, 2.0]", _format_info_value(np.array([1.0, 2.0])))

    def test_other_values_fall_back_to_repr(self) -> None:
        # A gym info dict may hold any type; the Environment tab must not raise.
        for value in (Path("a/b"), {1, 2}, object()):
            with self.subTest(value=value):
                self.assertEqual(repr(value), _format_info_value(value))
