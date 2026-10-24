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

import json
import unittest
from dataclasses import dataclass
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
from superdex.physics.viewer.unrealcv.unrealcv_actor_utils import (
    _build_actor_material_commands,
    _decimal,
    _log_material_dr_results,
    _log_mpc_results,
    _log_visibility_results,
    _resolve_mpc_value,
    apply_cast_shadow_dr,
    apply_hdri_lighting_dr,
    apply_mpc_dr,
    apply_scale_dr,
    capture_dr_state,
    get_light_info,
    get_material_params,
    KEEP_PREVIOUS_TOKEN,
)


@dataclass
class _FakeRangeConfig:
    low: float
    high: float


@dataclass
class _FakeMaterialOverride:
    """Minimal stand-in for ActorMaterialOverride for command-builder tests.

    Mirrors the fields ``_build_actor_material_commands`` reads (including the
    ``uses_rgb`` property) so tests do not depend on the fbcode config module.
    """

    hue: object = None
    saturation: object = None
    value: object = None
    red: object = None
    green: object = None
    blue: object = None
    alpha: object = None
    roughness: object = "default"
    metallic: object = "default"
    specular: object = "default"
    light_intensity: object = "default"
    source_radius: object = "default"
    texture_index: object = "default"
    texture_probability: object = None
    scalar_params: dict[str, object] | None = None
    vector_params: dict[str, list[object]] | None = None

    @property
    def uses_rgb(self) -> bool:
        return (
            (self.red is not None and self.red != "default")
            or (self.green is not None and self.green != "default")
            or (self.blue is not None and self.blue != "default")
        )


class ResolveMpcValueTest(unittest.TestCase):
    def test_none_returns_none(self):
        result = _resolve_mpc_value(None)
        self.assertIsNone(result)

    def test_scalar_float(self):
        result = _resolve_mpc_value(3.5)
        self.assertEqual(result, 3.5)

    def test_scalar_int(self):
        result = _resolve_mpc_value(7)
        self.assertEqual(result, 7.0)

    def test_range_config_returns_within_bounds(self):
        np.random.seed(42)
        rc = _FakeRangeConfig(low=1.0, high=5.0)
        result = _resolve_mpc_value(rc)
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result, 1.0)
        self.assertLessEqual(result, 5.0)

    def test_dict_converted_to_range(self):
        np.random.seed(42)
        result = _resolve_mpc_value({"low": 2.0, "high": 4.0})
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result, 2.0)
        self.assertLessEqual(result, 4.0)


class LogMpcResultsTest(unittest.TestCase):
    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_all_succeeded(self, mock_logger):
        commands = [
            "vset /object/Actor1/mpc_scalar Hue 0.5",
            "vset /object/Actor1/mpc_scalar Noise 0.3",
        ]
        responses = ["ok", "ok"]
        _log_mpc_results(commands, responses, ["Actor1"])
        self.assertEqual(mock_logger.info.call_count, 3)
        mock_logger.warning.assert_not_called()

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_mixed_results(self, mock_logger):
        commands = [
            "vset /object/Actor1/mpc_scalar Hue 0.5",
            "vset /object/Actor2/mpc_scalar Noise 0.3",
        ]
        responses = ["ok", "error"]
        _log_mpc_results(commands, responses, ["Actor1", "Actor2"])
        self.assertGreaterEqual(mock_logger.info.call_count, 1)
        mock_logger.error.assert_called_once()

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_empty_commands(self, mock_logger):
        _log_mpc_results([], [], [])
        mock_logger.info.assert_called_once()
        mock_logger.warning.assert_not_called()


class ApplyMpcDrTest(unittest.TestCase):
    def _make_cfg(self, mpc_overrides=None):
        cfg = SimpleNamespace(mpc_overrides=mpc_overrides or {})
        return cfg

    def _make_client(self, responses=None):
        client = MagicMock()
        if responses is None:
            responses = ["ok"]
        client._request_batch.return_value = responses
        return client

    def test_no_overrides_returns_early(self):
        client = self._make_client()
        cfg = self._make_cfg(mpc_overrides={})
        apply_mpc_dr(client, cfg)
        client._request_batch.assert_not_called()

    def test_missing_attr_returns_early(self):
        client = self._make_client()
        cfg = SimpleNamespace()
        apply_mpc_dr(client, cfg)
        client._request_batch.assert_not_called()

    def test_scalar_values_generate_commands(self):
        client = self._make_client(responses=["ok", "ok"])
        cfg = self._make_cfg(
            mpc_overrides={
                "Actor1": {"Hue": 0.5, "Noise": 0.3},
            }
        )
        apply_mpc_dr(client, cfg)
        client._request_batch.assert_called_once()
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(len(commands), 2)
        self.assertIn("mpc_scalar Hue 0.5", commands[0])
        self.assertIn("mpc_scalar Noise 0.3", commands[1])

    def test_none_values_skipped(self):
        client = self._make_client(responses=["ok"])
        cfg = self._make_cfg(
            mpc_overrides={
                "Actor1": {"Hue": 0.5, "Noise": None},
            }
        )
        apply_mpc_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(len(commands), 1)
        self.assertIn("Hue", commands[0])

    def test_empty_actor_params_skipped(self):
        client = self._make_client(responses=["ok"])
        cfg = self._make_cfg(
            mpc_overrides={
                "Actor1": {},
                "Actor2": {"Param": 1.0},
            }
        )
        apply_mpc_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(len(commands), 1)
        self.assertIn("Actor2", commands[0])

    def test_dict_range_resolved(self):
        np.random.seed(0)
        client = self._make_client(responses=["ok"])
        cfg = self._make_cfg(
            mpc_overrides={
                "Actor1": {"Hue": {"low": -30.0, "high": 30.0}},
            }
        )
        apply_mpc_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(len(commands), 1)
        self.assertIn("mpc_scalar Hue", commands[0])

    def test_range_config_object_resolved(self):
        np.random.seed(0)
        client = self._make_client(responses=["ok"])
        cfg = self._make_cfg(
            mpc_overrides={
                "Actor1": {"Hue": _FakeRangeConfig(low=-30.0, high=30.0)},
            }
        )
        apply_mpc_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(len(commands), 1)
        self.assertIn("mpc_scalar Hue", commands[0])

    def test_tuple_response_unwrapped(self):
        client = MagicMock()
        client._request_batch.return_value = (["ok"], 0.01)
        cfg = self._make_cfg(mpc_overrides={"Actor1": {"Hue": 1.0}})
        apply_mpc_dr(client, cfg)
        client._request_batch.assert_called_once()
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(len(commands), 1)
        self.assertIn("mpc_scalar Hue", commands[0])


class ApplyHdriLightingDrTest(unittest.TestCase):
    _DOME = "BP_HDRI_Skydome_C_1"
    _SKYLIGHT = "SkyLight_0"

    def _make_override(self, **overrides):
        cubemaps = [f"/Game/HDRI/TC_HDRI_{i:02d}" for i in range(1, 11)]
        base = {
            "skylight_actor": self._SKYLIGHT,
            "dome_actor": self._DOME,
            "cubemap_asset_paths": cubemaps,
            "dome_cube_param": "EnvCube",
            "offset_yaw": 30.0,
            "dome_yaw_sign": 1,
        }
        base.update(overrides)
        return SimpleNamespace(hdri_lighting_override=base)

    def _make_client(self, responses=None):
        client = MagicMock()
        client._request_batch.return_value = responses or ["ok", "ok", "ok", "ok"]
        return client

    def test_none_override_returns_early(self):
        client = self._make_client()
        cfg = SimpleNamespace(hdri_lighting_override=None)
        apply_hdri_lighting_dr(client, cfg, {})
        client._request_batch.assert_not_called()

    def test_missing_attr_returns_early(self):
        client = self._make_client()
        apply_hdri_lighting_dr(client, SimpleNamespace(), {})
        client._request_batch.assert_not_called()

    def test_incomplete_config_returns_early(self):
        client = self._make_client()
        cfg = self._make_override(skylight_actor="")
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 0.0, 0.0)})
        client._request_batch.assert_not_called()

    @patch("numpy.random.randint", return_value=3)
    def test_drives_dome_and_skylight_in_sync(self, _mock_randint):
        client = self._make_client()
        cfg = self._make_override()
        # Pre-seed the dome baseline so no rotation-capture round trip happens.
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 100.0, 0.0)})

        client._request_batch.assert_called_once()
        commands = client._request_batch.call_args[0][0]
        # One index + one yaw drive all four commands.
        self.assertEqual(
            commands,
            [
                f"vset /object/{self._DOME}/texture_cube_param EnvCube /Game/HDRI/TC_HDRI_04",
                f"vset /object/{self._DOME}/rotation 0.0 130.0 0.0",
                f"vset /object/{self._SKYLIGHT}/skylight_cubemap /Game/HDRI/TC_HDRI_04",
                f"vset /object/{self._SKYLIGHT}/skylight_angle 30.0",
            ],
        )

    @patch("numpy.random.randint", return_value=3)
    def test_dome_yaw_sign_flips_dome_rotation_only(self, _mock_randint):
        client = self._make_client()
        cfg = self._make_override(dome_yaw_sign=-1)
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 100.0, 0.0)})

        commands = client._request_batch.call_args[0][0]
        # Dome yaw is base - yaw; skylight angle stays the unsigned yaw.
        self.assertIn(f"/object/{self._DOME}/rotation 0.0 70.0 0.0", commands[1])
        self.assertIn(f"/object/{self._SKYLIGHT}/skylight_angle 30.0", commands[3])

    @patch("numpy.random.randint", return_value=7)
    def test_dome_and_skylight_use_same_cube_asset(self, _mock_randint):
        client = self._make_client()
        cfg = self._make_override()
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 0.0, 0.0)})
        commands = client._request_batch.call_args[0][0]
        # The unified design drives BOTH the dome and the SkyLight from ONE
        # cube asset path.
        cube = "/Game/HDRI/TC_HDRI_08"
        self.assertIn(f"texture_cube_param EnvCube {cube}", commands[0])
        self.assertIn(f"skylight_cubemap {cube}", commands[2])

    @patch("numpy.random.randint", return_value=0)
    def test_new_fields_omitted_keeps_current_four_commands(self, _mock_randint):
        client = self._make_client()
        cfg = self._make_override()
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 0.0, 0.0)})
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(len(commands), 4)

    @patch("numpy.random.randint", return_value=0)
    def test_skylight_intensity_fixed_emits_command(self, _mock_randint):
        client = self._make_client(responses=["ok"] * 5)
        cfg = self._make_override(skylight_intensity=0.4)
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 0.0, 0.0)})
        commands = client._request_batch.call_args[0][0]
        self.assertIn(f"vset /object/{self._SKYLIGHT}/skylight_intensity 0.4", commands)

    @patch("numpy.random.uniform", return_value=0.55)
    @patch("numpy.random.randint", return_value=0)
    def test_skylight_intensity_range_samples_per_rep(
        self, _mock_randint, _mock_uniform
    ):
        client = self._make_client(responses=["ok"] * 5)
        cfg = self._make_override(
            offset_yaw=None,
            skylight_intensity={"low": 0.3, "high": 0.9},
        )
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 0.0, 0.0)})
        commands = client._request_batch.call_args[0][0]
        self.assertIn(
            f"vset /object/{self._SKYLIGHT}/skylight_intensity 0.55", commands
        )

    @patch("numpy.random.randint", return_value=0)
    def test_skylight_color_emits_three_channel_command(self, _mock_randint):
        client = self._make_client(responses=["ok"] * 5)
        cfg = self._make_override(
            skylight_color={"red": 0.5, "green": 0.6, "blue": 1.0},
        )
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 0.0, 0.0)})
        commands = client._request_batch.call_args[0][0]
        self.assertIn(
            f"vset /object/{self._SKYLIGHT}/skylight_color 0.5 0.6 1.0", commands
        )

    @patch("numpy.random.randint", return_value=0)
    def test_skydome_brightness_emits_scalar_param_command(self, _mock_randint):
        client = self._make_client(responses=["ok"] * 5)
        cfg = self._make_override(skydome_brightness=0.5)
        apply_hdri_lighting_dr(client, cfg, {self._DOME: (0.0, 0.0, 0.0)})
        commands = client._request_batch.call_args[0][0]
        self.assertIn(
            f"vset /object/{self._DOME}/scalar_param Brightness 0.5", commands
        )


class ApplyCastShadowDrTest(unittest.TestCase):
    def _make_cfg(self, cast_shadow_overrides=None):
        return SimpleNamespace(cast_shadow_overrides=cast_shadow_overrides or {})

    def _make_client(self, responses=None):
        client = MagicMock()
        client._request_batch.return_value = responses or ["ok"]
        return client

    def test_no_overrides_returns_early(self):
        client = self._make_client()
        apply_cast_shadow_dr(client, self._make_cfg({}))
        client._request_batch.assert_not_called()

    def test_missing_attr_returns_early(self):
        client = self._make_client()
        apply_cast_shadow_dr(client, SimpleNamespace())
        client._request_batch.assert_not_called()

    def test_bool_toggles_map_to_one_and_zero(self):
        client = self._make_client(responses=["ok", "ok"])
        cfg = self._make_cfg({"WallA": False, "LightB": True})
        apply_cast_shadow_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(
            commands,
            [
                "vset /object/WallA/cast_shadow 0",
                "vset /object/LightB/cast_shadow 1",
            ],
        )

    def test_tuple_response_unwrapped(self):
        client = MagicMock()
        client._request_batch.return_value = (["ok"], 0.01)
        cfg = self._make_cfg({"WallA": False})
        apply_cast_shadow_dr(client, cfg)
        client._request_batch.assert_called_once()
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(commands, ["vset /object/WallA/cast_shadow 0"])


class ApplyScaleDrTest(unittest.TestCase):
    def _make_cfg(self, scale_overrides=None):
        return SimpleNamespace(scale_overrides=scale_overrides or {})

    def _make_client(self, responses=None):
        client = MagicMock()
        client._request_batch.return_value = responses or ["ok"]
        return client

    def _override(
        self, scale=None, scale_x=None, scale_y=None, scale_z=None, scale_group=None
    ):
        return SimpleNamespace(
            scale=scale,
            scale_x=scale_x,
            scale_y=scale_y,
            scale_z=scale_z,
            scale_group=scale_group,
        )

    def test_no_overrides_returns_early(self):
        client = self._make_client()
        apply_scale_dr(client, self._make_cfg({}))
        client._request_batch.assert_not_called()

    def test_missing_attr_returns_early(self):
        client = self._make_client()
        apply_scale_dr(client, SimpleNamespace())
        client._request_batch.assert_not_called()

    def test_none_override_skipped(self):
        client = self._make_client(responses=["ok"])
        cfg = self._make_cfg({"A": None, "B": self._override(scale=1.5)})
        apply_scale_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        self.assertEqual(commands, ["vset /object/B/scale 1.5 1.5 1.5"])

    def test_per_axis_precedence_and_uniform_fallback(self):
        client = self._make_client(responses=["ok"])
        cfg = self._make_cfg({"A": self._override(scale=1.2, scale_z=2.0)})
        apply_scale_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        # scale_x/y fall back to uniform 1.2; scale_z override wins.
        self.assertEqual(commands, ["vset /object/A/scale 1.2 1.2 2.0"])

    @patch("numpy.random.uniform", side_effect=[0.9, 1.05, 0.81, 1.11])
    def test_no_scale_group_samples_independently(self, mock_uniform):
        client = self._make_client(responses=["ok", "ok"])
        rc = _FakeRangeConfig
        cfg = self._make_cfg(
            {
                "Foam": self._override(scale_x=rc(0.8, 1.2), scale_y=rc(0.9, 1.1)),
                "Foam2": self._override(scale_x=rc(0.8, 1.2), scale_y=rc(0.9, 1.1)),
            }
        )
        apply_scale_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        # Ungrouped: each foam draws its own values → different sizes.
        self.assertEqual(
            commands,
            [
                "vset /object/Foam/scale 0.9 1.05 1.0",
                "vset /object/Foam2/scale 0.81 1.11 1.0",
            ],
        )
        self.assertEqual(mock_uniform.call_count, 4)

    @patch("numpy.random.uniform", side_effect=[0.9, 1.05, 0.5, 0.5])
    def test_scale_group_shares_single_sample(self, mock_uniform):
        client = self._make_client(responses=["ok", "ok"])
        rc = _FakeRangeConfig
        cfg = self._make_cfg(
            {
                "Foam": self._override(
                    scale_x=rc(0.8, 1.2), scale_y=rc(0.9, 1.1), scale_group="foam"
                ),
                "Foam2": self._override(scale_group="foam"),
            }
        )
        apply_scale_dr(client, cfg)
        commands = client._request_batch.call_args[0][0]
        # Grouped: Foam samples once (0.9, 1.05); Foam2 reuses the SAME scale.
        self.assertEqual(
            commands,
            [
                "vset /object/Foam/scale 0.9 1.05 1.0",
                "vset /object/Foam2/scale 0.9 1.05 1.0",
            ],
        )
        # Only Foam's two axes were drawn — Foam2 reused, no extra sampling.
        self.assertEqual(mock_uniform.call_count, 2)


class BuildActorMaterialCommandsExclusiveTest(unittest.TestCase):
    """Exclusive per-rep hue-vs-texture mode in _build_actor_material_commands."""

    _ACTOR = "Board"

    def test_texture_probability_omitted_matches_legacy_behavior(self):
        # Regression guard: with texture_probability unset, hue AND texture are
        # applied independently exactly as before.
        override = _FakeMaterialOverride(
            hue=_FakeRangeConfig(0.0, 360.0),
            roughness=_FakeRangeConfig(0.4, 0.9),
            texture_index=_FakeRangeConfig(0.0, 1.0),
        )
        with patch("numpy.random.uniform", side_effect=[123.0, 0.55, 0.42]):
            commands = _build_actor_material_commands(self._ACTOR, override, {})
        self.assertEqual(
            commands,
            [
                f"vset /object/{self._ACTOR}/hsva 123.0 -1000000.0 -1000000.0 -1000000.0",
                f"vset /object/{self._ACTOR}/roughness 0.55",
                f"vset /object/{self._ACTOR}/texture_index 0.42",
            ],
        )

    def _exclusive_override(self) -> _FakeMaterialOverride:
        return _FakeMaterialOverride(
            hue=_FakeRangeConfig(0.0, 360.0),
            saturation=_FakeRangeConfig(0.2, 0.8),
            value=_FakeRangeConfig(0.4, 0.9),
            roughness=_FakeRangeConfig(0.4, 0.9),
            metallic=_FakeRangeConfig(0.0, 0.3),
            specular=None,
            texture_index=_FakeRangeConfig(0.0, 1.0),
            texture_probability=0.8,
        )

    def test_forced_texture_mode_emits_texture_and_neutral_tint(self):
        # uniform() = 0.1 < 0.8 → texture mode. Draws: mode, texture, rough, metallic.
        override = self._exclusive_override()
        with patch("numpy.random.uniform", side_effect=[0.1, 0.5, 0.6, 0.2]):
            commands = _build_actor_material_commands(self._ACTOR, override, {})
        # Sampled texture + neutral white tint reset; NO sampled hue.
        self.assertEqual(
            commands,
            [
                f"vset /object/{self._ACTOR}/texture_index 0.5",
                f"vset /object/{self._ACTOR}/hsva 0.0 0.0 1.0 -1000000.0",
                f"vset /object/{self._ACTOR}/roughness 0.6",
                f"vset /object/{self._ACTOR}/metallic 0.2",
            ],
        )

    def test_forced_hue_mode_emits_hsva_and_flat_texture_slot(self):
        # uniform() = 0.9 >= 0.8 → hue mode. Draws: mode, hue, sat, val, rough, metallic.
        override = self._exclusive_override()
        with patch(
            "numpy.random.uniform", side_effect=[0.9, 200.0, 0.5, 0.7, 0.6, 0.2]
        ):
            commands = _build_actor_material_commands(self._ACTOR, override, {})
        # Sampled hue tint + flat-white texture slot (index 0); alpha kept.
        self.assertEqual(
            commands,
            [
                f"vset /object/{self._ACTOR}/hsva 200.0 0.5 0.7 -1000000.0",
                f"vset /object/{self._ACTOR}/texture_index 0.0",
                f"vset /object/{self._ACTOR}/roughness 0.6",
                f"vset /object/{self._ACTOR}/metallic 0.2",
            ],
        )

    def test_per_component_texture_mode_routes_with_component_token(self):
        # Per-child form routes every command through the component token.
        override = self._exclusive_override()
        with patch("numpy.random.uniform", side_effect=[0.1, 0.5, 0.6, 0.2]):
            commands = _build_actor_material_commands(
                self._ACTOR, override, {}, component=["NIST_Board_Render"]
            )
        self.assertEqual(
            commands,
            [
                f"vset /object/{self._ACTOR}/texture_index NIST_Board_Render 0.5",
                f"vset /object/{self._ACTOR}/hsva NIST_Board_Render 0.0 0.0 1.0 -1000000.0",
                f"vset /object/{self._ACTOR}/roughness NIST_Board_Render 0.6",
                f"vset /object/{self._ACTOR}/metallic NIST_Board_Render 0.2",
            ],
        )

    def test_texture_mode_preserves_explicit_alpha(self):
        override = self._exclusive_override()
        override.alpha = 0.75
        with patch("numpy.random.uniform", side_effect=[0.1, 0.5, 0.6, 0.2]):
            commands = _build_actor_material_commands(self._ACTOR, override, {})
        self.assertIn(f"vset /object/{self._ACTOR}/hsva 0.0 0.0 1.0 0.75", commands)


class NamedMaterialDrTest(unittest.TestCase):
    def test_named_ranges_use_the_supplied_generator(self):
        override = _FakeMaterialOverride(
            roughness=None,
            metallic=None,
            specular=None,
            scalar_params={
                "HueShift": _FakeRangeConfig(-180.0, 180.0),
                "Fixed": 0.2,
                "Skipped": None,
            },
            vector_params={
                "ColorTint": [
                    _FakeRangeConfig(0.1, 0.9),
                    0.4,
                    _FakeRangeConfig(0.1, 0.9),
                ]
            },
        )
        with patch("numpy.random.uniform", side_effect=AssertionError("unseeded draw")):
            first = _build_actor_material_commands(
                "Foam", override, {}, np.random.default_rng(77)
            )
            repeated = _build_actor_material_commands(
                "Foam", override, {}, np.random.default_rng(77)
            )
            other = _build_actor_material_commands(
                "Foam", override, {}, np.random.default_rng(78)
            )
        self.assertEqual(first, repeated)
        self.assertNotEqual(first, other)
        self.assertEqual(len(first), 3)
        self.assertGreaterEqual(float(first[0].split()[-1]), -180.0)
        self.assertLessEqual(float(first[0].split()[-1]), 180.0)
        self.assertEqual(first[1], "vset /object/Foam/scalar_param Fixed 0.2")
        rgb = [float(value) for value in first[2].split()[-3:]]
        self.assertGreaterEqual(rgb[0], 0.1)
        self.assertLessEqual(rgb[0], 0.9)
        self.assertEqual(rgb[1], 0.4)
        self.assertGreaterEqual(rgb[2], 0.1)
        self.assertLessEqual(rgb[2], 0.9)


class DecimalDrValuesTest(unittest.TestCase):
    """UnrealCV's [float] pattern has no exponent: DR floats go out positionally."""

    def test_only_floats_that_format_with_an_exponent_change(self):
        self.assertEqual("0.0000612", _decimal(6.12e-05))
        self.assertEqual("0.0000612", _decimal(np.float32(6.12e-05)))
        self.assertEqual("0.5625084527225299", _decimal(0.5625084527225299))
        self.assertEqual("-1000000.0", _decimal(KEEP_PREVIOUS_TOKEN))
        self.assertEqual("3", _decimal(3))

    def test_a_tiny_sampled_hue_reaches_unrealcv_in_decimal_form(self):
        override = _FakeMaterialOverride(
            hue=_FakeRangeConfig(0.0, 360.0), roughness=_FakeRangeConfig(0.4, 0.9)
        )
        with patch("numpy.random.uniform", side_effect=[6.12e-05, 0.55]):
            commands = _build_actor_material_commands("Board", override, {})
        self.assertEqual(
            commands,
            [
                "vset /object/Board/hsva 0.0000612 -1000000.0 -1000000.0 -1000000.0",
                "vset /object/Board/roughness 0.55",
            ],
        )


class AbsentActorDrFailuresTest(unittest.TestCase):
    """DR configs deliberately name actors from more than one map variant.

    Whichever name is not in the loaded map fails on every episode reset, so an
    intentional no-op was reported ~867 times at ERROR over a 200-rollout eval.
    """

    _GHOST = "Curtains_GEN_VARIABLE_Curtains_C_CAT_5"
    _ABSENT = "error Can not find object"

    def setUp(self) -> None:
        # The registry is process-scoped by design, and this target runs the
        # whole file in one interpreter, so isolation here is load-bearing.
        patcher = patch(
            "superdex.physics.viewer.unrealcv.unrealcv_actor_utils._ABSENT_ACTORS_REPORTED",
            None,
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_absent_actor_reported_once_across_resets(self, mock_logger):
        for _ in range(3):
            _log_visibility_results(
                [f"vset /object/{self._GHOST}/hide"], [self._ABSENT], [self._GHOST]
            )
        mock_logger.error.assert_called_once()
        self.assertIn(self._GHOST, mock_logger.error.call_args[0][3])

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_absent_actor_reported_once_across_dr_kinds(self, mock_logger):
        # Every property targeting a missing actor reports the same one fact,
        # so the key is the actor, not the command.
        _log_material_dr_results(
            [f"vset /object/{self._GHOST}/hsva 1 2 3 4"], [self._ABSENT], [self._GHOST]
        )
        _log_visibility_results(
            [f"vset /object/{self._GHOST}/hide"], [self._ABSENT], [self._GHOST]
        )
        mock_logger.error.assert_called_once()

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_each_absent_actor_gets_its_own_line(self, mock_logger):
        # A typo'd actor name must stay discoverable, not be folded into a peer.
        for actor in (self._GHOST, "BP_Typoed_Actor_C_0"):
            _log_visibility_results(
                [f"vset /object/{actor}/hide"], [self._ABSENT], [actor]
            )
        self.assertEqual(mock_logger.error.call_count, 2)

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_other_failures_are_never_suppressed(self, mock_logger):
        # A present actor that rejects a command, and a dropped batch (resp
        # None), are live faults -- they must report on every reset.
        for _ in range(3):
            _log_visibility_results(
                ["vset /object/RealActor_C_0/hide"], ["error something else"], ["a"]
            )
            _log_visibility_results(["vset /object/RealActor_C_0/hide"], [None], ["a"])
        self.assertEqual(mock_logger.error.call_count, 6)


class MaterialDrPartialResultsTest(unittest.TestCase):
    """A material that lacks a DR parameter lacks it on every reset; at ~800
    warnings/s per training host the repeats crowded out every other log line."""

    _CMD = "vset /object/BP_Robot_T3_Left_UCV_C_0/roughness 0.5"
    _PARTIAL = "warning: 9/22 materials set, 13 missing 'Roughness' parameter"

    def setUp(self) -> None:
        patcher = patch(
            "superdex.physics.viewer.unrealcv.unrealcv_actor_utils._MATERIAL_DR_REPORTED",
            None,
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_a_partial_result_is_reported_once_across_resets(self, mock_logger):
        for _ in range(3):
            _log_material_dr_results([self._CMD], [self._PARTIAL], ["a"])
        mock_logger.warning.assert_called_once()

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_each_property_and_diagnostic_gets_its_own_line(self, mock_logger):
        _log_material_dr_results([self._CMD], [self._PARTIAL], ["a"])
        _log_material_dr_results(
            ["vset /object/BP_Robot_T3_Left_UCV_C_0/hsva 1 2 3 4"],
            [self._PARTIAL],
            ["a"],
        )
        _log_material_dr_results(
            [self._CMD], ["warning: 21/22 materials set, 1 missing 'Roughness'"], ["a"]
        )
        _log_material_dr_results([self._CMD], ["0/22 materials set, 22 missing"], ["a"])
        self.assertEqual(mock_logger.warning.call_count, 4)

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_an_unsupported_result_is_reported_once_across_resets(self, mock_logger):
        unsupported = "warning: 0/22 materials set, 22 missing 'Roughness' parameter"
        for _ in range(3):
            _log_material_dr_results([self._CMD], [unsupported], ["a"])
        mock_logger.warning.assert_called_once()
        self.assertIn("UNSUPPORTED", mock_logger.warning.call_args.args[0])

    @patch("superdex.physics.viewer.unrealcv.unrealcv_actor_utils.logger")
    def test_failures_still_report_on_every_reset(self, mock_logger):
        for _ in range(3):
            _log_material_dr_results([self._CMD], ["error rejected"], ["a"])
        self.assertEqual(mock_logger.warning.call_count, 3)
        for logged in mock_logger.warning.call_args_list:
            self.assertIn("FAILED", logged.args[0])


class GetMaterialParamsTest(unittest.TestCase):
    _ACTOR = "BP_NIST_Board_Render_UCV_C_0"
    _PAYLOAD = '{"actor":"A","scope":"actor","materials":[]}'

    def _client(self, response):
        client = MagicMock()
        client._request.return_value = response
        return client

    def test_whole_actor_command_and_parsed_payload(self):
        client = self._client(self._PAYLOAD)
        result = get_material_params(client, self._ACTOR)
        client._request.assert_called_once_with(
            f"vget /object/{self._ACTOR}/material_params"
        )
        self.assertEqual(result["scope"], "actor")

    def test_component_is_appended(self):
        client = self._client(self._PAYLOAD)
        get_material_params(client, self._ACTOR, component="DSUB_Housing")
        client._request.assert_called_once_with(
            f"vget /object/{self._ACTOR}/material_params DSUB_Housing"
        )

    def test_star_requests_per_component_breakdown(self):
        client = self._client(self._PAYLOAD)
        get_material_params(client, self._ACTOR, component="*")
        client._request.assert_called_once_with(
            f"vget /object/{self._ACTOR}/material_params *"
        )

    def test_config_resolves_mochi_key_to_instance_name(self):
        # A bot-task config never names the UE actor, so callers pass the
        # mapping key; it must be resolved before the vget is issued.
        cfg = SimpleNamespace(
            actor_mapping={"scene/Box___Actor": "prefab://box_prefab"},
            articulated_actor_mapping={},
        )
        client = self._client(self._PAYLOAD)
        get_material_params(client, "scene/Box___Actor", unrealcvactor_cfg=cfg)
        client._request.assert_called_once_with(
            "vget /object/scene/Box___Actor/material_params"
        )

    def test_blueprint_desired_name_wins_over_key(self):
        cfg = SimpleNamespace(
            actor_mapping={"scene/Box___Actor": "blueprint:///Game/BP_Box@BP_Box_C_1"},
            articulated_actor_mapping={},
        )
        client = self._client(self._PAYLOAD)
        get_material_params(client, "scene/Box___Actor", unrealcvactor_cfg=cfg)
        client._request.assert_called_once_with(
            "vget /object/BP_Box_C_1/material_params"
        )

    def test_articulated_prefab_entry_resolves_to_key(self):
        # Bot-task derivation gives articulated entries a prefab:// ue_actor with
        # no "@DesiredName", so the UE name falls back to the spawn key. This is
        # the other mapping branch: a dict read through "ue_actor".
        cfg = SimpleNamespace(
            actor_mapping={},
            articulated_actor_mapping={
                "scene/Robot": {"ue_actor": "prefab://fr3_dg5f"}
            },
        )
        client = self._client(self._PAYLOAD)
        get_material_params(client, "scene/Robot", unrealcvactor_cfg=cfg)
        client._request.assert_called_once_with(
            "vget /object/scene/Robot/material_params"
        )

    def test_error_response_returns_empty(self):
        client = self._client("error Can not find object")
        self.assertEqual(get_material_params(client, self._ACTOR), {})

    def test_malformed_json_returns_empty(self):
        client = self._client("not json at all")
        self.assertEqual(get_material_params(client, self._ACTOR), {})

    def test_dropped_response_returns_empty(self):
        client = self._client(None)
        self.assertEqual(get_material_params(client, self._ACTOR), {})


class GetLightInfoTest(unittest.TestCase):
    _ACTOR = "BP_SpotlightTest_C0"

    def _client(self, response):
        client = MagicMock()
        client._request.return_value = response
        return client

    def test_command_and_parsed_payload(self):
        client = self._client('{"actor":"A","lights":[{"component":"Spot"}]}')
        result = get_light_info(client, self._ACTOR)
        client._request.assert_called_once_with(
            f"vget /object/{self._ACTOR}/light_info"
        )
        self.assertEqual(result["lights"], [{"component": "Spot"}])

    def test_actor_without_lights_is_a_valid_answer(self):
        client = self._client('{"actor":"A","lights":[]}')
        self.assertEqual(get_light_info(client, self._ACTOR)["lights"], [])

    def test_config_resolves_key_to_instance_name(self):
        cfg = SimpleNamespace(
            actor_mapping={"Lamp": "blueprint:///Game/BP_Spot@BP_SpotlightTest_C0"},
            articulated_actor_mapping={},
        )
        client = self._client('{"lights":[]}')
        get_light_info(client, "Lamp", unrealcvactor_cfg=cfg)
        client._request.assert_called_once_with(
            "vget /object/BP_SpotlightTest_C0/light_info"
        )

    def test_failures_return_empty(self):
        for response in ("error Can not find object", "not json", None):
            self.assertEqual(
                get_light_info(self._client(response), self._ACTOR), {}, response
            )


class CaptureDrStateTest(unittest.TestCase):
    def _cfg(self, **fields):
        return SimpleNamespace(actor_mapping={}, articulated_actor_mapping={}, **fields)

    def _capture(self, cfg, responses):
        client = MagicMock()
        client._request_batch.return_value = [json.dumps(r) for r in responses]
        return client, capture_dr_state(client, cfg)

    def test_queries_actors_from_every_override_section(self):
        cfg = self._cfg(
            material_overrides={"Mat": None},
            sphere_placement_overrides={"Sphere": None},
            visibility_overrides={"Vis": 0.5},
            cast_shadow_overrides={"Shadow": False},
            hidden_actors=["Hidden"],
            hdri_lighting_override=SimpleNamespace(
                skylight_actor="Sky", dome_actor="Dome"
            ),
        )
        client, _ = self._capture(cfg, [{}] * 7)
        self.assertEqual(
            client._request_batch.call_args[0][0],
            [
                f"vget /object/{name}/dr_state"
                for name in ("Dome", "Hidden", "Mat", "Shadow", "Sky", "Sphere", "Vis")
            ],
        )

    def test_all_and_config_modes_request_every_parameter(self):
        for mode in ("all", "config"):
            cfg = self._cfg(visibility_overrides={"Mug": True}, dr_state_params=mode)
            client, _ = self._capture(cfg, [{}])
            self.assertEqual(
                client._request_batch.call_args[0][0],
                ["vget /object/Mug/dr_state all"],
                mode,
            )

    def test_config_mode_keeps_only_configured_params_and_components(self):
        box = SimpleNamespace(
            scalar_params={"Dirt": None},
            vector_params={"Edge": [0.5, 0.8, 0.2]},
            texture_params={"Albedo": None},
            components=[SimpleNamespace(names=["Lid"], scalar_params={"Wear": None})],
        )
        cfg = self._cfg(material_overrides={"Box": box}, dr_state_params="config")
        captured = {
            "roughness": 0.4,
            "scalar_params": {"Dirt": 0.7, "Roughness": 0.4, "Wear": 0.1},
            "vector_params": {"Edge": [0.9, 0.8, 0.2, 1.0], "Tint": [1, 1, 1, 1]},
            "texture_params": {"Albedo": "T_Box"},
            "components": {
                "Lid": {
                    "metallic": 0.8,
                    "scalar_params": {"Wear": 0.2, "Metallic": 0.8},
                },
                "Handle": {"roughness": 0.5},
            },
        }
        _, state = self._capture(cfg, [captured])
        self.assertEqual(
            state["Box"],
            {
                "roughness": 0.4,
                "scalar_params": {"Dirt": 0.7, "Wear": 0.1},
                "vector_params": {"Edge": [0.9, 0.8, 0.2, 1.0]},
                "texture_params": {"Albedo": "T_Box"},
                "components": {
                    "Lid": {"metallic": 0.8, "scalar_params": {"Wear": 0.2}}
                },
            },
        )

    def test_config_mode_keeps_hdri_dome_params(self):
        cfg = self._cfg(
            hdri_lighting_override=SimpleNamespace(
                skylight_actor=None, dome_actor="Dome", dome_cube_param=None
            ),
            dr_state_params="config",
        )
        captured = {
            "scalar_params": {"Brightness": 2.0, "Opacity": 1.0},
            "texture_params": {"EnvCube": "HDRI_3", "Noise": "T_Noise"},
        }
        _, state = self._capture(cfg, [captured])
        self.assertEqual(
            state["Dome"],
            {
                "scalar_params": {"Brightness": 2.0},
                "texture_params": {"EnvCube": "HDRI_3"},
            },
        )

    def test_config_mode_strips_generic_params_from_unconfigured_actors(self):
        cfg = self._cfg(
            material_overrides={"Box": SimpleNamespace(scalar_params={"Dirt": None})},
            visibility_overrides={"Curtain": True},
            dr_state_params="config",
        )
        box = {"scalar_params": {"Dirt": 0.7, "Roughness": 0.4}}
        curtain = {
            "visible": True,
            "scalar_params": {"Dirt": 0.1},
            "components": {"Panel": {"scalar_params": {"Dirt": 0.2}}},
        }
        _, state = self._capture(cfg, [box, curtain])
        self.assertEqual(state["Box"], {"scalar_params": {"Dirt": 0.7}})
        # "Curtain" has no material config, so its generic params and
        # components are stripped while non-generic state is kept.
        self.assertEqual(state["Curtain"], {"visible": True})

    def test_invalid_dr_state_params_raises(self):
        cfg = self._cfg(visibility_overrides={"Mug": True}, dr_state_params="bogus")
        client = MagicMock()
        with self.assertRaises(ValueError):
            capture_dr_state(client, cfg)
        client._request_batch.assert_not_called()

    def test_failed_actor_is_reported_not_silently_dropped(self):
        cfg = self._cfg(visibility_overrides={"Good": True, "Bad": True})
        client = MagicMock()
        # Actors are queried in sorted order: Bad first, then Good.
        client._request_batch.return_value = [
            "error Can not find object",
            json.dumps({"visible": True}),
        ]
        state = capture_dr_state(client, cfg)
        self.assertEqual(state["Good"], {"visible": True})
        self.assertIn("_error", state["Bad"])

    def test_unparseable_response_is_reported_not_silently_dropped(self):
        cfg = self._cfg(visibility_overrides={"Mug": True})
        client = MagicMock()
        client._request_batch.return_value = ["not json at all"]
        state = capture_dr_state(client, cfg)
        self.assertIn("_error", state["Mug"])
