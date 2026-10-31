"""Validate task input contracts, synchronization, and hand geometry."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest


PROJECT = Path(__file__).resolve().parents[1]
from adaptive_control_project.tests import motion_fixtures as GENERATOR

def test_synthetic_hands_share_clock_and_preserve_bones():
    arrays = GENERATOR.generate_arrays(["left", "right"])
    np.testing.assert_allclose(np.diff(arrays["timestamps"]), 1. / 30.)
    for side in ("left", "right"):
        points = arrays[f"keypoints_{side}"]
        assert points.shape == (91, 21, 3)
        assert np.isfinite(points).all()
        for first in (1, 5, 9, 13, 17):
            lengths = np.linalg.norm(np.diff(points[:, first:first + 4], axis=1), axis=2)
            np.testing.assert_allclose(lengths, np.repeat(lengths[:1], len(points), axis=0), atol=1e-14)
        np.testing.assert_allclose(points[-1, 0] - points[0, 0], [0.04, 0., 0.04], atol=1e-14)
    np.testing.assert_allclose(arrays["keypoints_left"][:, 0] - arrays["keypoints_right"][:, 0],
                               np.repeat((arrays["keypoints_left"][0, 0] - arrays["keypoints_right"][0, 0])[None], 91, axis=0))


@pytest.mark.parametrize("hands", [[], ["left", "left"], ["invalid"]])
def test_reject_invalid_hands(hands):
    with pytest.raises(ValueError, match="hands"):
        GENERATOR.generate_arrays(hands)


@pytest.mark.parametrize("side", ["left", "right"])
def test_generated_wrist_orientation_matches_actual_hand_convention(side):
    # This checks the per-side YAML convention, which differs between hands.
    from scipy.spatial.transform import Rotation
    from wuji_retargeting.mediapipe import estimate_frame_from_hand_points

    import yaml

    config = yaml.safe_load((PROJECT / f"retarget_pipeline/configs/wuji_hand2_{side}_vector.yaml").read_text())
    euler = config["retarget"]["mediapipe_rotation"]
    rotation = Rotation.from_euler("xyz", [euler[axis] for axis in "xyz"], degrees=True).as_matrix()
    arrays = GENERATOR.generate_arrays([side])
    for index in (0, 45, 90):
        keypoints = arrays[f"keypoints_{side}"][index]
        actual = estimate_frame_from_hand_points(keypoints - keypoints[0]) @ GENERATOR.OPERATOR_TO_MANO[side] @ rotation.T
        np.testing.assert_allclose(actual, arrays[f"wrist_rot_{side}"][index], atol=1e-14)
