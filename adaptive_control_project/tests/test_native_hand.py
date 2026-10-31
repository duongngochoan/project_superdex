"""Regression checks against the original hand objectives and independent FK reference."""
import pickle

import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.hand_retarget import HandRetargeter
from adaptive_control_project.retarget_pipeline.paths import THIRD_PARTY
from wuji_retargeting import Retargeter
from wuji_retargeting.mediapipe import apply_mediapipe_transformations


@pytest.fixture(scope="module")
def hand_samples():
    with (THIRD_PARTY / "wuji-retargeting/example/data/avp1.pkl").open("rb") as stream:
        rec = pickle.load(stream)
    return {side: np.asarray([frame[f"{side}_fingers"] for frame in rec[:4]]) for side in ("left", "right")}


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("optimizer", ["vector", "adaptive"])
def test_native_hand_objective_and_gradient_match_reference(side, optimizer, hand_samples):
    native = HandRetargeter(side, optimizer)
    reference = Retargeter.from_yaml(str(native.cfg_path), hand_side=side).optimizer
    q = np.random.default_rng(12).uniform(native.lo, native.hi)
    previous = np.random.default_rng(22).uniform(native.lo, native.hi)
    kp = apply_mediapipe_transformations(hand_samples[side][0], side) @ native.Rot.T
    if optimizer == "vector":
        expected = reference._loss_and_grad(q, reference._compute_target_vectors(kp), previous)
    else:
        expected = reference._loss_and_grad_analytical(
            q, reference._compute_tip_vectors(kp, reference.scaling), reference._compute_tip_dirs(kp),
            reference._compute_full_hand_vectors(kp, reference.segment_scaling), reference._compute_pinch_alpha(kp), previous)
    actual = native.cost_and_gradient(q, kp, previous)
    np.testing.assert_allclose(actual[0], expected[0], rtol=1e-11, atol=1e-11)
    np.testing.assert_allclose(actual[1], expected[1], rtol=1e-9, atol=1e-9)


@pytest.mark.parametrize("optimizer", ["vector", "adaptive"])
def test_native_hand_sequence_is_deterministic_and_preserves_reference_solution(optimizer, hand_samples):
    native = HandRetargeter("right", optimizer)
    reference = Retargeter.from_yaml(str(native.cfg_path), hand_side="right")
    expected = np.asarray([reference.retarget(kp) for kp in hand_samples["right"]])
    result = native.retarget_sequence(hand_samples["right"])
    actual = result["q"]
    assert result["solver_status"].shape == (len(hand_samples["right"]),)
    assert np.all(result["solver_status"] > 0)
    np.testing.assert_allclose(actual, expected, atol=2e-4, rtol=2e-4)
    np.testing.assert_array_equal(actual, native.retarget_sequence(hand_samples["right"])["q"])


def test_native_squeeze_and_input_validation(hand_samples):
    native = HandRetargeter("right")
    q = np.repeat(((native.lo + native.hi) / 2)[None], 3, axis=0)
    weights = np.asarray([0., .5, 1.])
    expected = np.clip(q + weights[:, None] * native.flex_mask()[None] * .3, native.lo, native.hi)
    np.testing.assert_array_equal(native.apply_grasp_squeeze(q, weights, .3), expected)
    with pytest.raises(ValueError):
        native.retarget_sequence(np.zeros((0, 21, 3)))
    with pytest.raises(ValueError):
        native.retarget_sequence(np.full((1, 21, 3), np.nan))
    with pytest.raises(ValueError):
        native.apply_grasp_squeeze(q, [-1, 0, 1], .3)


def test_hand_c_abi_rejects_null_model_buffers():
    native = HandRetargeter("right")
    assert native._lib.rt_hand_create(0, 0, 0, None, None, None, None, None, None, None, None, None, 0) is None
    assert b"Invalid hand dimensions" in native._lib.rt_hand_last_error()
