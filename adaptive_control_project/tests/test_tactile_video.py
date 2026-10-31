"""Reject unsynchronized data before drawing forces on a reused hand view."""
import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.tactile_video import (
    synchronized_indices, validate_camera_clock, validate_same_motion,
)


def test_capture_samples_select_actual_measured_states_without_interpolation():
    times = np.arange(21) * .005
    captures = np.array([0., .035, .070, .100])
    np.testing.assert_array_equal(synchronized_indices(times, captures), [0, 7, 14, 20])
    with pytest.raises(ValueError, match="saved measured-state timestamp"):
        synchronized_indices(times, [0., .033333, .07])
    with pytest.raises(ValueError, match="saved measured-state timestamp"):
        synchronized_indices(times, [0., .105])


def test_shifted_camera_frames_cannot_receive_force_markers():
    force_times = np.array([0., .035, .07])
    validate_camera_clock(force_times.copy(), force_times)
    with pytest.raises(ValueError, match="identical capture timestamps"):
        validate_camera_clock(force_times + .005, force_times)
    with pytest.raises(ValueError, match="identical capture timestamps"):
        validate_camera_clock(force_times[:-1], force_times)


@pytest.mark.parametrize("key", ["t", "q", "cup_pos", "object_quat"])
def test_same_commands_do_not_certify_matching_measured_motion(key):
    original = {"t": np.array([0., 1.]), "q": np.zeros((2, 54)),
                "cup_pos": np.zeros((2, 3)), "object_quat": np.tile([0., 0., 0., 1.], (2, 1))}
    candidate = {name: value.copy() for name, value in original.items()}
    validate_same_motion(original, candidate)
    candidate[key].flat[0] += .01
    with pytest.raises(ValueError, match=f"measured {key}"):
        validate_same_motion(original, candidate)
