"""Native audits must distinguish invalid geometry from repairable timing."""
import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.constraints import (
    JointLimits, audit_trajectory, project_positions, retime_trajectory, prepare_trajectory,
)


def limits():
    return JointLimits(("arm", "hand"), np.array([-1., 0.]), np.array([1., 2.]),
                       np.array([.5, 1.]), np.array([1., 2.]))


def test_retime_preserves_path_and_bounds_both_derivatives():
    times = np.linspace(0., 1., 101)
    smooth = 3 * times**2 - 2 * times**3
    q = np.column_stack([.8 * smooth, 1.5 * smooth])
    original = q.copy()
    assert not audit_trajectory(q, times, limits(), assume_rest_at_ends=True)["passed"]
    slower, report = retime_trajectory(q, times, limits())
    assert report["passed"] and report["time_scale"] > 1
    np.testing.assert_array_equal(q, original)
    assert audit_trajectory(q, slower, limits(), assume_rest_at_ends=True)["passed"]


def test_retime_rejects_bad_positions_instead_of_hiding_them():
    q = np.array([[0., 0.], [1.1, 1.], [0., 1.]])
    report = audit_trajectory(q, [0., 1., 2.], limits())
    assert not report["position_passed"]
    assert report["max_position_violation_rad"] == pytest.approx(.1)
    with pytest.raises(ValueError, match="position-limit"):
        retime_trajectory(q, [0., 1., 2.], limits())
    projected = project_positions(q, limits(), margin=.05)
    assert np.all(projected >= limits().lower + .05)
    assert np.all(projected <= limits().upper - .05)


def test_supplied_velocity_is_audited_instead_of_inferred_stationarity():
    q = np.zeros((3, 2))
    v = np.array([[0., 0.], [2., 0.], [0., 0.]])
    report = audit_trajectory(q, [0., 1., 2.], limits(), velocity=v)
    assert not report["passed"] and report["max_velocity_limit_ratio"] == 4
    assert report["derivative_source"] == "measured_velocity"
    assert report["effort_audited"] is False


@pytest.mark.parametrize("times", [[0., 0., 1.], [0., np.nan, 1.], [1., 0., 2.]])
def test_malformed_clocks_rejected(times):
    with pytest.raises(ValueError):
        audit_trajectory(np.zeros((3, 2)), times, limits())


def test_position_projection_rejects_impossible_margin():
    with pytest.raises(ValueError, match="margins"):
        project_positions(np.zeros((3, 2)), limits(), margin=2.)


def test_effort_failure_requires_actual_effort_samples():
    bounded = JointLimits(("arm", "hand"), np.array([-1., 0.]), np.array([1., 2.]),
                          np.array([.5, 1.]), np.array([1., 2.]), np.array([40., .6]))
    q = np.zeros((3, 2))
    efforts = np.array([[0., 0.], [20., .7], [0., 0.]])
    report = audit_trajectory(q, [0., 1., 2.], bounded, efforts=efforts)
    assert not report["passed"] and report["effort_audited"]
    assert report["effort_passed"] is False
    assert report["max_effort_limit_ratio"] == pytest.approx(.7 / .6)
    assert report["effort_violation_samples"] == 1
    report = audit_trajectory(q, [0., 1., 2.], bounded)
    assert report["effort_passed"] is None and not report["effort_audited"]


def test_prepare_smooths_limit_corners_and_audits_exact_final_command_grid():
    times = np.linspace(0., 1., 101)
    q = np.column_stack([1.2 * np.sin(np.pi * times / 2), 1.8 * times])
    original = q.copy()
    output_times, output_q, report = prepare_trajectory(q, times, limits(), dt=.005, margin=.02)
    assert report["passed"]
    assert report["projection_max_change_rad"] == pytest.approx(.22)
    assert report["smoothing_max_change_rad"] > 0
    assert report["analytic_max_velocity_limit_ratio"] <= 1.
    assert report["analytic_max_acceleration_limit_ratio"] <= 1.
    np.testing.assert_allclose(np.diff(output_times), .005, atol=1e-12)
    np.testing.assert_allclose(output_q[0], [0., .02])
    np.testing.assert_allclose(output_q[-1], [.98, 1.8])
    assert audit_trajectory(output_q, output_times, limits(), assume_rest_at_ends=True)["passed"]
    np.testing.assert_array_equal(q, original)


def test_prepare_without_smoothing_preserves_input_knots_and_rest_bounds():
    q = np.array([[0., .1], [.2, .4], [.4, .8], [.4, .8]])
    t, prepared, report = prepare_trajectory(q, [0., 1., 2., 3.], limits(), smoothing_sigma_seconds=0.,
                                            margin=0., dt=.01)
    assert report["smoothing_max_change_rad"] == 0
    assert report["projection_max_change_rad"] == 0
    np.testing.assert_array_equal(prepared[0], q[0])
    np.testing.assert_allclose(prepared[-1], q[-1])


@pytest.mark.parametrize("kwargs", [dict(smoothing_sigma_seconds=-1), dict(dt=0), dict(safety_factor=.9)])
def test_prepare_rejects_invalid_settings(kwargs):
    with pytest.raises(ValueError):
        prepare_trajectory(np.zeros((3, 2)), [0., 1., 2.], limits(), **kwargs)


def test_analytic_retime_matches_known_rest_to_rest_cubic_extrema():
    bounded = JointLimits(("joint",), np.array([-2.]), np.array([2.]), np.array([1.]), np.array([2.]))
    times, q, report = prepare_trajectory([[0.], [1.]], [0., 1.], bounded, margin=0., dt=.001,
                                         smoothing_sigma_seconds=0., safety_factor=1.05)
    # Hermite rest-to-rest q(s)=3s²-2s³ has exact max |v|=1.5, max |a|=6.
    expected_scale = np.sqrt(6. / 2.) * 1.05
    assert report["time_scale"] == pytest.approx(expected_scale)
    assert report["analytic_peak_velocity_rad_s"] == pytest.approx(1.5 / expected_scale)
    assert report["analytic_peak_acceleration_rad_s2"] == pytest.approx(6. / expected_scale**2)
    source_t = np.minimum(times / expected_scale, 1.)
    np.testing.assert_allclose(q[:, 0], 3 * source_t**2 - 2 * source_t**3, atol=2e-14)


def test_nonuniform_turning_path_stays_inside_bounds_without_hidden_clipping():
    bounded = JointLimits(("joint",), np.array([-1.]), np.array([1.]), np.array([.5]), np.array([1.]))
    times, q, report = prepare_trajectory([[0.], [.8], [-.8], [.1]], [0., .3, 1.7, 2.], bounded,
                                         margin=0., smoothing_sigma_seconds=0., dt=.005)
    assert report["passed"] and report["projection_max_change_rad"] == 0
    assert q.min() >= -.8 - 1e-12 and q.max() <= .8 + 1e-12
