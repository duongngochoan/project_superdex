"""Robot limit metadata and Python data bindings to native trajectory constraint checks.

Joint position bounds come from the exact simulation prefab. Hand URDF velocity
limits further bound explicit task operating caps. Arm speed and all acceleration
caps are task policy because those limits are absent from the shipped prefab.
This audit covers positions, rates and accelerations; effort metadata alone does
not establish torque compliance without measured joint torques.
"""
from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
import re
import xml.etree.ElementTree as ET

import numpy as np

from .native import DOUBLE_POINTER, INT_POINTER, double_array, double_pointer, load_library
from .paths import WUJI_HAND2_URDF


@dataclass(frozen=True)
class JointLimits:
    names: tuple[str, ...]
    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    effort: np.ndarray | None = None
    velocity_sources: tuple[str, ...] = ()

    def to_json(self) -> dict:
        return {"joint_names": list(self.names), "lower_rad": self.lower.tolist(),
                "upper_rad": self.upper.tolist(), "velocity_rad_s": self.velocity.tolist(),
                "acceleration_rad_s2": self.acceleration.tolist(),
                "effort_nm": None if self.effort is None else self.effort.tolist(),
                "velocity_sources": list(self.velocity_sources),
                "acceleration_source": "task_operating_policy",
                "position_source": "loaded_superdex_prefab",
                "effort_audited": False}


def limits_from_prefab(prefab, sdp, *, arm_velocity: float = 1.0, hand_velocity: float = 1.5,
                       arm_acceleration: float = 2.0, hand_acceleration: float = 4.0) -> JointLimits:
    """Extract actual joint bounds; apply documented conservative task rate caps."""
    caps = double_array([arm_velocity, hand_velocity, arm_acceleration, hand_acceleration])
    if np.any(caps <= 0):
        raise ValueError("operating velocity/acceleration caps must be positive")
    hand_velocities = {}
    for side, path in WUJI_HAND2_URDF.items():
        prefix = "l_" if side == "left" else "r_"
        for joint in ET.parse(path).getroot().findall("joint"):
            limit = joint.find("limit")
            if limit is not None and "velocity" in limit.attrib:
                name = joint.attrib["name"]
                if name.startswith(prefix):
                    hand_velocities[(side, name[len(prefix):])] = float(limit.attrib["velocity"])
    names, lower, upper, velocity, acceleration, effort, sources = [], [], [], [], [], [], []
    for joint in prefab.joints:
        if joint.type != sdp.ArticulatedJointType.REVOLUTE:
            continue
        axis = np.asarray(joint.axis, dtype=float)
        axis_norm = np.linalg.norm(axis)
        if not np.isclose(axis_norm, 1., atol=1e-6):
            raise ValueError(f"Expected a unit revolute axis for {joint.name}")
        names.append(joint.name)
        lower.append(float(np.asarray(joint.min_limit) @ axis))
        upper.append(float(np.asarray(joint.max_limit) @ axis))
        effort.append(float(joint.effort_limit))
        hand = re.fullmatch(r"(left|right)_joint_\d+_(.*)", joint.name)
        if hand:
            model_cap = hand_velocities.get((hand[1], hand[2]))
            if model_cap is None or not np.isfinite(model_cap) or model_cap <= 0:
                raise ValueError(f"Missing hand URDF velocity limit for {joint.name}")
            velocity.append(min(hand_velocity, model_cap))
            acceleration.append(hand_acceleration)
            sources.append("min(pinned_urdf_velocity, task_hand_operating_cap)")
        else:
            velocity.append(arm_velocity)
            acceleration.append(arm_acceleration)
            sources.append("task_arm_operating_cap; no arm velocity in prefab")
    return JointLimits(tuple(names), *(double_array(v) for v in (lower, upper, velocity, acceleration, effort)),
                       tuple(sources))


def _library():
    library = load_library()
    library.ar_constraints_last_error.restype = ct.c_char_p
    library.ar_constraints_audit.argtypes = ([ct.c_int, ct.c_int] + [DOUBLE_POINTER] * 9 +
                                           [ct.c_int, ct.c_double, ct.c_double, DOUBLE_POINTER, DOUBLE_POINTER, INT_POINTER])
    library.ar_constraints_audit.restype = ct.c_int
    library.ar_constraints_retime.argtypes = ([ct.c_int, ct.c_int] + [DOUBLE_POINTER] * 6 +
                                            [ct.c_int, ct.c_double, ct.c_double, DOUBLE_POINTER, DOUBLE_POINTER])
    library.ar_constraints_retime.restype = ct.c_int
    library.ar_constraints_project.argtypes = [ct.c_int, ct.c_int] + [DOUBLE_POINTER] * 5
    library.ar_constraints_project.restype = ct.c_int
    return library


def _buffers(q, timestamps, limits):
    q = double_array(q)
    if q.ndim != 2 or q.shape[0] < 2 or q.shape[1] != len(limits.names):
        raise ValueError("joint positions must be (N,number_of_named_joints), N >= 2")
    times = double_array(timestamps, (len(q),))
    bounds = [double_array(value, (q.shape[1],)) for value in
              (limits.lower, limits.upper, limits.velocity, limits.acceleration)]
    return q, times, bounds


def _check(library, status):
    if status:
        raise ValueError(library.ar_constraints_last_error().decode())


def audit_trajectory(q, timestamps, limits: JointLimits, *, velocity=None, efforts=None,
                     assume_rest_at_ends: bool = False, position_tolerance: float = 1e-6,
                     rate_tolerance: float = 1e-6) -> dict:
    """Check sampled motion in C++; supplied actual velocities improve state audits.

    Otherwise velocities use interval slopes and accelerations use successive
    slope changes at interval midpoints. Rest endpoint checks are optional and
    should be enabled for planned paths expected to start/stop at rest.
    """
    q, times, bounds = _buffers(q, timestamps, limits)
    velocities = None if velocity is None else double_array(velocity, q.shape)
    measured_efforts = None if efforts is None else double_array(efforts, q.shape)
    effort_limits = None
    if efforts is not None:
        if limits.effort is None:
            raise ValueError("measured effort audit requires model effort limits")
        effort_limits = double_array(limits.effort, (q.shape[1],))
    summary = np.empty(13)
    per_joint = np.empty((q.shape[1], 9))
    counts = np.empty((q.shape[1], 4), dtype=np.int32)
    lib = _library()
    _check(lib, lib.ar_constraints_audit(len(q), q.shape[1], *map(double_pointer, (q, times, *bounds, velocities, measured_efforts, effort_limits)),
                                        int(assume_rest_at_ends), position_tolerance, rate_tolerance,
                                        double_pointer(summary), double_pointer(per_joint), counts.ctypes.data_as(INT_POINTER)))
    keys = ("minimum_position_margin_rad", "max_position_violation_rad", "peak_velocity_rad_s",
            "peak_acceleration_rad_s2", "velocity_limit_ratio", "acceleration_limit_ratio", "required_time_scale",
            "peak_effort_nm", "effort_limit_ratio")
    details = [{"joint": name, **dict(zip(keys, map(float, values))),
                "position_violation_samples": int(violations[0]), "velocity_violation_samples": int(violations[1]),
                "acceleration_violation_samples": int(violations[2]), "effort_violation_samples": int(violations[3])}
               for name, values, violations in zip(limits.names, per_joint, counts)]
    return {"passed": bool(np.all(counts == 0)), "position_passed": bool(np.all(counts[:, 0] == 0)),
            "velocity_passed": bool(np.all(counts[:, 1] == 0)), "acceleration_passed": bool(np.all(counts[:, 2] == 0)),
            "max_position_violation_rad": float(summary[0]), "minimum_position_margin_rad": float(summary[1]),
            "max_velocity_limit_ratio": float(summary[2]), "max_acceleration_limit_ratio": float(summary[3]),
            "peak_velocity_rad_s": float(summary[4]), "peak_acceleration_rad_s2": float(summary[5]),
            "required_time_scale": float(summary[6]), "position_violation_samples": int(summary[7]),
            "velocity_violation_samples": int(summary[8]), "acceleration_violation_samples": int(summary[9]),
            "max_effort_limit_ratio": None if efforts is None else float(summary[10]),
            "peak_effort_nm": None if efforts is None else float(summary[11]),
            "effort_violation_samples": None if efforts is None else int(summary[12]),
            "effort_passed": None if efforts is None else bool(np.all(counts[:, 3] == 0)),
            "derivative_source": "measured_velocity" if velocity is not None else "sampled_position_interval_slopes",
            "assume_rest_at_ends": bool(assume_rest_at_ends), "effort_audited": efforts is not None,
            "position_tolerance_rad": float(position_tolerance), "rate_relative_tolerance": float(rate_tolerance),
            "per_joint": details}


def retime_trajectory(q, timestamps, limits: JointLimits, *, safety_factor: float = 1.02,
                      assume_rest_at_ends: bool = True, position_tolerance: float = 1e-6):
    """Return (retimed timestamps, report), preserving every joint position exactly.

    Global dilation bounds sampled velocities and accelerations. Position-invalid
    paths raise: changing time cannot repair geometry. Resampling/interpolation
    can change derivative peaks, so audit the final replay command grid too.
    """
    q, times, bounds = _buffers(q, timestamps, limits)
    output, scale = np.empty_like(times), np.empty(1)
    lib = _library()
    _check(lib, lib.ar_constraints_retime(len(q), q.shape[1], *map(double_pointer, (q, times, *bounds)),
                                         int(assume_rest_at_ends), position_tolerance, safety_factor,
                                         double_pointer(output), double_pointer(scale)))
    report = audit_trajectory(q, output, limits, assume_rest_at_ends=assume_rest_at_ends,
                              position_tolerance=position_tolerance)
    report["time_scale"] = float(scale[0])
    return output, report


def project_positions(q, limits: JointLimits, *, margin=0.0):
    """Explicit native position clamping for planned commands; audit original state separately."""
    q = double_array(q)
    if q.ndim != 2 or q.shape[1] != len(limits.names) or len(q) < 1:
        raise ValueError("joint positions must be a nonempty (N,number_of_named_joints) array")
    margins = double_array(np.broadcast_to(margin, (q.shape[1],)))
    lower, upper = (double_array(value, (q.shape[1],)) for value in (limits.lower, limits.upper))
    output = np.empty_like(q)
    lib = _library()
    _check(lib, lib.ar_constraints_project(len(q), q.shape[1],
                                          *map(double_pointer, (q, lower, upper, margins, output))))
    return output


def prepare_trajectory(q, timestamps, limits: JointLimits, *, margin=.015, dt=.005,
                       smoothing_sigma_seconds=.08, safety_factor=1.05):
    """Return (uniform times, commands, report) from a native bounded smooth path.

    Explicit preprocessing projects into the position margin and optionally
    Gaussian-smooths each joint using positive weights. Native PCHIP then fits a
    bound-preserving C1 path with resting endpoints. Its exact polynomial
    velocity/acceleration extrema determine global dilation before uniform
    sampling. Projection/smoothing displacement is reported, never hidden as
    time-only retiming. Physical contact success must be rechecked afterward.
    """
    q, times, bounds = _buffers(q, timestamps, limits)
    margins = double_array(np.broadcast_to(margin, (q.shape[1],)))
    lib = _library()
    lib.ar_constraints_prepare.argtypes = ([ct.c_int, ct.c_int] + [DOUBLE_POINTER] * 7 +
                                           [ct.c_double] * 3 + [ct.c_int, INT_POINTER] + [DOUBLE_POINTER] * 3)
    lib.ar_constraints_prepare.restype = ct.c_int
    count = ct.c_int()
    metadata = np.empty(8)
    inputs = (len(q), q.shape[1], *map(double_pointer, (q, times, *bounds, margins)),
              smoothing_sigma_seconds, dt, safety_factor)
    _check(lib, lib.ar_constraints_prepare(*inputs, 0, ct.byref(count), None, None, double_pointer(metadata)))
    output_times = np.empty(count.value)
    output_q = np.empty((count.value, q.shape[1]))
    _check(lib, lib.ar_constraints_prepare(*inputs, count.value, ct.byref(count), double_pointer(output_times),
                                          double_pointer(output_q), double_pointer(metadata)))
    report = audit_trajectory(output_q, output_times, limits, assume_rest_at_ends=True)
    report.update(time_scale=float(metadata[0]), projection_max_change_rad=float(metadata[1]),
                  smoothing_max_change_rad=float(metadata[2]), smoothing_sigma_seconds=float(smoothing_sigma_seconds),
                  interpolation="native_bound_preserving_pchip_with_resting_endpoints",
                  analytic_peak_velocity_rad_s=float(metadata[3]), analytic_peak_acceleration_rad_s2=float(metadata[4]),
                  analytic_max_velocity_limit_ratio=float(metadata[5]), analytic_max_acceleration_limit_ratio=float(metadata[6]),
                  exact_path_duration_s=float(metadata[7]), uniform_dt_s=float(dt),
                  source_start_time_s=float(times[0]), source_duration_s=float(times[-1] - times[0]),
                  projection_margin_rad=margins.tolist())
    if not report["passed"]:
        raise RuntimeError("Prepared uniform command grid failed its native constraints audit")
    return output_times, output_q, report
