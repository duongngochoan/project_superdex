"""Phase 2.2-2.4 -- Python data binding for native Wuji Hand 2 controllers and
name-based mapping of those angles onto the SuperDex bot's DOF vector.

Locked decisions honoured here
    * Retargeting objectives: native ports of wuji-retargeting VectorOptimizer and
      AdaptiveOptimizerAnalytical, using native NLopt SLSQP, chosen by YAML in ``configs/``.
    * Joint mapping: by *name*.  The URDF joint ``r_index_finger_pip`` maps to the SuperDex joint
      ``right_joint_11_index_finger_pip`` through the regex ``{side}_joint_\\d+_{suffix}``; the numeric
      index in the SuperDex name is never used.

Wrist frame
-----------
``wuji_retargeting`` first expresses the keypoints in a wrist frame::

    kp_wf = (kp - kp[0]) @ F @ O @ Rot^T

with ``F = estimate_frame_from_hand_points`` (columns x, palm-normal, z), ``O`` the OPERATOR2MANO matrix
of the hand side and ``Rot`` the per-config ``mediapipe_rotation``.  All three are orthonormal, so the
world pose of the URDF wrist link that makes the robot fingers line up with the human fingers is::

    world_R_wrist = F @ O @ Rot^T ,   world_p_wrist = kp[0]

(:func:`wrist_pose_world`; checked numerically in :func:`self_check`).  This is the Cartesian target
handed to the arm IK (Phase 3.2).
"""
from __future__ import annotations

import ctypes as ct
import re
import weakref
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
import nlopt
import nlopt._nlopt as nlopt_extension
from scipy.spatial.transform import Rotation as R

from wuji_retargeting.robot import RobotWrapper
from wuji_retargeting.mediapipe import OPERATOR2MANO_LEFT, OPERATOR2MANO_RIGHT, estimate_frame_from_hand_points

from .paths import CONFIGS
from .native import load_library, double_array, double_pointer, DOUBLE_POINTER, INT_POINTER

FLEX_SUFFIXES = ("mcp_flex", "pip", "dip", "thumb_mcp", "thumb_ip")   # joints that close the grasp


def config_path(side: str, optimizer: str = "vector") -> Path:
    return CONFIGS / f"wuji_hand2_{side}_{optimizer}.yaml"


class HandRetargeter:
    def __init__(self, side: str, optimizer: str = "vector", config: Path | None = None, prefab=None, sdp=None):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        if optimizer not in ("vector", "adaptive"):
            raise ValueError("optimizer must be 'vector' or 'adaptive'")
        self.side = side
        self.cfg_path = Path(config) if config else config_path(side, optimizer)
        with self.cfg_path.open() as stream:
            cfg = yaml.safe_load(stream)
        opt, options = cfg["optimizer"], cfg.get("retarget", {})
        mode = {"VectorOptimizer": 0, "AdaptiveOptimizerAnalytical": 1}.get(opt.get("type"))
        if mode is None:
            raise ValueError(f"Unsupported native optimizer: {opt.get('type')!r}")
        if options.get("project_tip_dir", False):
            raise ValueError("project_tip_dir is not supported by the native hand objective")
        urdf = (self.cfg_path.parent / opt["urdf_path"]).resolve()
        # Pinocchio reads robot-description metadata. Controller FK, Jacobians, objective,
        # SLSQP callbacks, temporal regularization and filtering all execute in hand.cpp.
        self.robot = RobotWrapper(str(urdf), hand_side=side)
        self.joint_names = list(self.robot.dof_joint_names)
        self.num_joints = self.robot.model.nq
        self.lo, self.hi = (double_array(self.robot.joint_limits[:, i]) for i in (0, 1))
        rot = options.get("mediapipe_rotation") or {}
        self.Rot = R.from_euler("xyz", [rot.get("x", 0.0), rot.get("y", 0.0), rot.get("z", 0.0)], degrees=True).as_matrix()
        self.O = OPERATOR2MANO_RIGHT if side == "right" else OPERATOR2MANO_LEFT
        self.prefix = "r_" if side == "right" else "l_"
        self._wrist_offset = double_array(options.get("wrist_offset_cm", [0, 0, 0]), (3,)) / 100
        self._thumb_offset = double_array(options.get("thumb_offset_cm", [0, 0, 0]), (3,)) / 100
        self._lib = load_library()
        self._configure_abi()
        model = self.robot.model
        axes_by_name = {}
        for joint in ET.parse(urdf).getroot().findall("joint"):
            if joint.attrib["type"] == "fixed":
                continue
            if joint.attrib["type"] != "revolute":
                raise ValueError("Native hand models support fixed and bounded revolute joints")
            axis = joint.find("axis")
            values = np.fromstring("1 0 0" if axis is None else axis.attrib.get("xyz", "1 0 0"), sep=" ")
            if values.shape != (3,) or not np.isfinite(values).all() or np.linalg.norm(values) == 0:
                raise ValueError(f"Invalid axis for joint {joint.attrib['name']}")
            axes_by_name[joint.attrib["name"]] = values / np.linalg.norm(values)
        parents = self._integers(model.parents)
        qindex = self._integers([-1 if j == 0 else model.joints[j].idx_q for j in range(model.njoints)])
        axes = double_array([[0, 0, 0]] + [axes_by_name[str(model.names[j])] for j in range(1, model.njoints)])
        origins = double_array([t.homogeneous for t in model.jointPlacements])
        frame_parents = self._integers([f.parentJoint for f in model.frames])
        frame_offsets = double_array([f.placement.homogeneous for f in model.frames])
        self._handle = self._lib.rt_hand_create(
            self.num_joints, model.njoints, model.nframes,
            self._ip(parents), self._ip(qindex), double_pointer(origins), double_pointer(axes),
            self._ip(frame_parents), double_pointer(frame_offsets), double_pointer(self.lo), double_pointer(self.hi),
            str(nlopt_extension.__file__).encode(), nlopt.LD_SLSQP,
        )
        if not self._handle:
            raise RuntimeError(self._lib.rt_hand_last_error().decode())
        self._finalizer = weakref.finalize(self, self._lib.rt_hand_destroy, self._handle)
        naming = opt["link_naming"]
        def role(key):
            return [naming["prefix"] + naming[key].format(finger=f) for f in naming["fingers"]]
        pip_names, dip_names, tip_names = role("pip"), role("dip"), role("tip")
        wrist = self.robot.get_link_index(naming["prefix"] + naming["palm"])
        roles = self._integers([self.robot.get_link_index(n) for n in pip_names + dip_names + tip_names])
        pip = self._integers([self.robot.get_actuated_qpos_index(n) for n in pip_names])
        dip = self._integers([self.robot.get_actuated_qpos_index(n) for n in dip_names])
        vectors = options.get("key_vectors", [
            dict(origin=naming["prefix"] + naming["palm"], task=name, origin_kp=0, task_kp=4*f+seg+2, scale=1.)
            for f in range(5) for seg, name in enumerate((pip_names[f], dip_names[f], tip_names[f]))
        ])
        if not vectors:
            raise ValueError("key_vectors must be a nonempty list")
        vec = self._integers([[self.robot.get_link_index(v["origin"]), self.robot.get_link_index(v["task"]),
                               v["origin_kp"], v["task_kp"]] for v in vectors])
        scales = double_array([v.get("scale", 1.) for v in vectors])
        params = [options.get(k, default) for k, default in (
            ("huber_delta", 2.), ("norm_delta", .04), ("huber_delta_dir", .5), ("w_pos", 1.),
            ("w_dir", 10.), ("scaling", 1.), ("w_full_hand", 1.), ("thumb_skip_pip", False),
            ("w_hyper", 0.), ("soft_min", 0.), ("w_couple", 0.), ("couple_ratio", .7), ("lp_alpha", .2))]
        for finger in ("thumb", "index", "middle", "ring", "pinky"):
            segment = options.get("segment_scaling", {}).get(finger, [1., 1., 1.])
            if len(segment) not in (3, 4):
                raise ValueError("segment_scaling must have three or four values per finger")
            params.extend(segment[-3:])
        params.extend(options.get("pinch_thresholds", {}).get(f, {}).get("d1", 2.) for f in ("index", "middle", "ring", "pinky"))
        params.extend(options.get("pinch_thresholds", {}).get(f, {}).get("d2", 4.) for f in ("index", "middle", "ring", "pinky"))
        params = double_array(params, (36,))
        self._check(self._lib.rt_hand_configure(self._handle, mode, wrist, self._ip(roles), self._ip(pip), self._ip(dip),
                                              len(vectors), self._ip(vec), double_pointer(scales), double_pointer(params)))
        self.dof_indices = None
        if prefab is not None:
            self.bind_prefab(prefab, sdp)

    @staticmethod
    def _integers(values):
        return np.ascontiguousarray(values, dtype=np.int32)

    @staticmethod
    def _ip(array):
        return array.ctypes.data_as(INT_POINTER)

    def _check(self, status):
        if status != 0:
            raise ValueError(self._lib.rt_hand_last_error().decode())

    def _configure_abi(self):
        lib = self._lib
        lib.rt_hand_last_error.argtypes = []
        lib.rt_hand_last_error.restype = ct.c_char_p
        lib.rt_hand_create.argtypes = [ct.c_int]*3 + [INT_POINTER]*2 + [DOUBLE_POINTER]*2 + [INT_POINTER] + [DOUBLE_POINTER]*3 + [ct.c_char_p, ct.c_int]
        lib.rt_hand_create.restype = ct.c_void_p
        lib.rt_hand_destroy.argtypes = [ct.c_void_p]
        lib.rt_hand_destroy.restype = None
        lib.rt_hand_configure.argtypes = [ct.c_void_p, ct.c_int, ct.c_int] + [INT_POINTER]*3 + [ct.c_int, INT_POINTER, DOUBLE_POINTER, DOUBLE_POINTER]
        lib.rt_hand_sequence.argtypes = [ct.c_void_p, ct.c_int, DOUBLE_POINTER, ct.c_int, DOUBLE_POINTER, DOUBLE_POINTER, INT_POINTER]
        lib.rt_hand_fk_points.argtypes = [ct.c_void_p, DOUBLE_POINTER, ct.c_int, INT_POINTER, ct.c_int, DOUBLE_POINTER]
        lib.rt_hand_cost_grad.argtypes = [ct.c_void_p] + [DOUBLE_POINTER]*5
        lib.rt_hand_squeeze.argtypes = [ct.c_void_p, ct.c_int, DOUBLE_POINTER, INT_POINTER, DOUBLE_POINTER, ct.c_double, DOUBLE_POINTER]
        for name in ("configure", "sequence", "fk_points", "cost_grad", "squeeze"):
            getattr(lib, "rt_hand_" + name).restype = ct.c_int

    # ---- name-based mapping -------------------------------------------------------------
    def bind_prefab(self, prefab, sdp):
        revolute = [j.name for j in prefab.joints if j.type == sdp.ArticulatedJointType.REVOLUTE]
        idx = []
        for jn in self.joint_names:
            suffix = jn[len(self.prefix):]
            m = [i for i, n in enumerate(revolute) if re.fullmatch(rf"{self.side}_joint_\d+_{re.escape(suffix)}", n)]
            if len(m) != 1:
                raise KeyError(f"URDF joint {jn!r} -> SuperDex joint: {len(m)} matches ({m})")
            idx.append(m[0])
        self.dof_indices = np.array(idx)
        self.superdex_joint_names = [revolute[i] for i in idx]
        return self.dof_indices

    # ---- wrist pose -----------------------------------------------------------------------
    def wrist_pose_world(self, kp: np.ndarray) -> np.ndarray:
        kp = np.asarray(kp, dtype=np.float64)
        F = estimate_frame_from_hand_points(kp - kp[0:1])
        T = np.eye(4)
        T[:3, :3] = F @ self.O @ self.Rot.T
        T[:3, 3] = kp[0]
        return T

    # ---- retargeting ------------------------------------------------------------------------
    def retarget_sequence(self, keypoints: np.ndarray, apply_filter: bool = True):
        """(T,21,3) world keypoints -> dict(q (T,nq), wrist_T (T,4,4), cost (T,))."""
        keypoints = double_array(keypoints)
        if keypoints.ndim != 3 or keypoints.shape[1:] != (21, 3) or len(keypoints) == 0:
            raise ValueError("keypoints must have nonempty shape (T,21,3)")
        wrist = np.asarray([self.wrist_pose_world(kp) for kp in keypoints])
        transformed = np.asarray([(kp-kp[0]) @ W[:3, :3] for kp, W in zip(keypoints, wrist)])
        transformed[:, 5:] += self._wrist_offset
        transformed[:, 1:5] += self._thumb_offset
        transformed = double_array(transformed)
        q, cost = np.empty((len(keypoints), self.num_joints)), np.empty(len(keypoints))
        statuses = np.empty(len(keypoints), dtype=np.int32)
        self._check(self._lib.rt_hand_sequence(self._handle, len(keypoints), double_pointer(transformed), bool(apply_filter),
                                               double_pointer(q), double_pointer(cost), self._ip(statuses)))
        return {"q": q, "wrist_T": wrist, "cost": cost, "solver_status": statuses, "joint_names": self.joint_names}

    def fk_points(self, q: np.ndarray, link_names: list[str]) -> np.ndarray:
        """Native FK link origins in the wrist frame; Python resolves frame names only."""
        q = double_array(q, (self.num_joints,))
        indices = self._integers([self.robot.get_link_index(n) for n in link_names])
        points = np.empty((len(indices), 3), dtype=np.float64)
        self._check(self._lib.rt_hand_fk_points(self._handle, double_pointer(q), len(indices), self._ip(indices),
                                              self.robot.get_link_index(self.prefix + "wrist"), double_pointer(points)))
        return points

    def cost_and_gradient(self, q, wrist_keypoints, previous=None):
        """Evaluate the native objective in the configured wrist frame (useful for validation)."""
        q = double_array(q, (self.num_joints,))
        kp = double_array(wrist_keypoints, (21, 3))
        prev = None if previous is None else double_array(previous, (self.num_joints,))
        loss, grad = np.empty(1), np.empty(self.num_joints)
        self._check(self._lib.rt_hand_cost_grad(self._handle, double_pointer(q), double_pointer(kp),
                                              double_pointer(prev), double_pointer(loss), double_pointer(grad)))
        return float(loss[0]), grad

    def tip_links(self):
        return [self.prefix + f for f in ("thumb_tip", "index_finger_tip", "middle_finger_tip", "ring_finger_tip", "pinky_tip")]

    def robot_tips_world(self, q, wrist_T):
        p = self.fk_points(q, self.tip_links())
        return (wrist_T[:3, :3] @ p.T).T + wrist_T[:3, 3]

    def flex_mask(self) -> np.ndarray:
        return np.array([any(n.endswith(s) for s in FLEX_SUFFIXES) for n in self.joint_names])

    def apply_grasp_squeeze(self, q: np.ndarray, weight: np.ndarray, squeeze_rad: float) -> np.ndarray:
        """Phase 2.4 / 4 tuning knob: add extra flexion (rad) to the finger flexion joints, scaled by a
        per-frame weight in [0,1] (ramped on at grasp onset).  Clipped to the joint limits."""
        q = double_array(q)
        if q.ndim != 2 or q.shape[1] != self.num_joints or len(q) == 0:
            raise ValueError(f"q must have nonempty shape (T,{self.num_joints})")
        weight = double_array(weight, (len(q),))
        if np.any((weight < 0) | (weight > 1)):
            raise ValueError("squeeze weights must be in [0,1]")
        mask = self._integers(self.flex_mask())
        output = np.empty_like(q)
        self._check(self._lib.rt_hand_squeeze(self._handle, len(q), double_pointer(q), self._ip(mask),
                                            double_pointer(weight), float(squeeze_rad), double_pointer(output)))
        return output

    def to_superdex_pose(self, q_hand: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """Write hand joint angles (URDF order) into a full SuperDex DOF vector (copy)."""
        out = np.array(pose, dtype=np.float64, copy=True)
        out[self.dof_indices] = q_hand
        return out

    # ---- validation --------------------------------------------------------------------------
    def self_check(self, keypoints: np.ndarray, n: int = 20) -> dict:
        """Retarget a few frames and compare robot fingertips (FK + wrist pose) with the human
        fingertips in world: this validates the wrist-frame formula; the residual is the
        retargeting error itself (morphology mismatch)."""
        res = self.retarget_sequence(keypoints[:n], apply_filter=False)
        errs = []
        for kp, q, W in zip(keypoints[:n], res["q"], res["wrist_T"]):
            tips_r = self.robot_tips_world(q, W)
            tips_h = kp[[4, 8, 12, 16, 20]]
            errs.append(np.linalg.norm(tips_r - tips_h, axis=1))
        errs = np.asarray(errs)
        # control: a wrong wrist rotation gives errors of several cm; vector retargeting gives < ~1.5 cm
        return {"tip_err_mean_cm": float(errs.mean() * 100), "tip_err_max_cm": float(errs.max() * 100),
                "per_finger_mean_cm": (errs.mean(0) * 100).round(2).tolist()}
