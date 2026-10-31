"""Prefab parsing and Python buffer bindings for the C++ arm controller.

FK, Jacobians, damped least-squares IK, random restarts and trajectory warm
starts execute in ``cpp/arm.cpp``. This module reads robot model metadata and
preserves the public ArmChain interface used by alignment and replay.
"""
from __future__ import annotations

from dataclasses import dataclass

from .native import double_array, double_pointer

import numpy as np
from scipy.spatial.transform import Rotation as R


def _T(rot_xyzw, trans):
    M = np.eye(4)
    M[:3, :3] = R.from_quat(np.asarray(rot_xyzw, dtype=float)).as_matrix()
    M[:3, 3] = np.asarray(trans, dtype=float)
    return M


@dataclass
class ChainJoint:
    name: str
    dof_index: int                 # index into the actor's DOF vector
    axis: np.ndarray               # unit axis in the joint frame
    lo: float
    hi: float
    parent_link_from_joint: np.ndarray   # 4x4
    joint_from_link: np.ndarray          # 4x4 (child link frame from joint frame at q=0)


class ArmChain:
    """Kinematic chain base_link -> tip_link extracted from a SuperDex bot prefab."""

    def __init__(self, prefab, base_link: str, tip_link: str, sdp):
        import numpy as np
        links = {l.name: l for l in prefab.links}
        link_index = {l.name: i for i, l in enumerate(prefab.links)}
        joints = list(prefab.joints)
        # joint i connects links[i].parent_link -> links[i] (prefab joints and links are index-aligned)
        assert len(joints) == len(prefab.links)
        # walk from tip to base
        path = []
        i = link_index[tip_link]
        base_i = link_index[base_link]
        while i != base_i:
            path.append(i)
            parent = prefab.links[i].parent_link           # int index (or name in older assets)
            i = parent if isinstance(parent, int) else link_index[parent]
            if i < 0:
                raise ValueError(f"{base_link} is not an ancestor of {tip_link}")
        path = path[::-1]
        # DOF numbering: revolute joints in prefab order, skipping the root joint
        dof = 0
        dof_of = {}
        for i, j in enumerate(joints):
            if j.type == sdp.ArticulatedJointType.REVOLUTE:
                dof_of[i] = dof
                dof += 1
        self.num_actor_dofs = dof
        self.elements = []        # list of (ChainJoint | None, fixed 4x4) in order base->tip
        self.joints: list[ChainJoint] = []
        for i in path:
            j = joints[i]
            L = prefab.links[i]
            plj = _T(j.parent_link_from_joint.rotation, j.parent_link_from_joint.translation)
            jfl = _T(L.parent_joint_from_link.rotation, L.parent_joint_from_link.translation)
            if j.type == sdp.ArticulatedJointType.REVOLUTE:
                axis = np.asarray(j.axis, dtype=float)
                # SuperDex stores the limits as 3-vectors q_lim * axis (e.g. axis (0,-1,0),
                # min_limit (0, 1.396, 0) means q_min = -1.396), so project onto the axis.
                lo = float(np.dot(np.asarray(j.min_limit, dtype=float), axis))
                hi = float(np.dot(np.asarray(j.max_limit, dtype=float), axis))
                assert lo <= hi, (j.name, lo, hi)
                cj = ChainJoint(j.name, dof_of[i], axis / np.linalg.norm(axis), lo, hi, plj, jfl)
                self.joints.append(cj)
                self.elements.append((cj, None))
            else:
                self.elements.append((None, plj @ jfl))
        self.dof_indices = np.array([cj.dof_index for cj in self.joints])
        self.lo = np.array([cj.lo for cj in self.joints])
        self.hi = np.array([cj.hi for cj in self.joints])
        self.n = len(self.joints)
        self._init_native()

    def _init_native(self):
        import ctypes as ct
        import weakref

        from .native import DOUBLE_POINTER, INT_POINTER, load_library

        self._lib = load_library()
        lib = self._lib
        lib.ar_arm_last_error.restype = ct.c_char_p
        lib.ar_arm_create.restype = ct.c_void_p
        lib.ar_arm_create.argtypes = [ct.c_int, ct.c_int, INT_POINTER] + [DOUBLE_POINTER] * 5
        lib.ar_arm_destroy.argtypes = [ct.c_void_p]
        lib.ar_arm_destroy.restype = None
        lib.ar_arm_fk.argtypes = [ct.c_void_p] + [DOUBLE_POINTER] * 5
        lib.ar_arm_jacobian.argtypes = [ct.c_void_p] + [DOUBLE_POINTER] * 4
        lib.ar_arm_solve.argtypes = ([ct.c_void_p] + [DOUBLE_POINTER] * 5
                                     + [ct.c_int, ct.c_int, ct.c_uint] + [DOUBLE_POINTER] * 2)
        lib.ar_arm_trajectory.argtypes = ([ct.c_void_p, ct.c_int] + [DOUBLE_POINTER] * 5
                                          + [ct.c_int, ct.c_int, ct.c_uint] + [DOUBLE_POINTER] * 3)
        kinds, before, after, axes = [], [], [], []
        for joint, fixed in self.elements:
            kinds.append(int(joint is not None))
            before.append(fixed if joint is None else joint.parent_link_from_joint)
            after.append(np.eye(4) if joint is None else joint.joint_from_link)
            axes.append(np.zeros(3) if joint is None else joint.axis)
        kinds = np.ascontiguousarray(kinds, dtype=np.int32)
        before, after, axes = map(double_array, (before, after, axes))
        self._handle = lib.ar_arm_create(
            len(kinds), self.n, kinds.ctypes.data_as(INT_POINTER),
            *map(double_pointer, (before, after, axes, self.lo, self.hi)))
        if not self._handle:
            raise ValueError(lib.ar_arm_last_error().decode())
        self._finalizer = weakref.finalize(self, lib.ar_arm_destroy, self._handle)

    def _check(self, status):
        if status:
            raise ValueError(self._lib.ar_arm_last_error().decode())

    def fk(self, q: np.ndarray, base_T=np.eye(4)):
        """Native FK returns tip pose and world-space joint axes/origins."""
        q, base = double_array(q, (self.n,)), double_array(base_T, (4, 4))
        pose, axes, origins = np.empty((4, 4)), np.empty((self.n, 3)), np.empty((self.n, 3))
        self._check(self._lib.ar_arm_fk(self._handle, *map(double_pointer, (q, base, pose, axes, origins))))
        return pose, axes, origins

    def jacobian(self, q, base_T=np.eye(4)):
        """Native geometric Jacobian, translation first and rotation last."""
        q, base = double_array(q, (self.n,)), double_array(base_T, (4, 4))
        pose, jac = np.empty((4, 4)), np.empty((6, self.n))
        self._check(self._lib.ar_arm_jacobian(self._handle, *map(double_pointer, (q, base, pose, jac))))
        return pose, jac

    def _solver_buffers(self, base_T=np.eye(4), q_posture=None, w_posture=1e-4, w_rot=1.0,
                        damping=0.02, max_iters=100, max_step=0.2, tol_pos=1e-4, tol_rot=1e-3):
        base = double_array(base_T, (4, 4))
        posture = None if q_posture is None else double_array(q_posture, (self.n,))
        params = double_array([w_posture, w_rot, damping, max_step, tol_pos, tol_rot])
        if not isinstance(max_iters, (int, np.integer)) or max_iters <= 0:
            raise ValueError("max_iters must be a positive integer")
        return base, posture, params, int(max_iters)

    def _solve_native(self, target_T, q_init, starts=0, seed=0, **kw):
        if not isinstance(starts, (int, np.integer)) or starts < 0:
            raise ValueError("n_starts must be a nonnegative integer")
        if not isinstance(seed, (int, np.integer)) or not 0 <= seed <= 2**32 - 1:
            raise ValueError("seed must be an unsigned 32-bit integer")
        target, initial = double_array(target_T, (4, 4)), double_array(q_init, (self.n,))
        base, posture, params, iterations = self._solver_buffers(**kw)
        out, errors = np.empty(self.n), np.empty(3)
        self._check(self._lib.ar_arm_solve(
            self._handle, *map(double_pointer, (target, initial, base, posture, params)),
            iterations, int(starts), int(seed), double_pointer(out), double_pointer(errors)))
        return out, float(errors[0]), float(errors[1]), int(errors[2])

    def solve(self, target_T, q_init, base_T=np.eye(4), q_posture=None, w_posture=1e-4, w_rot=1.0,
              damping=0.02, max_iters=100, max_step=0.2, tol_pos=1e-4, tol_rot=1e-3):
        """Native damped least-squares IK -> (q, position error [m], rotation error [rad], iterations)."""
        return self._solve_native(target_T, q_init, base_T=base_T, q_posture=q_posture,
                                  w_posture=w_posture, w_rot=w_rot, damping=damping,
                                  max_iters=max_iters, max_step=max_step, tol_pos=tol_pos, tol_rot=tol_rot)

    def solve_multistart(self, target_T, q_init, n_starts: int = 24, seed: int = 0, **kw):
        """Native deterministic random restarts; keep the smallest position/rotation error."""
        return self._solve_native(target_T, q_init, starts=n_starts, seed=seed, **kw)

    def solve_trajectory(self, targets, q0, multistart: int = 0, **kw):
        """Native warm-started trajectory solve, with restarts on the first frame."""
        targets = double_array(targets)
        if targets.ndim != 3 or targets.shape[1:] != (4, 4):
            raise ValueError("targets must have shape (T, 4, 4)")
        if not isinstance(multistart, (int, np.integer)) or multistart < 0:
            raise ValueError("multistart must be a nonnegative integer")
        seed = kw.pop("seed", 0)
        if not isinstance(seed, (int, np.integer)) or not 0 <= seed <= 2**32 - 1:
            raise ValueError("seed must be an unsigned 32-bit integer")
        q0 = double_array(q0, (self.n,))
        base, posture, params, iterations = self._solver_buffers(**kw)
        count = len(targets)
        out, position, rotation = np.empty((count, self.n)), np.empty(count), np.empty(count)
        self._check(self._lib.ar_arm_trajectory(
            self._handle, count, *map(double_pointer, (targets, q0, base, posture, params)),
            iterations, int(multistart), int(seed), *map(double_pointer, (out, position, rotation))))
        return out, position, rotation
