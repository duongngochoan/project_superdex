"""Numerical regression checks for C++ controller behavior without a physics runtime."""
from types import SimpleNamespace as Obj

import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.arm_ik import ArmChain


def planar_chain():
    transform = lambda x: Obj(rotation=[0., 0., 0., 1.], translation=[x, 0., 0.])
    links = [Obj(name="base", parent_link=-1, parent_joint_from_link=transform(0)),
             Obj(name="first", parent_link=0, parent_joint_from_link=transform(1)),
             Obj(name="tip", parent_link=1, parent_joint_from_link=transform(1))]
    fixed = Obj(name="root", type="fixed", parent_link_from_joint=transform(0))
    joints = [fixed] + [Obj(name=f"j{i}", type="revolute", axis=[0., 0., 1.],
                           min_limit=[0., 0., -2.8], max_limit=[0., 0., 2.8],
                           parent_link_from_joint=transform(0)) for i in range(2)]
    return ArmChain(Obj(links=links, joints=joints), "base", "tip",
                    Obj(ArticulatedJointType=Obj(REVOLUTE="revolute")))


def test_fk_and_jacobian_finite_difference():
    chain = planar_chain()
    q = np.array([0.25, -0.4])
    pose, jac = chain.jacobian(q)
    expected = [np.cos(q[0]) + np.cos(q.sum()), np.sin(q[0]) + np.sin(q.sum()), 0]
    np.testing.assert_allclose(pose[:3, 3], expected, atol=1e-12)
    for i in range(chain.n):
        bumped = q.copy()
        bumped[i] += 1e-7
        numerical = (chain.fk(bumped)[0][:3, 3] - pose[:3, 3]) / 1e-7
        np.testing.assert_allclose(jac[:3, i], numerical, atol=2e-7)


def test_ik_and_trajectory_reach_targets_within_limits():
    chain = planar_chain()
    desired = np.array([[0.1, -0.25], [0.12, -0.27], [0.15, -0.28]])
    targets = np.array([chain.fk(q)[0] for q in desired])
    q, ep, er = chain.solve_trajectory(targets, np.zeros(2), multistart=3, w_posture=0)
    assert np.max(ep) < 1e-4
    assert np.max(er) < 1e-3
    assert np.all(q >= chain.lo) and np.all(q <= chain.hi)
    q2, ep2, er2 = chain.solve_trajectory(targets, np.zeros(2), multistart=3, w_posture=0)
    np.testing.assert_array_equal(q, q2)
    np.testing.assert_array_equal(ep, ep2)
    np.testing.assert_array_equal(er, er2)
    distant = targets[-1].copy()
    distant[0, 3] = 10
    q, ep, er, _ = chain.solve(distant, np.zeros(2))
    assert np.all(q >= chain.lo) and np.all(q <= chain.hi)
    assert ep > 1 and np.isfinite(er)


def test_invalid_native_inputs_raise_instead_of_crashing():
    chain = planar_chain()
    with pytest.raises(ValueError, match="shape"):
        chain.fk([0])
    with pytest.raises(ValueError, match="finite"):
        chain.fk([np.nan, 0])
    with pytest.raises(ValueError, match="max_iters"):
        chain.solve(np.eye(4), np.zeros(2), max_iters=0)
    with pytest.raises(ValueError, match="orthonormal"):
        invalid = np.eye(4)
        invalid[0, 0] = 2
        chain.solve(invalid, np.zeros(2))
    q, ep, er = chain.solve_trajectory(np.empty((0, 4, 4)), np.zeros(2))
    assert q.shape == (0, 2) and ep.shape == er.shape == (0,)
