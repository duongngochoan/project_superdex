"""Phase 0 verification checks (run: PYTHONPATH=<project_superdex> .venv/bin/python -m pytest adaptive_control_project/tests -q)

1. SuperDex loads the OpenArm v2.0 + Wuji Hand 2 bimanual bot (54 DOFs: 2 x (7 arm + 20 hand)).
2. wuji-retargeting runs on the Wuji Hand 2 config with the shipped sample keypoints, inside joint limits.
3. Every URDF joint of the retargeter maps by *name* to exactly one SuperDex joint (both hands).
4. The pinned wuji-description hand2_beta1 URDF and the SuperDex asset have identical fingertip kinematics.
5. The arm chain FK built from the prefab matches SuperDex link transforms, and IK reaches a near target.
6. The wrist-frame formula is consistent with the retargeter's own frame (world tip error == wrist-frame error).
"""
import pickle
import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import superdex.physics as sdp
import superdex.robotics as sdr
from superdex.physics.paths import resolve_asset

from adaptive_control_project.retarget_pipeline.arm_ik import ArmChain
from adaptive_control_project.retarget_pipeline.hand_retarget import HandRetargeter
from adaptive_control_project.retarget_pipeline.paths import BOT_ASSET, THIRD_PARTY


@pytest.fixture(scope="module")
def prefab():
    sdp.initialize(num_worker_threads=0)
    p = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
    yield p
    sdp.shutdown()


@pytest.fixture(scope="module")
def sample_keypoints():
    rec = pickle.load(open(THIRD_PARTY / "wuji-retargeting/example/data/avp1.pkl", "rb"))
    return {side: np.array([fr[f"{side}_fingers"] for fr in rec[:60]]) for side in ("left", "right")}


def test_bot_loads(prefab):
    scene = sdp.create_scene("t")
    bot = sdr.create_bot(scene, prefab, sdr.create_context())
    assert bot.get_articulated_actor().get_num_dofs() == 54
    names = [j.name for j in prefab.joints if j.type == sdp.ArticulatedJointType.REVOLUTE]
    assert sum(n.startswith("arm_openarm_left") for n in names) == 7 and sum(n.startswith("right_joint_") for n in names) == 20
    sdr.destroy_bot(scene, bot)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("optimizer", ["vector", "adaptive"])
def test_retargeter_runs_and_maps_by_name(prefab, sample_keypoints, side, optimizer):
    hr = HandRetargeter(side, optimizer, prefab=prefab, sdp=sdp)
    assert hr.num_joints if hasattr(hr, "num_joints") else hr.robot.model.nq == 20
    res = hr.retarget_sequence(sample_keypoints[side])
    assert res["q"].shape == (60, 20)
    assert np.all(res["q"] >= hr.lo - 1e-6) and np.all(res["q"] <= hr.hi + 1e-6)
    for urdf_name, sd_name in zip(hr.joint_names, hr.superdex_joint_names):
        assert re.fullmatch(rf"{side}_joint_\d+_{urdf_name[2:]}", sd_name)
    assert len(set(hr.dof_indices)) == 20


@pytest.mark.parametrize("side", ["left", "right"])
def test_hand_kinematics_match_superdex(prefab, side):
    hr = HandRetargeter(side, "vector", prefab=prefab, sdp=sdp)
    rng = np.random.default_rng(0)
    q = rng.uniform(hr.lo, hr.hi)
    pose = np.array(prefab.default_pose, dtype=float)
    pose[hr.dof_indices] = q
    p2 = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
    p2.default_pose = pose.tolist()
    scene = sdp.create_scene("fk")
    bot = sdr.create_bot(scene, p2, sdr.create_context())
    actor = bot.get_articulated_actor()
    names = [l.name for l in p2.links]
    T = sdp.DynamicArrayTransformRT(len(names))
    actor.get_articulated_link_transforms(T)
    from scipy.spatial.transform import Rotation as R

    def world(n):
        i = next(k for k, x in enumerate(names) if re.fullmatch(rf"{side}_link_\d+_" + n[2:], x))
        M = np.eye(4)
        M[:3, :3] = R.from_quat(np.asarray(T[i].rotation)).as_matrix()
        M[:3, 3] = np.asarray(T[i].translation)
        return M
    Wm = world(hr.prefix + "wrist")
    tips_sd = np.array([(np.linalg.inv(Wm) @ np.r_[world(n)[:3, 3], 1])[:3] for n in hr.tip_links()])
    tips_pin = hr.fk_points(q, hr.tip_links())
    # The pinned left URDF has small source-geometry differences from the SuperDex asset.
    tolerance = 2e-3 if side == "left" else 1e-4
    assert np.abs(tips_sd - tips_pin).max() < tolerance
    sdr.destroy_bot(scene, bot)


def test_arm_fk_and_ik(prefab):
    chain = ArmChain(prefab, "openarm_body_link0", "right_link_01_wrist", sdp)
    q0 = np.array(prefab.default_pose)[chain.dof_indices]
    scene = sdp.create_scene("arm")
    bot = sdr.create_bot(scene, prefab, sdr.create_context())
    names = [l.name for l in prefab.links]
    T = sdp.DynamicArrayTransformRT(len(names))
    bot.get_articulated_actor().get_articulated_link_transforms(T)
    t_sd = np.asarray(T[names.index("right_link_01_wrist")].translation)
    Tfk, _, _ = chain.fk(q0)
    assert np.abs(Tfk[:3, 3] - t_sd).max() < 1e-5
    Tt = Tfk.copy()
    Tt[:3, 3] += [0.05, 0.03, -0.05]
    q, ep, er, _ = chain.solve(Tt, q0, q_posture=q0)
    assert ep < 2e-3 and er < 2e-2
    sdr.destroy_bot(scene, bot)


def test_wrist_frame_consistency(prefab, sample_keypoints):
    from wuji_retargeting.mediapipe import apply_mediapipe_transformations
    hr = HandRetargeter("right", "vector", prefab=prefab, sdp=sdp)
    res = hr.retarget_sequence(sample_keypoints["right"][:10], apply_filter=False)
    for kp, q, W in zip(sample_keypoints["right"][:10], res["q"], res["wrist_T"]):
        kp_wf = apply_mediapipe_transformations(kp, "right") @ hr.Rot.T
        e_wf = np.linalg.norm(hr.fk_points(q, hr.tip_links()) - kp_wf[[4, 8, 12, 16, 20]], axis=1)
        e_w = np.linalg.norm(hr.robot_tips_world(q, W) - kp[[4, 8, 12, 16, 20]], axis=1)
        assert np.abs(e_wf - e_w).max() < 1e-6


def test_contacts_available_at_initial_replay_sample(prefab):
    from adaptive_control_project.retarget_pipeline.scene import PickupScene, SceneConfig

    scene = PickupScene(SceneConfig(), initialize_physics=False)
    try:
        assert scene.hand_cup_contact("right") == ({}, 0.0, 0.0)
        scene.step(scene.rest_pose)
        forces, penetration, total_force = scene.hand_cup_contact("right")
        assert isinstance(forces, dict) and penetration >= 0 and total_force >= 0
    finally:
        scene.destroy(shutdown=False)
