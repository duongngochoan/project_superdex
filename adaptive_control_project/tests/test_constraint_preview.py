"""Preview evidence must come from final robot commands and exact prefab FK."""
import json

import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.constraint_preview import RobotSkeleton, preview_samples, render_constraint_previews
from adaptive_control_project.retarget_pipeline.tasks import Task


def test_video_sampling_preserves_clock_duration_and_final_commands():
    times = np.array([3., 3.1, 3.2])
    commands = np.column_stack([times, -times])
    output_times, sampled = preview_samples(times, commands, 20.)
    np.testing.assert_allclose(output_times, [3., 3.05, 3.1, 3.15, 3.2])
    np.testing.assert_allclose(sampled, np.column_stack([output_times, -output_times]))


@pytest.mark.parametrize("times,q,fps", [([0., 0.], [[0.], [0.]], 15.),
    ([0., 1.], [[0.], [np.nan]], 15.), ([0., 1.], [[0.], [1.]], 0.)])
def test_invalid_preview_timing_and_commands_are_rejected(times, q, fps):
    with pytest.raises(ValueError):
        preview_samples(times, q, fps)


@pytest.mark.parametrize("side", ["left", "right"])
def test_preview_fk_matches_exact_superdex_fingertips(side):
    import superdex.physics as sdp
    import superdex.robotics as sdr
    from superdex.physics.paths import resolve_asset
    from adaptive_control_project.retarget_pipeline.paths import BOT_ASSET

    sdp.initialize(num_worker_threads=0)
    scene = bot = None
    try:
        prefab = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
        rng = np.random.default_rng(13)
        q = np.asarray(prefab.default_pose, dtype=float)
        for index, joint in enumerate(j for j in prefab.joints if j.type == sdp.ArticulatedJointType.REVOLUTE):
            axis = np.asarray(joint.axis)
            lo, hi = np.asarray(joint.min_limit) @ axis, np.asarray(joint.max_limit) @ axis
            q[index] = rng.uniform(lo + .01, hi - .01)
        prefab.default_pose = q.tolist()
        skeleton = RobotSkeleton(prefab, sdp, [side]).points(q[None])
        scene = sdp.create_scene("constraint preview FK")
        bot = sdr.create_bot(scene, prefab, sdr.create_context())
        transforms = sdp.DynamicArrayTransformRT(len(prefab.links))
        bot.get_articulated_actor().get_articulated_link_transforms(transforms)
        names = [link.name for link in prefab.links]
        import re
        for finger, points in zip(("thumb", "index_finger", "middle_finger", "ring_finger", "pinky"), skeleton[f"{side}_hand_world"][0]):
            link = next(i for i, name in enumerate(names) if re.fullmatch(rf"{side}_link_\d+_{finger}_tip", name))
            np.testing.assert_allclose(points[-1], np.asarray(transforms[link].translation), atol=1e-6)
        # The wrist in every finger chain connects to the exact arm endpoint.
        np.testing.assert_allclose(skeleton[f"{side}_hand_world"][0, :, 0],
            np.repeat(skeleton[f"{side}_arm_world"][0, -1][None], 5, axis=0), atol=1e-12)
    finally:
        if bot is not None:
            sdr.destroy_bot(scene, bot)
        sdp.shutdown()


def test_constraint_preview_refuses_out_of_bounds_commands_before_rendering(tmp_path):
    import superdex.physics as sdp
    import superdex.robotics as sdr
    from superdex.physics.paths import resolve_asset
    from adaptive_control_project.retarget_pipeline.paths import BOT_ASSET

    sdp.initialize(num_worker_threads=0)
    try:
        prefab = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
        names = [joint.name for joint in prefab.joints if joint.type == sdp.ArticulatedJointType.REVOLUTE]
        q = np.repeat(np.asarray(prefab.default_pose, dtype=float)[None], 2, axis=0)
        q[:, 0] = 100.
    finally:
        sdp.shutdown()
    path = tmp_path / "trajectory.npz"
    np.savez(path, task="test", hands=["right"], timestamps=[0., 1.], q_target=q,
             includes_replay_timeline=True, dof_names=names, constraint_policy=json.dumps({}))
    task = Task(tmp_path, {"id": "test", "hands": ["right"]})
    with pytest.raises(ValueError, match="constraint audit failed"):
        render_constraint_previews(task, path, run="invalid")
    assert not (tmp_path / "videos").exists()
