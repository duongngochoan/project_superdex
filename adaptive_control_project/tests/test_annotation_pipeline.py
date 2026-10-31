"""Annotation outputs are bounded complete control streams, and unsafe imports never step physics."""
import json
import hashlib
from pathlib import Path

import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.data_convert import save_sequence
from adaptive_control_project.retarget_pipeline.pipeline import replay_task, retarget_task
from adaptive_control_project.retarget_pipeline.tasks import Task
from adaptive_control_project.tests.motion_fixtures import generate_arrays


def annotation_task(tmp_path):
    count = 8
    arrays = generate_arrays(['right'])
    keypoints = arrays['keypoints_right'][:count]
    data = dict(keypoints=keypoints, wrist_pos=keypoints[:, 0].copy(),
                wrist_rot=arrays['wrist_rot_right'][:count],
                timestamps=arrays['timestamps'][:count], fps=30.,
                hand='right', frame='world', up_axis='z')
    save_sequence(tmp_path / 'input.npz', **data)
    return Task(tmp_path, {'id': 'annotation_test', 'hands': ['right'],
                           'sequences': {'right': 'input.npz'}, 'control': {'optimizer': 'vector'}})


def test_annotation_export_has_bounded_transitions_and_distinct_source_clock(tmp_path):
    task = annotation_task(tmp_path)
    out = tmp_path / 'run'
    trajectory = retarget_task(task, out)
    limits = json.loads((out / 'joint_limits.json').read_text())
    with np.load(trajectory, allow_pickle=False) as archive:
        q, times = archive['q_target'], archive['timestamps']
        assert archive['includes_replay_timeline'].item() is True
        assert archive['motion_source'].item() == 'annotations'
        provenance = json.loads(str(archive['source_sequences']))
        assert provenance['right']['sha256'] == hashlib.sha256((task.root / 'input.npz').read_bytes()).hexdigest()
        assert archive['fps'].item() == 200
        assert archive['source_fps'].item() == 30
        assert len(archive['source_timestamps']) == 8
        assert archive['right_arm_q'].shape == (8, 7)
        assert archive['right_hand_q'].shape == (8, 20)
        assert q.shape == (len(times), 54) and len(times) > 200
        np.testing.assert_allclose(np.diff(times), .005, atol=1e-12)
        assert np.isfinite(q).all()
        assert (q >= np.asarray(limits['lower_rad']) + .015 - 1e-8).all()
        assert (q <= np.asarray(limits['upper_rad']) - .015 + 1e-8).all()
        velocity = np.diff(q, axis=0) / np.diff(times)[:, None]
        acceleration = np.diff(velocity, axis=0) / .005
        assert (np.abs(velocity) <= np.asarray(limits['velocity_rad_s']) * (1 + 1e-6)).all()
        assert (np.abs(acceleration) <= np.asarray(limits['acceleration_rad_s2']) * (1 + 1e-6)).all()
        scene = json.loads(str(archive['scene_config']))
        assert scene['dt'] == .005
    metrics = json.loads((out / 'metrics.json').read_text())
    assert metrics['command_constraints_passed'] is True
    assert metrics['constraints_passed'] is None
    assert metrics['manipulation_success'] is None


def test_task_alignment_and_grip_assistance_preserve_inspectable_source_results(tmp_path):
    task = annotation_task(tmp_path)
    transform = np.eye(4)
    transform[:3, 3] = [.03, -.02, .01]
    (tmp_path / 'alignment.json').write_text(json.dumps(transform.tolist()))
    task.manifest['annotation_alignment'] = 'alignment.json'
    task.manifest['control']['grasp_assist'] = {
        'angle_rad': .15, 'start_time_s': 0., 'ramp_time_s': .1,
    }
    path = retarget_task(task, tmp_path / 'run')
    with np.load(path) as saved, np.load(tmp_path / 'input.npz') as source:
        np.testing.assert_allclose(saved['right_source_keypoints_world'], source['keypoints'] + transform[:3, 3])
        np.testing.assert_allclose(saved['robot_from_source'], transform)
        # The first pose has no extra squeeze, later poses retain an explicit adjustment.
        np.testing.assert_allclose(saved['right_hand_q'][0], saved['right_optimizer_hand_q'][0])
        assert np.any(np.abs(saved['right_hand_q'][1:] - saved['right_optimizer_hand_q'][1:]) > 1e-6)
        assert saved['right_grasp_weight'][0] == 0.
        assert saved['right_grasp_weight'][-1] == 1.


def test_unsafe_complete_archive_rejected_before_warmup_or_replay(tmp_path, monkeypatch):
    task = annotation_task(tmp_path)
    trajectory = retarget_task(task, tmp_path / 'safe')
    with np.load(trajectory, allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files}
    arrays['q_target'][:, 0] = 100.
    unsafe = tmp_path / 'unsafe.npz'
    np.savez_compressed(unsafe, **arrays)
    from adaptive_control_project.retarget_pipeline.scene import PickupScene

    def forbidden_step(*args, **kwargs):
        pytest.fail('An unsafe command reached the physics stepping method')

    monkeypatch.setattr(PickupScene, 'step', forbidden_step)
    out = tmp_path / 'rejected'
    with pytest.raises(ValueError, match='command trajectory fails robot constraints'):
        replay_task(task, unsafe, out, cameras=[], no_video=True)
    report = json.loads((out / 'rejected_commands.json').read_text())
    assert report['position_passed'] is False
    assert report['max_position_violation_rad'] > 90


def test_measured_motor_failure_is_written_as_failure_and_raised(tmp_path, monkeypatch):
    task = annotation_task(tmp_path)
    out = tmp_path / 'run'
    trajectory = retarget_task(task, out)
    from adaptive_control_project.retarget_pipeline.scene import PickupScene

    monkeypatch.setattr(PickupScene, 'controller_efforts', lambda scene: np.full(scene.num_dofs, 1e6))
    with pytest.raises(ValueError, match='measured replay violates robot constraints'):
        replay_task(task, trajectory, out, cameras=[], no_video=True)
    metrics = json.loads((out / 'metrics.json').read_text())
    assert metrics['command_constraints']['passed'] is True
    assert metrics['measured_constraints']['effort_passed'] is False
    assert metrics['constraints_passed'] is False
    assert metrics['replay']['constraints_passed'] is False
    assert metrics['success'] is False
    assert metrics['stage'] == 'replay'


def test_explicit_scene_registration_preserves_raw_derived_hand_pose(tmp_path):
    task = annotation_task(tmp_path)
    before = (task.root / 'input.npz').read_bytes()
    base = retarget_task(task, tmp_path / 'base')
    transform = np.eye(4)
    transform[:3, 3] = [.05, -.02, .01]
    calibrated = retarget_task(task, tmp_path / 'calibrated', robot_from_source=transform)
    with np.load(base) as first, np.load(calibrated) as second:
        # Rigid registration changes wrist/arm placement; the finger shape
        # remains the annotation optimizer result, never a support posture.
        np.testing.assert_allclose(first['right_hand_q'], second['right_hand_q'], atol=1e-7)
        np.testing.assert_allclose(second['robot_from_source'], transform)
        np.testing.assert_allclose(second['right_wrist_T'][:, :3, 3] - first['right_wrist_T'][:, :3, 3],
                                   np.broadcast_to(transform[:3, 3], (8, 3)), atol=1e-12)
    assert (task.root / 'input.npz').read_bytes() == before


def test_annotation_replay_cannot_mislabel_a_procedural_plan(tmp_path):
    task = annotation_task(tmp_path)
    path = tmp_path / 'procedural.npz'
    np.savez(path, task=task.id)
    with pytest.raises(ValueError, match='annotation-generated'):
        replay_task(task, path, tmp_path / 'run', cameras=[], no_video=True, annotation_only=True)


def test_imported_annotation_replay_saves_named_targets_and_provenance(tmp_path, monkeypatch):
    from adaptive_control_project.retarget_pipeline.scene import PickupScene

    task = annotation_task(tmp_path)
    source = retarget_task(task, tmp_path / 'source')
    out = tmp_path / 'imported'
    out.mkdir()
    (out / 'metrics.json').write_text(json.dumps({'success': True, 'stale': 'unrelated run'}))
    monkeypatch.setattr(PickupScene, 'controller_efforts', lambda scene: np.full(scene.num_dofs, 1e6))
    with pytest.raises(ValueError, match='measured replay'):
        replay_task(task, source, out, cameras=[], no_video=True, annotation_only=True)
    assert (out / 'trajectory.npz').read_bytes() == source.read_bytes()
    metrics = json.loads((out / 'metrics.json').read_text())
    assert 'stale' not in metrics and metrics['motion_source'] == 'annotations'
    assert metrics['source_sequences']['right']['sha256'] == hashlib.sha256((task.root / 'input.npz').read_bytes()).hexdigest()
    assert json.loads((out / 'run_meta.json').read_text())['robot_from_source'] == np.eye(4).tolist()
    with np.load(out / 'trajectory.npz', allow_pickle=False) as archive:
        assert archive['q_target'].shape[1] == 54


def test_annotation_replay_rejects_fps_that_would_speed_up_playback(tmp_path):
    task = annotation_task(tmp_path)
    source = retarget_task(task, tmp_path / 'source')
    with pytest.raises(ValueError, match='physics sample rate'):
        replay_task(task, source, tmp_path / 'run', cameras=[], no_video=True, annotation_only=True, video_fps=201.)
