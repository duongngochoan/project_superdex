"""The default command rebuilds one sequence before producing its robot video."""
import json
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from adaptive_control_project.retarget_pipeline.tasks import Task
from adaptive_control_project.scripts import run_task


def stubs(monkeypatch, tmp_path):
    task = Task(tmp_path / 'pickup_the_cup', {'id': 'pickup_the_cup', 'hands': ['right']})
    monkeypatch.setattr(run_task, 'load_task', Mock(return_value=task))
    events = []
    canonical = task.root / 'data/processed/input.npz'
    trajectory = task.root / 'retarget/runs/pickup_the_cup/trajectory.npz'
    convert = Mock(side_effect=lambda *a, **k: events.append('convert') or canonical)
    retarget = Mock(side_effect=lambda *a, **k: events.append('retarget') or trajectory)
    replay = Mock(side_effect=lambda *a, **k: events.append('replay') or {'constraints_passed': True, 'replay': {}})
    preview = Mock(return_value=[])
    base = 'adaptive_control_project.retarget_pipeline.'
    monkeypatch.setitem(sys.modules, base + 'data_convert', SimpleNamespace(convert_task_sequence=convert))
    monkeypatch.setitem(sys.modules, base + 'pipeline', SimpleNamespace(retarget_task=retarget, replay_task=replay))
    monkeypatch.setitem(sys.modules, base + 'constraint_preview', SimpleNamespace(render_constraint_previews=preview))
    return SimpleNamespace(task=task, events=events, canonical=canonical, trajectory=trajectory,
                           convert=convert, retarget=retarget, replay=replay, preview=preview)


def test_default_rebuild_uses_fresh_raw_inputs_and_the_requested_filename(monkeypatch, tmp_path):
    s = stubs(monkeypatch, tmp_path)
    run_task.main([])
    assert s.events == ['convert', 'retarget', 'replay']
    assert s.replay.call_args.args[1] == s.trajectory
    assert s.replay.call_args.args[2].name == 'pickup_the_cup'
    assert s.replay.call_args.kwargs['cameras'] == ['external']
    assert s.replay.call_args.kwargs['annotation_only'] is True


def test_task_defaults_render_three_views_in_external_folder(monkeypatch, tmp_path, capsys):
    s = stubs(monkeypatch, tmp_path)
    s.task.manifest.update(robot_video_layout='external_folder',
                           default_cameras=['external', 'head', 'wrist_right'])
    run_task.main(['--stage', 'replay'])
    assert s.replay.call_args.kwargs['cameras'] == ['external', 'head', 'wrist_right']
    output = capsys.readouterr().out
    for name in ('pickup_the_cup.mp4', 'pickup_the_cup_head.mp4', 'pickup_the_cup_wrist_right.mp4'):
        assert str(s.task.root / 'videos/robot/external' / name) in output


def test_explicit_camera_selection_overrides_task_defaults(monkeypatch, tmp_path):
    s = stubs(monkeypatch, tmp_path)
    s.task.manifest['default_cameras'] = ['external', 'head', 'wrist_right']
    run_task.main(['--stage', 'replay', '--cameras', 'head'])
    assert s.replay.call_args.kwargs['cameras'] == ['head']


@pytest.mark.parametrize('stage', ['all', 'replay'])
def test_task_defaults_render_arm_hand_preview_during_video_stages(monkeypatch, tmp_path, stage):
    s = stubs(monkeypatch, tmp_path)
    s.task.manifest['default_constraint_preview_modes'] = ['arm_hand']
    run_task.main(['--stage', stage])
    s.preview.assert_called_once()
    assert s.preview.call_args.args[1] == s.trajectory
    assert s.preview.call_args.kwargs['modes'] == ['arm_hand']


@pytest.mark.parametrize('flags', [['--no-video'], ['--stage', 'replay', '--no-video'],
                                  ['--stage', 'convert'], ['--stage', 'retarget']])
def test_task_default_preview_is_suppressed_without_video_stage(monkeypatch, tmp_path, flags):
    s = stubs(monkeypatch, tmp_path)
    s.task.manifest['default_constraint_preview_modes'] = ['arm_hand']
    run_task.main(flags)
    s.preview.assert_not_called()


def test_explicit_constraint_previews_preserve_both_mode_function_default(monkeypatch, tmp_path):
    s = stubs(monkeypatch, tmp_path)
    s.task.manifest['default_constraint_preview_modes'] = ['arm_hand']
    run_task.main(['--constraint-previews'])
    s.preview.assert_called_once()
    assert 'modes' not in s.preview.call_args.kwargs


@pytest.mark.parametrize('cameras', [[], 'head', ['unknown']])
def test_invalid_default_cameras_fail_before_conversion(monkeypatch, tmp_path, cameras):
    s = stubs(monkeypatch, tmp_path)
    s.task.manifest['default_cameras'] = cameras
    with pytest.raises(SystemExit) as error:
        run_task.main([])
    assert error.value.code == 2
    assert not s.events


@pytest.mark.parametrize('stage,events', [('convert', ['convert']), ('retarget', ['retarget']), ('replay', ['replay'])])
def test_independent_stages_do_not_repeat_other_work(monkeypatch, tmp_path, stage, events):
    s = stubs(monkeypatch, tmp_path)
    run_task.main(['--stage', stage])
    assert s.events == events


def test_saved_replay_previews_and_overlays_use_one_trajectory(monkeypatch, tmp_path, capsys):
    s = stubs(monkeypatch, tmp_path)
    supplied = tmp_path / 'saved.npz'
    run_task.main(['--stage', 'replay', '--trajectory', str(supplied), '--human-keypoints',
                  '--constraint-previews', '--tactile', '--video-fps', '12',
                  '--cameras', 'external', 'head', 'external'])
    assert s.preview.call_args.args[1] == supplied
    assert s.replay.call_args.args[1] == supplied
    assert s.replay.call_args.kwargs['human_keypoints'] is True
    assert s.replay.call_args.kwargs['tactile'] is True
    assert s.replay.call_args.kwargs['cameras'] == ['external', 'head']
    assert s.replay.call_args.kwargs['video_fps'] == 12.
    output = capsys.readouterr().out
    assert str(s.task.root / 'videos/robot/external/pickup_the_cup.mp4') in output
    assert str(s.task.root / 'videos/robot/head/pickup_the_cup.mp4') in output


@pytest.mark.parametrize('flags', [['--no-video', '--human-keypoints'], ['--no-video', '--tactile'],
                                  ['--no-video', '--constraint-previews'], ['--stage', 'retarget', '--human-keypoints'],
                                  ['--trajectory', 'input.npz'], ['--run', '../other']])
def test_invalid_requests_are_rejected_before_conversion(monkeypatch, tmp_path, flags):
    s = stubs(monkeypatch, tmp_path)
    with pytest.raises(SystemExit) as error:
        run_task.main(flags)
    assert error.value.code == 2
    assert not s.events


@pytest.mark.parametrize('absolute', [False, True])
def test_camera_override_preserves_original_manifest(monkeypatch, tmp_path, absolute):
    s = stubs(monkeypatch, tmp_path)
    s.task.manifest['camera_calibration'] = 'assets/original.json'
    original = dict(s.task.manifest)
    path = s.task.root / 'assets/override.json'
    path.parent.mkdir(parents=True)
    path.write_text('{"wrist_from_camera": {}}')
    monkeypatch.chdir(tmp_path)
    argument = path if absolute else path.relative_to(tmp_path)
    run_task.main(['--stage', 'replay', '--camera-calibration', str(argument)])
    selected = s.replay.call_args.args[0]
    assert selected.manifest['camera_calibration'] == 'assets/override.json'
    assert s.task.manifest == original


@pytest.mark.parametrize('case', ['outside', 'symlink', 'missing'])
def test_camera_override_rejects_invalid_paths(monkeypatch, tmp_path, case):
    s = stubs(monkeypatch, tmp_path)
    outside = tmp_path / 'outside.json'
    outside.write_text('{}')
    candidate = outside
    if case != 'outside':
        candidate = s.task.root / 'assets/cameras.json'
        candidate.parent.mkdir(parents=True)
        if case == 'symlink': candidate.symlink_to(outside)
    with pytest.raises(SystemExit) as error:
        run_task.main(['--camera-calibration', str(candidate)])
    assert error.value.code == 2
    assert not s.events


def test_failed_measured_audit_stays_a_failure_and_reports_saved_diagnostics(monkeypatch, tmp_path, capsys):
    s = stubs(monkeypatch, tmp_path)
    out = s.trajectory.parent
    out.mkdir(parents=True)
    (out / 'metrics.json').write_text(json.dumps({'constraints_passed': False, 'replay': {}}))
    s.replay.side_effect = ValueError('measured replay violates robot constraints')
    with pytest.raises(SystemExit, match='saved as diagnostics'):
        run_task.main(['--stage', 'replay'])
    assert 'pickup_the_cup.mp4' in capsys.readouterr().out


def test_unsafe_commands_are_not_mistaken_for_completed_diagnostics(monkeypatch, tmp_path):
    s = stubs(monkeypatch, tmp_path)
    s.replay.side_effect = ValueError('command trajectory fails robot constraints before replay')
    with pytest.raises(ValueError, match='before replay'):
        run_task.main(['--stage', 'replay'])
