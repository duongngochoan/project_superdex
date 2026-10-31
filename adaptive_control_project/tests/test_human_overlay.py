"""Source overlays follow retimed annotations while plain replays stay available."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.cameras import CameraRecorder
from adaptive_control_project.retarget_pipeline.pipeline import annotation_overlay, retarget_task
from adaptive_control_project.retarget_pipeline.tasks import Task
from adaptive_control_project.tests.motion_fixtures import generate_arrays


def test_retarget_saves_calibrated_source_geometry_on_prepared_clock(tmp_path):
    arrays = generate_arrays(["right"])
    points = arrays["keypoints_right"][:8]
    np.savez(tmp_path / "source.npz", keypoints=points, timestamps=arrays["timestamps"][:8],
             fps=30., hand="right", frame="world", up_axis="z")
    task = Task(tmp_path, {"id": "test", "hands": ["right"], "sequences": {"right": "source.npz"}})
    transform = np.eye(4)
    transform[:3, 3] = [.05, -.02, .01]
    path = retarget_task(task, tmp_path / "run", robot_from_source=transform)
    with np.load(path) as saved:
        np.testing.assert_allclose(saved["right_source_keypoints_world"], points + transform[:3, 3])
        sample = annotation_overlay(saved, ["right"])
        times = saved["source_replay_timestamps"]
        assert times[0] > 0 and times[-1] < saved["timestamps"][-1]
        np.testing.assert_allclose(np.diff(times), np.full(7, times[0] / 30.))
        np.testing.assert_allclose(sample(times[3])["right"], points[3] + transform[:3, 3])


def test_source_overlay_uses_replay_clock_and_holds_endpoint_poses():
    points = np.stack([np.zeros((21, 3)), np.ones((21, 3))])
    sample = annotation_overlay({"source_replay_timestamps": [2., 4.],
                                 "right_source_keypoints_world": points}, ["right"])
    np.testing.assert_array_equal(sample(0.)["right"], points[0])
    np.testing.assert_array_equal(sample(3.)["right"], np.full((21, 3), .5))
    np.testing.assert_array_equal(sample(6.)["right"], points[-1])


@pytest.mark.parametrize("data", [{}, {"source_replay_timestamps": [1., 1.]},
    {"source_replay_timestamps": [1., 2.]},
    {"source_replay_timestamps": [1., 2.], "right_source_keypoints_world": np.zeros((2, 20, 3))}])
def test_missing_or_invalid_source_overlay_is_rejected(data):
    with pytest.raises(ValueError, match="[Hh]uman"):
        annotation_overlay(data, ["right"])


def test_camera_writes_plain_and_overlay_frames_with_matching_geometry(monkeypatch, tmp_path):
    monkeypatch.setattr("adaptive_control_project.retarget_pipeline.cameras.rd.annotate", lambda frame, _: frame)
    recorder = object.__new__(CameraRecorder)
    recorder.task = Task(tmp_path, {"id": "test", "hands": ["right"]})
    recorder.run, recorder.fps = "test", 15.
    recorder._closed = recorder._capture_failed = False
    recorder.scene = SimpleNamespace(link_names=[])
    recorder.times, recorder.human_values = [], []
    recorder.human_keypoints, recorder.tactile = True, False
    recorder.writers, recorder.human_writers = {"external": Mock()}, {"external": Mock()}
    recorder.mounts, recorder.fixed, recorder.poses = {}, {"external": np.eye(4)}, {"external": []}
    directory = tmp_path / "videos/robot/external"
    directory.mkdir(parents=True)
    plain_path, overlay_path = directory / "test.mp4", directory / "test_human_keypoints.mp4"
    plain_path.touch()
    overlay_path.touch()
    recorder.paths, recorder.human_paths = {"external": plain_path}, {"external": overlay_path}
    recorder.renderer = Mock(nodes={})
    plain, overlay = np.zeros((2, 2, 3), dtype=np.uint8), np.ones((2, 2, 3), dtype=np.uint8)
    recorder.renderer.render.side_effect = [plain, overlay]
    points = np.arange(63).reshape(21, 3)
    recorder.capture(3., [], (np.zeros(3), [0., 0., 0., 1.]), "test", human_keypoints={"right": points})
    np.testing.assert_array_equal(recorder.writers["external"].append_data.call_args.args[0], plain)
    np.testing.assert_array_equal(recorder.human_writers["external"].append_data.call_args.args[0], overlay)
    np.testing.assert_array_equal(recorder.renderer.add_points.call_args.args[1], points)
    recorder.renderer.remove.assert_called_once_with("source_keypoints")
    recorder.close()
    with np.load(overlay_path.with_suffix(".keypoints.npz")) as saved:
        np.testing.assert_array_equal(saved["timestamps"], [3.])
        np.testing.assert_array_equal(saved["right_keypoints_world"], points[None])
        np.testing.assert_array_equal(saved["world_from_camera"], np.eye(4)[None])
