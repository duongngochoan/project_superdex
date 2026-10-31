"""Capture cleanup protects complete clips if another encoder fails."""
from unittest.mock import Mock
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import trimesh

from adaptive_control_project.retarget_pipeline.cameras import CameraRecorder
from adaptive_control_project.retarget_pipeline.tasks import Task


def test_invalid_camera_rejected_before_allocating_renderer():
    with pytest.raises(ValueError, match="supported"):
        CameraRecorder(None, "test", None, ["unknown"], 320, 240)


def test_close_cleans_all_encoders_and_renderer_on_failure():
    recorder = object.__new__(CameraRecorder)
    recorder._closed = False
    recorder.times = []
    failing, healthy = Mock(), Mock()
    failing.close.side_effect = RuntimeError("encoder failed")
    recorder.writers = {"head": failing, "external": healthy}
    recorder.renderer = Mock()
    with pytest.raises(RuntimeError, match="encoder failed"):
        recorder.close(register=False)
    healthy.close.assert_called_once()
    recorder.renderer.close.assert_called_once()
    recorder.close()
    healthy.close.assert_called_once()


def test_failed_capture_cannot_register_mismatched_video_sidecars(monkeypatch):
    recorder = object.__new__(CameraRecorder)
    recorder._closed = False
    recorder._capture_failed = False
    recorder._capture = Mock(side_effect=RuntimeError("render failed after first camera"))
    recorder.times = [0.]
    recorder.writers = {"external": Mock()}
    recorder.paths = {"external": Path("/must-not-publish.mp4")}
    recorder.renderer = Mock()
    monkeypatch.setattr("adaptive_control_project.retarget_pipeline.cameras.record_video",
                        Mock(side_effect=AssertionError("Incomplete capture was registered")))
    with pytest.raises(RuntimeError, match="render failed"):
        recorder.capture(0., [], None, "test")
    recorder.close()
    recorder.writers["external"].close.assert_called_once()
    recorder.renderer.close.assert_called_once()


def test_flat_camera_outputs_and_overlays_use_separate_encoders(monkeypatch, tmp_path):
    task = Task(tmp_path, {"id": "test", "hands": ["right"], "robot_video_layout": "external_folder"})
    scene = SimpleNamespace(
        prefab=SimpleNamespace(links=[]), table_box=([1, 1, .1], [0, 0, 0]), static_props=[],
        cfg=SimpleNamespace(object_kind="mesh", object_mesh="unused.stl", object_color=[255, 255, 255, 255]),
        link_names=["right_link_01_wrist"],
        link_transforms=Mock(return_value=[(np.zeros(3), [0, 0, 0, 1])]), link_index=Mock(return_value=0))
    renderer = Mock()
    monkeypatch.setattr("adaptive_control_project.retarget_pipeline.cameras.rd.SceneRenderer", Mock(return_value=renderer))
    monkeypatch.setattr("adaptive_control_project.retarget_pipeline.cameras.rd.box_mesh", Mock())
    monkeypatch.setattr(trimesh, "load", Mock(return_value=trimesh.creation.box()))
    writer = Mock(side_effect=lambda *args, **kwargs: Mock())
    monkeypatch.setattr("imageio.get_writer", writer)
    recorder = CameraRecorder(task, "pickup_the_cup", scene, ["external", "head", "wrist_right"],
                              320, 240, tactile=True, human_keypoints=True)
    try:
        paths = [Path(call.args[0]) for call in writer.call_args_list]
        assert len(paths) == len(set(paths)) == 9
        assert {path.parent for path in paths} == {tmp_path / "videos/robot/external"}
        for camera, path in recorder.paths.items():
            assert recorder.tactile_paths[camera].stem == f"{path.stem}_tactile"
            assert recorder.human_paths[camera].stem == f"{path.stem}_human_keypoints"
    finally:
        recorder.close(register=False)
