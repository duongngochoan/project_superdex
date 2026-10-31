"""Boundary checks before data reaches native robot controllers."""
import json

import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.tasks import Task, list_tasks, load_task, robot_video_path, validate_sequence


def sequence(side="right"):
    return dict(keypoints=np.zeros((3, 21, 3)), timestamps=np.arange(3) / 30, fps=30,
                hand=side, frame="world", up_axis="z")


@pytest.mark.parametrize("change", [dict(timestamps=[0, 0, 1]), dict(fps=0), dict(frame="camera"),
                                     dict(keypoints=np.full((3, 21, 3), np.nan))])
def test_invalid_input_is_rejected_before_control(change):
    with pytest.raises(ValueError):
        validate_sequence(sequence() | change, "right")


def test_bimanual_timestamp_mismatch_rejected(tmp_path):
    for side in ("left", "right"):
        data = sequence(side)
        if side == "left":
            data["timestamps"] = data["timestamps"] + .01
        np.savez(tmp_path / f"{side}.npz", **data)
    task = Task(tmp_path, {"id": "test", "hands": ["left", "right"],
                           "sequences": {side: f"{side}.npz" for side in ("left", "right")}})
    with pytest.raises(ValueError, match="same timestamps"):
        task.load_sequences()


def test_manifest_cannot_escape_task_root(tmp_path):
    task = Task(tmp_path, {"id": "test"})
    with pytest.raises(ValueError, match="escapes"):
        task.path("../unrelated.npz")
    with pytest.raises(ValueError):
        load_task("../other", tmp_path)


def test_cup_is_the_only_project_task():
    assert [task.id for task in list_tasks()] == ["pickup_the_cup"]


def test_cup_manifest_has_valid_canonical_input():
    task = load_task("pickup_the_cup")
    # Dataset archives may be absent in fresh checkouts.
    if not all(task.path(p).exists() for p in task.manifest["sequences"].values()):
        pytest.skip("Cup annotation data is not present in this checkout")
    assert set(task.load_sequences()) == set(task.hands)


def test_robot_views_share_external_folder_with_distinct_filenames(tmp_path):
    task = Task(tmp_path, {"id": "test", "robot_video_layout": "external_folder"})
    paths = [robot_video_path(task, "pickup_the_cup", camera)
             for camera in ("external", "head", "wrist_right")]
    assert {path.parent for path in paths} == {tmp_path / "videos/robot/external"}
    assert {path.name for path in paths} == {
        "pickup_the_cup.mp4", "pickup_the_cup_head.mp4", "pickup_the_cup_wrist_right.mp4"
    }
    assert len(set(paths)) == 3


def test_robot_video_path_preserves_per_camera_layout_without_manifest_setting(tmp_path):
    task = Task(tmp_path, {"id": "test"})
    assert robot_video_path(task, "clip", "head") == tmp_path / "videos/robot/head/clip.mp4"


@pytest.mark.parametrize("run,camera", [("../escape", "head"), ("", "head"), ("clip", "unknown")])
def test_invalid_robot_video_paths_are_rejected(tmp_path, run, camera):
    with pytest.raises(ValueError):
        robot_video_path(Task(tmp_path, {"id": "test"}), run, camera)


def test_simultaneous_video_registration_preserves_every_clip(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    from adaptive_control_project.retarget_pipeline.tasks import record_video

    task = Task(tmp_path, {"id": "test"})
    directory = tmp_path / "videos/robot/head"
    directory.mkdir(parents=True)
    for i in range(16):
        (directory / f"clip_{i}.mp4").touch()
    with ThreadPoolExecutor(max_workers=4) as workers:
        list(workers.map(lambda i: record_video(task, "robot", "head", directory / f"clip_{i}.mp4",
                                                kind="simulation_render"), range(16)))
    index = json.loads((tmp_path / "videos/index.json").read_text())
    assert {clip["path"] for clip in index["clips"]} == {f"videos/robot/head/clip_{i}.mp4" for i in range(16)}
