"""Video products distinguish dataset motion, commanded geometry and robot replay."""
import json

import pytest

from adaptive_control_project.retarget_pipeline.paths import CUP_TASK
from adaptive_control_project.retarget_pipeline.tasks import Task, record_video, robot_video_path, video_product


PRODUCTS = [
    ("raw", "keypoint_preview", None, "dataset_keypoints"),
    ("robot", "constrained_keypoint_preview", "hand", "constrained_hand_keypoints"),
    ("robot", "constrained_keypoint_preview", "arm_and_hand", "constrained_arm_hand_keypoints"),
    ("robot", "simulation_render", None, "robot_replay"),
    ("robot", "simulated_tactile_render", None, "robot_replay_tactile"),
    ("robot", "human_keypoint_render", None, "robot_replay_human_keypoints"),
    ("robot", "historical_simulation_render", None, "historical_robot_replay"),
]


@pytest.mark.parametrize("domain,kind,skeleton,product", PRODUCTS)
def test_registration_preserves_distinct_video_products(tmp_path, domain, kind, skeleton, product):
    assert video_product(domain, kind, skeleton=skeleton) == product
    task = Task(tmp_path, {"id": "test"})
    path = tmp_path / "videos" / domain / "external" / "clip.mp4"
    path.parent.mkdir(parents=True)
    path.touch()
    metadata = {"skeleton": skeleton} if skeleton is not None else {}
    record_video(task, domain, "external", path, kind=kind, **metadata)
    clip = json.loads((tmp_path / "videos/index.json").read_text())["clips"][0]
    assert clip["product"] == product
    assert clip["kind"] == kind
    assert clip["path"] == f"videos/{domain}/external/clip.mp4"


@pytest.mark.parametrize("domain,kind,skeleton,camera", [
    ("raw", "simulation_render", None, "external"),
    ("raw", "constrained_keypoint_preview", "hand", "external"),
    ("robot", "keypoint_preview", None, "external"),
    ("robot", "constrained_keypoint_preview", None, "external"),
    ("robot", "constrained_keypoint_preview", "arm", "external"),
    ("other", "simulation_render", None, "external"),
    ("robot", "simulation_render", None, "unsupported"),
])
def test_invalid_registration_cannot_create_an_index(tmp_path, domain, kind, skeleton, camera):
    task = Task(tmp_path, {"id": "test"})
    with pytest.raises(ValueError):
        record_video(task, domain, camera, tmp_path / "clip.mp4", kind=kind, skeleton=skeleton)
    assert not (tmp_path / "videos/index.json").exists()


def test_custom_synthetic_input_cannot_be_labelled_as_dataset_motion(tmp_path):
    task = Task(tmp_path, {"id": "test"})
    path = tmp_path / "videos/raw/external/preview.mp4"
    path.parent.mkdir(parents=True)
    path.touch()
    record_video(task, "raw", "external", path,
                 kind="keypoint_preview", synthetic=True)
    clip = json.loads((tmp_path / "videos/index.json").read_text())["clips"][0]
    assert clip["product"] == "synthetic_keypoints"


def test_missing_clip_or_sidecar_cannot_create_a_catalog_entry(tmp_path):
    task = Task(tmp_path, {"id": "test"})
    path = tmp_path / "videos/robot/external/clip.mp4"
    path.parent.mkdir(parents=True)
    with pytest.raises(ValueError, match="missing video"):
        record_video(task, "robot", "external", path, kind="simulation_render")
    path.touch()
    with pytest.raises(ValueError, match="missing camera_poses"):
        record_video(task, "robot", "external", path, kind="simulation_render",
                     camera_poses="videos/robot/external/clip.camera.npz")
    assert not (tmp_path / "videos/index.json").exists()


def test_video_cannot_be_registered_under_a_different_camera(tmp_path):
    task = Task(tmp_path, {"id": "test"})
    path = tmp_path / "videos/robot/head/clip.mp4"
    path.parent.mkdir(parents=True)
    path.touch()
    with pytest.raises(ValueError, match="selected task video directory"):
        record_video(task, "robot", "external", path, kind="simulation_render")
    assert not (tmp_path / "videos/index.json").exists()


def test_flat_folder_keeps_actual_camera_identity_in_index(tmp_path):
    task = Task(tmp_path, {"id": "test", "robot_video_layout": "external_folder"})
    expected = {}
    for camera in ("external", "head", "wrist_right"):
        path = robot_video_path(task, "pickup_the_cup", camera)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
        sidecar = path.with_suffix(".camera.npz")
        sidecar.touch()
        relative = str(path.relative_to(tmp_path))
        expected[camera] = relative
        record_video(task, "robot", camera, path, kind="simulation_render", run="pickup_the_cup",
                     camera_poses=str(sidecar.relative_to(tmp_path)))
    clips = json.loads((tmp_path / "videos/index.json").read_text())["clips"]
    assert {clip["camera"]: clip["path"] for clip in clips} == expected
    assert all(clip["product"] == "robot_replay" for clip in clips)


def test_cup_index_matches_existing_clips_and_sidecars():
    index = json.loads((CUP_TASK / "videos/index.json").read_text())
    indexed_paths = []
    for clip in index["clips"]:
        expected = video_product(clip["domain"], clip["kind"], skeleton=clip.get("skeleton"),
                                 synthetic=clip.get("synthetic", False))
        assert clip["product"] == expected
        indexed_paths.append(clip["path"])
        for field in ("path", "timestamps", "camera_poses", "keypoints", "report", "tactile_signals", "source_video"):
            if field in clip:
                path = (CUP_TASK / clip[field]).resolve()
                assert path.is_relative_to(CUP_TASK.resolve())
                assert path.is_file(), (field, clip[field])
    assert len(indexed_paths) == len(set(indexed_paths))
    assert set(indexed_paths) == {
        str(path.relative_to(CUP_TASK)) for path in (CUP_TASK / "videos").rglob("*.mp4")
    }
    manifest = json.loads((CUP_TASK / "task.json").read_text())
    for cameras in manifest["cameras"].values():
        for camera in cameras.values():
            if "latest_clip" in camera:
                assert camera["latest_clip"] in indexed_paths
