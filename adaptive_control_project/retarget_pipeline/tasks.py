"""Task manifests and paths, independent of simulation and native controllers."""
from __future__ import annotations

import json
import fcntl
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .data_convert import load_sequence
from .paths import TASKS

CAMERA_POSITIONS = ("external", "head", "wrist_left", "wrist_right")
CUP_TASK_ID = "pickup_the_cup"


def video_product(domain: str, kind: str, *, skeleton: str | None = None, synthetic: bool = False) -> str:
    """Identify the source preview or robot evaluation product without changing clip kinds."""
    if domain == "raw" and kind == "keypoint_preview":
        return "synthetic_keypoints" if synthetic else "dataset_keypoints"
    if domain == "robot":
        products = {
            "simulation_render": "robot_replay",
            "simulated_tactile_render": "robot_replay_tactile",
            "human_keypoint_render": "robot_replay_human_keypoints",
            "historical_simulation_render": "historical_robot_replay",
        }
        if kind in products:
            return products[kind]
        if kind == "constrained_keypoint_preview":
            if skeleton == "hand":
                return "constrained_hand_keypoints"
            if skeleton == "arm_and_hand":
                return "constrained_arm_hand_keypoints"
    raise ValueError(f"Unsupported video domain/kind/skeleton: {domain}/{kind}/{skeleton}")


@dataclass(frozen=True)
class Task:
    root: Path
    manifest: dict

    @property
    def id(self) -> str:
        return self.manifest["id"]

    @property
    def hands(self) -> list[str]:
        return self.manifest["hands"]

    def path(self, relative: str) -> Path:
        path = (self.root / relative).resolve()
        if not path.is_relative_to(self.root.resolve()):
            raise ValueError(f"Task path escapes {self.id}: {relative}")
        return path

    def load_sequences(self) -> dict[str, dict]:
        sequences = {}
        for side, relative in self.manifest["sequences"].items():
            seq = load_sequence(self.path(relative))
            validate_sequence(seq, side)
            sequences[side] = seq
        if set(sequences) != set(self.hands):
            raise ValueError("Task sequences must provide exactly the declared hands")
        first = next(iter(sequences.values()))
        for seq in sequences.values():
            if seq["timestamps"].shape != first["timestamps"].shape or not np.allclose(
                seq["timestamps"], first["timestamps"], atol=1e-8, rtol=0
            ) or not np.isclose(seq["fps"], first["fps"]):
                raise ValueError("Both hands must use the same timestamps and fps")
        return sequences


def validate_sequence(seq: dict, side: str) -> None:
    kp = np.asarray(seq["keypoints"])
    ts = np.asarray(seq["timestamps"])
    fps = float(seq["fps"])
    if kp.ndim != 3 or kp.shape[1:] != (21, 3) or len(kp) < 2 or not np.isfinite(kp).all():
        raise ValueError("keypoints must be finite (T,21,3), with T >= 2")
    if ts.shape != (len(kp),) or not np.isfinite(ts).all() or np.any(np.diff(ts) <= 0):
        raise ValueError("timestamps must be finite, strictly increasing and match keypoints")
    if not np.isfinite(fps) or fps <= 0 or not np.allclose(np.diff(ts), 1 / fps, atol=1e-5):
        raise ValueError("fps must be positive and match the uniform timestamp spacing")
    if seq["hand"] != side or seq.get("frame") != "world" or seq.get("up_axis") != "z":
        raise ValueError("Expected the declared hand in a calibrated Z-up world frame")


def load_task(task_id: str, tasks_root: Path = TASKS) -> Task:
    if not task_id or Path(task_id).name != task_id or task_id in (".", ".."):
        raise ValueError("Task id must be a single directory name")
    root = tasks_root / task_id
    with (root / "task.json").open() as f:
        manifest = json.load(f)
    if manifest.get("schema_version") != 1 or manifest.get("id") != task_id:
        raise ValueError(f"Invalid task manifest: {root / 'task.json'}")
    hands = manifest.get("hands", [])
    if not hands or len(set(hands)) != len(hands) or not set(hands) <= {"left", "right"}:
        raise ValueError("hands must contain left, right, or both once each")
    return Task(root, manifest)


def list_tasks(tasks_root: Path = TASKS) -> list[Task]:
    return [load_task(p.parent.name, tasks_root) for p in sorted(tasks_root.glob("*/task.json"))]


def video_directory(task: Task, domain: str, camera: str) -> Path:
    """Resolve the configured output folder without changing camera identity."""
    if domain not in ("raw", "robot"):
        raise ValueError(f"Unknown video domain: {domain}")
    if camera not in CAMERA_POSITIONS:
        raise ValueError(f"Unknown camera position: {camera}")
    if task.manifest.get("video_layout") == "flat":
        return task.path(f"videos/{domain}")
    if domain == "robot" and task.manifest.get("robot_video_layout") == "external_folder":
        return task.path("videos/robot/external")
    return task.path(f"videos/{domain}/{camera}")


def robot_video_path(task: Task, run: str, camera: str) -> Path:
    """Name robot views while keeping their camera identity in video metadata."""
    directory = video_directory(task, "robot", camera)
    if not run or Path(run).name != run or run in (".", ".."):
        raise ValueError("Video run must be a single directory name")
    if (task.manifest.get("video_layout") == "flat"
            or task.manifest.get("robot_video_layout") == "external_folder"):
        suffix = "" if camera == "external" else f"_{camera}"
        return task.path(str(directory / f"{run}{suffix}.mp4"))
    return task.path(str(directory / f"{run}.mp4"))


def record_video(task: Task, domain: str, camera: str, path: Path, *, kind: str, **metadata) -> None:
    """Register each clip without overwriting earlier runs or claiming sensor footage."""
    directory = video_directory(task, domain, camera)
    product = video_product(domain, kind, skeleton=metadata.get("skeleton"),
                            synthetic=metadata.get("synthetic", False))
    path = path.resolve()
    if path.parent != directory or path.suffix != ".mp4":
        raise ValueError("Video must be an MP4 inside the selected task video directory")
    if not path.is_file():
        raise ValueError(f"Cannot register a missing video: {path}")
    for field in ("timestamps", "camera_poses", "keypoints", "report", "tactile_signals", "source_video"):
        if field in metadata and not task.path(metadata[field]).is_file():
            raise ValueError(f"Cannot register a video with a missing {field}: {metadata[field]}")
    entry = {**metadata, "path": str(path.resolve().relative_to(task.root.resolve())),
             "kind": kind, "product": product}
    manifest_path = task.root / "videos" / "index.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    # Separate runs can finish different camera encoders at the same time.
    with (manifest_path.parent / ".index.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        index = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"schema_version": 1, "clips": []}
        index["clips"] = [clip for clip in index["clips"] if clip["path"] != entry["path"]]
        index["clips"].append({"domain": domain, "camera": camera, **entry})
        with tempfile.NamedTemporaryFile(mode="w", dir=manifest_path.parent, delete=False) as temporary:
            temporary.write(json.dumps(index, indent=2) + "\n")
            temporary_path = Path(temporary.name)
        temporary_path.replace(manifest_path)
