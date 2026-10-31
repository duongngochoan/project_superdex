"""Rebuild pickup_the_cup.mp4 from DexYCB sequence 20200709_151032."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from adaptive_control_project.retarget_pipeline.tasks import CAMERA_POSITIONS, CUP_TASK_ID, Task, load_task, robot_video_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["all", "convert", "retarget", "replay"], default="all",
                        help="all: raw labels -> canonical motion -> robot commands -> video")
    parser.add_argument("--run", default="pickup_the_cup", help="output name; default creates pickup_the_cup.mp4")
    parser.add_argument("--out", type=Path, help="advanced: override saved-run directory")
    parser.add_argument("--trajectory", type=Path, help="saved commands for --stage replay")
    parser.add_argument("--optimizer", choices=["vector", "adaptive"], help="default: task's configured hand objective")
    parser.add_argument("--robot-from-source", type=Path, help="override the task's source-to-robot transform JSON")
    parser.add_argument("--human-keypoints", action="store_true", help="also render a source-point overlay for comparison")
    parser.add_argument("--constraint-previews", action="store_true", help="also render the commanded hand/arm skeletons")
    parser.add_argument("--tactile", action="store_true", help="also render simulated hand-object contact forces")
    parser.add_argument("--no-video", action="store_true", help="simulate and write measurements without OpenGL")
    parser.add_argument("--cameras", nargs="+", choices=CAMERA_POSITIONS,
                        help="virtual views; default: task's default_cameras, or external")
    parser.add_argument("--camera-calibration", type=Path, help="advanced: task-local virtual wrist-camera JSON")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--video-fps", type=float, default=30.)
    args = parser.parse_args(argv)
    task = load_task(CUP_TASK_ID)
    cameras = args.cameras if args.cameras is not None else task.manifest.get("default_cameras", ["external"])
    if not isinstance(cameras, list) or not cameras or any(camera not in CAMERA_POSITIONS for camera in cameras):
        parser.error("default_cameras must contain supported camera positions")
    cameras = list(dict.fromkeys(cameras))
    if Path(args.run).name != args.run or args.run in (".", ".."):
        parser.error("--run must be a single directory name")
    if args.trajectory and args.stage != "replay":
        parser.error("--trajectory requires --stage replay")
    if args.robot_from_source and args.stage not in ("all", "retarget"):
        parser.error("--robot-from-source applies during retargeting; replay uses the saved transform")
    if args.no_video and (args.human_keypoints or args.constraint_previews or args.tactile):
        parser.error("video overlays and previews require video capture")
    if args.human_keypoints and args.stage not in ("all", "replay"):
        parser.error("--human-keypoints requires a replay stage")
    preview_modes = []
    if not args.constraint_previews and args.stage in ("all", "replay") and not args.no_video:
        preview_modes = task.manifest.get("default_constraint_preview_modes", [])
        if (not isinstance(preview_modes, list) or any(mode not in ("hand", "arm_hand") for mode in preview_modes)
                or len(set(preview_modes)) != len(preview_modes)):
            parser.error("default_constraint_preview_modes must contain distinct hand and/or arm_hand modes")
    if args.camera_calibration:
        try:
            calibration_path = task.path(str(args.camera_calibration.resolve()))
        except ValueError:
            parser.error("--camera-calibration must be inside the task directory")
        if not calibration_path.is_file():
            parser.error("--camera-calibration must name an existing file")
        relative = str(calibration_path.relative_to(task.root.resolve()))
        task = Task(task.root, {**task.manifest, "camera_calibration": relative})
    out = args.out or task.root / "retarget/runs" / args.run
    trajectory = args.trajectory or out / "trajectory.npz"
    if args.stage in ("all", "convert"):
        from adaptive_control_project.retarget_pipeline.data_convert import convert_task_sequence
        print(f"[1/3 convert] {convert_task_sequence(task)}", flush=True)
    if args.stage == "convert":
        return
    from adaptive_control_project.retarget_pipeline.pipeline import replay_task, retarget_task
    if args.stage in ("all", "retarget"):
        calibration = json.loads(args.robot_from_source.read_text()) if args.robot_from_source else None
        trajectory = retarget_task(task, out, args.optimizer, robot_from_source=calibration)
        print(f"[2/3 retarget] {trajectory}", flush=True)
    if args.constraint_previews or preview_modes:
        from adaptive_control_project.retarget_pipeline.constraint_preview import render_constraint_previews
        options = {"modes": preview_modes} if preview_modes else {}
        render_constraint_previews(task, trajectory, run=out.name, fps=args.video_fps,
                                   width=args.width, height=args.height, **options)
    if args.stage in ("all", "replay"):
        options = {"human_keypoints": True} if args.human_keypoints else {}
        failed = False
        try:
            result = replay_task(task, trajectory, out, cameras=cameras,
                no_video=args.no_video, width=args.width, height=args.height, annotation_only=True,
                video_fps=args.video_fps, tactile=args.tactile, **options)
        except ValueError as error:
            if "measured replay violates robot constraints" not in str(error):
                raise
            result = json.loads((out / "metrics.json").read_text())
            failed = True
        print(json.dumps({"constraints_passed": result["constraints_passed"],
                          "replay": result["replay"]}, indent=2))
        if not args.no_video:
            for camera in cameras:
                print(f"[3/3 video] {robot_video_path(task, out.name, camera)}", flush=True)
        if failed:
            raise SystemExit("Video/log saved as diagnostics; measured robot constraints failed. Inspect metrics.json.")


if __name__ == "__main__":
    main()
