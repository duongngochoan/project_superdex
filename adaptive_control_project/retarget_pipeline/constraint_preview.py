"""Robot skeleton previews reconstructed from the final constrained command stream.

The loaded SuperDex prefab supplies both arm and finger geometry, including the
left hand. No raw human keypoints or unsmoothed retargeting samples are drawn.
The preview certifies commanded position/rate feasibility, not physical tracking.
"""
from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

import numpy as np

from .arm_ik import ArmChain
from .constraints import audit_trajectory, limits_from_prefab
from .tasks import Task, record_video, robot_video_path

FINGERS = ("thumb", "index_finger", "middle_finger", "ring_finger", "pinky")
FINGER_COLORS = ("#b55aa6", "#2574b6", "#279579", "#d39b26", "#d36242")


def preview_samples(timestamps, q_target, fps):
    """Uniform video sampling on the original controller clock, at normal speed."""
    times, q = np.asarray(timestamps, dtype=float), np.asarray(q_target, dtype=float)
    if times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Preview requires finite, increasing timestamps with at least two samples")
    if q.ndim != 2 or len(q) != len(times) or not np.isfinite(q).all():
        raise ValueError("Preview requires finite joint targets matching timestamps")
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("Preview FPS must be finite and positive")
    count = int(np.floor((times[-1] - times[0]) * fps + 1e-8)) + 1
    output_times = times[0] + np.arange(count) / fps
    samples = np.column_stack([np.interp(output_times, times, column) for column in q.T])
    return output_times, samples


class RobotSkeleton:
    """Exact prefab FK for arm joint origins and each finger's joints/tip."""

    def __init__(self, prefab, sdp, hands):
        self.arms, self.fingers = {}, {}
        names = [link.name for link in prefab.links]
        for side in hands:
            wrist = f"{side}_link_01_wrist"
            self.arms[side] = ArmChain(prefab, "world", wrist, sdp)
            self.fingers[side] = {}
            for finger in FINGERS:
                tips = [name for name in names if re.fullmatch(rf"{side}_link_\d+_{finger}_tip", name)]
                if len(tips) != 1:
                    raise ValueError(f"Expected one exact prefab tip for {side} {finger}")
                self.fingers[side][finger] = ArmChain(prefab, wrist, tips[0], sdp)

    def points(self, samples):
        result = {}
        for side, arm in self.arms.items():
            arms, world_hands, local_hands, wrists = [], [], [], []
            for q in samples:
                wrist, _, origins = arm.fk(q[arm.dof_indices])
                arms.append(np.vstack([origins, wrist[:3, 3]]))
                wrists.append(wrist)
                local = []
                for finger in FINGERS:
                    chain = self.fingers[side][finger]
                    tip, _, finger_origins = chain.fk(q[chain.dof_indices])
                    # Include the wrist, all four actual joint origins, and tip.
                    local.append(np.vstack([np.zeros(3), finger_origins, tip[:3, 3]]))
                local = np.asarray(local)
                local_hands.append(local)
                world_hands.append(local @ wrist[:3, :3].T + wrist[:3, 3])
            result[f"{side}_arm_world"] = np.asarray(arms)
            result[f"{side}_hand_world"] = np.asarray(world_hands)
            result[f"{side}_hand_wrist"] = np.asarray(local_hands)
            result[f"{side}_wrist_world"] = np.asarray(wrists)
        return result


def _prepare_axis(axis, points, minimum_radius):
    points = points.reshape(-1, 3)
    lower, upper = points.min(axis=0), points.max(axis=0)
    center = (lower + upper) / 2
    radius = max(float((upper - lower).max()) * .60, minimum_radius)
    for setter, value in zip((axis.set_xlim, axis.set_ylim, axis.set_zlim), center):
        setter(value - radius, value + radius)
    axis.set_box_aspect((1, 1, 1))
    axis.view_init(elev=25, azim=-55)
    axis.set_xlabel("x [m]", fontsize=7, labelpad=-2)
    axis.set_ylabel("y [m]", fontsize=7, labelpad=-2)
    axis.set_zlabel("z [m]", fontsize=7, labelpad=-2)
    axis.tick_params(labelsize=6, pad=-1)


def _render(task, run, mode, times, q, points, hands, fps, width, height, provenance):
    import imageio
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    hand_only = mode == "hand"
    suffix = "hand_constraints" if hand_only else "arm_hand_constraints"
    output = robot_video_path(task, f"{run}_{suffix}", "external")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure = plt.figure(figsize=(width / 100, height / 100), dpi=100)
    figure.patch.set_facecolor("#f7f9fb")
    figure.text(.5, .965, "CONSTRAINED HAND KEYPOINTS" if hand_only else "CONSTRAINED ARM + HAND SKELETON",
                ha="center", va="top", fontsize=11, weight="bold", color="#172b3d")
    figure.text(.5, .915, "COMMAND FK | position, velocity and acceleration audit: PASS",
                ha="center", va="top", fontsize=8, color="#21704a")
    figure.text(.5, .87, "Exact robot joint origins and fingertips; physical tracking is measured separately",
                ha="center", va="top", fontsize=7, color="#425466")
    caption = figure.text(.5, .035, "", ha="center", va="bottom", fontsize=8)
    hand_lines, arm_lines = {}, {}
    if hand_only:
        axes = {side: figure.add_axes([.01 + .98 * i / len(hands), .14, .98 / len(hands), .67], projection="3d")
                for i, side in enumerate(hands)}
        for side, axis in axes.items():
            _prepare_axis(axis, points[f"{side}_hand_wrist"], .09)
            axis.set_title(f"{side} hand | wrist frame", fontsize=9, pad=-2)
    else:
        axis = figure.add_axes([.02, .12, .96, .70], projection="3d")
        _prepare_axis(axis, np.concatenate([np.concatenate([
            points[f"{side}_arm_world"].reshape(-1, 3), points[f"{side}_hand_world"].reshape(-1, 3)])
            for side in hands]), .3)
        axes = {side: axis for side in hands}
        for side in hands:
            arm_lines[side] = axis.plot([], [], [], "o-", linewidth=2.8, markersize=3,
                                        color="#255e90" if side == "right" else "#c27831", label=f"{side} arm")[0]
        axis.legend(loc="upper left", fontsize=7, frameon=False)
    for side, axis in axes.items():
        hand_lines[side] = [axis.plot([], [], [], "o-", color=color, linewidth=1.9, markersize=3)[0]
                            for color in FINGER_COLORS]
    try:
        with imageio.get_writer(str(output), fps=fps, codec="libx264", quality=8, macro_block_size=1) as writer:
            for index, timestamp in enumerate(times):
                for side in hands:
                    hand = points[f"{side}_hand_{'wrist' if hand_only else 'world'}"][index]
                    for line, finger in zip(hand_lines[side], hand):
                        line.set_data_3d(finger[:, 0], finger[:, 1], finger[:, 2])
                    if not hand_only:
                        arm = points[f"{side}_arm_world"][index]
                        arm_lines[side].set_data_3d(arm[:, 0], arm[:, 1], arm[:, 2])
                caption.set_text(f"{task.id} | {run} | controller t={timestamp:.3f}s")
                figure.canvas.draw()
                writer.append_data(np.asarray(figure.canvas.buffer_rgba())[..., :3])
    finally:
        plt.close(figure)
    sidecar = output.with_suffix(".keypoints.npz")
    np.savez_compressed(sidecar, timestamps=times, video_timestamps=np.arange(len(times)) / fps,
                        q_target=q, dof_names=np.asarray(provenance["dof_names"]),
                        hands=np.asarray(hands), fingers=np.asarray(FINGERS),
                        provenance=json.dumps(provenance), **points)
    report = output.with_suffix(".json")
    report.write_text(json.dumps(provenance | {"video": str(output.relative_to(task.root)),
        "sidecar": str(sidecar.relative_to(task.root)), "frames": len(times), "fps": fps,
        "view": "wrist_frame_hand_closeup" if hand_only else "world_frame_arm_and_hand"}, indent=2) + "\n")
    record_video(task, "robot", "external", output, kind="constrained_keypoint_preview", run=run,
                 fps=fps, frames=len(times), keypoints=str(sidecar.relative_to(task.root)),
                 report=str(report.relative_to(task.root)), constraint_passed=True,
                 skeleton="hand" if hand_only else "arm_and_hand", state="commanded",
                 label="Exact robot FK of constrained q_target; not source annotations or measured tracking")
    return output


def render_constraint_previews(task: Task, trajectory: Path, *, run: str | None = None,
                               fps=15., width=640, height=480, modes=("hand", "arm_hand")):
    """Audit the full command trajectory before deriving either robot preview."""
    import superdex.physics as sdp
    import superdex.robotics as sdr
    from superdex.physics.paths import resolve_asset
    from .paths import BOT_ASSET

    if width < 160 or height < 160 or width % 2 or height % 2:
        raise ValueError("Video dimensions must be even integers >= 160")
    if not modes or not set(modes) <= {"hand", "arm_hand"} or len(set(modes)) != len(modes):
        raise ValueError("Select distinct hand and/or arm_hand preview modes")
    trajectory = Path(trajectory).resolve()
    run = run or trajectory.parent.name
    if Path(run).name != run or run in (".", ".."):
        raise ValueError("Run must be a single directory name")
    with np.load(trajectory, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    if str(data["task"]) != task.id or set(data["hands"].tolist()) != set(task.hands):
        raise ValueError("Preview trajectory task/hands do not match the task manifest")
    if not bool(data.get("includes_replay_timeline", False)):
        raise ValueError("Preview requires the complete prepared replay command timeline")
    times, samples = preview_samples(data["timestamps"], data["q_target"], fps)
    policy = {"arm_velocity": 1., "hand_velocity": 1.5, "arm_acceleration": 2., "hand_acceleration": 4.}
    policy.update(task.manifest.get("constraint_policy", {}))
    requested = json.loads(str(data.get("constraint_policy", "{}")))
    if set(requested) - set(policy) or any(requested[key] > policy[key] for key in requested):
        raise ValueError("Trajectory cannot relax task operating caps")
    policy.update(requested)
    sdp.initialize(num_worker_threads=0)
    try:
        prefab = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
        limits = limits_from_prefab(prefab, sdp, **policy)
        if not np.array_equal(data["dof_names"], limits.names):
            raise ValueError("Preview trajectory DOF order does not match the loaded robot")
        report = audit_trajectory(data["q_target"], data["timestamps"], limits, assume_rest_at_ends=True)
        if not report["passed"]:
            raise ValueError("Cannot publish a constrained preview: command constraint audit failed")
        points = RobotSkeleton(prefab, sdp, task.hands).points(samples)
    finally:
        sdp.shutdown()
    provenance = {"task": task.id, "run": run, "source_trajectory": str(trajectory),
                  "trajectory_sha256": hashlib.sha256(trajectory.read_bytes()).hexdigest(),
                  "state": "commanded", "source_field": "q_target", "robot_asset": BOT_ASSET,
                  "raw_keypoints_used_for_drawing": False, "constraint_policy": policy,
                  "command_constraints": report, "joint_limits": limits.to_json(),
                  "dof_names": list(limits.names), "playback": 1.,
                  "audit_scope": "Full controller timeline position/velocity/acceleration. No measured-state, effort or contact claim."}
    if "motion_source" in data:
        provenance["motion_source"] = str(data["motion_source"])
    if "source_sequences" in data:
        provenance["source_sequences"] = json.loads(str(data["source_sequences"]))
    if "robot_from_source" in data:
        provenance["robot_from_source"] = data["robot_from_source"].tolist()
    return [_render(task, run, mode, times, samples, points, task.hands, fps, width, height, provenance)
            for mode in modes]
