"""Bounded annotation retargeting and replay, including synchronized two-hand input."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np

from .tasks import Task


def _scene_spec(task: Task) -> dict:
    """Save the task-owned scene, or an explicit legacy paper-cup scene."""
    from .scene import SceneConfig

    relative = task.manifest.get("scene")
    if relative:
        path = task.path(relative)
    else:
        candidate = task.root / "assets/scene.json"
        path = candidate if candidate.is_file() else None
    if path is not None:
        spec = json.loads(path.read_text())
        if spec.get("object_mesh"):
            task.path(spec["object_mesh"])
    else:
        spec = asdict(SceneConfig(table_top=.24, cup_xy=(.42, -.25)))
    cfg = SceneConfig(**spec)
    if not np.isfinite(cfg.dt) or cfg.dt <= 0:
        raise ValueError("Scene control period must be positive and finite")
    return asdict(cfg)


def _timing(control: dict, dt: float, *, approach_time=1., hold_time=.5) -> tuple[float, float, float]:
    approach = float(control.get("approach_time", approach_time))
    hold = float(control.get("hold_time", hold_time))
    warmup = float(control.get("warmup_time_s", 1.))
    if not np.isfinite([approach, hold, warmup]).all() or approach <= 0 or hold < 0 or warmup < 0:
        raise ValueError("Approach must be positive; hold and warmup must be finite and nonnegative")
    return approach, max(hold, dt), warmup


def retarget_task(task: Task, out: Path, optimizer: str | None = None, *, robot_from_source=None) -> Path:
    import superdex.physics as sdp
    import superdex.robotics as sdr
    from superdex.physics.paths import resolve_asset

    from .arm_ik import ArmChain
    from .constraints import limits_from_prefab, prepare_trajectory
    from .hand_retarget import HandRetargeter
    from .paths import BOT_ASSET

    sequences = task.load_sequences()
    out.mkdir(parents=True, exist_ok=True)
    first = next(iter(sequences.values()))
    source_times = np.asarray(first["timestamps"], dtype=float)
    # One calibration for the whole scene preserves relative two-hand geometry.
    if robot_from_source is None and task.manifest.get("annotation_alignment"):
        robot_from_source = json.loads(task.path(task.manifest["annotation_alignment"]).read_text())
    robot_from_source = np.asarray(task.manifest.get("robot_from_source", np.eye(4))
                                  if robot_from_source is None else robot_from_source, dtype=float)
    if robot_from_source.shape != (4, 4) or not np.isfinite(robot_from_source).all() or not np.allclose(
        robot_from_source[3], [0, 0, 0, 1]
    ) or not np.allclose(robot_from_source[:3, :3].T @ robot_from_source[:3, :3], np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(robot_from_source[:3, :3]), 1
    ):
        raise ValueError("robot_from_source must be a rigid 4x4 transform")
    control = task.manifest.get("control", {})
    optimizer = optimizer or control.get("optimizer", "vector")
    scene_spec = _scene_spec(task)
    dt = float(scene_spec["dt"])
    approach, hold, warmup = _timing(control, dt)
    policy = task.manifest.get("constraint_policy", {})
    source_sequences = {side: {"path": path, "sha256": hashlib.sha256(task.path(path).read_bytes()).hexdigest(),
                               "source": sequences[side].get("source", "")}
                        for side, path in task.manifest["sequences"].items()}
    sdp.initialize(num_worker_threads=0)
    try:
        prefab = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
        rest = np.asarray(prefab.default_pose, dtype=float)
        source_targets = np.repeat(rest[None], len(source_times), axis=0)
        arrays = {"source_timestamps": source_times.copy(), "source_fps": float(first["fps"]),
                  "hands": np.asarray(task.hands), "task": task.id, "controller_backend": "cpp",
                  "robot_from_source": robot_from_source,
                  "motion_source": "annotations", "source_sequences": json.dumps(source_sequences),
                  "optimizer": optimizer,
                  "dof_names": np.asarray([j.name for j in prefab.joints if j.type == sdp.ArticulatedJointType.REVOLUTE])}
        metrics = {"task": task.id, "controller_backend": "cpp", "optimizer": optimizer,
                   "motion_source": "annotations", "source_sequences": source_sequences, "stage": "retarget", "hands": {},
                   "manipulation_success": None}
        for side, seq in sequences.items():
            kp = np.asarray(seq["keypoints"]) @ robot_from_source[:3, :3].T + robot_from_source[:3, 3]
            arrays[f"{side}_source_keypoints_world"] = kp
            hand = HandRetargeter(side, optimizer, prefab=prefab, sdp=sdp)
            arm = ArmChain(prefab, "openarm_body_link0", f"{side}_link_01_wrist", sdp)
            result = hand.retarget_sequence(kp)
            arrays[f"{side}_optimizer_hand_q"] = result["q"].copy()
            assist = control.get("grasp_assist")
            if assist:
                angle = float(assist["angle_rad"])
                start = float(assist["start_time_s"])
                ramp = float(assist["ramp_time_s"])
                if not np.isfinite([angle, start, ramp]).all() or angle < 0 or start < 0 or ramp <= 0:
                    raise ValueError("Grasp assistance needs a nonnegative angle/start and positive ramp")
                weight = np.clip((source_times - source_times[0] - start) / ramp, 0., 1.)
                result["q"] = hand.apply_grasp_squeeze(result["q"], weight, angle)
                arrays[f"{side}_grasp_weight"] = weight
            arm_q, pos_error, rot_error = arm.solve_trajectory(result["wrist_T"], rest[arm.dof_indices],
                                                             q_posture=rest[arm.dof_indices], multistart=8)
            source_targets[:, arm.dof_indices] = arm_q
            source_targets[:, hand.dof_indices] = result["q"]
            for key, value in {"arm_q": arm_q, "hand_q": result["q"], "wrist_T": result["wrist_T"],
                               "retarget_cost": result["cost"], "solver_status": result["solver_status"],
                               "ik_pos_err": pos_error, "ik_rot_err": rot_error,
                               "arm_dofs": arm.dof_indices, "hand_dofs": hand.dof_indices}.items():
                arrays[f"{side}_{key}"] = value
            metrics["hands"][side] = {"ik_pos_err_max_mm": float(pos_error.max() * 1000),
                                      "ik_rot_err_max_deg": float(np.degrees(rot_error.max())),
                                      "retarget_cost_mean": float(result["cost"].mean()),
                                      "failed_solver_frames": int(np.count_nonzero(result["solver_status"] < 0))}
        # Include all transitions before native projection, smoothing and time scaling.
        relative = source_times - source_times[0]
        key_times = np.r_[0., approach + relative, approach + relative[-1] + hold]
        key_poses = np.vstack([rest, source_targets, source_targets[-1]])
        limits = limits_from_prefab(prefab, sdp, **policy)
        times, commands, preparation = prepare_trajectory(key_poses, key_times, limits, margin=.015,
                                                           smoothing_sigma_seconds=.1, dt=dt)
        scale = preparation["time_scale"]
        phase_bounds = {"approach": [0., approach * scale],
                        "demo": [approach * scale, (approach + relative[-1]) * scale],
                        "hold": [(approach + relative[-1]) * scale, float(times[-1])]}
        arrays.update(timestamps=times, q_target=commands, fps=1. / dt, source_q_target=source_targets,
                      source_replay_timestamps=(approach + relative) * scale,
                      includes_replay_timeline=True, scene_config=json.dumps(scene_spec),
                      constraint_policy=json.dumps(policy), warmup_time_s=warmup,
                      phase_bounds=json.dumps(phase_bounds), preparation=json.dumps(preparation))
        metrics["command_constraints"] = preparation
        metrics["command_constraints_passed"] = preparation["passed"]
        metrics["constraints_passed"] = None
        metrics["measured_constraints"] = None
        np.savez_compressed(out / "trajectory.npz", **arrays)
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
        (out / "joint_limits.json").write_text(json.dumps(limits.to_json(), indent=2) + "\n")
        (out / "run_meta.json").write_text(json.dumps({"task": task.id, "optimizer": optimizer,
            "controller_backend": "cpp", "manifest": task.manifest, "scene_config": scene_spec,
            "motion_source": "annotations", "source_sequences": source_sequences,
            "robot_from_source": robot_from_source.tolist(),
            "constraint_policy": policy, "trajectory_preparation": preparation,
            "source_timestamps": "trajectory.npz/source_timestamps", "source_fps": float(first["fps"]),
            "control_fps": 1. / dt, "phase_bounds": phase_bounds, "warmup_time_s": warmup,
            "source_joint_arrays": "Per-side arm_q/hand_q retain unsmoothed source samples; q_target is the prepared command stream."},
            indent=2) + "\n")
    finally:
        sdp.shutdown()
    return out / "trajectory.npz"


def annotation_overlay(data: dict, hands: list[str]):
    """Validate saved source geometry and return its samples on the replay clock."""
    if "source_replay_timestamps" not in data:
        raise ValueError("Human keypoints require a new annotation trajectory; retarget with --motion-source annotations first")
    times = np.asarray(data["source_replay_timestamps"], dtype=float)
    if times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Human keypoint replay timestamps must be finite and increasing")
    points = {}
    for side in hands:
        key = f"{side}_source_keypoints_world"
        if key not in data:
            raise ValueError(f"Human keypoints missing for {side}; retarget the annotation trajectory")
        value = np.asarray(data[key], dtype=float)
        if value.shape != (len(times), 21, 3) or not np.isfinite(value).all():
            raise ValueError(f"Human keypoints for {side} must be finite (T,21,3) samples")
        points[side] = value.reshape(len(times), -1)

    def sample(time):
        return {side: np.array([np.interp(time, times, column) for column in value.T]).reshape(21, 3)
                for side, value in points.items()}

    return sample


def replay_task(task: Task, trajectory: Path, out: Path, *, cameras: list[str], no_video: bool = False,
                width: int = 640, height: int = 480, approach_time: float = 1.0, hold_time: float = .5,
                annotation_only=False, video_fps=15., tactile=False, human_keypoints=False) -> dict:
    if human_keypoints and (not annotation_only or no_video):
        raise ValueError("Human keypoints require annotation replay with video capture")
    from .constraints import audit_trajectory, limits_from_prefab, prepare_trajectory
    from .scene import PickupScene, SceneConfig

    with np.load(trajectory, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    if annotation_only and str(data.get("motion_source", "")) != "annotations":
        raise ValueError("Annotation replay requires an annotation-generated trajectory; retarget with --motion-source annotations first")
    overlay = annotation_overlay(data, task.hands) if human_keypoints else None
    if not np.isfinite(video_fps) or video_fps <= 0:
        raise ValueError("video_fps must be positive and finite")
    if str(data["task"]) != task.id:
        raise ValueError("Trajectory belongs to a different task")
    source_times = np.asarray(data["timestamps"], dtype=float)
    source_q = np.asarray(data["q_target"], dtype=float)
    if source_times.ndim != 1 or len(source_times) < 2 or source_q.ndim != 2 or len(source_times) != len(source_q) or not np.isfinite(source_q).all() or not np.isfinite(source_times).all() or np.any(np.diff(source_times) <= 0):
        raise ValueError("Trajectory must contain finite joint targets and increasing timestamps")
    if not np.isfinite([approach_time, hold_time]).all() or approach_time <= 0 or hold_time < 0:
        raise ValueError("approach_time must be positive and hold_time finite and nonnegative")
    includes_timeline = bool(data.get("includes_replay_timeline", False))
    scene_spec = json.loads(str(data["scene_config"])) if "scene_config" in data else _scene_spec(task)
    if scene_spec.get("object_mesh"):
        scene_spec["object_mesh"] = str(task.path(scene_spec["object_mesh"]))
    cfg = SceneConfig(**scene_spec)
    if not np.isfinite(cfg.dt) or cfg.dt <= 0:
        raise ValueError("Scene control period must be positive and finite")
    if video_fps > 1 / cfg.dt:
        raise ValueError("video_fps cannot exceed the physics sample rate")
    scene = PickupScene(cfg)
    recorder = None
    out.mkdir(parents=True, exist_ok=True)
    try:
        if source_q.shape[1] != scene.num_dofs or not np.array_equal(data["dof_names"], scene.revolute_joint_names):
            raise ValueError("Trajectory DOF names/order do not match the current robot")
        # The current task policy remains authoritative when replaying an imported archive.
        import superdex.physics as sdp
        policy = task.manifest.get("constraint_policy", {})
        limits = limits_from_prefab(scene.prefab, sdp, **policy)
        legacy_preparation = None
        if includes_timeline:
            keys = source_times - source_times[0]
            times = np.arange(int(np.floor(keys[-1] / cfg.dt + 1e-9)) + 1) * cfg.dt
            commands = np.stack([np.interp(times, keys, column) for column in source_q.T], axis=1)
        else:
            approach, hold, _ = _timing({}, cfg.dt, approach_time=approach_time, hold_time=hold_time)
            relative = source_times - source_times[0]
            keys = np.r_[0., approach + relative, approach + relative[-1] + hold]
            poses = np.vstack([scene.rest_pose, source_q, source_q[-1]])
            times, commands, legacy_preparation = prepare_trajectory(poses, keys, limits, margin=.015,
                                                                      smoothing_sigma_seconds=.1, dt=cfg.dt)
        if len(times) < 2:
            raise ValueError("Trajectory is shorter than one simulation step")
        command_audit = audit_trajectory(commands, times, limits, assume_rest_at_ends=True)
        if not command_audit["passed"]:
            (out / "rejected_commands.json").write_text(json.dumps(command_audit, indent=2) + "\n")
            (out / "metrics.json").write_text(json.dumps({"task": task.id, "stage": "replay_preflight",
                "success": False, "constraints_passed": False, "command_constraints": command_audit,
                "measured_constraints": None, "note": "Rejected before any warmup or replay command was sent."},
                indent=2) + "\n")
            raise ValueError(f"Task {task.id}: command trajectory fails robot constraints before replay")
        warmup = float(data.get("warmup_time_s", 1.))
        if not np.isfinite(warmup) or warmup < 0:
            raise ValueError("warmup_time_s must be finite and nonnegative")
        for _ in range(int(round(warmup / cfg.dt))):
            scene.step(commands[0])
        if not no_video:
            from .cameras import CameraRecorder
            center = cfg.object_translation or (cfg.cup_xy[0], cfg.cup_xy[1], cfg.table_top + .12)
            options = {"human_keypoints": True} if human_keypoints else {}
            recorder = CameraRecorder(task, out.name, scene, cameras, width, height,
                                      target=(center[0], center[1], center[2] + .05), fps=video_fps, tactile=tactile, **options)
        log = {key: [] for key in ("t", "q_target", "q", "q_vel", "effort_nm", "cup_pos", "object_quat")}
        next_render = 0.
        for index, t in enumerate(times):
            target = commands[index]
            if index:
                scene.step(target)
            else:
                scene.set_target(target)
            cup_pose = scene.cup_pose()
            values = (t, target, scene.joint_positions(), scene.joint_velocities(), scene.controller_efforts(), cup_pose[0], cup_pose[1])
            for key, value in zip(log, values):
                log[key].append(value)
            if recorder is not None and t >= next_render - 1e-9:
                options = {"human_keypoints": overlay(float(t))} if overlay is not None else {}
                recorder.capture(float(t), scene.link_transforms(), cup_pose, f"{task.id} | annotations | t={t:.2f}s", **options)
                next_render += 1 / video_fps
        arrays = {key: np.asarray(value) for key, value in log.items()}
        measured_audit = audit_trajectory(arrays["q"], times, limits, velocity=arrays["q_vel"], efforts=arrays["effort_nm"])
        saved_trajectory = out / "trajectory.npz"
        imported = trajectory.resolve() != saved_trajectory.resolve()
        if imported:
            shutil.copyfile(trajectory, saved_trajectory)
        (out / "joint_limits.json").write_text(json.dumps(limits.to_json(), indent=2) + "\n")
        provenance = {"task": task.id, "motion_source": str(data.get("motion_source", "annotations")),
            "source_sequences": json.loads(str(data.get("source_sequences", "{}"))),
            "robot_from_source": np.asarray(data.get("robot_from_source", np.eye(4))).tolist(),
            "controller_backend": "cpp", "optimizer": str(data.get("optimizer", "vector"))}
        meta_path = out / "run_meta.json"
        meta = json.loads(meta_path.read_text()) if meta_path.exists() and not imported else {}
        meta.update(provenance, scene_config=scene_spec, replay_trajectory=str(trajectory.resolve()),
                    replay_trajectory_sha256=hashlib.sha256(saved_trajectory.read_bytes()).hexdigest(),
                    warmup_s=warmup, video_fps=video_fps,
                    camera_calibration=task.manifest.get("camera_calibration"))
        meta_path.write_text(json.dumps(meta, indent=2) + "\n")
        np.savez_compressed(out / "log.npz", **arrays)
        (out / "constraints.json").write_text(json.dumps({"command": command_audit, "measured": measured_audit,
            "legacy_trajectory_preparation": legacy_preparation}, indent=2) + "\n")
        metrics_path = out / "metrics.json"
        metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() and not imported else {}
        metrics.update(provenance)
        passed = bool(command_audit["passed"] and measured_audit["passed"])
        metrics.update(stage="replay", success=None if passed else False, constraints_passed=passed,
                       command_constraints=command_audit, measured_constraints=measured_audit)
        metrics["replay"] = {"steps": len(times), "duration_s": float(times[-1]),
            "tracking_err_max_rad": float(np.abs(arrays["q_target"] - arrays["q"]).max()),
            "constraints_passed": passed, "manipulation_success": None, "warmup_s": warmup,
            "object_origin_z_change_m": float(arrays["cup_pos"][-1, 2] - arrays["cup_pos"][0, 2]),
            "max_object_origin_z_change_m": float(arrays["cup_pos"][:, 2].max() - arrays["cup_pos"][0, 2]),
            "note": "Joint compliance and object-origin displacement are measured. A temporary rise does not certify a sustained upright grasp or successful pickup."}
        metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
        if not passed:
            raise ValueError(f"Task {task.id}: measured replay violates robot constraints; inspect {metrics_path}")
        return metrics
    finally:
        try:
            if recorder is not None:
                recorder.close()
        finally:
            scene.destroy()
