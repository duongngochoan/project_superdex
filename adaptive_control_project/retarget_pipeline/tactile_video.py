"""Rebuild the force dashboard from synchronized saved video and measurements.

No physics is stepped here. Camera footage, measured joint states and force
samples must share the same replay clock and robot/object motion.
"""
from __future__ import annotations

from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np

from .tasks import Task, record_video, robot_video_path
from .tactile import FORCE_SCALE_N, SIGNAL_DESCRIPTION


def _clock(values, label):
    values = np.asarray(values, dtype=float)
    if (values.ndim != 1 or len(values) < 2 or not np.isfinite(values).all()
            or np.any(np.diff(values) <= 0)):
        raise ValueError(f"{label} must contain finite, strictly increasing timestamps")
    return values


def synchronized_indices(times, capture_times):
    """Find measured states at capture instants, rejecting missing samples."""
    times = _clock(times, "Measured log")
    captures = _clock(capture_times, "Force captures")
    right = np.clip(np.searchsorted(times, captures), 0, len(times) - 1)
    left = np.maximum(right - 1, 0)
    indices = np.where(abs(times[left] - captures) < abs(times[right] - captures), left, right)
    if not np.allclose(times[indices], captures, atol=1e-7, rtol=0):
        raise ValueError("Every force frame must match a saved measured-state timestamp")
    return indices


def validate_camera_clock(camera_times, force_times):
    camera_times = _clock(camera_times, "Camera captures")
    force_times = _clock(force_times, "Force captures")
    if (camera_times.shape != force_times.shape
            or not np.allclose(camera_times, force_times, atol=1e-7, rtol=0)):
        raise ValueError("Camera frames and force samples must have identical capture timestamps")


def validate_same_motion(source_log, force_log):
    """A reused view must depict the same measured state, not just commands."""
    for key in ("t", "q", "cup_pos", "object_quat"):
        a, b = np.asarray(source_log[key]), np.asarray(force_log[key])
        if (a.shape != b.shape or not np.isfinite(a).all() or not np.isfinite(b).all()
                or not np.allclose(a, b, atol=1e-9, rtol=0)):
            raise ValueError(f"Source camera replay and tactile replay differ in measured {key}")


def render_tactile_dashboard(task: Task, *, source_run="pickup_the_cup",
                             tactile_run="pickup_the_cup_tactile",
                             source_log_path: Path | None = None,
                             force_log_path: Path | None = None) -> Path:
    """Replace only the external tactile MP4 with a 1280x720 multi-view dashboard.

    Plain external/wrist footage belongs to ``source_run``; the force log belongs
    to ``tactile_run``. The external force sidecar keeps its original camera poses
    as capture provenance. A separate JSON describes the composite's two views.
    """
    import imageio.v2 as imageio

    from .tactile_dashboard import TactileDashboard
    from .tactile_projection import project_points, recorded_link_centers

    external_path = robot_video_path(task, source_run, "external")
    wrist_path = robot_video_path(task, source_run, "wrist_right")
    output = external_path.with_name(f"{external_path.stem}_tactile.mp4")
    signal_path = output.with_suffix(".tactile.npz")
    source_log_path = source_log_path or task.path(f"retarget/runs/{source_run}/log.npz")
    # Validate run names with the same rules as camera outputs.
    robot_video_path(task, tactile_run, "external")
    force_log_path = force_log_path or task.path(f"retarget/runs/{tactile_run}/log.npz")
    with ExitStack() as stack:
        signals = stack.enter_context(np.load(signal_path, allow_pickle=False))
        source_log = stack.enter_context(np.load(source_log_path, allow_pickle=False))
        force_log = stack.enter_context(np.load(force_log_path, allow_pickle=False))
        wrist_camera = stack.enter_context(np.load(wrist_path.with_suffix(".camera.npz"), allow_pickle=False))
        external_camera = stack.enter_context(np.load(external_path.with_suffix(".camera.npz"), allow_pickle=False))
        times = _clock(signals["timestamps"], "Force captures")
        validate_camera_clock(wrist_camera["timestamps"], times)
        validate_camera_clock(external_camera["timestamps"], times)
        validate_same_motion(source_log, force_log)
        indices = synchronized_indices(force_log["t"], times)
        names = signals["link_names"].astype(str)
        values = np.asarray(signals["force_n"], dtype=float)
        if (names.ndim != 1 or len(set(names)) != len(names)
                or values.shape != (len(times), len(names))
                or not np.isfinite(values).all() or np.any(values < 0)):
            raise ValueError("Force samples must be finite, nonnegative and match unique ordered link names")
        if not np.isclose(float(signals["force_color_max_n"]), FORCE_SCALE_N):
            raise ValueError("Force sidecar and dashboard color scales differ")
        cameras = np.asarray(wrist_camera["world_from_camera"], dtype=float)
        if cameras.shape != (len(times), 4, 4) or not np.isfinite(cameras).all():
            raise ValueError("Wrist camera poses must be finite and match all force frames")
        # Check both rendered views against the original force-capture camera.
        if (signals["world_from_camera"].shape != external_camera["world_from_camera"].shape
                or not np.allclose(signals["world_from_camera"], external_camera["world_from_camera"],
                                   atol=1e-7, rtol=0)):
            raise ValueError("External view poses differ from force-capture provenance")
        wrist_reader = imageio.get_reader(str(wrist_path))
        stack.callback(wrist_reader.close)
        external_reader = imageio.get_reader(str(external_path))
        stack.callback(external_reader.close)
        wrist_meta, external_meta = wrist_reader.get_meta_data(), external_reader.get_meta_data()
        fps = float(wrist_meta["fps"])
        if (not np.isfinite(fps) or fps <= 0 or not np.isclose(fps, float(external_meta["fps"]))
                or not np.allclose(times - times[0], np.arange(len(times)) / fps, atol=.005001, rtol=0)):
            raise ValueError("Source videos must share the force capture frame rate")
        source_width, source_height = wrist_meta["size"]
        active = np.flatnonzero(np.char.startswith(names, "right_link_"))
        if not len(active):
            raise ValueError("Dashboard requires right-hand force channels")
        import superdex.physics as sdp
        import superdex.robotics as sdr
        from superdex.physics.paths import resolve_asset
        from .paths import BOT_ASSET

        print("Computing measured hand-link marker positions (no physics replay)...", flush=True)
        sdp.initialize(num_worker_threads=0)
        try:
            prefab = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
            centers = recorded_link_centers(prefab, sdp, force_log["q"][indices], names[active])
        finally:
            sdp.shutdown()
        subset_pixels, subset_visible = project_points(centers, cameras, source_width, source_height)
        pixels = np.full((len(times), len(names), 2), np.nan)
        visible = np.zeros((len(times), len(names)), dtype=bool)
        pixels[:, active], visible[:, active] = subset_pixels, subset_visible
        dashboard = TactileDashboard(times, names, values)
        output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(suffix=".mp4", prefix=".tactile-", dir=output.parent, delete=False) as file:
            temporary = Path(file.name)
        try:
            with imageio.get_writer(str(temporary), fps=fps, codec="libx264", quality=8, macro_block_size=1) as writer:
                for index in range(len(times)):
                    try:
                        wrist_frame = wrist_reader.get_next_data()
                        external_frame = external_reader.get_next_data()
                    except (IndexError, StopIteration) as error:
                        raise ValueError("A source video ends before the force samples") from error
                    writer.append_data(dashboard.frame(index, wrist_frame, external_frame,
                                                       pixels[index], visible[index]))
                    if index % 300 == 0:
                        print(f"Dashboard frame {index + 1}/{len(times)}", flush=True)
                for reader in (wrist_reader, external_reader):
                    try:
                        reader.get_next_data()
                    except (IndexError, StopIteration):
                        pass
                    else:
                        raise ValueError("A source video has extra frames without force samples")
            # Count by decoding the completed file before replacing the published clip.
            with imageio.get_reader(str(temporary)) as check:
                metadata = check.get_meta_data()
                if check.count_frames() != len(times) or tuple(metadata["size"]) != (1280, 720):
                    raise ValueError("Encoded dashboard has incorrect dimensions or frame count")
            temporary.replace(output)
        finally:
            temporary.unlink(missing_ok=True)
        def relative(path):
            path = path.resolve()
            return str(path.relative_to(task.root.resolve()) if path.is_relative_to(task.root.resolve()) else path)
        report_path = output.with_suffix(".dashboard.json")
        report = {
            "schema_version": 1, "presentation": "synchronized_multi_view_tactile_dashboard",
            "width": 1280, "height": 720, "fps": fps, "frames": len(times),
            "source_run": source_run, "tactile_run": tactile_run,
            "views": {"primary": relative(wrist_path), "context": relative(external_path)},
            "camera_poses": {"wrist_right": relative(wrist_path.with_suffix(".camera.npz")),
                             "external": relative(external_path.with_suffix(".camera.npz"))},
            "tactile_signals": relative(signal_path),
            "tactile_signals_sha256": hashlib.sha256(signal_path.read_bytes()).hexdigest(),
            "measured_states": relative(force_log_path),
            "alignment": "Identical capture clocks and measured q, object pose and replay times; no physics rerun",
            "projection": "Exact prefab FK of measured q; GLB-to-Z-up rendered mesh bounds centers; 42 degree vertical FOV",
            "marker_meaning": "Aggregated per-link force magnitude at schematic link center; not actual contact points, pressure taxels or force-vector direction. Markers are overlays without occlusion testing.",
            "signal_description": SIGNAL_DESCRIPTION, "force_color_max_n": FORCE_SCALE_N,
            "force_numbers_clipped": False,
            "composite_calibration": "No single camera pose describes the dashboard. The unchanged tactile NPZ world_from_camera is the original external capture pose, not wrist view calibration.",
            "right_hand_contact_intervals_s": [[float(a), float(b)] for a, b in dashboard.contact_intervals],
            "right_hand_peak_n": float(dashboard.totals.max()),
            "right_hand_peak_time_s": float(times[np.argmax(dashboard.totals)]),
        }
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        record_video(task, "robot", "external", output, kind="simulated_tactile_render",
                     run=tactile_run, source_run=source_run, fps=fps, frames=len(times),
                     width=1280, height=720, tactile_signals=relative(signal_path), report=relative(report_path),
                     signal_description=SIGNAL_DESCRIPTION, force_color_max_n=FORCE_SCALE_N,
                     primary_view="wrist_right", context_view="external",
                     presentation=report["presentation"],
                     render_method="Saved synchronized wrist/external views, measured link-force markers, finger bars and force timeline")
    return output
