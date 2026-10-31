"""Render canonical hand annotations for comparison with robot videos.

The MP4 is an annotation preview, not a camera recording. Its sidecar preserves
the sample times in the source sequence so it can be aligned to a robot replay.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from adaptive_control_project.retarget_pipeline.data_convert import MEDIAPIPE_CHAINS, load_sequence
from adaptive_control_project.retarget_pipeline.tasks import CUP_TASK_ID, load_task, record_video, validate_sequence

def resample_preview(sequences: dict[str, dict], fps: float | None, max_frames: int | None):
    """Interpolate on the source clock, preserving duration at a requested FPS."""
    first = next(iter(sequences.values()))
    rate = float(first["fps"] if fps is None else fps)
    if not np.isfinite(rate) or rate <= 0:
        raise ValueError("fps must be finite and positive")
    if max_frames is not None and max_frames < 1:
        raise ValueError("max-frames must be positive")
    source_times = np.asarray(first["timestamps"], dtype=float)
    count = int(np.floor((source_times[-1] - source_times[0]) * rate + 1e-8)) + 1
    times = source_times[0] + np.arange(count) / rate
    if max_frames is not None:
        times = times[:max_frames]
    result = {}
    for side, sequence in sequences.items():
        source = np.asarray(sequence["timestamps"], dtype=float)
        if source.shape != source_times.shape or not np.allclose(source, source_times, atol=1e-8, rtol=0):
            raise ValueError("Preview hands must share the same timestamps")
        flat = np.asarray(sequence["keypoints"]).reshape(len(source), -1)
        result[side] = np.stack([np.interp(times, source, column) for column in flat.T], axis=1).reshape(-1, 21, 3)
    return result, times, rate


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=[CUP_TASK_ID], default=CUP_TASK_ID)
    parser.add_argument("--input", type=Path, help="A canonical NPZ sequence instead of the manifest inputs")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, help="Output FPS; default is the source FPS")
    parser.add_argument("--max-frames", type=int, help="Limit preview length for a smoke check")
    args = parser.parse_args()
    if args.width < 160 or args.height < 160 or args.width % 2 or args.height % 2:
        parser.error("width and height must be even integers of at least 160 pixels")
    task = load_task(args.task)
    if args.input is not None:
        source_path = args.input.resolve()
        sequence = load_sequence(source_path)
        validate_sequence(sequence, sequence["hand"])
        sequences = {sequence["hand"]: sequence}
        sources = [str(source_path)]
        name = source_path.stem
    else:
        sequences = task.load_sequences()
        sources = list(task.manifest["sequences"].values())
        name = task.manifest.get("default_sequence", task.id)
    keypoints, times, fps = resample_preview(sequences, args.fps, args.max_frames)

    import imageio
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    output = task.root / "videos/raw/external" / f"{name}_keypoints.mp4"
    output.parent.mkdir(parents=True, exist_ok=True)
    dpi = 100
    figure = plt.figure(figsize=(args.width / dpi, args.height / dpi), dpi=dpi)
    axis = figure.add_axes([0.01, 0.08, 0.98, 0.72], projection="3d")
    all_points = np.concatenate([points.reshape(-1, 3) for points in keypoints.values()])
    lower, upper = all_points.min(axis=0), all_points.max(axis=0)
    center = (lower + upper) / 2
    radius = max(float((upper - lower).max()) * 0.62, 0.12)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(min(0., center[2] - radius), center[2] + radius)
    axis.set_box_aspect((1, 1, 1))
    ground_x = [center[0] - radius, center[0] + radius]
    ground_y = [center[1] - radius, center[1] + radius]
    xx, yy = np.meshgrid(ground_x, ground_y)
    axis.plot_surface(xx, yy, np.zeros_like(xx), color="#9da7b1", alpha=.12, shade=False)
    axis.view_init(elev=23, azim=-55)
    axis.set_xlabel("x [m]", fontsize=6, labelpad=-6)
    axis.set_ylabel("y [m]", fontsize=6, labelpad=-6)
    axis.set_zlabel("z [m]", fontsize=6, labelpad=-6)
    axis.tick_params(labelsize=5, pad=-3)
    source_kinds = []
    for sequence in sequences.values():
        try:
            provenance = json.loads(sequence.get("source", "{}"))
        except (TypeError, json.JSONDecodeError):
            provenance = {}
        source_kinds.append(provenance.get("kind", "dataset" if "dataset" in provenance else
                                           task.manifest.get("provenance", {}).get("kind")))
    synthetic = any(kind == "synthetic" for kind in source_kinds)
    annotation_label = "SYNTHETIC ANNOTATIONS" if synthetic else "DATASET ANNOTATIONS"
    figure.text(.5, .965, "KEYPOINT PREVIEW", ha="center", va="top", fontsize=9, weight="bold")
    figure.text(.5, .91, f"{annotation_label} - NOT SOURCE CAMERA FOOTAGE", ha="center", va="top", fontsize=6)
    caption = figure.text(.5, .025, "", ha="center", va="bottom", fontsize=6)
    colors = {"right": "#2473b3", "left": "#e88a28"}
    lines = {}
    for side in keypoints:
        lines[side] = [axis.plot([], [], [], "o-", color=colors[side], linewidth=1.4, markersize=2)[0]
                       for _ in MEDIAPIPE_CHAINS]
    object_sequence = next((sequence for sequence in sequences.values() if "object_pose" in sequence), None)
    object_line = None
    object_positions = None
    if object_sequence is not None:
        original_times = object_sequence["timestamps"]
        positions = np.asarray(object_sequence["object_pose"])[:, :3, 3]
        object_positions = np.column_stack([np.interp(times, original_times, positions[:, dim]) for dim in range(3)])
        object_line = axis.plot([], [], [], "s", color="#3c9963", markersize=4, label="object origin")[0]
    writer = None
    try:
        writer = imageio.get_writer(str(output), fps=fps, codec="libx264", quality=8, macro_block_size=1)
        for index, timestamp in enumerate(times):
            for side, points in keypoints.items():
                for line, chain in zip(lines[side], MEDIAPIPE_CHAINS):
                    selected = points[index, chain]
                    line.set_data_3d(selected[:, 0], selected[:, 1], selected[:, 2])
            if object_line is not None:
                position = object_positions[index]
                object_line.set_data_3d([position[0]], [position[1]], [position[2]])
            caption.set_text(f"{task.id} | source t={timestamp:.3f}s | {', '.join(keypoints)}")
            figure.canvas.draw()
            pixels = np.asarray(figure.canvas.buffer_rgba())[..., :3]
            writer.append_data(pixels)
    finally:
        if writer is not None:
            writer.close()
        plt.close(figure)
    sidecar = output.with_suffix(".timestamps.npz")
    np.savez_compressed(sidecar, timestamps=times, video_timestamps=np.arange(len(times)) / fps)
    record_video(
        task, "raw", "external", output, kind="keypoint_preview",
        fps=fps, frames=len(times), source_sequences=sources,
        timestamps=str(sidecar.relative_to(task.root)), source_start_s=float(times[0]),
        source_end_s=float(times[-1]), playback=1., synthetic=synthetic,
        label="Annotation visualization, not source camera footage",
    )
    print(json.dumps({"video": str(output), "timestamps": str(sidecar), "frames": len(times), "fps": fps}))


if __name__ == "__main__":
    main()
