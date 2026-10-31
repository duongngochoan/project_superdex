"""Render hand and arm/hand robot skeletons from an audited command trajectory."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from adaptive_control_project.retarget_pipeline.constraint_preview import render_constraint_previews
from adaptive_control_project.retarget_pipeline.tasks import CUP_TASK_ID, load_task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=[CUP_TASK_ID], default=CUP_TASK_ID)
    parser.add_argument("--run", default="pickup_the_cup")
    parser.add_argument("--trajectory", type=Path, help="Override the task-local saved trajectory")
    parser.add_argument("--modes", nargs="+", choices=("hand", "arm_hand"), default=["hand", "arm_hand"])
    parser.add_argument("--fps", type=float, default=15.)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()
    task = load_task(args.task)
    trajectory = args.trajectory or task.root / "retarget/runs" / args.run / "trajectory.npz"
    outputs = render_constraint_previews(task, trajectory, run=args.run, fps=args.fps,
        width=args.width, height=args.height, modes=args.modes)
    print(json.dumps({"videos": [str(path) for path in outputs]}))


if __name__ == "__main__":
    main()
