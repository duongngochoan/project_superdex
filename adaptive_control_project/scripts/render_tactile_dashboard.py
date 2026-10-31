"""Improve pickup_the_cup_tactile.mp4 using saved forces and camera videos."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from adaptive_control_project.retarget_pipeline.tasks import CUP_TASK_ID, load_task
from adaptive_control_project.retarget_pipeline.tactile_video import render_tactile_dashboard


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", default="pickup_the_cup", help="run providing synchronized wrist/external videos")
    parser.add_argument("--tactile-run", default="pickup_the_cup_tactile", help="run providing the measured force replay log")
    args = parser.parse_args(argv)
    print(render_tactile_dashboard(load_task(CUP_TASK_ID), source_run=args.source_run,
                                   tactile_run=args.tactile_run), flush=True)


if __name__ == "__main__":
    main()
