"""Rebuild the canonical motion archive from DexYCB 20200709_151032 labels."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from adaptive_control_project.retarget_pipeline.data_convert import convert_task_sequence
from adaptive_control_project.retarget_pipeline.tasks import load_task


def main(argv=None):
    argparse.ArgumentParser(description=__doc__).parse_args(argv)
    print(convert_task_sequence(load_task("pickup_the_cup")))


if __name__ == "__main__":
    main()
