"""Shared code/assets and task-local data paths."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]          # adaptive_control_project/
THIRD_PARTY = PROJECT_ROOT / "third_party"
TASKS = PROJECT_ROOT / "tasks"
CUP_TASK = TASKS / "pickup_the_cup"
DATA_RAW = CUP_TASK / "data" / "raw"
DATA_PROCESSED = CUP_TASK / "data" / "processed"
CONFIGS = Path(__file__).resolve().parent / "configs"

# Wuji Hand 2 (Beta 1) URDF at the wuji-description revision pinned by wuji-retargeting.
# Right-hand FK matches SuperDex closely; the pinned left model has millimetre-scale
# fingertip geometry differences (see tests/test_pipeline.py and cpp/README.md).
WUJI_DESCRIPTION_PINNED = THIRD_PARTY / "wuji-description-pinned"
WUJI_HAND2_URDF = {
    side: WUJI_DESCRIPTION_PINNED / "hand2" / "hand2_beta1" / "body" / "urdf" / f"{side}.urdf"
    for side in ("left", "right")
}

BOT_ASSET = "bots/arm_hand_combos/openarm_v20_wuji2_beta1/openarm_v20_wuji2_beta1.superdex_bot"
CUP_PREFAB = "prefabs/paper_cups/paper_cup.mochi_prefab"
