#!/usr/bin/env bash
# Environment for the cup video pipeline (headless Linux x86_64).
# Run from the project_superdex checkout root.  Everything is installed into ./.venv with uv.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
[ -d .venv ] || uv venv --python 3.12
uv pip install superdex                                  # SuperDex Physics / Robotics / Studio wheels (+ trimesh, imageio-ffmpeg)
uv pip install pin nlopt pyyaml scipy pyrender pyopengl matplotlib pytest
# retargeting + robot descriptions (third_party/)
mkdir -p adaptive_control_project/third_party && cd adaptive_control_project/third_party
[ -d wuji-retargeting ] || git clone --depth 1 https://github.com/wuji-technology/wuji-retargeting.git
if [ ! -d wuji-description-pinned ]; then
  # the revision wuji-retargeting pins as its submodule: ships hand2/hand2_beta1, which matches the SuperDex asset
  git clone https://github.com/wuji-technology/wuji-description.git wuji-description-pinned
  git -C wuji-description-pinned checkout 8271644a78d69ed9a4adcf9165d882c64ad33dfa
fi
uv pip install --python "$ROOT/.venv/bin/python" -e wuji-retargeting --no-deps
cd "$ROOT"
# Build and check the retained video pipeline.
"$ROOT/.venv/bin/python" adaptive_control_project/scripts/build_controllers.py
PYTHONPATH="$ROOT" .venv/bin/python -m pytest adaptive_control_project/tests -q
echo "Headless rendering: PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 .venv/bin/python <script>"
