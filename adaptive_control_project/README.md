# Egocentric hand motion retargeting → OpenArm v2.0 + Wuji Hand 2 (bimanual) in simulation

**Focus**: Reusable motion data → kinematic/optimization-based retargeting → simulation control.  
**Not focus**: Training new hand-pose / reconstruction models from raw video.

---

## 1. Problem statement

**Goal**: Take **pre-processed** 3D human hand motion (wrist pose + finger joints / keypoints) from existing datasets, retarget it to the **OpenArm v2.0 + Wuji Hand 2 (bimanual)** morphology, and replay the resulting joint trajectories in a physics simulator so that the robot performs the same manipulation with natural, feasible hand postures.

**Core technical challenges**:
- Morphology mismatch (human MANO / MediaPipe skeleton ≠ Wuji Hand 2 20-DoF anatomical joints).
- Wrist + arm base frame alignment (egocentric camera / world frame → robot base).
- Joint-limit, collision, and contact feasibility under robot kinematics.
- Object assets and trajectories (when available) must be loaded into the same simulator scene.
- Prefer **explainable** methods (IK, vector/position optimization, analytical retargeting) over black-box learned retargeters.

**Success criteria**:
- Robot completes the demonstrated contact / grasp / placement.
- Hand posture remains visually similar to the human demonstration within mechanical limits.
- Trajectory is smooth and stable (no violent joint jumps, no excessive self-collision).

---

## 2. Target robot assets

### 2.1 OpenArm v2.0 and Wuji Hand 2 models

| Component | Repository / Docs | Formats | Notes |
|-----------|-------------------|---------|-------|
| **OpenArm v2.0** | [enactic/openarm_description](https://github.com/enactic/openarm_description) | URDF/xacro, ROS 2 | Preset `default_bimanual`, `right_arm`, `left_arm`. Generate with xacro. |
| **Wuji Hand 2** | [wuji-technology/wuji-description](https://github.com/wuji-technology/wuji-description) | URDF, MJCF, USD, STL | Beta 1 / Beta 2. 20 actuated revolute joints per hand (5 fingers × 4). Anatomical naming. |
| **Combined OpenArm + Wuji** | [Project SuperDex bot assets](https://projectsuperdex.com/robotics/docs/bot_assets/) | `.superdex_bot` | Pre-composed bimanual station. |


### 2.2 SuperDex-Ready assets (Recommended simulation backend)

Project SuperDex ships production-ready bots under `assets/bots/`:

```
assets/bots/
├── arms/openarm_v20/{left,right}/
├── hands/wuji_hand2_beta1/{left,right}/
├── torsos/openarm_v20/
└── arm_hand_combos/openarm_v20/          # bimanual OpenArm
    └── openarm_v20_wuji/                 # OpenArm v2.0 + Wuji Hand 2 (Bimanual)
```

- Collision: Remeshed CAD → SuperDex SDF (`.mochi.h5`)
- Visual: GLB with custom PBR
- Dynamics: Manufacturer values (Mfgr.)

**Primary recommended path**: Load the pre-composed `openarm_v20_wuji` (or equivalent ModBot) inside SuperDex Physics / SuperDex Robotics. This avoids manual URDF stitching and gives consistent collision / contact behavior.

### 2.3 Alternative simulation stacks

| Stack | Strength | Weakness for this project |
|-------|----------|---------------------------|
| **SuperDex** | Native OpenArm + Wuji assets, contact-first physics, bot composition | Newer ecosystem, steeper initial learning curve |
| **MuJoCo + wuji-description MJCF** | Mature, excellent for hand control / RL | Need to attach OpenArm URDF yourself |
| **Isaac Sim / Isaac Lab** | USD assets available for both robots, strong rendering | Heavier setup, GPU required |
| **ROS 2 + MoveIt 2 / ros2_control** | Good for real-robot transfer later | Overkill for pure offline retargeting + replay |

**Recommendation for the team**: Start with **SuperDex** for the integrated bimanual station, keep **MuJoCo** as a lightweight hand-only validation path, and keep **ROS 2** packages only for later hardware transfer.

---

## 3. Recommended data sources (Pre-processed preference)


###  Ready 3D hand + object poses

| Dataset | Content | Why useful | Caveats |
|---------|---------|------------|---------|
| **OakInk / OakInk2** | MANO params, 3D joints, object poses, affordance/intent labels, multi-view RGB-D | Strong object assets + interaction intents; OakInk2 is bimanual | Need to map MANO → Wuji |
| **DexYCB** | 3D hand pose, 6D object pose, multi-view RGB-D, 20 YCB objects | Clean grasps, well-benchmarked | Mostly single-hand |
| **GRAB** | Marker-based full body + hand + object | High-quality trajectories | Body-centric, not egocentric |



**Action for team**: Download OakInk-Shape / OakInk2 motion subsets first. Convert MANO pose + translation into 21-keypoint trajectories (OpenPose / MediaPipe order) once, then feed the same format into the retargeter.

---

## 4. Motion retargeting codebases (Production-ready)

### `wuji-retargeting` + `dex-retargeting`

| Repo | Role |
|------|------|
| [wuji-technology/wuji-retargeting](https://github.com/wuji-technology/wuji-retargeting) | Official Wuji Hand retargeting. Adaptive analytical + key-vector optimizers. Already ships Wuji Hand 2 configs. Supports video, Vision Pro, glove, RealSense, ZED. |
| [dexsuite/dex-retargeting](https://github.com/dexsuite/dex-retargeting) | Upstream library used by Wuji. Position / vector / DexPilot optimizers. Clean API for offline sequences. |

**Key technical points**:
- Optimizers operate on **key vectors** (finger segment directions) or **fingertip positions**.
- Joint mapping is config-driven (`link_naming`, `urdf_path`, joint-name remapping). Never rely on joint index order.
- Output is joint-position trajectory that can be fed directly to MuJoCo / SuperDex / ROS joint commands.
- Real-time capable (~2 ms/frame with NLopt SLSQP).

**Typical offline usage pattern**:
```python
from dex_retargeting.seq_retarget import SeqRetargeting
from dex_retargeting.retargeting_config import RetargetingConfig

config = RetargetingConfig.load_from_file("wuji_hand2_right.yaml")
retargeter = SeqRetargeting(config)
# human_keypoints: (T, 21, 3) in robot-aligned frame
robot_qpos = retargeter.retarget(human_keypoints)  # (T, n_dof)
```

## 5. End-to-End pipeline

```
┌─────────────────────┐
│  Dataset (OakInk /  │
│  DexYCB / preproc.) │
└──────────┬──────────┘
           │  wrist SE(3) + 21 kpts or MANO
           ▼
┌─────────────────────┐
│  Coordinate Align   │  camera/world → robot base
│  (rigid transform)  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Hand Retargeter    │  wuji-retargeting / dex-retargeting
│  (vector or pos.)   │  → Wuji Hand 2 joint trajectory
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Arm IK / Trajectory│  wrist pose → OpenArm 7-DoF
│  (optional)         │  (mink, Pinocchio, or SuperDex OSC)
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Simulator Replay   │  SuperDex (preferred) or MuJoCo
│  + Object assets    │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Evaluation         │  success, penetration, smoothness,
│                     │  visual similarity
└─────────────────────┘
```

### 5.1 Coordinate convention checklist

1. Confirm dataset wrist pose is in **world** or **camera** frame.
2. Estimate or load rigid transform .
3. Transform all keypoints and wrist poses into robot base frame **before** retargeting.
4. Wuji Hand URDF root is `{l,r}_wrist`. Align human wrist orientation to this frame (usually a fixed rotation offset).
5. Keep left/right hands separate; never swap joint names.

### 5.2 Handling morphology difference

- Human has continuous palm + different thumb saddle.
- Wuji has discrete 4-DoF fingers with anatomical limits.
- Preferred strategy:
  - Optimize **fingertip positions** + **key-vector directions** (not full joint angles).
  - Soft joint-limit penalties + self-collision cost inside the optimizer.
  - Accept that exact joint-angle matching is impossible and not required.

---

## 6. Simulation environment setup

### 6.1 SuperDex (Primary recommendation)

**Requirements**:
- Linux x86_64 / Windows / macOS ARM
- Python 3.12
- `uv` package manager
- OpenGL 4.1 capable GPU for Studio / Debugger

```bash
# 1. Clone dev branch
git clone --branch dev https://github.com/duongngochoan/project_superdex.git
cd project_superdex

# 2. Create environment
uv venv
source .venv/bin/activate   # or Windows equivalent

# 3. Install wheels
uv pip install superdex

# 4. Quick test
uv run --no-project superdex_robotics/examples/control/example_osc_jsc_control.py
uv run --no-project superdex-studio   # visual authoring
```

**Load the bimanual OpenArm + Wuji bot**:
```python
from superdex_robotics import load_bot_prefab_from_file

bot = load_bot_prefab_from_file(
    "assets/bots/arm_hand_combos/openarm_v20/openarm_v20_wuji.superdex_bot"
)
# or construct via ModBot (AttachBot + ReplaceLinkWithBot) if you need custom mounting
```

See official docs:
- Bot assets: https://projectsuperdex.com/robotics/docs/bot_assets/
- Modifying bots: https://projectsuperdex.com/robotics/docs/modifying_bots/

### 6.2 MuJoCo Path (Hand-only or full arm later)

```bash
git clone --recurse-submodules https://github.com/wuji-technology/wuji-description.git
# or the older wuji-hand-description if needed

# Preview
python -m mujoco.viewer --mjcf=hand2/hand2_beta1/body/mjcf/right.xml
```

Attach OpenArm URDF via a common base link or use SuperDex for the combined system.


## 7. Concrete implementation steps for the team

### Phase 0 – Setup environment 
1. Install SuperDex + verify OpenArm + Wuji bot loads.
2. Install `wuji-retargeting` (or `dex-retargeting`) + verify a sample Wuji Hand 2 config runs on a synthetic keypoint sequence.
3. Clone `wuji-description` and confirm joint names match the retargeter config.

### Phase 1 – Data preparation
1. Download OakInk / OakInk2 (or DexYCB) motion + object assets.
2. Write a one-time converter: MANO / dataset joints → MediaPipe-order 21 keypoints + wrist SE(3).
3. Save sequences as simple NumPy / Parquet / pickle with consistent frame rate and timestamps.

### Phase 2 – Retargeting core
1. Implement coordinate alignment module (dataset frame → robot base).
2. Configure `wuji-retargeting` YAML for left and right Wuji Hand 2 (use official shipped configs as base).
3. Run offline retargeting on several sequences; visualize in MuJoCo or SuperDex.
4. Tune optimizer weights (tip position vs. vector direction vs. joint regularization) until postures look natural and stay inside limits.

### Phase 3 – Full-body station + objects
1. Load `openarm_v20_wuji` in SuperDex.
2. Solve arm IK / use Cartesian trajectory tracking for the wrist pose (SuperDex OSC or external IK).
3. Import object meshes / trajectories from the dataset into the same scene.
4. Replay synchronized arm + hand + object motion.

### Phase 4 – Evaluation & Iteration
- Metrics: task success (grasp / place), max penetration depth, joint velocity / acceleration peaks, visual similarity (optional keypoint error after retarget).
- Iterate optimizer costs and trajectory smoothing (e.g., cubic spline or QP smoothing on joint space).

---

## 8. Key code / Config references

```text
# Wuji Hand 2 official retargeting
https://github.com/wuji-technology/wuji-retargeting

# Upstream optimizer library
https://github.com/dexsuite/dex-retargeting

# Robot descriptions
https://github.com/enactic/openarm_description
https://github.com/wuji-technology/wuji-description

# SuperDex (simulation + assets)
https://github.com/facebookresearch/project_superdex
https://projectsuperdex.com/robotics/docs/bot_assets/

# Datasets
https://oakink.net/
https://dex-ycb.github.io/

# Full video→robot pipeline (optional later to expend project)
https://github.com/3587jjh/HuRo
https://3587jjh.github.io/HuRo/
https://github.com/zju3dv/EasyMocap
https://github.com/geopavlakos/hamer

```
---

## 9. Design decisions (Locked for the project)

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Simulation backend | SuperDex first | Native OpenArm + Wuji bimanual asset, contact-first physics |
| Retargeting method | Optimization (vector / position) via wuji-retargeting | Explainable, already supports Wuji Hand 2, no training required |
| Data priority | Pre-processed 3D (OakInk / DexYCB) | Avoids HaMeR / EasyMocap development |
| Joint mapping | Name-based + config | Robust across URDF parsers |


---
