# One sequence, one upright mug, three camera views

This project focuses on **DexYCB `20200709_151032`**, a right-hand mug demonstration. The main output is:

[`tasks/pickup_the_cup/videos/robot/pickup_the_cup.mp4`](tasks/pickup_the_cup/videos/robot/pickup_the_cup.mp4)

The current command creates a reproducible reconstruction from the identified sequence and mug geometry, with explicit settings and measurements. 

**Start with the [detailed input/output and workflow manual](tasks/pickup_the_cup/data/README.md).** It explains the raw data, every intermediate output, complete run commands, validation, and troubleshooting. The [task walkthrough](tasks/pickup_the_cup/docs/task.md) provides a shorter explanation.

## Run it

Run from `project_superdex` using the existing environment:

```bash
PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 \
.venv/bin/python adaptive_control_project/scripts/run_task.py
```

That one command reads raw labels, rebuilds the processed motion, computes robot commands, simulates, and renders external, head-position, and right-wrist views. It also creates an arm-and-hand commanded-keypoint preview. All four videos go directly into `tasks/pickup_the_cup/videos/robot/`:

- `pickup_the_cup.mp4`: external measured simulation view.
- `pickup_the_cup_head.mp4`: stationary virtual camera near the robot's head position.
- `pickup_the_cup_wrist_right.mp4`: virtual camera fixed to the moving right wrist.
- `pickup_the_cup_arm_hand_constraints.mp4`: robot arm and finger keypoints reconstructed from commanded joint angles.

The mug now starts with its open rim facing upward and its lowest mesh point 1 mm above the table. Its handle keeps the former horizontal heading.

To understand the stages individually:

```bash
# 1. Raw labels -> 62-frame canonical motion archive.
.venv/bin/python adaptive_control_project/scripts/run_task.py --stage convert

# Optional: see the original human keypoints before robot retargeting.
.venv/bin/python adaptive_control_project/scripts/render_raw_preview.py

# 2. Human keypoints -> bounded robot joint commands; no simulation yet.
.venv/bin/python adaptive_control_project/scripts/run_task.py --stage retarget

# 3. Saved commands -> physics simulation -> MP4 and measurements.
PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 \
.venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay
```

The source preview is `videos/raw/dexycb_mug_keypoints.mp4`. Each camera video has a `.camera.npz` sidecar with capture times and camera poses. The commanded-keypoint preview has `.keypoints.npz` and `.json` sidecars; its audit covers commands rather than measured tracking. Intermediate commands, measurements and audits are in `retarget/runs/pickup_the_cup/`. `--no-video` suppresses camera videos and the automatic keypoint preview.

To rebuild the tactile-force dashboard from the existing videos and saved measurements:

```bash
.venv/bin/python adaptive_control_project/scripts/render_tactile_dashboard.py
```

This rebuilds `videos/robot/pickup_the_cup_tactile.mp4` as a 1280×720 dashboard with a large right-wrist view, external context inset, per-link force markers, readable palm/finger totals, contact status, and force history. It checks that the saved camera clocks and measured motion match before combining them. It runs no physics or OpenGL rendering and preserves the original five videos and force NPZ.

The markers show simulated object-contact force magnitude sums per link, placed at measured robot mesh-bound centers. They are overlays without an occlusion test, rather than contact-point locations, force vectors, pressure, taxels, or hardware readings. Marker colors saturate at 5 N; numbers remain unclipped. The `.dashboard.json` sidecar describes both camera views and data provenance. The original `.tactile.npz` camera poses still describe the external force capture, not the composite. See the [tactile data guide](tasks/pickup_the_cup/data/README.md#simulated-contact-force-display) for inspection and replay commands.

The task uses `video_layout: "flat"`: source previews go in `videos/raw/` and robot products in `videos/robot/`. The video index retains camera identities `external`, `head`, and `wrist_right`; `_head` and `_wrist_right` filename suffixes distinguish the latter views.

The force samples came from a separate `retarget/runs/pickup_the_cup_tactile/` measurement run whose measured state arrays exactly match the original replay. Contact occurs from 18.835 to 21.500 s; inspect around 19.2 s for the peak. The improved presentation uses the same 1,300 force captures at 30 FPS; all 194 regression tests pass. The [detailed guide](tasks/pickup_the_cup/data/README.md#current-diagnostic-outcome) records verification evidence and remaining measured constraint failures.

## What remains

```text
tasks/pickup_the_cup/
  task.json                       One source sequence and its control settings
  assets/
    mug.stl                       DexYCB 025_mug geometry
    asset.json                    Mesh description and configured mass
    scene.json                    Table, mug pose, gravity and controller gains
    annotation_alignment.json     Dataset-to-robot coordinate transform
    cameras.json                  Fixed virtual right-wrist camera mount
  data/
    raw/
      20200709_151032/             meta.yml + eight camera-label folders
      calibration/                Required camera-to-table calibration
    processed/
      dexycb_mug_20200709_151032.npz
  retarget/runs/pickup_the_cup/     Current commands, logs and audit results
  videos/
    raw/                          Human keypoint preview
    robot/                        Camera views, arm/hand preview, tactile overlay
  docs/task.md                    Detailed beginner guide
```

Other sequences, the wide-base training cup, scene-planned comparison runs, and obsolete outputs have been removed. Shared robot descriptions and the retargeting dependency remain under `third_party/`; the robot prefab itself is shared with the main SuperDex checkout. These camera views are simulated visualization placements, not real sensor calibration.

## Setup and checks

For a new environment, `bash adaptive_control_project/setup_env.sh` installs dependencies and builds the native controllers. Local raw inputs are excluded from Git and must be copied with the project separately.

```bash
.venv/bin/python adaptive_control_project/scripts/build_controllers.py
.venv/bin/python -m pytest adaptive_control_project/tests -q
```

Python handles files, coordinate conventions, simulation and rendering. C++ implements hand optimization, arm kinematics/IK and trajectory constraint calculations. See [native controller details](retarget_pipeline/cpp/README.md) after learning the main workflow.
