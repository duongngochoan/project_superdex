# DexYCB 20200709_151032: a detailed guide from raw data to robot video

This project currently studies **one right-hand mug demonstration: DexYCB sequence 20200709_151032**. The main output is [pickup_the_cup.mp4](../videos/robot/external/pickup_the_cup.mp4), a rendered SuperDex simulation of an OpenArm v2.0 arm and Wuji Hand 2.

The mug starts upright. The default run also creates head-position and moving right-wrist views, plus a video showing the commanded right-arm and hand keypoints. All four videos are saved directly in `videos/robot/external/`; their differences and sidecars are explained in [section 12](#12-rendering-and-video-sidecars).

The input already contains annotated **3D hand points and object poses**. The project does not recover 3D poses from photographs, train a vision model, or turn a source RGB video directly into robot control.

Three representations appear throughout the project:

| Representation | What it describes | Main file |
| --- | --- | --- |
| Human demonstration | Positions of 21 human hand points and the recorded mug pose | processed/dexycb_mug_20200709_151032.npz |
| Robot commands | Joint angles the controller is asked to follow | ../retarget/runs/pickup_the_cup/trajectory.npz |
| Simulated measurements | How the robot and mug actually move during physics replay | ../retarget/runs/pickup_the_cup/log.npz |

These are different data. A well-formed video, legal requested joint angles, accurate tracking, and a successful grasp are separate things to check.

## Contents

1. [Quick start and prerequisites](#1-quick-start-and-prerequisites)
2. [Project folders and input files](#2-project-folders-and-input-files)
3. [What is inside the raw DexYCB labels?](#3-what-is-inside-the-raw-dexycb-labels)
4. [Conversion into one common coordinate system](#4-conversion-into-one-common-coordinate-system)
5. [The processed motion archive](#5-the-processed-motion-archive)
6. [Aligning dataset coordinates with the robot](#6-aligning-dataset-coordinates-with-the-robot)
7. [Hand retargeting and arm inverse kinematics](#7-hand-retargeting-and-arm-inverse-kinematics)
8. [Explicit grip assistance](#8-explicit-grip-assistance)
9. [Smoothing, constraints, and the two clocks](#9-smoothing-constraints-and-the-two-clocks)
10. [The robot command archive](#10-the-robot-command-archive)
11. [Physics replay and measured data](#11-physics-replay-and-measured-data)
12. [Rendering and video sidecars](#12-rendering-and-video-sidecars)
13. [Commands for each stage](#13-commands-for-each-stage)
14. [Optional diagnostic views](#14-optional-diagnostic-views)
15. [Reading metrics and exit codes](#15-reading-metrics-and-exit-codes)
16. [Checking a complete run yourself](#16-checking-a-complete-run-yourself)
17. [Changing settings without confusing the results](#17-changing-settings-without-confusing-the-results)
18. [Troubleshooting](#18-troubleshooting)
19. [Code map and glossary](#19-code-map-and-glossary)

## 1. Quick start and prerequisites

All commands below run from the repository root:

    cd /home/hoan/project_superdex

Use the project's Python environment explicitly:

    .venv/bin/python --version

The current environment uses Python 3.12. There may be no global command named "python"; the examples therefore use .venv/bin/python.

For an existing environment, build the native controller library:

    .venv/bin/python adaptive_control_project/scripts/build_controllers.py

Then run the complete focused pipeline:

    export PYOPENGL_PLATFORM=egl
    export LIBGL_ALWAYS_SOFTWARE=1
    export MPLCONFIGDIR=/tmp/dexycb-retarget-matplotlib
    .venv/bin/python adaptive_control_project/scripts/run_task.py

The default stage is "all": **convert raw labels → retarget robot commands → create the commanded arm-and-hand preview → simulate and render three views**. It writes these videos in one directory:

    adaptive_control_project/tasks/pickup_the_cup/videos/robot/external/pickup_the_cup.mp4
    adaptive_control_project/tasks/pickup_the_cup/videos/robot/external/pickup_the_cup_head.mp4
    adaptive_control_project/tasks/pickup_the_cup/videos/robot/external/pickup_the_cup_wrist_right.mp4
    adaptive_control_project/tasks/pickup_the_cup/videos/robot/external/pickup_the_cup_arm_hand_constraints.mp4

The default command overwrites that run's generated outputs. It also regenerates the trajectory's embedded scene settings, which is necessary to replace an older upside-down mug initialization. Source annotations and requested robot joint motion are unaffected by the upright initialization correction.

The source-keypoint preview is a separate optional command:

    .venv/bin/python adaptive_control_project/scripts/render_raw_preview.py

It writes:

    adaptive_control_project/tasks/pickup_the_cup/videos/raw/external/dexycb_mug_keypoints.mp4

The EGL setting selects headless OpenGL. LIBGL_ALWAYS_SOFTWARE selects software rendering, which can take several minutes even though the simulated motion is much shorter. MPLCONFIGDIR gives Matplotlib a writable cache location; it does not alter the motion.

**Important:** the current reconstruction saves its video and measurements but returns a nonzero exit status when measured robot constraints fail. That diagnostic outcome is described in [section 15](#15-reading-metrics-and-exit-codes). Do not interpret a nonzero status as proof that the video is missing, or a playable MP4 as proof that the grasp succeeded.

For a new Python environment, the setup script prepares dependencies and builds the library:

    bash adaptive_control_project/setup_env.sh

Setup needs network access for package/repository downloads. It does not download this task's raw DexYCB annotations. Preserve or copy the local raw data separately.

The main dependencies are SuperDex, NumPy, SciPy, PyYAML, Pinocchio, NLopt, wuji-retargeting, trimesh, pyrender/OpenGL, Matplotlib, and imageio/FFmpeg. Building the controller library also requires a C++17 compiler and Eigen 3 headers. The build produces:

    adaptive_control_project/retarget_pipeline/_native/libretarget_controllers.so

Python loads that library through ctypes. Rebuild it after modifying the C++ sources and start a new Python process afterward; an already running process caches its loaded library.

## 2. Project folders and input files

The task directory is [pickup_the_cup/](../). Its relevant structure is:

    pickup_the_cup/
      task.json
      assets/
        mug.stl
        asset.json
        scene.json
        annotation_alignment.json
        cameras.json
      data/
        README.md
        raw/
          20200709_151032/
            meta.yml
            836212060125/
              labels_000000.npz
              ...
              labels_000073.npz
            ... seven more camera folders
          calibration/
            extrinsics_20200702_151821/
              extrinsics.yml
        processed/
          dexycb_mug_20200709_151032.npz
      retarget/runs/pickup_the_cup/
        trajectory.npz
        log.npz
        metrics.json
        constraints.json
        joint_limits.json
        run_meta.json
        system_check.json
      videos/
        index.json
        raw/external/
          dexycb_mug_keypoints.mp4
          dexycb_mug_keypoints.timestamps.npz
        robot/external/
          pickup_the_cup.mp4
          pickup_the_cup.camera.npz
          pickup_the_cup_head.mp4
          pickup_the_cup_head.camera.npz
          pickup_the_cup_wrist_right.mp4
          pickup_the_cup_wrist_right.camera.npz
          pickup_the_cup_arm_hand_constraints.mp4
          pickup_the_cup_arm_hand_constraints.keypoints.npz
          pickup_the_cup_arm_hand_constraints.json

The system_check.json file is a compact report from the full system verification. The ordinary run_task.py command creates the command/replay artifacts; it does not automatically run the entire test suite or regenerate that system-check report on every invocation.

### Files needed before conversion and replay

| Input | Purpose |
| --- | --- |
| [task.json](../task.json) | Selects this sequence, its raw/processed paths, alignment file, hand optimizer, timing, grip assistance, and operating caps |
| [raw/20200709_151032/meta.yml](raw/20200709_151032/meta.yml) | Declares camera serial numbers, 74 time steps, hand identity, calibration ID, and the grasped object index |
| Eight camera folders of labels | Supply measured hand points and object poses for each time step |
| [extrinsics.yml](raw/calibration/extrinsics_20200702_151821/extrinsics.yml) | Relates each camera and the AprilTag table coordinate frame |
| [mug.stl](../assets/mug.stl) | The 025_mug triangle mesh used for rendering and physical collision |
| [scene.json](../assets/scene.json) | Actual mug pose, table geometry, mass, gravity, controller gains, and physics timestep |
| [annotation_alignment.json](../assets/annotation_alignment.json) | Dataset-table-to-robot transform |
| [cameras.json](../assets/cameras.json) | A fixed virtual right-wrist mounting transform; used for visualization rather than real sensor calibration |
| Shared robot prefab and pinned hand URDFs | Supply robot geometry, joint names, kinematics, and joint limits |
| Native controller library | Implements arm IK, hand optimization, and trajectory constraints |

The manifest also contains `default_cameras: ["external", "head", "wrist_right"]`, `default_constraint_preview_modes: ["arm_hand"]`, `robot_video_layout: "external_folder"`, and `camera_calibration: "assets/cameras.json"`. These settings choose the default views, default commanded skeleton preview, common robot-video output directory, and wrist mount. They do not replace the source dataset's camera extrinsics.

The robot is shared from the main SuperDex checkout, using the prefab:

    assets/bots/arm_hand_combos/openarm_v20_wuji2_beta1/openarm_v20_wuji2_beta1.superdex_bot

The active hand configurations resolve their URDFs under adaptive_control_project/third_party/wuji-description-pinned/. Keep the wuji-retargeting dependency too. Although this demonstration uses the right side, the loaded robot has both arms and hands; full-model joint-limit checks still need the left-hand model metadata.

### Files that can be regenerated

The processed NPZ, robot trajectory, replay log, metrics, and videos are generated from the retained inputs and settings. Raw annotations are original inputs: do not edit or replace them to make an output look successful.

Raw data, processed data, generated native binaries, and large replay logs are excluded by the project's Git ignore rules. A code-only checkout may therefore lack essential local inputs even when all Python files are present.

## 3. What is inside the raw DexYCB labels?

The sequence has **74 time steps** observed by **eight cameras**. Each camera folder contains 74 files, so there are:

    74 times × 8 cameras = 592 label files

Eight files at the same frame number describe the same moment from different coordinate systems. They are not eight additional moments.

The camera serials in this sequence are:

    836212060125   839512060362   840412060917   841412060263
    932122060857   932122060861   932122061900   932122062010

An NPZ is a ZIP-based NumPy container holding several named arrays. The retained label files contain:

| Raw field | Shape | Stored dtype | Meaning | Used by conversion? |
| --- | --- | --- | --- | --- |
| joint_3d | (1, 21, 3) | float32 | One hand's 21 XYZ points in that camera frame | Yes |
| pose_y | (5, 3, 4) | float32 | Rotation and translation for five objects | Yes, the grasped mug entry |
| joint_2d | (1, 21, 2) | float32 | Image-plane hand coordinates | No |
| pose_m | (1, 51) | float32 | MANO hand-model parameters | No |
| seg | (480, 640) | uint8 | Image-sized segmentation labels | No |

The converter reads existing 3D labels. It does not use joint_2d to reconstruct depth, and it does not run a neural pose estimator. The segmentation array remains inside the original annotation archive but is not a source RGB photograph.

### Which of the five objects is the mug?

The metadata contains:

    ycb_ids: [1, 6, 7, 10, 14]
    ycb_grasp_ind: 4
    mano_sides: [right]

The grasp index is zero-based: 4 selects the fifth object, ID 14. The converter maps that ID to **025_mug**. It takes pose_y[4] from each label file. The other object poses remain in the original raw label archives, but their meshes are unnecessary for this focused replay.

Each selected pose_y matrix has three rows and four columns:

    [ r11 r12 r13 tx ]
    [ r21 r22 r23 ty ]
    [ r31 r32 r33 tz ]

The 3×3 part describes orientation and the last column describes position. Appending [0, 0, 0, 1] makes a 4×4 homogeneous transform.

### Inspect raw labels without modifying them

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import numpy as np
    import yaml

    root = Path('adaptive_control_project/tasks/pickup_the_cup/data')
    sequence = root / 'raw/20200709_151032'
    metadata = yaml.safe_load((sequence / 'meta.yml').read_text())
    print('time steps:', metadata['num_frames'])
    print('cameras:', metadata['serials'])
    print('grasped object ID:',
          metadata['ycb_ids'][metadata['ycb_grasp_ind']])

    labels = sequence / metadata['serials'][0] / 'labels_000012.npz'
    with np.load(labels, allow_pickle=False) as archive:
        for name in archive.files:
            array = archive[name]
            print(name, array.shape, array.dtype)
        print('one camera wrist XYZ:', archive['joint_3d'][0, 0])
        print('mug pose in this camera:', archive['pose_y'][4])
    PY

The calibration YAML contains a tuple notation supported by the project's dedicated safe-loader subclass. Use the project's conversion command to read it; a generic YAML command that rejects that tag does not imply that the calibration data is missing.

## 4. Conversion into one common coordinate system

The implementation is [data_convert.py](../../../retarget_pipeline/data_convert.py), mainly convert_task_sequence(), convert_dexycb_sequence(), and _dexycb_cam_to_tag().

### Camera coordinates, table coordinates, robot coordinates

A coordinate frame is an origin plus three directions used to describe position. One physical wrist has different numerical coordinates when viewed from different cameras.

Conversion first makes all cameras agree on the **AprilTag/table frame**. Robot alignment is a later stage.

Using transform notation:

    T_table_from_camera =
        inverse(T_master_from_table) × T_master_from_camera

    p_table = T_table_from_camera × [x_camera, y_camera, z_camera, 1]

For the mug:

    T_table_from_mug = T_table_from_camera × T_camera_from_mug

The calibration file provides the camera-to-master and AprilTag-to-master relationships. The converter applies their inverse/composition as shown above.

### Exact conversion steps

1. Read the sequence metadata and the matching extrinsics_20200702_151821 calibration.
2. For each of the 74 frame numbers, read all eight camera label files.
3. Select the grasped mug pose using ycb_grasp_ind.
4. Transform each camera's hand points and mug pose into the table frame.
5. Ignore a camera's hand observation when its entire hand-point array is marked -1.
6. Average the available hand observations; average the transformed mug-pose matrices.
7. Fill time steps with no valid hand observation using NumPy interpolation of the available samples. Internal gaps are linearly interpolated; endpoint gaps use the nearest available endpoint.
8. Find the first frame where the horizontal wrist-to-mug distance is below **0.30 m**.
9. Keep that frame and the rest of the sequence; reset its first timestamp to zero.
10. Check the hand-point ordering/bone-length plausibility and save the common-format NPZ.

In this sequence the crop removes the first 12 raw time steps, leaving frames 12–73: **62 samples at 30 FPS**. The converter uses a 30 Hz source clock for these labels.

The horizontal crop uses X and Y distance only. It does not delete frames because the hand is vertically above the mug. This crop is a preparation choice for the robot workspace, not a newly recorded demonstration.

Run conversion alone:

    .venv/bin/python adaptive_control_project/scripts/run_task.py --stage convert

An equivalent conversion-only entrypoint is:

    .venv/bin/python adaptive_control_project/scripts/convert_task_data.py

Both use the paths declared in task.json. They overwrite the same processed file.

## 5. The processed motion archive

The result is [processed/dexycb_mug_20200709_151032.npz](processed/dexycb_mug_20200709_151032.npz).

"Canonical" here means a common format used by the project. It does not mean that the data has already become robot motion.

| Field | Shape / type | Units or meaning |
| --- | --- | --- |
| keypoints | (62, 21, 3), float64 | Human hand-point XYZ positions in metres |
| wrist_pos | (62, 3), float64 | Same positions as keypoints[:, 0] |
| wrist_rot | (62, 3, 3), float64 | Identity placeholders in this DexYCB converter |
| timestamps | (62,), float64 | Source time in seconds |
| fps | Scalar float64 | 30.0 |
| hand | Scalar string | "right" |
| frame | Scalar string | "world", meaning the calibrated dataset/table world |
| up_axis | Scalar string | "z" |
| object_pose | (62, 4, 4), float64 | Recorded mug pose in the table frame |
| object_name | Scalar string | "025_mug" |
| camera_pos | (62, 3), float64 | Zero placeholders, not measured simulation camera poses |
| camera_forward | (62, 3), float64 | A repeated horizontal approach-direction proxy |
| text | Scalar string | A readable description of the recording |
| source | Scalar JSON string | Dataset name, sequence ID, object ID, frame count, and frame convention |

Two details prevent common misunderstandings:

- The "world" flag here describes the **dataset's table frame**. It is not yet the robot base frame.
- wrist_rot is not the robot's wrist orientation. The retargeter derives the desired wrist frame from the 21-point hand geometry.

The first source timestamp is 0; the last is approximately **2.033333 seconds**, since (62 − 1) / 30 = 2.033333. A 62-frame MP4 at 30 FPS displays for about 2.067 seconds because the final frame also occupies a display interval. That small difference is expected.

The hand-point order is:

| Indices | Body part |
| --- | --- |
| 0 | Wrist |
| 1–4 | Thumb |
| 5–8 | Index finger |
| 9–12 | Middle finger |
| 13–16 | Ring finger |
| 17–20 | Little finger |

Inspect the processed data:

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import json
    import numpy as np

    path = Path('adaptive_control_project/tasks/pickup_the_cup/data/processed/'
                'dexycb_mug_20200709_151032.npz')
    with np.load(path, allow_pickle=False) as motion:
        print('fields:', motion.files)
        print('hand points:', motion['keypoints'].shape,
              motion['keypoints'].dtype)
        print('first wrist XYZ [m]:', motion['keypoints'][0, 0])
        print('first/last time [s]:', motion['timestamps'][[0, -1]])
        print('source:', json.loads(str(motion['source'])))
    PY

The first processed wrist position in the current file is approximately:

    [0.479474, 0.446240, 0.273548] metres in the dataset table frame

Regenerated NPZ container bytes can differ while array values remain identical. To check reproducibility, compare named arrays and provenance, not only a ZIP file's modification time.

## 6. Aligning dataset coordinates with the robot

The robot is not automatically sitting at the origin and orientation of the dataset table. [annotation_alignment.json](../assets/annotation_alignment.json) supplies a rigid 4×4 transform:

    T_robot_from_table = [ R  t ]
                        [ 0  1 ]

    p_robot = R × p_table + t

R is a 3×3 rotation. The three entries of t are translation in metres. The final row is [0, 0, 0, 1]. A rigid transform changes origin and orientation, not hand size.

The current transform rotates around Z by approximately **130.54 degrees** and translates by:

    [0.771886, -0.494543, 0.221028] metres

For example, the first wrist above becomes approximately:

    [0.121120, -0.420187, 0.494576] metres in the robot frame

You can verify the calculation:

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import json
    import numpy as np

    root = Path('adaptive_control_project/tasks/pickup_the_cup')
    transform = np.asarray(json.loads(
        (root / 'assets/annotation_alignment.json').read_text()))
    with np.load(root / 'data/processed/dexycb_mug_20200709_151032.npz',
                 allow_pickle=False) as motion:
        point = motion['keypoints'][0, 0]
        print('table frame:', point)
        print('robot frame:', transform[:3, :3] @ point + transform[:3, 3])
    PY

The alignment values were based on a related earlier mug run. They are explicit current reconstruction settings, not verified complete metadata for the old historical MP4.

The source points are saved after alignment as right_source_keypoints_world in the trajectory. This field is in the robot/simulation world frame. The source-keypoint preview, by contrast, shows the processed points in the dataset table frame.

Applying this transform to the hand does not automatically derive the simulated mug's starting pose from object_pose. The mug's initial translation and rotation are explicit settings in scene.json. If you change the alignment, check that the hand and the independently configured mug still share the intended arrangement.

## 7. Hand retargeting and arm inverse kinematics

The main orchestration is [pipeline.py](../../../retarget_pipeline/pipeline.py), in retarget_task().

### Hand shape: 21 points become 20 joint angles

The human hand is supplied as point positions. The Wuji hand is controlled by mechanical joint angles. These quantities cannot be copied directly.

[hand_retarget.py](../../../retarget_pipeline/hand_retarget.py) reads the hand model, estimates a wrist frame from the point geometry, expresses the finger points relative to that wrist, and calls the native optimizer. The default objective is configured by [wuji_hand2_right_adaptive.yaml](../../../retarget_pipeline/configs/wuji_hand2_right_adaptive.yaml).

The optimizer finds bounded finger angles whose robot geometry approximates the observed human finger relationships. The adaptive objective blends hand-vector relationships with fingertip/pinch terms. Previous-pose regularization and filtering promote continuity.

Its per-source-frame output includes 20 angles, a desired wrist pose, objective costs, and solver statuses. Angles are in radians. Costs describe the configured geometric objective, not a probability of grasp success.

### Wrist placement: the seven arm angles

[arm_ik.py](../../../retarget_pipeline/arm_ik.py) solves inverse kinematics for the right arm:

- Forward kinematics computes a wrist pose from known joint angles.
- Inverse kinematics searches for joint angles that place the wrist near a desired position and orientation.

The native solver uses model kinematics/Jacobians, damping, joint bounds, posture regularization, and warm starts. Retargeting saves the position/orientation residuals so the remaining mismatch can be inspected.

### Why are there 54 columns in the final commands?

The complete robot has:

    2 arms × 7 joints + 2 hands × 20 joints = 54 controlled joints

Only the right arm and hand follow this demonstration: **27 active source-driven joints**. The other 27 columns retain the robot's rest commands. The simulator expects the complete model's ordered vector, so the command still has 54 columns.

Use dof_names and the per-side right_arm_dofs/right_hand_dofs arrays to identify columns. Do not assume an arbitrary column order or remove the left-side columns.

## 8. Explicit grip assistance

The task adds a configured finger-flexion adjustment after the hand retargeter. Its settings are in task.json:

| Setting | Current value | Clock |
| --- | --- | --- |
| angle_rad | 0.15 rad, about 8.6 degrees | Angle, not a time |
| start_time_s | 0.733333 s, processed sample 22 (raw frame 34) | Source demonstration clock |
| ramp_time_s | 0.3 s | Source demonstration clock |

The ramp weight is conceptually:

    weight = clip((source_time − start_time) / ramp_time, 0, 1)

The native hand helper applies the weighted angle to selected flexion joints and clips the result to hand bounds. This is an explicit heuristic adjustment, not an angle measured from the person and not a learned grasp policy.

The trajectory preserves both versions:

- right_optimizer_hand_q: retargeter output before this extra grip assistance.
- right_hand_q: source-sampled hand commands after assistance.
- right_grasp_weight: the applied 0-to-1 weight.

Subsequent trajectory preparation can further project/smooth the commands. Neither the adjustment nor clipping guarantees contact, a stable hold, or actuator-effort compliance.

## 9. Smoothing, constraints, and the two clocks

There is a **source clock** and a **robot replay clock**. They are intentionally different.

Before retiming, the task constructs:

    2.5 s approach + 2.033333 s demonstration + 2.0 s hold
    = 6.533333 s nominal timeline

The approach moves from the robot rest command to the first retargeted source pose. The hold repeats the final source command. A separate one-second stationary physics initialization happens before the scored replay/video clock starts.

The native preparation then:

1. Projects commands inside the loaded model's position bounds with a 0.015-radian margin.
2. Smooths the joint commands using the configured preparation smoothing.
3. Fits bounded continuous PCHIP interpolation with resting endpoints.
4. Estimates derivative extrema and scales time to meet speed/acceleration caps.
5. Samples the resulting command stream uniformly at the physics period.
6. Audits the final samples again.

The Python call uses a smoothing sigma of 0.1 seconds on the nominal pre-retiming timeline.

### Limits and where they come from

| Quantity | Arm | Hand | Source |
| --- | --- | --- | --- |
| Position bounds | Per joint | Per joint | Loaded robot prefab |
| Velocity cap | 1.0 rad/s | Up to 1.5 rad/s | Task policy; hand cap also respects the smaller pinned-URDF velocity limit |
| Acceleration cap | 2.0 rad/s² | 4.0 rad/s² | Task operating policy |
| Effort bound | Per joint | Per joint | Loaded prefab; checked using measured controller effort during replay |

The operating speed and acceleration caps are project choices. They are not claimed to be manufacturer-certified limits.

### Current timing example

With the current configuration, preparation uses a time scale of approximately **6.6273**, producing:

| Phase | Approximate replay time |
| --- | --- |
| Approach | 0–16.5683 s |
| Demonstration | 16.5683–30.0438 s |
| Hold | 30.0438–43.3000 s |

The full stream has **8,661 samples at 200 Hz**. Its last timestamp is 43.3 seconds. Actual values should always be read from the current trajectory's phase_bounds and preparation fields; changing an optimizer or setting can change the time scale.

A longer video therefore does not imply that the raw recording contained 43 seconds of human motion. Increasing --video-fps also does not speed up the controller: it changes the number of captured video frames, while physics timing remains at 200 Hz.

## 10. The robot command archive

Run retargeting without simulation:

    .venv/bin/python adaptive_control_project/scripts/run_task.py --stage retarget

It reads the existing processed NPZ and writes [trajectory.npz](../retarget/runs/pickup_the_cup/trajectory.npz). This stage does not automatically reconvert raw data; run "convert" first if the processed input is absent or stale.

For the current settings, the main trajectory fields are:

| Field | Shape / type | Meaning |
| --- | --- | --- |
| source_timestamps | (62,), float64 | Original processed sample times |
| source_fps | Scalar float64 | 30 Hz source rate |
| right_source_keypoints_world | (62, 21, 3), float64 | Human points aligned into robot coordinates |
| right_optimizer_hand_q | (62, 20), float64 | Finger angles before extra grip assistance |
| right_grasp_weight | (62,), float64 | Assistance ramp |
| right_hand_q | (62, 20), float64 | Source-sampled finger commands after assistance |
| right_arm_q | (62, 7), float64 | Source-sampled arm IK solutions |
| right_wrist_T | (62, 4, 4), float64 | Desired wrist poses in robot coordinates |
| right_retarget_cost | (62,), float64 | Hand objective costs |
| right_solver_status | (62,), int32 | Native hand-solver statuses |
| right_ik_pos_err / right_ik_rot_err | (62,), float64 | IK residuals in metres/radians |
| right_arm_dofs / right_hand_dofs | (7,) / (20,), integers | Indices into the complete robot vector |
| source_q_target | (62, 54), float64 | Full robot commands at source samples, before final preparation |
| timestamps | (8661,), float64 | Prepared replay times, seconds |
| q_target | (8661, 54), float64 | Final prepared joint commands, radians |
| fps | Scalar float64 | 200 Hz controller sample rate |
| source_replay_timestamps | (62,), float64 | Source samples located on the retimed replay clock |
| dof_names | (54,), strings | The name corresponding to each joint column |
| robot_from_source | (4, 4), float64 | Alignment used for this export |
| scene_config | Scalar JSON string | Scene snapshot used by the trajectory |
| source_sequences | Scalar JSON string | Processed source path, source-file hash, and provenance |
| constraint_policy / preparation / phase_bounds | Scalar JSON strings | Limit policy, preprocessing report, and timing information |
| task / optimizer / motion_source / controller_backend | Scalar strings | Identity and method metadata |
| hands | (1,), strings | Right hand for this task |
| includes_replay_timeline | Scalar bool | True: transitions are already included |
| warmup_time_s | Scalar float64 | Stationary initialization duration |

The source-sampled arrays and the prepared stream have different lengths. Comparing row 10 in each array would compare different moments.

Inspect the trajectory:

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import json
    import numpy as np

    run = Path('adaptive_control_project/tasks/pickup_the_cup/'
               'retarget/runs/pickup_the_cup')
    with np.load(run / 'trajectory.npz', allow_pickle=False) as command:
        print('source frames:', command['source_timestamps'].shape)
        print('prepared commands:', command['q_target'].shape)
        print('replay time [s]:', command['timestamps'][[0, -1]])
        print('phases:', json.loads(str(command['phase_bounds'])))
        print('first five joint names:', command['dof_names'][:5])
        print('source provenance:', json.loads(str(command['source_sequences'])))
    PY

Retargeting also writes:

| File | Purpose before replay |
| --- | --- |
| metrics.json | Planning/retargeting diagnostics; measured results remain unset |
| joint_limits.json | Ordered model bounds and task rate caps |
| run_meta.json | Settings snapshot, source provenance, preprocessing, and method information |

A successful retarget stage certifies the checked command properties. It has not yet tested robot dynamics, tracking, contact, or object holding.

## 11. Physics replay and measured data

Run only replay:

    export PYOPENGL_PLATFORM=egl
    export LIBGL_ALWAYS_SOFTWARE=1
    .venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay

Replay reads the saved trajectory. It does not repeat hand optimization, arm IK, or raw conversion.

The scene loader creates the robot, table, ground, and dynamic mug. The current scene specifies:

| Parameter | Value |
| --- | --- |
| Mesh | assets/mug.stl |
| Configured mug mass | 0.118 kg |
| Table top height | 0.22 m |
| Initial mug translation | [0.2590971915, -0.1812029467, 0.260003] m |
| Initial rotation quaternion, [x, y, z, w] | [0, 0, 0.3978501697, 0.9174504033] |
| Initial rim direction | Mesh local +Z becomes world +Z |
| Initial lowest-point clearance | 0.001 m above the tabletop |
| Gravity | [0, 0, -9.81] m/s² |
| Physics timestep | 0.005 s, or 200 Hz |
| Stationary initialization | 1.0 s before replay time zero |

The mass is a declared simulation setting inherited from the reconstruction setup. It is not inferred from the images or hand labels.

### Why the mug is now upright

The STL has its open rim at the local **+Z** end and its solid base at the local **−Z** end. The previous scene quaternion sent local +Z almost directly downward: its world direction was approximately [−0.0154, −0.0437, −0.9989]. That made the rim face the table. The corrected rotation sends the same axis to [0, 0, 1], while preserving the handle's horizontal heading, approximately 0.818345 rad or 46.89° about world Z. The STL itself has not been mirrored or edited.

Placing the mesh origin at table height would put part of the mesh inside the table because the origin is not at the base. For the corrected rotation, the lowest actual mesh vertex is at local Z = −0.039003 m. The initialization therefore uses:

    origin_z = table_top - lowest_rotated_mesh_z + clearance
             = 0.22 - (-0.039003) + 0.001
             = 0.260003 metres

The lowest initial world point is 0.221 m. The body can then settle, move, or tip under physics. Upright initialization does not constrain it to stay upright, attach it to the hand, or change the recorded human motion.

The full default command rebuilds the scene snapshot in `trajectory.npz`. If running stages separately, use **retarget → replay** after changing `scene.json`. Replaying a previously saved archive uses that archive's embedded scene; it can reproduce the old upside-down pose despite the corrected current JSON.

The pose controller receives q_target and applies forces/torques to follow it. The resulting measured joints are q. Contact, gravity, and tracking dynamics can make q differ from q_target.

The current scene uses SceneConfig's default enforce_effort_limits=false. Its pose-controller saturation is therefore disabled; preparing position/rate-limited commands does not clamp the resulting actuator efforts. The replay still measures those efforts and checks them against the model's effort bounds. This distinction helps explain why a legal command stream can fail the measured effort audit.

The recorded object_pose is **not** used to prescribe the simulated mug trajectory. The mug is not attached to the robot hand. Its movement comes from physics. This is why the human demonstration may hold the object while the robot replay drops it.

### Replay log schema

[log.npz](../retarget/runs/pickup_the_cup/log.npz) currently contains:

| Field | Current shape | Units / meaning |
| --- | --- | --- |
| t | (8661,) | Replay timestamps in seconds |
| q_target | (8661, 54) | Desired joint angles, radians |
| q | (8661, 54) | Measured simulated joint angles, radians |
| q_vel | (8661, 54) | Measured joint velocities, rad/s |
| effort_nm | (8661, 54) | Measured articulated-controller efforts, N·m |
| cup_pos | (8661, 3) | Simulated mug-origin XYZ position, metres |
| object_quat | (8661, 4) | Simulated mug orientation as quaternion [x, y, z, w] |

"cup" remains the variable name in parts of the code, but the active object is the handled DexYCB mug.

Inspect tracking and object movement:

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import numpy as np

    path = Path('adaptive_control_project/tasks/pickup_the_cup/'
                'retarget/runs/pickup_the_cup/log.npz')
    with np.load(path, allow_pickle=False) as log:
        print('samples:', len(log['t']))
        print('max joint tracking error [rad]:',
              np.max(np.abs(log['q_target'] - log['q'])))
        height = log['cup_pos'][:, 2]
        print('maximum mug-origin rise [m]:', height.max() - height[0])
        print('final mug-origin change [m]:', height[-1] - height[0])
    PY

An origin-height increase is not proof that the object stayed upright, remained held, or cleared all supports. The current focused replay does not implement a complete physical pickup-success certificate.

## 12. Rendering and video sidecars

During simulation, the renderer draws meshes at the **measured** robot-link and mug poses. It does not simply draw the desired commands as though the robot had followed them perfectly.

Three virtual views are recorded by default. The external camera is fixed at eye position [1.4, -1.1, 1.0] m, aimed near the initial mug location. The head-position camera is fixed at [0.05, 0, 1.1] m, looking toward the task workspace; it is stationary because this robot has no moving head-camera link. The right-wrist camera uses a transform from `assets/cameras.json` that stays fixed relative to the moving right wrist. It moves with the wrist, rather than continuously turning to track the cup.

These are simulation camera placements. They are not one of the eight source dataset cameras, do not use source RGB recordings, and do not claim measured head/wrist sensor calibration. The wrist mount is fitted for a useful view of the hand during this replay; mesh occlusion and the mug leaving the view remain possible.

The default video uses:

| Property | Default |
| --- | --- |
| Resolution | 640×480 pixels |
| Video frame rate | 30 FPS |
| Encoding | H.264 MP4 through imageio/FFmpeg |
| Audio | None |
| Cameras | external, head, wrist_right |
| Robot-video directory | videos/robot/external for all three views |
| Expected current frame count | 1,300 |
| Expected replay timestamp span | 0–43.3 s |

At 30 FPS, 1,300 displayed frames span about 43.333 seconds. The controller's final sample is at 43.3 seconds. As with the source preview, this is the distinction between sample timestamps and frame-display intervals.

The camera captures on the physics sample grid, so adjacent capture timestamps can differ slightly from exactly 1/30 second. The difference is bounded by the 0.005-second physics sampling resolution.

### Main output files

| Output | Contents |
| --- | --- |
| [pickup_the_cup.mp4](../videos/robot/external/pickup_the_cup.mp4) | Rendered measured robot and mug motion |
| [pickup_the_cup_head.mp4](../videos/robot/external/pickup_the_cup_head.mp4) | The same measured replay from the stationary virtual head position |
| [pickup_the_cup_wrist_right.mp4](../videos/robot/external/pickup_the_cup_wrist_right.mp4) | The same measured replay from the moving right-wrist mount |
| [pickup_the_cup.camera.npz](../videos/robot/external/pickup_the_cup.camera.npz) | External camera timestamps: (N,), world_from_camera: (N, 4, 4) |
| [pickup_the_cup_head.camera.npz](../videos/robot/external/pickup_the_cup_head.camera.npz) | Head-position camera timestamps and poses with the same array schema |
| [pickup_the_cup_wrist_right.camera.npz](../videos/robot/external/pickup_the_cup_wrist_right.camera.npz) | Right-wrist camera timestamps and moving poses with the same array schema |
| [pickup_the_cup_arm_hand_constraints.mp4](../videos/robot/external/pickup_the_cup_arm_hand_constraints.mp4) | Default right-arm and hand skeleton preview reconstructed from commanded q_target |
| [pickup_the_cup_arm_hand_constraints.keypoints.npz](../videos/robot/external/pickup_the_cup_arm_hand_constraints.keypoints.npz) | Preview timestamps, video timestamps, sampled q_target, joint names, arm/finger points, and provenance |
| [pickup_the_cup_arm_hand_constraints.json](../videos/robot/external/pickup_the_cup_arm_hand_constraints.json) | Full-command constraint audit, preview settings and provenance |
| [videos/index.json](../videos/index.json) | Product, camera, run, frame count, FPS, clip path, and sidecar paths |

The camera matrix maps camera coordinates into the simulation world. These saved poses describe virtual camera placement, not measured hardware sensor calibration.

The arm-and-hand preview is generated before physics replay from the final prepared joint command stream. Forward kinematics places robot arm-joint origins and finger joints/tips at each video sample. It shows both the right arm and its fingers, including their relative motion in world coordinates. Its JSON audit checks the full command timeline's position, velocity and acceleration bounds; it does not use measured robot state, actuator effort or contact. Its `.keypoints.npz` is not a copy of the 21 human source points. In contrast, the three camera movies render the **measured** simulation state.

Although all these files share the `external` directory, `videos/index.json` retains each camera's actual identity (`external`, `head`, or `wrist_right`) and distinguishes measured replay from commanded-keypoint preview. Directory placement does not mean that the wrist/head videos were recorded by the external camera.

The ordinary main command does not create the human source preview. To regenerate that additional learning aid:

    .venv/bin/python adaptive_control_project/scripts/render_raw_preview.py

Its outputs are:

| Output | Contents |
| --- | --- |
| [dexycb_mug_keypoints.mp4](../videos/raw/external/dexycb_mug_keypoints.mp4) | 62-frame visualization of source hand points and recorded object origin |
| [dexycb_mug_keypoints.timestamps.npz](../videos/raw/external/dexycb_mug_keypoints.timestamps.npz) | Source timestamps and zero-based video timestamps |

The source preview is drawn from annotations and is labeled accordingly. It is not original camera footage or proof of a robot grasp.

## 13. Commands for each stage

### Full pipeline from raw data

    cd /home/hoan/project_superdex
    .venv/bin/python adaptive_control_project/scripts/build_controllers.py
    export PYOPENGL_PLATFORM=egl
    export LIBGL_ALWAYS_SOFTWARE=1
    export MPLCONFIGDIR=/tmp/dexycb-retarget-matplotlib
    .venv/bin/python adaptive_control_project/scripts/run_task.py

The expected progress markers are:

    [1/3 convert] .../data/processed/dexycb_mug_20200709_151032.npz
    [2/3 retarget] .../retarget/runs/pickup_the_cup/trajectory.npz
    [3/3 video] .../videos/robot/external/pickup_the_cup.mp4
    [3/3 video] .../videos/robot/external/pickup_the_cup_head.mp4
    [3/3 video] .../videos/robot/external/pickup_the_cup_wrist_right.mp4

The default arm-and-hand preview is also generated from the saved commands before replay. When the measured audit fails, the final markers still identify the saved diagnostic camera videos, followed by an explanatory failure message.

### Conversion only

    .venv/bin/python adaptive_control_project/scripts/run_task.py --stage convert

Reads raw labels/calibration and overwrites the processed motion NPZ. No native retargeting, physics scene, video capture, or automatic keypoint preview is required for this stage.

### Retargeting only

    .venv/bin/python adaptive_control_project/scripts/run_task.py --stage retarget

Reads the processed file and current task/scene/alignment configuration. It computes a new trajectory and planning metadata. It does not reconvert raw data, test physical execution, or automatically render the default keypoint preview. This stage is necessary before replay if an older archive still contains the upside-down scene snapshot.

### Replay only

    PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 .venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay

Reads the current saved trajectory, regenerates its commanded arm-and-hand preview, and records the default three camera views and measured logs. The saved trajectory contains its scene and alignment; edit-and-replay alone is not the way to apply a new mug orientation, alignment, or changed source motion.

### Simulate without a renderer

    .venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay --no-video

This still runs physics, writes measurements, and checks constraints. It does not create or refresh any camera MP4 or the automatic arm-and-hand preview. It is useful for separating physics/model problems from OpenGL/encoding problems. A video already present from an older run is not evidence that this no-video command generated one.

### Keep a separate experiment

    PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 .venv/bin/python adaptive_control_project/scripts/run_task.py --run study_01

This creates retarget/runs/study_01/ and four videos directly in videos/robot/external/: study_01.mp4, study_01_head.mp4, study_01_wrist_right.mp4, and study_01_arm_hand_constraints.mp4, with matching sidecars. The canonical processed input is still shared because there is only one source sequence. Reusing a run name overwrites that run's generated outputs.

Use --run only when you want another experiment; it is unnecessary for the default focused video.

### Select a saved command archive

    PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 .venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay --trajectory adaptive_control_project/tasks/pickup_the_cup/retarget/runs/pickup_the_cup/trajectory.npz

--trajectory is accepted only with --stage replay. The task and joint ordering are validated before use.

## 14. Optional diagnostic views

These options generate additional outputs. They are not required to produce the default pickup_the_cup.mp4.

### Source points over the robot

    PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 .venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay --human-keypoints

This adds separate source-overlay videos for each selected camera, all directly in videos/robot/external/: pickup_the_cup_human_keypoints.mp4, pickup_the_cup_head_human_keypoints.mp4, and pickup_the_cup_wrist_right_human_keypoints.mp4 for the default views, each with a .keypoints.npz sidecar. Green markers show aligned human annotations, not measured robot joints. Their timing follows source_replay_timestamps, including held endpoint poses outside the demonstration interval.

### Commanded robot skeletons

The combined arm-and-hand preview already runs automatically with the default full/replay command, at --video-fps (30 FPS by default). To recreate only that preview from saved commands:

    .venv/bin/python adaptive_control_project/scripts/render_constraint_preview.py --run pickup_the_cup --modes arm_hand --fps 30

To request both available skeleton views:

    .venv/bin/python adaptive_control_project/scripts/render_constraint_preview.py --run pickup_the_cup --modes hand arm_hand --fps 30

This writes two videos in videos/robot/external/:

- pickup_the_cup_hand_constraints.mp4
- pickup_the_cup_arm_hand_constraints.mp4

Each has a .keypoints.npz file and JSON audit/provenance report. These previews reconstruct robot forward kinematics from final q_target. They describe commanded geometry, not measured robot tracking or contact success.

Alternatively, add --constraint-previews explicitly to run_task.py. That requests **both** hand-only and arm-and-hand previews, replacing the manifest's default selection of only arm_hand. The runner uses --video-fps for those previews. Conversion-only and retarget-only commands do not automatically generate the manifest's preview; --no-video disables it during full/replay runs.

### Simulated contact-force display

    PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 .venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay --tactile

This writes the ordinary replays and default arm-and-hand preview, plus one contact-overlay video per selected camera. With default views, those extra files are pickup_the_cup_tactile.mp4, pickup_the_cup_head_tactile.mp4, and pickup_the_cup_wrist_right_tactile.mp4 in videos/robot/external/, each with a .tactile.npz sidecar. The sidecar contains timestamps, world_from_camera, link_names, and force_n with shape (video frames, number of sampled hand links).

The values are sums of object-contact force magnitudes per hand link in **newtons**. The color scale is blue at 0 N through red at 5 N or more. This is a simulated force proxy, not a hardware pressure/taxel measurement. The focused replay's ordinary log.npz does not become a full-rate tactile recording merely because this camera overlay is enabled.

### Other camera views and settings

Supported --cameras values are external, head, wrist_left, and wrist_right. For example:

    PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 .venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay --cameras external head wrist_right --width 640 --height 480 --video-fps 15

The task already selects external, head, and wrist_right by default. With `robot_video_layout` set to `external_folder`, all ordinary robot videos are written directly under `videos/robot/external/`: `<run>.mp4` for external and `<run>_<camera>.mp4` for other views. Selecting `--cameras external` records only the external camera; selecting a subset changes rendering rather than the saved robot commands. The automatic arm-and-hand preview is independent of this camera selection.

The task's camera_calibration setting loads assets/cameras.json automatically. To override the virtual wrist mount, pass --camera-calibration followed by an existing task-local JSON file containing wrist_from_camera matrices. Its 4×4 matrix maps camera-local coordinates into wrist coordinates. The current camera then moves with world_from_camera = world_from_wrist × wrist_from_camera. Without a configured matrix, the renderer fits a virtual mount at initialization and holds it fixed relative to the wrist. This is visualization geometry, not hardware calibration. A camera that follows the wrist is not guaranteed to keep the mug in view throughout arbitrary movement, especially after the mug moves away.

The head-position view stays fixed in world coordinates at [0.05, 0, 1.1] m. It approximates a useful view from the robot's upper body; this model has no moving head-camera link.

Video dimensions must be even and at least 160 pixels. Replay video FPS must be positive and cannot exceed the 200 Hz physics sample rate. Changing video FPS changes capture density, not the simulated joint commands.

Options --human-keypoints, --constraint-previews, and --tactile cannot be combined with --no-video.

## 15. Reading metrics and exit codes

The output video answers, "What did the renderer see?" The audit answers, "Did the saved motion satisfy the checks?" Read both.

| Result | Where to look | What it establishes |
| --- | --- | --- |
| Conversion completed | Processed NPZ, source fields, conversion test | The raw annotations were converted into the expected schema |
| Command audit passed | command_constraints in metrics.json | Requested position/speed/acceleration samples pass the current joint checks |
| Measured audit passed | measured_constraints in metrics.json | Simulated robot position, velocity, acceleration and controller effort pass their audited bounds |
| Video decoded | Video decode check and camera sidecar | The MP4 is complete and its saved timing matches |
| Successful physical pickup | Not certified by this focused workflow | Requires sustained upright holding/contact/support checks beyond these joint audits |

A value such as max_acceleration_limit_ratio = 1.2 means that the measured peak is 1.2 times the configured limit. A ratio at or below 1 is within that reported limit, subject to the audit's declared tolerance.

The command audit has no measured actuator-effort input. Its effort_passed may therefore be null. The measured audit uses controller effort samples and reports an actual effort result.

### Current diagnostic outcome

The corrected full pipeline was rerun on **2026-10-31**. All required runtime inputs were present, **143 tests passed**, and all five catalog videos fully decoded: the unchanged raw preview has 62 frames, and the four requested robot videos each have **1,300 frames at 640×480, 30 FPS**. Camera and keypoint sidecars were verified. The source annotations, prepared `q_target`, and command timestamps are exactly unchanged by the mug-orientation correction.

The upright initialization remains stable through the one-second stationary physics initialization: the measured rim direction is approximately [−0.01600, −0.00296, 0.99987], or about 0.93° from vertical. The corrected replay then records **8,661 physics samples over 43.3 seconds**:

| Corrected replay result | Measured outcome |
| --- | --- |
| Command position/velocity/acceleration audit | Pass |
| Measured joint position and velocity checks | Pass |
| Measured acceleration and actuator-effort checks | Fail |
| Maximum joint tracking error | Approximately 0.02915 rad |
| Maximum mug-origin rise from its first logged height | 0.003445 m, about 3.45 mm |
| Final mug-origin height change | −0.0002783 m, about −0.278 mm |
| Sustained physical lift | Not observed; the mug largely remains on the table |

The height values describe the mesh origin, not bottom clearance. The workflow does not certify physical manipulation success (`manipulation_success` remains null). The pipeline completes video generation and saves its measurements, then returns **exit status 1** because the measured acceleration/effort checks fail. See the fresh metrics.json, log.npz, and [system_check.json](../retarget/runs/pickup_the_cup/system_check.json) for this checked run. Earlier upside-down replay measurements are superseded; they describe different contact and dynamics.

When measured robot constraints fail, the CLI saves the replay log, metrics and camera videos, then exits nonzero with:

    Video/log saved as diagnostics; measured robot constraints failed. Inspect metrics.json.

That is an intentional failure result from the audit, not a missing Python package or missing mesh. The appropriate response is to inspect the measurements and controller/retargeting behavior, not to delete the failure status or loosen bounds until they appear to pass.

A missing input, invalid command stream, model-order mismatch, OpenGL exception, or encoder failure is a different execution problem and must be resolved separately. Commands rejected before replay do not establish a completed simulation.

Check the shell exit status immediately after the command:

    .venv/bin/python adaptive_control_project/scripts/run_task.py --stage replay --no-video
    echo "exit_status=$?"

The audit-failure exit status is 1. Read the actual current exit status and metrics instead of assuming an outcome from an older replay.

### Inspect the current result

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import json

    run = Path('adaptive_control_project/tasks/pickup_the_cup/'
               'retarget/runs/pickup_the_cup')
    metrics = json.loads((run / 'metrics.json').read_text())
    print('stage:', metrics.get('stage'))
    for field in ('command_constraints', 'measured_constraints'):
        audit = metrics.get(field) or {}
        print(field)
        for key in ('passed', 'position_passed', 'velocity_passed',
                    'acceleration_passed', 'effort_passed',
                    'max_acceleration_limit_ratio', 'max_effort_limit_ratio'):
            print(' ', key, audit.get(key))
    print('replay:', metrics.get('replay'))
    print('manipulation_success:', metrics.get('manipulation_success'))
    PY

A successfully saved measured result uses stage "replay". During a retarget-only run, measured values are not yet available. A null manipulation_success means that physical pickup success was not certified; it does not mean true.

## 16. Checking a complete run yourself

The checks below verify different layers. Passing unit tests does not turn a failed physical grasp into a successful one.

### Check that all raw labels are present and readable

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import numpy as np
    import yaml

    root = Path('adaptive_control_project/tasks/pickup_the_cup/data')
    sequence = root / 'raw/20200709_151032'
    meta = yaml.safe_load((sequence / 'meta.yml').read_text())
    count = 0
    for serial in meta['serials']:
        for frame in range(meta['num_frames']):
            path = sequence / serial / f'labels_{frame:06d}.npz'
            assert path.is_file(), path
            with np.load(path, allow_pickle=False) as labels:
                assert labels['joint_3d'].shape == (1, 21, 3), path
                assert labels['pose_y'].shape == (5, 3, 4), path
            count += 1
    calibration = (root / 'raw/calibration' /
                   f"extrinsics_{meta['extrinsics']}" / 'extrinsics.yml')
    assert calibration.is_file(), calibration
    print('Readable label files:', count)
    print('Calibration:', calibration)
    PY

Expected count: **592**.

### Run the regression tests

    .venv/bin/python -m pytest adaptive_control_project/tests -q

The suite checks conversion reproducibility, input validation, native hand/arm calculations, command limits, source timing, upright mug orientation and real-mesh table clearance, replay failure reporting, camera layout and video registration, and related behavior. The complete data-dependent checks require the local annotation data.

### Verify source provenance and prepared/measured dimensions

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import hashlib
    import json
    import numpy as np

    root = Path('adaptive_control_project/tasks/pickup_the_cup')
    run = root / 'retarget/runs/pickup_the_cup'
    with np.load(run / 'trajectory.npz', allow_pickle=False) as command:
        source = json.loads(str(command['source_sequences']))['right']
        actual_hash = hashlib.sha256((root / source['path']).read_bytes()).hexdigest()
        assert actual_hash == source['sha256']
        assert command['q_target'].shape[1] == 54
        assert np.isfinite(command['q_target']).all()
        assert np.all(np.diff(command['timestamps']) > 0)
        command_times = command['timestamps'].copy()
        expected = command['q_target'].copy()
    with np.load(run / 'log.npz', allow_pickle=False) as measured:
        np.testing.assert_allclose(measured['t'], command_times)
        np.testing.assert_allclose(measured['q_target'], expected)
        assert measured['q'].shape == expected.shape
        assert np.isfinite(measured['q']).all()
        assert np.isfinite(measured['cup_pos']).all()
    print('Source provenance and replay arrays are consistent.')
    PY

The hash verifies which processed archive the trajectory used. If you reconvert the processed file after planning, rerun retargeting too before asserting that the old trajectory describes the newly written input archive.

### Decode the entire output and compare its camera sidecar

    .venv/bin/python - <<'PY'
    from pathlib import Path
    import imageio.v2 as imageio
    import numpy as np

    directory = Path('adaptive_control_project/tasks/pickup_the_cup/'
                     'videos/robot/external')
    camera_names = ('pickup_the_cup', 'pickup_the_cup_head',
                    'pickup_the_cup_wrist_right')
    for name in (*camera_names, 'pickup_the_cup_arm_hand_constraints'):
        path = directory / f'{name}.mp4'
        frames = 0
        with imageio.get_reader(path) as reader:
            print(name, 'video metadata:', reader.get_meta_data())
            for frame in reader:
                assert frame.shape[:2] == (480, 640)
                frames += 1
        suffix = '.camera.npz' if name in camera_names else '.keypoints.npz'
        with np.load(path.with_suffix(suffix), allow_pickle=False) as sidecar:
            assert len(sidecar['timestamps']) == frames
            assert np.all(np.diff(sidecar['timestamps']) > 0)
            if name in camera_names:
                assert sidecar['world_from_camera'].shape == (frames, 4, 4)
                assert np.isfinite(sidecar['world_from_camera']).all()
            else:
                assert sidecar['q_target'].shape == (frames, 54)
                assert np.isfinite(sidecar['right_arm_world']).all()
                assert np.isfinite(sidecar['right_hand_world']).all()
        print(name, 'decoded frames:', frames)
    PY

For the unchanged 43.3-second command timeline at the default 30 FPS, expect **1,300 frames per video**. Adjust the resolution or frame expectations if you deliberately changed those settings.

The compact [system_check.json](../retarget/runs/pickup_the_cup/system_check.json) records the full verification rerun's build/input/test/decode checks separately from its measured constraint failure. It is evidence for that checked run, not a promise that future edited inputs or settings will pass.

## 17. Changing settings without confusing the results

The files have different responsibilities:

| Change | Where | What must be rerun |
| --- | --- | --- |
| Raw annotation/calibration change | data/raw | Convert, retarget, replay |
| Dataset-to-robot position/orientation | assets/annotation_alignment.json | Retarget and replay |
| Hand optimizer or grip assistance | task.json / hand configuration | Retarget and replay |
| Joint speed/acceleration policy | task.json | Retarget and replay; old plans cannot override current policy |
| Mug geometry, initial pose, mass or controller gains | assets/scene.json and mug.stl | Retarget to save the new scene, then replay |
| Video resolution, FPS or view selection | Replay command flags | Replay only |
| Fixed virtual wrist mount | assets/cameras.json or --camera-calibration | Replay only |
| C++ calculation changes | retarget_pipeline/cpp | Rebuild library, restart Python, retarget and replay |

The simulator uses scene.json's operational mass and pose. asset.json is a readable catalog; changing its mass description alone does not change physics.

The saved trajectory contains a scene snapshot. Editing the current scene file and replaying an old trajectory does not automatically replace that saved snapshot. Use --stage retarget after changing scene parameters.

Use --optimizer vector or --optimizer adaptive during retargeting to select a different hand objective. Replay uses the optimizer output already stored in its trajectory. Likewise, --robot-from-source applies during retargeting, while replay uses the saved transform.

The advanced --out option changes the saved-run directory. All robot videos still go directly under the task's videos/robot/external/ directory. They use that output directory's basename as the run stem, with _head, _wrist_right, and _arm_hand_constraints suffixes for the additional default products. The default --run behavior is easier to follow when learning.

## 18. Troubleshooting

| Symptom | What to check |
| --- | --- |
| "python: command not found" | Use .venv/bin/python from /home/hoan/project_superdex |
| Raw label or meta.yml missing | Check task.json paths and the 592-label preflight above; setup does not fetch the local task data |
| Processed NPZ missing | Run --stage convert, then --stage retarget |
| trajectory.npz missing | Run --stage retarget or the default full command |
| "Build the native controllers" | Run scripts/build_controllers.py with the same Python environment, then start a new process |
| Compiler/Eigen error | Check that a C++17 compiler and Eigen headers exist; build script supports --cxx and --eigen-include or CXX/EIGEN3_INCLUDE_DIR |
| Hand URDF or robot mesh missing | Keep the shared SuperDex assets, pinned description and installed wuji-retargeting checkout |
| OpenGL/EGL initialization error | Confirm headless EGL/Mesa availability; use --no-video to isolate simulation from rendering |
| Matplotlib cache warning | Set MPLCONFIGDIR to a writable directory such as /tmp/dexycb-retarget-matplotlib |
| Unsupported older CLI flags | Use scripts/run_task.py --help; this focused runner no longer has --motion-source, --task, or --list |
| Dimensions/FPS rejected | Use even dimensions of at least 160 pixels and positive FPS no higher than physics rate |
| Camera override rejected | The JSON must exist inside the task directory; the option accepts an absolute or repository-relative path |
| "command trajectory fails ... before replay" | Inspect rejected_commands.json; invalid requested commands were rejected, so there is no completed new replay |
| "measured robot constraints failed" after video saved | Read measured_constraints and replay in the fresh metrics.json; videos/logs can be complete while measured checks fail |
| Cup still begins upside down after editing scene.json | Run --stage retarget before replay; the saved trajectory embeds its initial scene |
| Head/wrist videos seem absent | Look directly in videos/robot/external/ for the _head and _wrist_right filenames; no separate camera directory is used by this task |
| MP4 exists but cannot be read while a job is running | Wait for encoding/close to finish; an open encoder may not yet have finalized the MP4 |
| Source and robot videos have different lengths | Source motion is about 2 s; the robot adds transitions and retimes to limits, currently about 43.3 s |
| Robot drops the mug | Retargeting approximates hand geometry; grasp stability and measured compliance remain separate controller/contact problems |
| A file is absent from a code-only checkout | Raw/processed inputs, native binaries and logs can be Git-ignored local artifacts |

If the pipeline returns nonzero, preserve its log and inspect the actual final message. A useful capture command is:

    PYOPENGL_PLATFORM=egl LIBGL_ALWAYS_SOFTWARE=1 .venv/bin/python -u adaptive_control_project/scripts/run_task.py > /tmp/pickup_the_cup_run.log 2>&1
    echo "exit_status=$?"
    tail -n 30 /tmp/pickup_the_cup_run.log

Do not use shell constructs that force a success status and then report the failed audit as passed.

## 19. Code map and glossary

### Where each stage is implemented

| Code | Responsibility |
| --- | --- |
| [scripts/run_task.py](../../../scripts/run_task.py) | Stage selection, CLI validation, progress output and diagnostic failure exit |
| [scripts/convert_task_data.py](../../../scripts/convert_task_data.py) | Conversion-only entrypoint |
| [data_convert.py](../../../retarget_pipeline/data_convert.py) | Raw labels/calibration to canonical hand/object archive |
| [tasks.py](../../../retarget_pipeline/tasks.py) | Task paths, sequence validation and video catalog |
| [pipeline.py](../../../retarget_pipeline/pipeline.py) | Alignment, retargeting orchestration, command preparation and replay |
| [hand_retarget.py](../../../retarget_pipeline/hand_retarget.py) | Hand model/frame binding and native optimizer calls |
| [arm_ik.py](../../../retarget_pipeline/arm_ik.py) | Arm model binding, forward kinematics and inverse kinematics |
| [constraints.py](../../../retarget_pipeline/constraints.py) | Model-limit extraction, native preparation and audits |
| [scene.py](../../../retarget_pipeline/scene.py) | Robot, dynamic mug, table, controller and measured state |
| [cameras.py](../../../retarget_pipeline/cameras.py) | Video capture, camera poses and optional overlays |
| [render.py](../../../retarget_pipeline/render.py) | Mesh rendering and frame captions |
| [cpp/](../../../retarget_pipeline/cpp/) | Native numerical calculations loaded through ctypes |

### Terms used in this project

| Term | Meaning here |
| --- | --- |
| Annotation | A supplied measurement/label, such as an XYZ hand point |
| Keypoint | One named point on the hand; 21 points describe one source hand sample |
| NPZ | A container of named NumPy arrays |
| Coordinate frame | An origin and set of axes used to express position/orientation |
| Extrinsic calibration | A transform describing the spatial relationship between camera/world frames |
| Pose | Position together with orientation |
| Homogeneous transform | A 4×4 matrix containing rotation and translation |
| Retargeting | Finding robot motion that approximates a demonstration made by another body |
| Joint angle | A rotation parameter controlling a mechanical robot joint |
| DOF | Degree of freedom; one independent controlled joint coordinate here |
| Forward kinematics | Compute link/wrist pose from joint angles |
| Inverse kinematics | Find joint angles for a desired link/wrist pose |
| q_target | Requested robot joint angles |
| q | Measured simulated robot joint angles |
| Retiming | Changing motion timing to satisfy limits while preparing the command trajectory |
| Pose controller | The simulation controller that tries to follow target joint positions |
| Tracking error | Difference between requested and measured robot positions |
| Sidecar | A data file stored alongside a video, such as its timestamps/camera poses |
| Diagnostic replay | A recorded execution used for inspection, including executions with failed checks |
| Physical task success | A sustained acceptable manipulation outcome; not established solely by a playable MP4 or passing commanded-joint checks |

The recommended learning order is: inspect one raw label → inspect the canonical archive → watch the source-keypoint preview → inspect a few named robot command columns → compare q_target with q → watch the robot video while reading its metrics.
