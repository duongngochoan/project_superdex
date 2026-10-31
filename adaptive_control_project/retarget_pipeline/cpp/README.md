# Native robot controllers

`arm.cpp` and `hand.cpp` contain the controller calculations. The adjacent Python
modules read model/configuration data, convert frames, map joint names, and pass
contiguous buffers through `ctypes`. The simulation's implicit pose tracking is
provided by the compiled SuperDex physics engine.

## Build and dependencies

From the `project_superdex` repository root:

```bash
.venv/bin/python adaptive_control_project/scripts/build_controllers.py
```

The build requires a C++17 compiler, Eigen 3 headers, and a Linux/POSIX dynamic
loader. No Python headers or pybind11 are required. Set `CXX` and
`EIGEN3_INCLUDE_DIR`, or use `--cxx` and `--eigen-include`, for another local
compiler or Eigen installation. The script replaces the library atomically at
`retarget_pipeline/_native/libretarget_controllers.so`. Restart Python processes
after rebuilding because `native.load_library()` caches the loaded library.

An equivalent CMake build is available:

```bash
cmake -S adaptive_control_project/retarget_pipeline/cpp \
      -B /tmp/superdex-controller-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/superdex-controller-build --parallel
```

The hand binding also requires the Python packages `pin`, `nlopt`, and the local
`wuji-retargeting` checkout. Pinocchio reads the pinned URDF's model metadata;
the controller does not call its Python FK or Jacobian methods. The binding
passes the installed `nlopt._nlopt` extension path to C++. `hand.cpp` opens that
shared object using `dlopen` and resolves its exported `nlopt_*` C functions with
`dlsym`. NLopt's SLSQP calls the C++ objective directly, with no Python objective
or gradient callback. A package build that hides these C symbols is unsupported
and produces a descriptive load error. The build links `${CMAKE_DL_LIBS}` / `-ldl`.

## Algorithms and tuning

| Source | Calculation | Tuning entry point |
| --- | --- | --- |
| `arm.cpp` | Serial-chain FK and geometric Jacobian; orientation error as an axis-angle vector | `ArmChain` model metadata in `arm_ik.py` |
| `arm.cpp` | Damped least-squares pose IK with joint limits, step limits and a posture regularizer | `ArmChain.solve(..., w_posture, w_rot, damping, max_iters, max_step, tol_pos, tol_rot)` |
| `arm.cpp` | First-frame random restarts; subsequent frames warm-start from the preceding solution | `solve_multistart(..., n_starts, seed)` / `solve_trajectory(..., multistart, seed)` |
| `hand.cpp` | Tree FK, link-position Jacobians, vector or adaptive analytical objective, bounded native SLSQP | `configs/wuji_hand2_{left,right}_{vector,adaptive}.yaml` |
| `hand.cpp` | Temporal joint regularization and low-pass filtering | YAML `norm_delta`, `lp_alpha`; `retarget_sequence(..., apply_filter)` |
| `hand.cpp` | Additional grasp flexion and joint-limit clipping | `apply_grasp_squeeze(q, weight, squeeze_rad)` |
| `constraints.cpp` | Joint position/rate/acceleration audit, measured effort audit | `constraints.audit_trajectory(...)` |
| `constraints.cpp` | Explicit bound-margin projection, positive-weight Gaussian smoothing, bounded PCHIP, analytic derivative retiming and uniform command sampling | `constraints.prepare_trajectory(...)` |

Arm pose errors use metres and radians. Translation occupies the first three
Jacobian rows; rotation occupies the final three. The rotation weight applies to
both the rotation residual and the corresponding Jacobian rows. The posture
weight is added to the normal equations alongside squared damping. Random
restarts select the lowest `position_error + 0.05 * w_rot * rotation_error`.
C++ uses `std::mt19937`; the same seed is reproducible with the same build/toolchain,
but its random initial configurations differ from the former NumPy generator.
Historical placement searches can therefore select different solutions after the
refactor; compare recorded errors and task outcomes instead of exact restart poses.

`constraints.py` reads real position and effort limits from the loaded prefab and
hand velocities from the pinned URDF. Arm speed and acceleration limits are
explicit task operating policies. `prepare_trajectory` reports every projection
and smoothing change; it does not certify contact success. Audit the generated
command grid and actual measured positions, velocities and controller efforts.
Missing effort samples never count as verified motor compliance. The focused
DexYCB replay records those measurements and object motion; it does not certify
a sustained grasp, collision-free motion, or physical task success.

The hand objectives preserve the original wuji-retargeting definitions. Vector
mode averages Huber distances for configured origin/task link pairs. Adaptive
mode blends fingertip position/direction losses with full-hand vector losses,
using thumb-to-finger pinch weights, and adds the configured hyperextension and
DIP/PIP coupling penalties. Position losses operate internally in centimetres;
direction losses are unitless. Bounds, squeeze angles, and temporal joint
regularization use radians. SLSQP retains the former 50-evaluation limit and
absolute function tolerance `1e-4`. Each sequence resets warm-start/filter state,
while consecutive frames within the sequence share the preceding solution.

A negative native SLSQP status retains that frame's initial command, matching
the previous wrapper's failed-solve behavior. `retarget_sequence` returns
`solver_status` alongside `q`, `wrist_T`, `cost`, and `joint_names`, so callers can
record these failures. Costs describe the unfiltered solution without temporal
regularization; `q` contains the filtered command when filtering is enabled.

## Buffer contract and supported models

The exported API is intentionally small and internal. All numeric buffers are
contiguous and use C row-major ordering. Floating-point buffers are `float64`;
index, mask, and solver-status buffers use C `int` / NumPy `int32`. Handles own
copied model/configuration data. Callers retain ownership of input and output
buffers for the duration of each synchronous call.

| Symbol family | Contract |
| --- | --- |
| `ar_arm_create` / `rt_hand_create` | Return an opaque controller handle or null; read the matching `*_last_error` on failure |
| `ar_arm_destroy` / `rt_hand_destroy` | Release the handle; Python uses `weakref.finalize` |
| `ar_arm_fk`, `ar_arm_jacobian`, `ar_arm_solve`, `ar_arm_trajectory` | Return 0 on success, -1 on failure; errors are thread-local |
| `rt_hand_configure`, `rt_hand_sequence`, `rt_hand_fk_points`, `rt_hand_cost_grad`, `rt_hand_squeeze` | Return 0 on success, -1 on failure; errors are thread-local |

The internal hand configuration packs 36 doubles: indices 0–12 are
`huber_delta`, `norm_delta`, `huber_delta_dir`, `w_pos`, `w_dir`, `scaling`,
`w_full_hand`, `thumb_skip_pip`, `w_hyper`, `soft_min`, `w_couple`, `couple_ratio`,
and `lp_alpha`; 13–27 are the three segment scales for each finger; 28–31 are
index/middle/ring/pinky `d1`; 32–35 are their `d2`. Keep this packing synchronized
with `HandRetargeter.__init__` when extending the ABI.

Python bindings validate dimensions and finite values before entering C++.
Native entry points also validate model/configuration values and required
pointers; raw C pointers cannot convey allocation lengths, so a new binding must
preserve the dimensions declared in `arm_ik.py` / `hand_retarget.py`. Do not share
a hand handle between simultaneous calls: its NLopt object is mutable. Separate
hands use separate handles.

The shipped bounded revolute Wuji Hand 2 models and fixed joints are supported.
Floating bases, prismatic joints, continuous unbounded joints, and mimic-joint
custom models require another explicit model implementation. Adaptive configs
must resolve five distinct PIP/DIP joint pairs and five finger roles in thumb to
pinky order. Each segment scale has three values (PIP/DIP/TIP), or four values
whose first MCP scale is omitted by the optimizer. Pinch thresholds require
`d2 > d1`, Huber deltas must be positive, and `lp_alpha` must lie in `(0, 1]`.
`project_tip_dir: true` is rejected. The shipped YAML files use supported settings.

## Validation

```bash
.venv/bin/python -m pytest adaptive_control_project/tests -q
```

The regression suite compares native arm FK with SuperDex and checks IK on
reachable targets. Hand checks compare both native objectives and analytical
gradients against the original implementation for both sides, compare initial
SLSQP solutions, check FK against SuperDex, and verify deterministic sequence
reset, filtering, squeeze, joint limits, and invalid-input handling. The upstream
MIT copyright/license notice is included in `hand.cpp`.

The pinned left-hand source URDF has small geometry differences from the SuperDex
left-hand asset: a sampled pose showed about 1.3 mm fingertip displacement. Native
FK still matches the original Pinocchio source model to numerical precision. The
SuperDex comparison uses a 2 mm coordinate tolerance on the left and a 0.1 mm
tolerance on the right; this source-model difference is separate from the native
controller regression checks.
