"""Deterministic hand-motion fixtures; these never create project task directories."""
import numpy as np

SIDES = ("left", "right")
REST_WRISTS = {
    # Default OpenArm v2.0 + Wuji Hand 2 beta1 asset, XYZW quaternion.
    "left": ([0.28708680, 0.28608360, 0.46515906],
             [0.19826685, -0.62088530, -0.16636577, 0.73994200]),
    "right": ([0.28708680, -0.28608364, 0.46515903],
              [-0.19826692, -0.62088510, 0.16636567, 0.73994225]),
}
OPERATOR_TO_MANO = {
    "right": np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]]),
    "left": np.array([[0., 0., -1.], [1., 0., 0.], [0., -1., 0.]]),
}


def quaternion_matrix(xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(xyzw, dtype=float) / np.linalg.norm(xyzw)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def _human_hand(closure: float, side: str) -> np.ndarray:
    """MediaPipe-order skeleton, metres; flexion preserves every bone length."""
    points = np.zeros((21, 3), dtype=float)
    bases = [(0.030, 0.025), (0.030, 0.070), (0., 0.085),
             (-0.025, 0.079), (-0.045, 0.065)]
    lengths = [(0.036, 0.027, 0.022), (0.037, 0.024, 0.020),
               (0.041, 0.027, 0.022), (0.037, 0.025, 0.020),
               (0.030, 0.020, 0.017)]
    for finger, ((x, y), segments) in enumerate(zip(bases, lengths)):
        first = 1 + finger * 4
        points[first] = [x, y, 0.]
        for segment, length in enumerate(segments):
            angle = closure * (segment + 1) * (0.50 if finger else 0.35)
            # Thumb points diagonally; other fingers point along the palm.
            direction = np.array([0.70, 0.70, 0.]) if finger == 0 else np.array([0., 1., 0.])
            direction /= np.linalg.norm(direction)
            delta = length * (np.cos(angle) * direction + [0., 0., -np.sin(angle)])
            points[first + segment + 1] = points[first + segment] + delta
    if side == "left":
        points[:, 0] *= -1
    return points


def _hand_frame(points: np.ndarray) -> np.ndarray:
    palm = points[[0, 5, 9]]
    x_axis = palm[0] - palm[2]
    centered = palm - palm.mean(axis=0)
    _, _, vh = np.linalg.svd(centered)
    normal = vh[2]
    x_axis -= np.dot(x_axis, normal) * normal
    x_axis /= np.linalg.norm(x_axis)
    z_axis = np.cross(x_axis, normal)
    if np.dot(z_axis, centered[1] - centered[2]) < 0:
        normal *= -1
        z_axis *= -1
    return np.column_stack([x_axis, normal, z_axis])


def _smoothstep(value: np.ndarray) -> np.ndarray:
    value = np.clip(value, 0., 1.)
    return value * value * (3. - 2. * value)


def generate_arrays(hands: list[str], fps: float = 30., duration: float = 3.) -> dict:
    """Return one shared clock and anatomically ordered hand arrays for a task."""
    if not hands or len(set(hands)) != len(hands) or any(side not in SIDES for side in hands):
        raise ValueError("hands must contain distinct 'left' and/or 'right'")
    if not np.isfinite(fps) or not np.isfinite(duration) or fps <= 0 or duration <= 0:
        raise ValueError("fps and duration must be finite and positive")
    count = max(2, int(round(duration * fps)) + 1)
    timestamps = np.arange(count, dtype=float) / fps
    progress = timestamps / timestamps[-1]
    reach = _smoothstep(progress / 0.4)
    closure = _smoothstep((progress - 0.3) / 0.3)
    lift = _smoothstep((progress - 0.65) / 0.25)
    arrays = {"timestamps": timestamps, "fps": np.array(fps), "closure": closure}
    for side in hands:
        # Exact per-side Euler rotations in configs/wuji_hand2_*_vector.yaml.
        config_rotation = (np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
                           if side == "right" else
                           np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]]))
        origin, quaternion = REST_WRISTS[side]
        desired_rotation = quaternion_matrix(quaternion)
        local_hand = _human_hand(0., side)
        local_wrist_rotation = _hand_frame(local_hand) @ OPERATOR_TO_MANO[side] @ config_rotation.T
        world_from_local = desired_rotation @ local_wrist_rotation.T
        wrist = np.repeat(np.array(origin)[None], count, axis=0)
        wrist[:, 0] += 0.04 * reach
        wrist[:, 2] += 0.04 * lift
        arrays[f"keypoints_{side}"] = np.array([
            _human_hand(float(grip), side) @ world_from_local.T + position
            for grip, position in zip(closure, wrist)
        ])
        arrays[f"wrist_rot_{side}"] = np.repeat(desired_rotation[None], count, axis=0)
    return arrays

