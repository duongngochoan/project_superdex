"""DexYCB labels -> canonical hand/object motion for sequence 20200709_151032.

Canonical NPZ arrays use metres, seconds, a Z-up table frame, and 21 hand points
per sample. The converter reads all calibrated camera labels, averages their
world-space estimates, and crops the initial distant approach. It does not read
RGB images, estimate poses from video, or produce robot joint commands.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .paths import DATA_PROCESSED

MEDIAPIPE_CHAINS = [[0, 1, 2, 3, 4], [0, 5, 6, 7, 8], [0, 9, 10, 11, 12], [0, 13, 14, 15, 16], [0, 17, 18, 19, 20]]


# --------------------------------------------------------------------------------------------------
# generic helpers
# --------------------------------------------------------------------------------------------------
def check_mediapipe_order(kp: np.ndarray) -> dict:
    """Sanity check that ``kp`` (T,21,3) follows the MediaPipe chain layout.

    In MediaPipe order every chain wrist->MCP->PIP->DIP->TIP has a long first bone (palm, 7-10 cm)
    followed by three short phalanges (1.5-4 cm).  MANO's native order (index/middle/pinky/ring/thumb
    joints then tips) violates this, so the check distinguishes the two layouts.
    """
    bones = {}
    ok = True
    for c in MEDIAPIPE_CHAINS:
        L = [float(np.linalg.norm(kp[:, c[i + 1]] - kp[:, c[i]], axis=1).mean()) for i in range(4)]
        bones[c[1]] = L
        if not (0.02 < L[0] < 0.13 and all(0.008 < l < 0.06 for l in L[1:])):
            ok = False
    return {"ok": ok, "bone_lengths_m": bones}


def resample_uniform(kp: np.ndarray, t: np.ndarray, fps: float):
    """Linear resampling of (T,...) arrays to a uniform grid at ``fps``."""
    t = np.asarray(t, dtype=float)
    tu = np.arange(t[0], t[-1] + 1e-9, 1.0 / fps)
    flat = kp.reshape(len(t), -1)
    out = np.stack([np.interp(tu, t, flat[:, j]) for j in range(flat.shape[1])], axis=1)
    return out.reshape((len(tu),) + kp.shape[1:]), tu


def save_sequence(path: Path, **arrays) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)
    return path


def load_sequence(path: Path) -> dict:
    with np.load(path, allow_pickle=False) as z:
        return {k: (z[k].item() if z[k].shape == () else z[k]) for k in z.files}


# --------------------------------------------------------------------------------------------------
# DexYCB (official per-subject archive: <subject>/<seq>/{meta.yml, <serial>/labels_XXXXXX.npz}, calibration/)
# --------------------------------------------------------------------------------------------------
class _DexYcbYaml(__import__("yaml").SafeLoader):
    pass


_DexYcbYaml.add_constructor("tag:yaml.org,2002:python/tuple", lambda l, n: list(l.construct_sequence(n)))

YCB_NAMES = {1: "002_master_chef_can", 2: "003_cracker_box", 3: "004_sugar_box", 4: "005_tomato_soup_can", 5: "006_mustard_bottle",
             6: "007_tuna_fish_can", 7: "008_pudding_box", 8: "009_gelatin_box", 9: "010_potted_meat_can", 10: "011_banana",
             11: "019_pitcher_base", 12: "021_bleach_cleanser", 13: "024_bowl", 14: "025_mug", 15: "035_power_drill",
             16: "036_wood_block", 17: "037_scissors", 18: "040_large_marker", 19: "051_large_clamp", 20: "052_extra_large_clamp", 21: "061_foam_brick"}


def _dexycb_cam_to_tag(extr: dict, serial: str) -> np.ndarray:
    """camera -> AprilTag (table) frame.  ``extrinsics[serial]`` is the 3x4 camera->master transform and
    ``extrinsics['apriltag']`` the tag->master transform (dex-ycb-toolkit convention)."""
    T = np.asarray(extr[serial], dtype=np.float64).reshape(3, 4)
    tag = np.asarray(extr["apriltag"], dtype=np.float64).reshape(3, 4)
    M = np.eye(4)
    M[:3, :3] = tag[:, :3].T @ T[:, :3]
    M[:3, 3] = tag[:, :3].T @ (T[:, 3] - tag[:, 3])
    return M


def convert_dexycb_sequence(seq_dir: Path, calib_dir: Path, out_name: str, out_dir: Path = DATA_PROCESSED,
                            crop_approach_m: float | None = 0.30, end_frame: int | None = None) -> Path:
    """Convert DexYCB's 21 hand joints (MediaPipe order, checked by bone lengths)
    and the grasped object's 6-D pose per frame, both in the
    AprilTag table frame (Z-up, z=0 on the table).  Labels from all 8 cameras agree to <0.1 mm after
    the transform, so they are simply averaged; frames without a visible hand (``joint_3d == -1`` in
    every camera, typically the first few) are filled by linear extrapolation of the first valid frames.
    """
    import yaml
    seq_dir, calib_dir = Path(seq_dir), Path(calib_dir)
    meta = yaml.load(open(seq_dir / "meta.yml"), Loader=_DexYcbYaml)
    extr = yaml.load(open(calib_dir / f"extrinsics_{meta['extrinsics']}" / "extrinsics.yml"), Loader=_DexYcbYaml)["extrinsics"]
    serials = meta["serials"]
    nf = int(meta["num_frames"])
    gi = int(meta["ycb_grasp_ind"])
    ycb_id = int(meta["ycb_ids"][gi])
    hand = meta["mano_sides"][0]
    fps = 30.0
    M = {s: _dexycb_cam_to_tag(extr, s) for s in serials}
    kp = np.full((nf, 21, 3), np.nan)
    obj = np.zeros((nf, 4, 4))
    for f in range(nf):
        js, os_ = [], []
        for s in serials:
            z = np.load(seq_dir / s / f"labels_{f:06d}.npz")
            P = np.vstack([z["pose_y"][gi].astype(np.float64), [0, 0, 0, 1]])
            os_.append(M[s] @ P)
            j = z["joint_3d"][0].astype(np.float64)
            if not np.allclose(j, -1):
                js.append((M[s][:3, :3] @ j.T).T + M[s][:3, 3])
        obj[f] = np.mean(os_, axis=0)
        if js:
            kp[f] = np.mean(js, axis=0)
    valid = ~np.isnan(kp[:, 0, 0])
    t = np.arange(nf) / fps
    flat = kp.reshape(nf, -1)
    for j in range(flat.shape[1]):
        flat[~valid, j] = np.interp(t[~valid], t[valid], flat[valid, j])
    kp = flat.reshape(nf, 21, 3)
    # DexYCB subjects start with the hand far from the object (0.5 m+), beyond what the OpenArm can
    # reach from a fixed torso: keep only the part of the approach within ``crop_approach_m`` of the object
    if crop_approach_m is not None:
        d = np.linalg.norm(kp[:, 0, :2] - obj[:, :2, 3], axis=1)
        f0 = int(np.argmax(d < crop_approach_m))
        f1 = nf if end_frame is None else min(nf, end_frame)
        kp, obj, valid, t = kp[f0:f1], obj[f0:f1], valid[f0:f1], t[f0:f1] - t[f0]
        nf = len(kp)
    chk = check_mediapipe_order(kp)
    if not chk["ok"]:
        raise ValueError(f"joint layout is not MediaPipe order: {chk}")
    rot = np.repeat(np.eye(3)[None], nf, axis=0)
    # 'camera forward' proxy (DexYCB has no egocentric camera): the horizontal approach direction from
    # the first wrist position to the object, so the robot reaches forward the way the subject did
    v = obj[0, :2, 3] - kp[0, 0, :2]
    cam_fwd = np.repeat(np.r_[v / (np.linalg.norm(v) + 1e-9), 0.0][None], nf, axis=0)
    out = save_sequence(
        out_dir / f"{out_name}.npz", keypoints=kp, wrist_pos=kp[:, 0].copy(), wrist_rot=rot, timestamps=t, fps=fps,
        camera_pos=np.zeros((nf, 3)), camera_forward=cam_fwd, hand=hand, frame="world", up_axis="z", object_pose=obj,
        object_name=YCB_NAMES.get(ycb_id, f"ycb_{ycb_id}"), text=f"DexYCB {seq_dir.name}: pick up the {YCB_NAMES.get(ycb_id, ycb_id)} ({hand} hand)",
        source=json.dumps({"dataset": "DexYCB", "sequence": seq_dir.name, "n_valid_hand_frames": int(valid.sum()), "ycb_id": ycb_id,
                           "frame": "apriltag (table) frame"}),
    )
    return out


def convert_task_sequence(task) -> Path:
    """Rebuild the task's one canonical sequence from its declared raw inputs."""
    dataset = task.manifest["dataset"]
    if dataset["name"] != "DexYCB" or dataset["sequence"] != "20200709_151032":
        raise ValueError("This project supports DexYCB sequence 20200709_151032")
    target = task.path(task.manifest["sequences"]["right"])
    return convert_dexycb_sequence(
        task.path(dataset["sequence_directory"]),
        task.path(dataset["calibration_directory"]),
        target.stem, out_dir=target.parent,
    )
