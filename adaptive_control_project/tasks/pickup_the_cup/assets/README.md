# Assets for the one DexYCB mug replay

| File | Role |
| --- | --- |
| `mug.stl` | One retained copy of DexYCB `025_mug` geometry, used for rendering and collision. Coordinates are interpreted in metres. |
| `asset.json` | Readable geometry/provenance description. The configured simulation mass is 0.118 kg. |
| `scene.json` | Actual simulation configuration: initial mug pose, table, gravity, timestep and controller gains. |
| `annotation_alignment.json` | 4×4 transform taking the dataset table coordinates into robot coordinates. |
| `cameras.json` | Fixed virtual right-wrist camera mounting transform, selected through `task.json`. This is a visualization placement rather than measured hardware calibration. |

The mug is a freely dynamic rigid body; no hand attachment or prescribed object path is created. The table is constructed as a box in code. The OpenArm/Wuji robot assets are shared from the main SuperDex checkout, outside this task folder.

The mesh's open rim points along **local +Z**. `scene.json` rotates that axis onto **world +Z**, keeping the handle's former horizontal heading (yaw approximately 0.818345 rad, or 46.89°). The initial quaternion is `[0, 0, 0.39785017, 0.91745040]` in `[x, y, z, w]` order. Its translation is approximately `[0.259097, -0.181203, 0.260003]` m. The mesh's lowest local Z is −0.039003 m, so its lowest initial world point is 0.221 m: **1 mm above the 0.22 m tabletop**. This height comes from the actual mesh vertices, not a guessed mug centre or height.

The dataset-to-robot alignment and horizontal scene arrangement are based on the related former `native_validation` mug run. The upright orientation is an explicit visualization/initialization correction. These are reconstruction settings, not verified exact metadata for the historical reference MP4. The source labels, processed annotations, and requested robot joint motion are unaffected by this pose correction. See [the workflow guide](../docs/task.md).

**Rerun retargeting after changing the scene:** `trajectory.npz` embeds a scene snapshot. Replaying an older archive preserves its saved upside-down initialization even if `scene.json` has been corrected.

The task defaults to external, stationary virtual head-position, and moving right-wrist views. All three videos and their camera sidecars are saved directly in `videos/robot/external/`, alongside the default arm-and-hand commanded-keypoint preview. The right-wrist camera uses `cameras.json`; its transform stays fixed relative to the wrist as the wrist moves. The head view is stationary because this robot has no moving head-camera link. Neither view is a recording from a real head or wrist sensor, and the wrist camera does not track the mug automatically.
