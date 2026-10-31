"""Virtual head, wrist and external video capture; sensor recordings remain task inputs."""
from __future__ import annotations

from pathlib import Path
import json

import numpy as np
from scipy.spatial.transform import Rotation

from . import render as rd
from .tasks import CAMERA_POSITIONS, Task, record_video, robot_video_path


class CameraRecorder:
    """Stream frames to MP4 and save camera poses on the replay clock."""

    def __init__(self, task: Task, run: str, scene, cameras: list[str], width: int, height: int,
                 target=(0.4, 0, 0.4), fps: float = 30, tactile: bool = False, human_keypoints: bool = False):
        if not cameras or len(set(cameras)) != len(cameras) or not set(cameras) <= set(CAMERA_POSITIONS):
            raise ValueError("Select distinct supported camera positions")
        if width < 160 or height < 160 or width % 2 or height % 2 or not np.isfinite(fps) or fps <= 0:
            raise ValueError("Video dimensions must be even and >= 160, and fps must be positive")
        if Path(run).name != run or run in (".", ".."):
            raise ValueError("Video run must be a single directory name")
        self.renderer = None
        self.writers, self.paths, self.poses, self.mounts = {}, {}, {}, {}
        self.tactile_writers, self.tactile_paths, self.tactile_values = {}, {}, []
        self.human_writers, self.human_paths, self.human_values = {}, {}, []
        self.human_keypoints = human_keypoints
        self.tactile = tactile
        self.times = []
        self._closed = False
        self._capture_failed = False
        try:
            self._initialize(task, run, scene, cameras, width, height, target, fps)
        except BaseException:
            self.close(register=False)
            raise

    def _initialize(self, task, run, scene, cameras, width, height, target, fps):
        import imageio

        self.task, self.run, self.scene = task, run, scene
        if self.tactile:
            from .tactile import hand_link_names
            self.tactile_names = hand_link_names(scene)
        self.fps = fps
        self.renderer = rd.SceneRenderer(width, height, eye=(1.4, -1.1, 1.0), target=target)
        for link in scene.prefab.links:
            if link.render_model_file:
                color = rd.link_color(link.name)
                is_tactile_link = self.tactile and link.name in self.tactile_names
                # Keep GPU vertex colours neutral so the force material really
                # follows the blue/red legend, instead of multiplying orange.
                mesh = rd.load_link_mesh(link.render_model_file, [255,255,255,255] if is_tactile_link else color)
                node = self.renderer.add_mesh(link.name, mesh)
                if is_tactile_link:
                    for primitive in node.mesh.primitives:
                        primitive.material.baseColorFactor = np.asarray(color) / 255.
        self.renderer.add_mesh("table", rd.box_mesh(*scene.table_box, rd.TABLE_COLOR))
        for _, prop in scene.static_props:
            self.renderer.add_mesh(prop["name"], rd.box_mesh(prop["size"], prop["position"], prop.get("color", rd.TABLE_COLOR)))
        self.renderer.add_mesh("ground", rd.box_mesh([3, 3, .01], [0, 0, -.005], rd.GROUND_COLOR))
        if scene.cfg.object_kind == "paper_cup":
            from superdex.physics.paths import resolve_asset
            mesh = rd.load_link_mesh(str(resolve_asset("prefabs/paper_cups/render/paper_cup.glb")), rd.CUP_COLOR)
            mesh.apply_scale(scene.cfg.cup_scale)
        else:
            import trimesh
            mesh = trimesh.load(scene.cfg.object_mesh, force="mesh")
            mesh.visual = trimesh.visual.ColorVisuals(mesh, face_colors=scene.cfg.object_color)
        self.renderer.add_mesh("cup", mesh)
        self.writers, self.paths, self.poses, self.mounts = {}, {}, {}, {}
        self.times = []
        self.fixed = {"external": rd.look_at((1.4, -1.1, 1.0), target),
                      "head": rd.look_at((.05, 0, 1.1), target)}
        links = scene.link_transforms()
        calibration = {}
        if task.manifest.get("camera_calibration"):
            calibration = json.loads(task.path(task.manifest["camera_calibration"]).read_text()).get("wrist_from_camera", {})
        for camera in cameras:
            if camera not in CAMERA_POSITIONS:
                raise ValueError(f"Unknown camera {camera!r}")
            path = robot_video_path(task, run, camera)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.paths[camera] = path
            self.writers[camera] = imageio.get_writer(str(path), fps=fps, codec="libx264", quality=8, macro_block_size=1)
            if self.tactile:
                tactile_path = path.with_name(f"{path.stem}_tactile.mp4")
                self.tactile_paths[camera] = tactile_path
                self.tactile_writers[camera] = imageio.get_writer(str(tactile_path), fps=fps, codec="libx264", quality=8, macro_block_size=1)
            if self.human_keypoints:
                human_path = path.with_name(f"{path.stem}_human_keypoints.mp4")
                self.human_paths[camera] = human_path
                self.human_writers[camera] = imageio.get_writer(str(human_path), fps=fps, codec="libx264", quality=8, macro_block_size=1)
            self.poses[camera] = []
            if camera.startswith("wrist_"):
                side = camera.removeprefix("wrist_")
                idx = scene.link_index(f"{side}_link_01_wrist")
                wrist = self._matrix(*links[idx])
                # Fit the virtual mounting extrinsic once, with clearance above the palm.
                # Keep this extrinsic fixed as the wrist moves throughout the replay.
                eye = wrist[:3, 3] + np.array([0, 0, .12])
                mount = np.asarray(calibration.get(camera, np.linalg.inv(wrist) @ rd.look_at(eye, target)), dtype=float)
                if mount.shape != (4, 4) or not np.isfinite(mount).all() or not np.allclose(mount[3], [0,0,0,1]) or not np.allclose(mount[:3,:3].T @ mount[:3,:3], np.eye(3), atol=1e-6):
                    raise ValueError(f"Invalid virtual wrist camera mounting transform: {camera}")
                self.mounts[camera] = (idx, mount)

    @staticmethod
    def _matrix(position, quaternion):
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        matrix[:3, 3] = position
        return matrix

    def capture(self, time: float, links, cup_pose, caption: str, *, human_keypoints=None):
        try:
            options = {"human_keypoints": human_keypoints} if human_keypoints is not None else {}
            self._capture(time, links, cup_pose, caption, **options)
        except BaseException:
            self._capture_failed = True
            raise

    def _capture(self, time: float, links, cup_pose, caption: str, *, human_keypoints=None):
        renderer = self.renderer
        if self.human_keypoints:
            if human_keypoints is None or set(human_keypoints) != set(self.task.hands):
                raise ValueError("Human overlay requires source keypoints for each task hand")
            points = {side: np.asarray(value, dtype=float) for side, value in human_keypoints.items()}
            if any(value.shape != (21, 3) or not np.isfinite(value).all() for value in points.values()):
                raise ValueError("Human overlay requires finite (21,3) keypoints per hand")
            self.human_values.append({side: value.copy() for side, value in points.items()})
        for (translation, quaternion), name in zip(links, self.scene.link_names):
            if name in renderer.nodes:
                renderer.set_pose(name, translation, quaternion)
        renderer.set_pose("cup", *cup_pose)
        self.times.append(time)
        if self.tactile:
            from .tactile import sample_tactile
            tactile_values = sample_tactile(self.scene, self.tactile_names)
            self.tactile_values.append(tactile_values)
        for camera, writer in self.writers.items():
            if camera in self.mounts:
                idx, mount = self.mounts[camera]
                pose = self._matrix(*links[idx]) @ mount
            else:
                pose = self.fixed[camera]
            self.poses[camera].append(pose)
            renderer.set_camera_pose(pose)
            writer.append_data(rd.annotate(renderer.render(), f"{caption} | virtual {camera}"))
            if self.tactile:
                from .tactile import annotate_tactile, force_color
                original = []
                try:
                    for name, force in zip(self.tactile_names, tactile_values):
                        node = renderer.nodes.get(name)
                        if node is not None:
                            for primitive in node.mesh.primitives:
                                material = primitive.material
                                original.append((material, material.baseColorFactor.copy()))
                                material.baseColorFactor = force_color(force)
                    frame = rd.annotate(renderer.render(), f"{caption} | virtual {camera}")
                    self.tactile_writers[camera].append_data(annotate_tactile(frame, self.tactile_names, tactile_values, self.task.hands))
                finally:
                    for material, color in original:
                        material.baseColorFactor = color
            if self.human_keypoints:
                try:
                    renderer.add_points("source_keypoints", np.concatenate(list(points.values())))
                    frame = rd.annotate(renderer.render(), f"{caption} | virtual {camera} | green: source annotations")
                    self.human_writers[camera].append_data(frame)
                finally:
                    renderer.remove("source_keypoints")

    def close(self, register=True):
        if self._closed:
            return
        self._closed = True
        register = register and not getattr(self, "_capture_failed", False)
        errors = []
        try:
            for camera, writer in self.writers.items():
                try:
                    writer.close()
                    if register and self.times:
                        path = self.paths[camera]
                        poses_path = path.with_suffix(".camera.npz")
                        np.savez_compressed(poses_path, timestamps=self.times, world_from_camera=self.poses[camera])
                        record_video(self.task, "robot", camera, path, kind="simulation_render", run=self.run,
                                     fps=self.fps, frames=len(self.times), camera_poses=str(poses_path.relative_to(self.task.root)))
                except Exception as exc:
                    errors.append(exc)
            for camera, writer in getattr(self, "tactile_writers", {}).items():
                try:
                    writer.close()
                    if register and self.times:
                        from .tactile import FORCE_SCALE_N, SIGNAL_DESCRIPTION
                        path = self.tactile_paths[camera]
                        signal_path = path.with_suffix(".tactile.npz")
                        np.savez_compressed(signal_path, timestamps=self.times,
                            world_from_camera=self.poses[camera], link_names=self.tactile_names,
                            force_n=self.tactile_values, force_color_max_n=FORCE_SCALE_N,
                            signal_description=SIGNAL_DESCRIPTION)
                        record_video(self.task, "robot", camera, path, kind="simulated_tactile_render", run=self.run,
                            fps=self.fps, frames=len(self.times), tactile_signals=str(signal_path.relative_to(self.task.root)),
                            signal_description=SIGNAL_DESCRIPTION, force_color_max_n=FORCE_SCALE_N)
                except Exception as exc:
                    errors.append(exc)
            for camera, writer in getattr(self, "human_writers", {}).items():
                try:
                    writer.close()
                    if register and self.times:
                        path = self.human_paths[camera]
                        signal_path = path.with_suffix(".keypoints.npz")
                        points = {f"{side}_keypoints_world": np.asarray([frame[side] for frame in self.human_values])
                                  for side in self.task.hands}
                        np.savez_compressed(signal_path, timestamps=self.times,
                            world_from_camera=self.poses[camera], **points)
                        record_video(self.task, "robot", camera, path, kind="human_keypoint_render", run=self.run,
                            fps=self.fps, frames=len(self.times), keypoints=str(signal_path.relative_to(self.task.root)),
                            label="Green markers: calibrated source annotations on the retimed replay clock")
                except Exception as exc:
                    errors.append(exc)
        finally:
            if self.renderer is not None:
                self.renderer.close()
        if errors:
            raise errors[0]
