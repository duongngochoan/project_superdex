"""Offscreen rendering of a SuperDex scene to PNG frames / MP4.

Why a custom renderer: SuperDex Studio and the Physics Debugger need an X11 display and OpenGL 4.1;
this machine is headless.  The bot assets ship PBR GLB render meshes per link, so we draw them with
pyrender under Xvfb (Mesa llvmpipe, CPU only) at the link transforms reported by the physics engine.

GLB meshes are authored Y-up (glTF convention) while SuperDex link frames are Z-up: the loader applies
a fixed +90 deg rotation about X (x, y, z)_zup = (x, -z, y)_glb -- verified against the collision mesh
bounds of `openarm_right_link1` (see docs/CHANGES.md).

Run any script that uses this module with:
    LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1280x720x24" python ...
"""
from __future__ import annotations

import os
import numpy as np
import trimesh

if not os.environ.get("DISPLAY"):
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
import pyrender  # noqa: E402

GLB_TO_ZUP = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])

ARM_COLOR = [190, 194, 200, 255]
HAND_COLOR = [235, 130, 60, 255]
TORSO_COLOR = [120, 124, 130, 255]
CUP_COLOR = [70, 140, 220, 255]
TABLE_COLOR = [205, 180, 140, 255]
GROUND_COLOR = [235, 235, 235, 255]
HUMAN_KP_COLOR = [30, 170, 60, 255]


def _xyzw_to_matrix(t, q):
    q = np.asarray(q, dtype=float)
    M = trimesh.transformations.quaternion_matrix([q[3], q[0], q[1], q[2]])
    M[:3, 3] = np.asarray(t, dtype=float)
    return M


def load_link_mesh(path: str, color) -> trimesh.Trimesh:
    tm = trimesh.load(path, force="mesh", process=False)
    if isinstance(tm, trimesh.Scene):
        tm = tm.dump(concatenate=True)
    tm.apply_transform(GLB_TO_ZUP)
    tm.visual = trimesh.visual.ColorVisuals(tm, face_colors=color)
    return tm


def link_color(link_name: str):
    if "link_" in link_name and ("left_" in link_name or "right_" in link_name):
        return HAND_COLOR
    if "body_link0" in link_name:
        return TORSO_COLOR
    return ARM_COLOR


def look_at(eye, target, up=(0, 0, 1.0)):
    eye, target, up = (np.asarray(v, dtype=float) for v in (eye, target, up))
    z = eye - target
    z /= np.linalg.norm(z)
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    C = np.eye(4)
    C[:3, 0], C[:3, 1], C[:3, 2], C[:3, 3] = x, y, z, eye
    return C


class SceneRenderer:
    """Keeps pyrender nodes for every SuperDex actor and updates their poses each frame."""

    def __init__(self, width=1280, height=720, eye=(1.45, -1.15, 1.05), target=(0.35, -0.05, 0.45), yfov_deg=42):
        self.width, self.height = width, height
        self.scene = pyrender.Scene(bg_color=[1, 1, 1, 1], ambient_light=[0.45, 0.45, 0.45])
        cam = pyrender.PerspectiveCamera(yfov=np.deg2rad(yfov_deg), aspectRatio=width / height)
        self.cam_pose = look_at(eye, target)
        self.camera_node = self.scene.add(cam, pose=self.cam_pose)
        self.scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=3.0), pose=self.cam_pose)
        self.scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=1.5),
                       pose=look_at((-1.0, 1.0, 2.0), (0.3, 0, 0.4)))
        self.nodes = {}
        self.renderer = pyrender.OffscreenRenderer(width, height)

    def add_mesh(self, name: str, tm: trimesh.Trimesh, pose=np.eye(4), smooth=False):
        node = self.scene.add(pyrender.Mesh.from_trimesh(tm, smooth=smooth), pose=np.asarray(pose), name=name)
        self.nodes[name] = node
        return node

    def add_points(self, name: str, pts: np.ndarray, color=HUMAN_KP_COLOR, radius=0.006):
        sph = trimesh.creation.icosphere(subdivisions=1, radius=radius)
        sph.visual = trimesh.visual.ColorVisuals(sph, face_colors=color)
        poses = np.repeat(np.eye(4)[None], len(pts), axis=0)
        poses[:, :3, 3] = pts
        node = self.scene.add(pyrender.Mesh.from_trimesh(sph, poses=poses, smooth=False), name=name)
        self.nodes[name] = node
        return node

    def remove(self, name: str):
        node = self.nodes.pop(name, None)
        if node is not None:
            self.scene.remove_node(node)

    def set_pose(self, name: str, t, q_xyzw):
        self.scene.set_pose(self.nodes[name], _xyzw_to_matrix(t, q_xyzw))

    def render(self) -> np.ndarray:
        color, _ = self.renderer.render(self.scene)
        return color

    def set_camera_pose(self, world_from_camera):
        self.cam_pose = np.asarray(world_from_camera, dtype=float)
        self.scene.set_pose(self.camera_node, self.cam_pose)

    def close(self):
        self.renderer.delete()


def box_mesh(size, center, color):
    tm = trimesh.creation.box(extents=size)
    tm.apply_translation(center)
    tm.visual = trimesh.visual.ColorVisuals(tm, face_colors=color)
    return tm


def annotate(img: np.ndarray, text: str) -> np.ndarray:
    """Burn a one-line caption into the top-left corner of a frame."""
    from PIL import Image, ImageDraw
    im = Image.fromarray(np.ascontiguousarray(img))
    d = ImageDraw.Draw(im)
    d.rectangle([8, 8, 8 + 7 * len(text) + 12, 30], fill=(255, 255, 255))
    d.text((14, 12), text, fill=(20, 20, 20))
    return np.asarray(im)
