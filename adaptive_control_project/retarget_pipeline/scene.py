"""Load the robot, table, and freely dynamic task object into SuperDex.

The active DexYCB task passes assets/scene.json: its object is assets/mug.stl,
with an explicit initial pose and configured 0.118 kg mass. Support geometry
is procedural. The built-in paper-cup defaults are used by isolated tests.
The pose controller follows joint targets; gravity and contact move the object.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import superdex.physics as sdp
import superdex.robotics as sdr
from superdex.physics.paths import get_assets_root, resolve_asset

from .paths import BOT_ASSET, CUP_PREFAB


def _box_tri_mesh(size, center):
    sx, sy, sz = (np.asarray(size, dtype=float) / 2.0)
    cx, cy, cz = center
    V = np.array([[cx - sx, cy - sy, cz - sz], [cx + sx, cy - sy, cz - sz], [cx + sx, cy + sy, cz - sz], [cx - sx, cy + sy, cz - sz],
                  [cx - sx, cy - sy, cz + sz], [cx + sx, cy - sy, cz + sz], [cx + sx, cy + sy, cz + sz], [cx - sx, cy + sy, cz + sz]], dtype=np.float32)
    F = np.array([(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)], dtype=np.int32)
    return V, F


@dataclass
class SceneConfig:
    table_top: float = 0.32
    table_size: tuple = (0.70, 1.10, 0.03)
    table_center_xy: tuple = (0.50, 0.0)
    cup_xy: tuple = (0.42, -0.22)
    cup_yaw: float = 0.0
    object_kind: str = "paper_cup"        # "paper_cup" (SuperDex prefab) | "mesh" (STL/OBJ via load_shape_from_file)
    object_mesh: str = ""                 # path for object_kind == "mesh"
    object_mass: float = 0.118            # [kg] for object_kind == "mesh" (YCB mug: 118 g)
    object_bottom_offset: float = 0.0     # distance from the mesh origin down to its bottom [m] (mesh objects)
    object_rotation_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)   # full initial rotation for mesh objects (overrides cup_yaw)
    object_translation: tuple | None = None
    object_color: tuple = (70, 140, 220, 255)
    static_props: list = field(default_factory=list)
    enforce_effort_limits: bool = False
    effort_elastic_fraction: float = 0.8
    cup_scale: float = 1.0                # uniform scale of the paper-cup prefab
    arm_stiffness: float = 1000.0
    arm_damping: float = 60.0
    hand_stiffness: float = 25.0
    hand_damping: float = 1.0
    dt: float = 1.0 / 200.0
    gravity: float = -9.81
    num_threads: int = 0

    def __post_init__(self):
        if not np.isfinite(self.dt) or self.dt <= 0 or self.object_mass <= 0:
            raise ValueError("Physics timestep and dynamic object mass must be positive")
        if self.enforce_effort_limits and (self.hand_stiffness <= 0 or self.arm_stiffness <= 0 or not 0 < self.effort_elastic_fraction <= 1):
            raise ValueError("Effort-limited pose tracking needs positive stiffness and an elastic fraction in (0,1]")


class PickupScene:
    def __init__(self, cfg: SceneConfig, initialize_physics: bool = True):
        self.cfg = cfg
        if initialize_physics:
            sdp.initialize(num_worker_threads=cfg.num_threads)
        self.scene = sdp.create_scene("Cup pick-up replay")
        self.scene.set_gravity([0, 0, cfg.gravity])
        self.ground = self.scene.create_rigid_actor(name="ground", shape=sdp.create_plane_shape(normal=[0, 0, 1], distance=0), is_static=True)
        V, F = _box_tri_mesh(cfg.table_size, (cfg.table_center_xy[0], cfg.table_center_xy[1], cfg.table_top - cfg.table_size[2] / 2))
        self.table = self.scene.create_rigid_actor(name="table", shape=sdp.create_tri_mesh_shape(coordinates=V.flatten(), connectivity=F.flatten()), is_static=True)
        self.table_box = (cfg.table_size, (cfg.table_center_xy[0], cfg.table_center_xy[1], cfg.table_top - cfg.table_size[2] / 2))
        # bot
        self.prefab = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
        self.ctx = sdr.create_context()
        self.bot = sdr.create_bot(self.scene, self.prefab, self.ctx)
        self.actor = self.bot.get_articulated_actor()
        self.link_names = [l.name for l in self.prefab.links]
        self.num_dofs = self.actor.get_num_dofs()
        self.revolute_joint_names = [j.name for j in self.prefab.joints if j.type == sdp.ArticulatedJointType.REVOLUTE]
        assert len(self.revolute_joint_names) == self.num_dofs
        self._install_pose_controller()
        self.actor.register_query(sdp.QueryType.ARTICULATED_CONTROLLER_FORCE)
        self.rest_pose = np.array(self.prefab.default_pose, dtype=np.float64)
        self._target = sdp.DynamicArrayReal(self.num_dofs)
        self._pose = sdp.DynamicArrayReal(self.num_dofs)
        self._T = sdp.DynamicArrayTransformRT(len(self.link_names))
        self.static_props = []
        for prop in cfg.static_props:
            vertices, faces = _box_tri_mesh(prop["size"], prop["position"])
            actor = self.scene.create_rigid_actor(name=prop["name"],
                shape=sdp.create_tri_mesh_shape(coordinates=vertices.flatten(), connectivity=faces.flatten()), is_static=True)
            self.static_props.append((actor, prop))
        # object
        if cfg.object_kind == "paper_cup":
            res = sdp.prefab.add_to_scene(prefab_path=str(resolve_asset(CUP_PREFAB)), root_path=str(get_assets_root()), scene=self.scene,
                                          params=sdp.prefab.PrefabParams(name="cup", scale=cfg.cup_scale, rotation=sdp.Quaternion.rotation_z(cfg.cup_yaw),
                                                                         translation=[cfg.cup_xy[0], cfg.cup_xy[1], cfg.table_top + 0.001]))
            self.cup = list(res.actors)[0]
            self.cup_bottom_offset = 0.0
        else:
            shape = sdp.load_shape_from_file(cfg.object_mesh)
            self.cup = self.scene.create_rigid_actor(
                name="cup", shape=shape, mass=cfg.object_mass,
                world_from_local=sdp.TransformRT(rotation=list(cfg.object_rotation_xyzw),
                                                 translation=list(cfg.object_translation) if cfg.object_translation is not None else
                                                 [cfg.cup_xy[0], cfg.cup_xy[1], cfg.table_top + cfg.object_bottom_offset + 0.001]))
            self.cup_bottom_offset = cfg.object_bottom_offset
        self.cup_height = float(np.asarray(self.cup.get_aabb_local().max)[2] - np.asarray(self.cup.get_aabb_local().min)[2])
        self.cup_radius = float(np.asarray(self.cup.get_aabb_local().max)[0])
        self.cup.register_query(sdp.QueryType.CONTACT_POINTS)
        self.time = 0.0

    # ---- control --------------------------------------------------------------------------------
    def _install_pose_controller(self):
        cfg = self.cfg
        n_links = len(self.prefab.links)
        pc = sdp.PoseControllerParams(n_links)
        # joint_tracking is indexed by link (the link's parent joint).  Hand links are the ones whose
        # parent joint name starts with "<side>_joint_"; everything else revolute is an arm joint.
        arr = sdp.DynamicArrayPoseTrackingParams(n_links)
        for i, j in enumerate(self.prefab.joints):
            if j.type != sdp.ArticulatedJointType.REVOLUTE:
                arr[i] = sdp.PoseTrackingParams(stiffness=0.0, damping=0.0)
            elif j.name.startswith(("left_joint_", "right_joint_")):
                saturation = cfg.effort_elastic_fraction * j.effort_limit / cfg.hand_stiffness if cfg.enforce_effort_limits else -1
                arr[i] = sdp.PoseTrackingParams(stiffness=cfg.hand_stiffness, damping=cfg.hand_damping, saturation=saturation)
            else:
                saturation = cfg.effort_elastic_fraction * j.effort_limit / cfg.arm_stiffness if cfg.enforce_effort_limits else -1
                arr[i] = sdp.PoseTrackingParams(stiffness=cfg.arm_stiffness, damping=cfg.arm_damping, saturation=saturation)
        pc.joint_tracking = arr
        self.actor.add_articulated_pose_controller(pc)

    def set_target(self, pose: np.ndarray):
        for i, v in enumerate(pose):
            self._target[i] = float(v)
        self.actor.set_articulated_target_pose(self._target)

    def step(self, target_pose: np.ndarray | None = None):
        if target_pose is not None:
            self.set_target(target_pose)
        self.scene.step(self.cfg.dt)
        self.time += self.cfg.dt

    # ---- state -----------------------------------------------------------------------------------
    def joint_positions(self) -> np.ndarray:
        self.actor.get_articulated_pose(self._pose)
        return np.array(self._pose, dtype=np.float64)

    def joint_velocities(self) -> np.ndarray:
        v = sdp.DynamicArrayReal(self.num_dofs)
        self.actor.get_articulated_joint_velocities(v)
        return np.array(v, dtype=np.float64)

    def controller_efforts(self) -> np.ndarray:
        if self.time == 0.0:
            return np.zeros(self.num_dofs)
        return np.array(self.actor.get_articulated_controller_force(), dtype=np.float64)

    def link_transforms(self):
        """list of (translation(3,), quat_xyzw(4,)) per link, world frame."""
        self.actor.get_articulated_link_transforms(self._T)
        return [(np.asarray(self._T[i].translation, dtype=float), np.asarray(self._T[i].rotation, dtype=float)) for i in range(len(self.link_names))]

    def link_index(self, name: str) -> int:
        return self.link_names.index(name)

    def cup_pose(self):
        T = self.cup.get_root_transform()
        return np.asarray(T.translation, dtype=float), np.asarray(T.rotation, dtype=float)

    def cup_velocity(self):
        return np.asarray(self.cup.get_linear_velocity(), dtype=float)

    def hand_cup_contact(self, side: str):
        """Contact between the object and the hand links, from the object's CONTACT_POINTS query:
        returns (force_per_link {link: |F| [N]}, max_penetration [m], total_force_N on the object from
        the hand)."""
        # SuperDex only populates registered contact queries after a physics step.
        # The initial replay sample records the untouched scene at t=0.
        if self.time == 0.0:
            return {}, 0.0, 0.0
        forces = {}
        pen = 0.0
        tot = np.zeros(3)
        cup_name = self.cup.get_name()
        for cp in self.cup.get_contact_points_world():
            other = cp.actor_b if self.scene.get_actor(cp.actor_a).get_name() == cup_name else cp.actor_a
            name = self.scene.get_actor(other).get_name().split("/")[-1]
            if not name.startswith(f"{side}_link_"):
                continue
            f = np.asarray(cp.force, dtype=float)
            forces[name] = forces.get(name, 0.0) + float(np.linalg.norm(f))
            tot += f
            pen = max(pen, -float(cp.distance))
        return forces, pen, float(np.linalg.norm(tot))

    def destroy(self, shutdown: bool = True):
        sdr.destroy_bot(self.scene, self.bot)
        if shutdown:
            sdp.shutdown()
