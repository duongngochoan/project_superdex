# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Articulated actor updater for UnrealCV-based scene synchronization.

This class handles the updating of mochi articulated actors (robots) via UnrealCV,
using get_articulated_link_transforms to efficiently fetch all link transforms,
then computing parent-relative transforms for each bone.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import numpy as np
import numpy.typing as npt
import superdex.physics as sdp
from scipy.spatial.transform import Rotation
from superdex.physics.utils.coordinate_systems import (
    COORDINATE_SYSTEMS,
    CoordinateTransform,
)

from .unrealcv_updater import UnrealCVUpdater

if TYPE_CHECKING:
    from ..unrealcv_client import UnrealCVClient

logger = logging.getLogger(__name__)


class UnrealCVArticulatedActorUpdater(UnrealCVUpdater):
    """
    Updater for mochi articulated actors (robots) via UnrealCV.

    This updater gets the world transforms of each link from the scene,
    computes parent-relative transforms, and sends both position and rotation
    updates to Unreal Engine for each bone.
    """

    ####################################################################################
    # Members
    ####################################################################################

    _actor: sdp.Actor
    """The mochi articulated actor being updated."""

    _ue_skeletal_mesh_actor_name: str
    """Name of the UE actor containing the skeletal/poseable mesh."""

    _link_to_bone_mapping: dict[str, str]
    """Mapping from mochi link names to UE bone names."""

    _meters_to_cm: float
    """Scale factor for converting meters to centimeters."""

    _parents: list[int]
    """Parent index for each link (-1 for root links)."""

    _bone_coordinate_transform: CoordinateTransform

    _ue_bone_names: set[str]
    """Set of bone names that exist in the UE posable mesh skeleton."""

    _hack_fixup_root_xform: bool
    """If True, apply additional root transform fixup for Allegro robot orientation."""

    ####################################################################################
    # Constructor
    ####################################################################################

    def __init__(
        self,
        actor: sdp.Actor,
        ue_skeletal_mesh_actor_name: str,
        client: "UnrealCVClient",
        coordinate_transform: CoordinateTransform,
        link_to_bone_mapping: dict[str, str] | None = None,
        meters_to_cm: float = 100.0,
        ue_bone_names: set[str] | None = None,
        hack_fixup_root_xform: bool = False,
    ):
        """
        Initialize the articulated actor updater.

        Args:
            actor: The mochi articulated actor to update.
            ue_skeletal_mesh_actor_name: Name of the UE actor containing the skeletal mesh.
            client: Reference to the UnrealCV client.
            coordinate_transform: Converter for coordinate system transformations.
            link_to_bone_mapping: Optional mapping from mochi link names to UE bone names.
                                  If not provided, link names are used directly.
            meters_to_cm: Scale factor for converting meters to centimeters.
            ue_bone_names: Optional set of bone names in the UE skeleton. Used to
                           handle hierarchy mismatches between mochi and UE. If not
                           provided, all mochi link bone names are assumed to exist
                           in UE (original behavior).
            hack_fixup_root_xform: If True, apply additional root transform fixup for
                              Allegro robot orientation. Defaults to False.
        """
        # So, this is an annoying hack to get the articulated meshes rendering
        # properly in UE.  ROS -> UE is just a handedness swap, but
        # the other world transforms are clearly in "mochi" space for our test level
        # This needs to be revisited and really figured out.
        self._bone_coordinate_transform = CoordinateTransform(
            COORDINATE_SYSTEMS["ros"], COORDINATE_SYSTEMS["unreal"]
        )
        super().__init__(
            name=actor.get_name(),
            ue_actor_name=ue_skeletal_mesh_actor_name,
            client=client,
            coordinate_transform=coordinate_transform,
        )

        self._actor = actor
        self._ue_skeletal_mesh_actor_name = ue_skeletal_mesh_actor_name
        self._link_to_bone_mapping = link_to_bone_mapping or {}
        self._meters_to_cm = meters_to_cm

        # Get shape info for link names, parent indices, and count
        shape_info = actor.get_articulated_shape_info()
        self._link_names = list(shape_info.link_names)
        self._parents = list(shape_info.parents)
        # Pre-allocate transform array for get_articulated_link_transforms
        self._link_transforms = sdp.DynamicArrayTransformRT(len(self._link_names))

        # Build set of UE bone names for hierarchy mismatch handling.
        # If ue_bone_names was provided (e.g. queried from UE), use it.
        # Otherwise fall back to the set of all mapped mochi link names.
        self._bone_names = [
            self._link_to_bone_mapping.get(name, name) for name in self._link_names
        ]
        if ue_bone_names is not None:
            self._ue_bone_names = ue_bone_names
        else:
            self._ue_bone_names = set(self._bone_names)
        # The hierarchy is static, so each bone's effective UE parent is resolved
        # once rather than walked on every update.
        self._ue_parents = [
            self._closest_ue_ancestor(link_idx)
            for link_idx in range(len(self._link_names))
        ]

        self._hack_fixup_root_xform = hack_fixup_root_xform

        logger.debug(
            f"Created articulated updater for '{actor.get_name()}' "
            f"with {len(self._link_transforms)} links, "
            f"{len(self._ue_bone_names)} UE bones"
        )

    ####################################################################################
    # Update Methods
    ####################################################################################

    def get_all_bone_transform_data(
        self,
    ) -> tuple[
        tuple[str, npt.NDArray[np.floating], npt.NDArray[np.floating]],
        list[
            tuple[str, str, npt.NDArray[np.floating] | None, npt.NDArray[np.floating]]
        ],
    ]:
        """
        Get all bone transform data for batched updates.

        Returns:
            Tuple of:
            - (ue_actor_name, actor_position_ue, actor_rotation_quat_ue): The actor
              root transform that positions the entire skeletal mesh in world space
            - List of tuples (ue_actor_name, bone_name, position, rotation_quat)
              for each link with valid transforms. All bone transforms are in
              bone-local (parent-relative) space.
        """
        if not self._is_visible:
            return (
                (
                    self._ue_skeletal_mesh_actor_name,
                    np.zeros(3),
                    np.array([0, 0, 0, 1]),
                ),
                [],
            )

        # Compute bone transforms
        actor_root_transform, bone_transforms = self._compute_bone_transforms()
        actor_root_pos, actor_root_quat = actor_root_transform

        # Build list of transform data (all bones are now local transforms)
        bone_list = [
            (self._ue_skeletal_mesh_actor_name, bone_name, *transform)
            for bone_name, transform in zip(self._bone_names, bone_transforms)
            if transform is not None
        ]

        return (
            (self._ue_skeletal_mesh_actor_name, actor_root_pos, actor_root_quat),
            bone_list,
        )

    def remove(self):
        """Hide the actor in Unreal Engine."""
        self.set_visible(False)

    ####################################################################################
    # Bone Transform Computation
    ####################################################################################

    def _compute_bone_transforms(
        self,
    ) -> tuple[
        tuple[npt.NDArray[np.floating], npt.NDArray[np.floating]],
        list[tuple[npt.NDArray[np.floating], npt.NDArray[np.floating]] | None],
    ]:
        """
        Compute bone transforms relative to their parent bones.

        Uses get_articulated_link_transforms to efficiently get all link world transforms
        in a single call, then computes each bone's transform relative to its
        effective UE parent (see ``_closest_ue_ancestor``) and converts all of
        them to UE coordinates in one batch.

        Returns:
            Tuple of:
            - (actor_position_ue, actor_rotation_ue): The root transform for the UE actor
            - List of (position, quaternion) tuples for each link relative to parent.
        """
        # Get all link world transforms in a single call
        self._actor.get_articulated_link_transforms(self._link_transforms)

        # Get the articulated actor's root transform (this is the base link in world space)
        actor_root = self._actor.get_root_transform()

        # Convert actor root to UE coordinates (decompose only for the final output)
        # Note: actor_root.translation is Real3 and actor_root.rotation is Quaternion,
        # so we convert them to numpy arrays for the coordinate transform functions
        actor_root_pos_ue = self._convert_position_to_ue(
            np.array(actor_root.translation)
        )
        actor_root_quat_ue = self._convert_rotation_to_ue(np.array(actor_root.rotation))

        world_transforms = self._link_transforms.tolist()
        relative_transforms: list[sdp.TransformRT | None] = []
        for bone_name, parent, child in zip(
            self._bone_names, self._ue_parents, world_transforms
        ):
            # An out-of-range parent is a structural bug, so only the transform math is guarded.
            parent_transform = world_transforms[parent] if parent >= 0 else actor_root
            try:
                relative_transforms.append(parent_transform.inverse() * child)
            except Exception as e:  # noqa: BLE001 - one bad bone must not drop the actor
                logger.warning(f"Skipping bone {bone_name}: {e}")
                relative_transforms.append(None)
        valid = [tf for tf in relative_transforms if tf is not None]
        if not valid:
            return (actor_root_pos_ue, actor_root_quat_ue), [None] * len(
                relative_transforms
            )
        positions = self._bone_coordinate_transform.position_to_target(
            np.array([tf.translation for tf in valid]),
            scale=self._meters_to_cm,
        ).astype(np.float64)
        rotations = self._bone_coordinate_transform.rotation_to_target(
            np.array([tf.rotation for tf in valid])
        )
        converted = iter(zip(positions, rotations))
        return (actor_root_pos_ue, actor_root_quat_ue), [
            None if tf is None else next(converted) for tf in relative_transforms
        ]

    def _closest_ue_ancestor(self, link_idx: int) -> int:
        """
        Walk up the mochi parent chain from link_idx to the closest ancestor
        whose UE bone name exists in the UE skeleton.

        If the mochi hierarchy has extra bones that don't exist in UE, this skips
        them, so each bone's relative transform is computed against the parent
        UE actually uses.

        Args:
            link_idx: The index of the link whose parent we're looking for.

        Returns:
            The ancestor's link index, or -1 when the actor root is the parent.
        """
        current_idx = self._parents[link_idx]
        while 0 <= current_idx < len(self._link_names):
            if self._bone_names[current_idx] in self._ue_bone_names:
                return current_idx
            current_idx = self._parents[current_idx]
        return -1

    def _convert_position_to_ue(
        self, position: npt.NDArray[np.floating]
    ) -> npt.NDArray[np.floating]:
        """
        Convert a position from mochi coordinates to UE coordinates.

        Args:
            position: Position in mochi coordinates (meters).

        Returns:
            Position in UE coordinates (centimeters).
        """
        position_ue = self._coordinate_transform.position_to_target(
            position, scale=self._meters_to_cm
        )
        return position_ue.astype(np.float64)

    def _convert_rotation_to_ue(
        self, quat_mochi: npt.NDArray[np.floating]
    ) -> npt.NDArray[np.floating]:
        """
        Convert a quaternion from mochi coordinates to UE coordinates.

        Args:
            quat_mochi: Quaternion in mochi coordinates (x, y, z, w).

        Returns:
            Quaternion in UE coordinates (x, y, z, w).
        """
        # Use CoordinateTransform for proper quaternion conversion
        quat_ue = self._coordinate_transform.rotation_to_target(quat_mochi)

        if self._hack_fixup_root_xform:
            # Apply additional root transform fixup for Allegro robot orientation.
            # This is related to the bone space orientation problem.
            rot_ue = Rotation.from_quat(quat_ue)
            additional_rot = Rotation.from_euler("x", -90, degrees=True)
            additional_rot2 = Rotation.from_euler("y", -90, degrees=True)

            quat_ue = (rot_ue * additional_rot * additional_rot2).as_quat()

        return quat_ue

    ####################################################################################
    # Actor Properties
    ####################################################################################

    def get_actor(self) -> sdp.Actor:
        """Returns the mochi actor."""
        return self._actor

    def get_num_links(self) -> int:
        """Returns the number of links in the articulated actor."""
        return len(self._link_transforms)

    def get_link_names(self) -> list[str]:
        """Returns the list of link names."""
        return self._link_names

    def get_aabb(self):
        """
        Returns the axis-aligned bounding box of this articulated actor in world
        coordinates.

        Uses the mochi actor's get_aabb_world() for efficient AABB computation
        directly from the physics engine, which includes all links.
        """
        from superdex.physics.viewer.utils.aabb import AABB

        try:
            mochi_aabb = self._actor.get_aabb_world()
            aabb = AABB(
                min=np.array([mochi_aabb.min[0], mochi_aabb.min[1], mochi_aabb.min[2]]),
                max=np.array([mochi_aabb.max[0], mochi_aabb.max[1], mochi_aabb.max[2]]),
            )
            return aabb
        except Exception:
            return AABB.empty()
