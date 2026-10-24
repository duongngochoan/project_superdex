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

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import superdex.physics as sdp
from scipy.spatial.transform import Rotation
from superdex.physics.utils.coordinate_systems import (
    COORDINATE_SYSTEMS,
    CoordinateTransform,
)
from superdex.physics.viewer.unrealcv.updaters.unrealcv_articulated_actor_updater import (
    UnrealCVArticulatedActorUpdater,
)

LINK_NAMES = ["base", "mochi_only", "finger", "tip"]


class _Articulation:
    """A chain base -> mochi_only -> finger -> tip with fixed world transforms."""

    def __init__(
        self,
        world: list[sdp.TransformRT],
        root: sdp.TransformRT,
        link_names: tuple[str, ...] = tuple(LINK_NAMES),
        parents: tuple[int, ...] = (-1, 0, 1, 2),
    ) -> None:
        self._world = world
        self._root = root
        self._link_names = list(link_names)
        self._parents = list(parents)

    def get_name(self) -> str:
        return "robot"

    def get_articulated_shape_info(self) -> SimpleNamespace:
        return SimpleNamespace(link_names=self._link_names, parents=self._parents)

    def get_articulated_link_transforms(self, out: sdp.DynamicArrayTransformRT) -> None:
        for index, transform in enumerate(self._world):
            out[index] = transform

    def get_root_transform(self) -> sdp.TransformRT:
        return self._root


class _LinkTransforms:
    """Stands in for the link transform array, so one link can hold a broken value."""

    def __init__(self, transforms: list[object]) -> None:
        self._transforms = transforms

    def __setitem__(self, index: int, value: object) -> None:
        pass

    def tolist(self) -> list[object]:
        return list(self._transforms)


def _random_transform(seed: int) -> sdp.TransformRT:
    rng = np.random.default_rng(seed)
    return sdp.TransformRT(
        translation=sdp.Real3(rng.normal(size=3)),
        rotation=sdp.Quaternion(Rotation.random(random_state=seed).as_quat()),
    )


class TestUnrealCVArticulatedActorUpdater(unittest.TestCase):
    def test_bone_transforms_match_the_per_bone_parent_walk(self) -> None:
        world = [_random_transform(seed) for seed in range(len(LINK_NAMES))]
        root = _random_transform(len(LINK_NAMES))
        updater = UnrealCVArticulatedActorUpdater(
            _Articulation(world, root),
            "BP_Robot",
            client=None,
            coordinate_transform=CoordinateTransform(
                COORDINATE_SYSTEMS["mochi"], COORDINATE_SYSTEMS["unreal"]
            ),
            ue_bone_names={"base", "finger", "tip"},
        )
        bone_frame = CoordinateTransform(
            COORDINATE_SYSTEMS["ros"], COORDINATE_SYSTEMS["unreal"]
        )
        basis = bone_frame.source_to_target[:3, :3]
        sign = -1 if bone_frame.encodes_reflection else 1
        # mochi_only has no UE bone, so finger is posed relative to base.
        ue_parents = [root, world[0], world[0], world[2]]

        _, bones = updater.get_all_bone_transform_data()

        self.assertEqual(LINK_NAMES, [bone for _, bone, _, _ in bones])
        for (_, bone, position, rotation), parent, child in zip(
            bones, ue_parents, world
        ):
            relative = parent.inverse() * child
            quat = np.array(relative.rotation)
            with self.subTest(bone=bone):
                np.testing.assert_array_equal(
                    position,
                    (basis @ np.array(relative.translation) * 100.0).astype(np.float64),
                )
                np.testing.assert_array_equal(
                    rotation, np.append(basis @ (sign * quat[:3]), quat[3])
                )

    def test_a_bone_whose_transform_fails_is_skipped_not_the_actor(self) -> None:
        world = [_random_transform(seed) for seed in range(len(LINK_NAMES))]
        updater = UnrealCVArticulatedActorUpdater(
            _Articulation(world, _random_transform(len(LINK_NAMES))),
            "BP_Robot",
            client=None,
            coordinate_transform=CoordinateTransform(
                COORDINATE_SYSTEMS["mochi"], COORDINATE_SYSTEMS["unreal"]
            ),
            ue_bone_names={"base", "finger", "tip"},
        )
        # The tip's transform cannot be composed with its parent's.
        broken = _LinkTransforms([*world[:3], object()])

        with (
            patch.object(updater, "_link_transforms", broken),
            self.assertLogs(level="WARNING") as logs,
        ):
            _, bones = updater.get_all_bone_transform_data()

        self.assertEqual(LINK_NAMES[:3], [bone for _, bone, _, _ in bones])
        self.assertIn("Skipping bone tip", logs.output[0])

    def test_an_actor_without_bones_has_no_bone_transforms(self) -> None:
        updater = UnrealCVArticulatedActorUpdater(
            _Articulation([], _random_transform(0), link_names=(), parents=()),
            "BP_Robot",
            client=None,
            coordinate_transform=CoordinateTransform(
                COORDINATE_SYSTEMS["mochi"], COORDINATE_SYSTEMS["unreal"]
            ),
            ue_bone_names=set(),
        )

        _, bones = updater.get_all_bone_transform_data()

        self.assertEqual([], bones)
