/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <mochi_physics/mochi_physics.h>

#include <mochi_core/utils/dynamic_array.h>
#include <mochi_core/utils/dynamic_string.h>
#include <mochi_core/utils/span.h>

#include <limits>

// Builders of articulated actors shared by tests.
namespace mochi::test {

/**************************************************************************************************
  Three-bone skinned actors: a Hard -> Spherical -> Spherical skeleton of massless, gravity-free
  unit cubes without colliders, with links offset by 1 along x and y, and a unit-cube skin or nested
  soft actor skinned to the three bones.
*/

// Shape of a unit tet cube skinned to three bones. Bone 0 is static. Nodes with y > 0.5 have no
// weight on bone 2, so the samples of the cube face y = 1 depend on bone 1 only, and the samples of
// other faces depend on bones 1 and 2. If blendedSoftName is non-empty, the nodes with
// x < blendedMaxX are blended with the soft actor of that name.
ShapeHandle CreateThreeBoneShape(
    Context* context,
    DynamicArray<int> constrainedNodes = {},
    DynamicString const& blendedSoftName = {},
    real blendedMaxX = std::numeric_limits<real>::max());

// Linear ROM version of CreateThreeBoneShape, with node 0 constrained and a single mode that moves
// all other nodes along x.
ShapeHandle CreateThreeBoneTetRomShape(Context* context);

ArticulatedActorParams MakeThreeBoneSkeletonParams(Context* context);

// An articulated actor with the skeleton of MakeThreeBoneSkeletonParams and the skin of
// CreateThreeBoneShape in the "Colliding" layer.
Actor* CreateThreeBoneSkinnedActor(
    Context* context,
    Scene* scene,
    real coulombCoefficient,
    real viscousCoefficient,
    real dampingCoefficient);

// Variants of a soft-skinned actor with the skeleton of MakeThreeBoneSkeletonParams and a nested
// soft actor named "soft":
// - Fom, Rom: the soft actor (shape of CreateThreeBoneShape or CreateThreeBoneTetRomShape) collides
//   in the "Colliding" layer.
// - BlendedFom, BlendedRom: the soft actor does not collide. Instead, the articulated actor has a
//   colliding skin blended with it.
// - PartiallyBlendedFom: like BlendedFom, but only the skin nodes with x < 0.5 are blended.
// - PosedFom: like Fom, but with posed (world-space) gravity and stress of the soft actor.
enum class ThreeBoneSoftSkinnedMode {
  Fom,
  Rom,
  BlendedFom,
  BlendedRom,
  PartiallyBlendedFom,
  PosedFom
};

[[nodiscard]] inline bool UsesRom(ThreeBoneSoftSkinnedMode mode) {
  return mode == ThreeBoneSoftSkinnedMode::Rom || mode == ThreeBoneSoftSkinnedMode::BlendedRom;
}

[[nodiscard]] inline bool UsesBlendedSurface(ThreeBoneSoftSkinnedMode mode) {
  return mode == ThreeBoneSoftSkinnedMode::BlendedFom ||
      mode == ThreeBoneSoftSkinnedMode::BlendedRom ||
      mode == ThreeBoneSoftSkinnedMode::PartiallyBlendedFom;
}

// Creates the soft-skinned actor of the given mode. The soft actor is soft (Young's modulus 1e-10),
// without inertia, and constrained at node 0. Contact coefficients apply to the colliding surface.
Actor* CreateThreeBoneSoftSkinnedActor(
    Context* context,
    Scene* scene,
    ThreeBoneSoftSkinnedMode mode,
    TransformRT const& worldFromRoot,
    real coulombCoefficient,
    real viscousCoefficient,
    real dampingCoefficient);

/**************************************************************************************************
  Chains of unit cubes
*/

// A chain of box-colliding unit cubes in the "Articulated" layer, with the given joint types, all
// with axis x. Joint 0 is offset by (1, 1, 1) from the root, and the other links are co-located.
// Non-free joints get the given inertia, viscous damping and Coulomb friction, if positive.
Actor* CreateUnitCubeChain(
    Scene* scene,
    ShapeHandle cubeShape,
    Span<ArticulatedJointType const> joints,
    real dampingCoeff,
    real inertiaCoeff,
    real frictionCoeff);

} // namespace mochi::test
