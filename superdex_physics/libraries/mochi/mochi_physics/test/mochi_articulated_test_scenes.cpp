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

#include "mochi_articulated_test_scenes.h"

#include "mochi_physics_test_fixture.h"

#include <mochi_core/geometry/model_data.h>
#include <mochi_core/geometry/tetrahedral_mesh.h>
#include <mochi_physics/mochi_physics_experimental.h>
#include <mochi_physics/src/mochi_context.h>
#include <mochi_physics/src/mochi_shape.h>

#include <memory>
#include <utility>

namespace mochi::test {

namespace {
SkinningData MakeThreeBoneSkinning(Span<Real3 const> coordinates) {
  int const numNodes = isize(coordinates);
  SkinningData skinning;
  skinning.weightsPerNode = 3;
  skinning.indices.reserve(3 * numNodes);
  skinning.weights.reserve(3 * numNodes);
  for (int node = 0; node < numNodes; ++node) {
    skinning.indices.push_back(0);
    skinning.indices.push_back(1);
    skinning.indices.push_back(2);
    skinning.weights.push_back(0.2_r);
    real const weight1 = coordinates[node][1] > 0.5_r ? 0.8_r : (node % 2 == 0 ? 0.3_r : 0.5_r);
    skinning.weights.push_back(weight1);
    skinning.weights.push_back(0.8_r - weight1);
  }
  return skinning;
}
} // namespace

ShapeHandle CreateThreeBoneShape(
    Context* context,
    DynamicArray<int> constrainedNodes,
    DynamicString const& blendedSoftName,
    real blendedMaxX) {
  auto&& [coordinates, connectivity] = CreateMinimalTetMeshUnitCube();
  auto mesh = std::make_shared<TetrahedralMesh const>(coordinates, connectivity);
  int const numNodes = mesh->GetNumNodes();
  auto skinning =
      std::make_shared<SkinningData const>(MakeThreeBoneSkinning(MakeConstSpan(coordinates)));

  std::shared_ptr<ConstrainedNodesData const> constrainedNodesData;
  if (!constrainedNodes.empty()) {
    constrainedNodesData =
        std::make_shared<ConstrainedNodesData const>(std::move(constrainedNodes));
  }

  std::shared_ptr<BlendingDataMap> blending;
  if (!blendedSoftName.empty()) {
    BlendingDataTargetMesh target;
    target.indices.resize(numNodes);
    target.weights.resize(numNodes, 0_r);
    for (int node = 0; node < numNodes; ++node) {
      target.indices[node] = node;
      if (coordinates[node][0] < blendedMaxX) {
        target.weights[node] = 0.4_r;
      }
    }
    blending = std::make_shared<BlendingDataMap>();
    blending->perSourceShapeData.emplace(blendedSoftName, std::move(target));
  }

  auto shape =
      std::make_shared<TetrahedralMeshShape>(mesh, skinning, constrainedNodesData, blending);
  return assert_cast<ContextImpl*>(context)->RegisterShape(shape, ExpectOK{});
}

ShapeHandle CreateThreeBoneTetRomShape(Context* context) {
  auto&& [coordinates, connectivity] = CreateMinimalTetMeshUnitCube();
  ModelData model;
  model.mesh.emplace();
  model.mesh->nodesPerElement = 4;
  model.mesh->coordinates = Flatten(MakeSpan(coordinates));
  model.mesh->connectivity = Flatten(MakeSpan(connectivity));
  model.mesh->skinning = MakeThreeBoneSkinning(MakeConstSpan(coordinates));
  model.constrainedNodes = DynamicArray<int>{0};

  DynamicArray<real> basis(model.mesh->coordinates.size(), 0_r);
  for (int node = 1; node < model.mesh->GetNumNodes(); ++node) {
    basis[3 * node] = 1_r;
  }
  return experimental::CreateModelShapeWithLinearRom(
      context, model, "contact_test", MakeConstSpan(basis), ExpectOK{});
}

ArticulatedActorParams MakeThreeBoneSkeletonParams(Context* context) {
  ArticulatedActorParams params;
  params.joints = {
      {.name = "joint_0", .type = ArticulatedJointType::Hard},
      {.name = "joint_1",
       .type = ArticulatedJointType::Spherical,
       .parentLinkFromJoint = TransformRT{Real3{1_r, 0_r, 0_r}}},
      {.name = "joint_2",
       .type = ArticulatedJointType::Spherical,
       .parentLinkFromJoint = TransformRT{Real3{0_r, 1_r, 0_r}}}};
  ShapeHandle const linkShape = CreateUnitCubeTetMeshShape(context);
  params.links = {
      {.name = "link_0",
       .parentLink = -1,
       .shape = linkShape,
       .colliderType = ColliderType::None,
       .hasGravity = false,
       .density = 1e-10_r},
      {.name = "link_1",
       .parentLink = 0,
       .shape = linkShape,
       .colliderType = ColliderType::None,
       .hasGravity = false,
       .density = 1e-10_r},
      {.name = "link_2",
       .parentLink = 1,
       .shape = linkShape,
       .colliderType = ColliderType::None,
       .hasGravity = false,
       .density = 1e-10_r}};
  return params;
}

Actor* CreateThreeBoneSkinnedActor(
    Context* context,
    Scene* scene,
    real coulombCoefficient,
    real viscousCoefficient,
    real dampingCoefficient) {
  ArticulatedActorParams params = MakeThreeBoneSkeletonParams(context);
  ArticulatedSkinParams skinParams;
  skinParams.shape = CreateThreeBoneShape(context);
  skinParams.layer = "Colliding";
  skinParams.contact.coulombFrictionCoefficient = coulombCoefficient;
  skinParams.contact.viscousFrictionCoefficient = viscousCoefficient;
  skinParams.contact.normalViscousDampingCoefficient = dampingCoefficient;
  params.skin = std::move(skinParams);
  return scene->CreateArticulatedActor(params, ExpectOK{});
}

Actor* CreateThreeBoneSoftSkinnedActor(
    Context* context,
    Scene* scene,
    ThreeBoneSoftSkinnedMode mode,
    TransformRT const& worldFromRoot,
    real coulombCoefficient,
    real viscousCoefficient,
    real dampingCoefficient) {
  bool const isPosed = mode == ThreeBoneSoftSkinnedMode::PosedFom;
  SoftSkinnedActorParams params;
  params.hasGravity = isPosed;
  params.hasStress = isPosed;
  params.skeletonParams = MakeThreeBoneSkeletonParams(context);
  params.skeletonParams.worldFromRoot = worldFromRoot;
  if (UsesBlendedSurface(mode)) {
    ArticulatedSkinParams skinParams;
    skinParams.shape = CreateThreeBoneShape(
        context,
        {},
        "soft",
        mode == ThreeBoneSoftSkinnedMode::PartiallyBlendedFom ? 0.5_r
                                                              : std::numeric_limits<real>::max());
    skinParams.layer = "Colliding";
    skinParams.contact.coulombFrictionCoefficient = coulombCoefficient;
    skinParams.contact.viscousFrictionCoefficient = viscousCoefficient;
    skinParams.contact.normalViscousDampingCoefficient = dampingCoefficient;
    params.skeletonParams.skin = std::move(skinParams);
  }

  SoftActorParams softParams;
  softParams.name = "soft";
  softParams.shape = UsesRom(mode) ? CreateThreeBoneTetRomShape(context)
                                   : CreateThreeBoneShape(context, DynamicArray<int>{0});
  softParams.layer = UsesBlendedSurface(mode) ? "" : "Colliding";
  softParams.hasGravity = false;
  softParams.hasInertia = false;
  softParams.hasStress = !isPosed;
  softParams.material.neoHookean.youngsModulus = 1e-10_r;
  softParams.contact.coulombFrictionCoefficient = coulombCoefficient;
  softParams.contact.viscousFrictionCoefficient = viscousCoefficient;
  softParams.contact.normalViscousDampingCoefficient = dampingCoefficient;
  params.softParams.push_back(std::move(softParams));

  if (!UsesRom(mode)) {
    return scene->CreateSoftSkinnedActor(params, ExpectOK{});
  }
  experimental::ExperimentalSoftSkinnedActorParams experimentalParams;
  experimental::ExperimentalSoftActorParams softExperimentalParams;
  softExperimentalParams.rom.emplace();
  softExperimentalParams.rom->source = "contact_test";
  experimentalParams.softParams.push_back(std::move(softExperimentalParams));
  return experimental::CreateSoftSkinnedActor(scene, params, experimentalParams, ExpectOK{});
}

Actor* CreateUnitCubeChain(
    Scene* scene,
    ShapeHandle cubeShape,
    Span<ArticulatedJointType const> joints,
    real dampingCoeff,
    real inertiaCoeff,
    real frictionCoeff) {
  auto const numBones = isize(joints);
  TransformRT const offset{Real3{1_r, 1_r, 1_r}};

  // Build per-joint friction params (if any)
  ArticulatedJointFrictionParams fp{};
  bool const hasFriction = (dampingCoeff > 0 || frictionCoeff > 0);
  if (dampingCoeff > 0) {
    fp.viscous = dampingCoeff;
  }
  if (frictionCoeff > 0) {
    fp.coulomb = frictionCoeff;
  }

  // Build joints and links
  ArticulatedActorParams params;
  params.joints.resize(numBones);
  params.links.resize(numBones);
  for (int i = 0; i < numBones; ++i) {
    bool const isFreeJoint = (joints[i] == ArticulatedJointType::Free);

    auto& joint = params.joints[i];
    joint.type = joints[i];
    joint.axis = Real3{1_r, 0_r, 0_r};
    // The first link is offset from the root; subsequent links are co-located.
    if (i == 0) {
      joint.parentLinkFromJoint = offset;
    }
    // Broadcast inertia/friction to non-free joints (matches InitJointParams behavior).
    if (inertiaCoeff > 0 && !isFreeJoint) {
      joint.inertia = inertiaCoeff;
    }
    if (hasFriction && !isFreeJoint) {
      joint.friction = fp;
    }

    auto& link = params.links[i];
    link.parentLink = i - 1;
    link.shape = cubeShape;
    link.layer = "Articulated";
    link.colliderType = ColliderType::Box;
  }
  return scene->CreateArticulatedActor(params, ExpectOK{});
}

} // namespace mochi::test
