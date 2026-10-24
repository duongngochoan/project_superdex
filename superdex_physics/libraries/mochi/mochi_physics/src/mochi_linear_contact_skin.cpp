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

#include "mochi_linear_contact_skin.h"

#include "mochi_deformable.h"
#include "mochi_discretization_functions.h"

#include <mochi_core/geometry/geometry_utils.h>
#include <mochi_core/memory/filo_allocator.h>
#include <mochi_core/utils/profile.h>

using namespace mochi;

// Evaluates the active contact-skin node positions for the given physics-mesh displacements.
static void ComputeNodePositions(
    CContactSkinMesh const& contactSkin,
    CDeformedContactSkinNodes const& deformedNodes,
    ColumnVectorView<real const> displacements,
    Span<Real3> outPositions) {
  MOCHI_ASSERT_VERBOSE(contactSkin.embedding != nullptr, "Contact skin requires an embedding.");
  UpdateLinearEmbeddedNodePositionsFromDisplacements(
      *contactSkin.embedding,
      Unflatten<Real3 const>(MakeConstSpan(deformedNodes.referencePositions)),
      Unflatten<Real3 const>(displacements.GetConstSpan()),
      contactSkin.mesh->GetActiveNodes(),
      outPositions);
}

void mochi::linear_contact_skin::EmplaceComponents(
    entt::registry& reg,
    entt::entity e,
    CFemSurfaceDiscretization const& surfaceDisc,
    TriangularMesh const& contactSkin,
    LinearMeshEmbedding const& embedding,
    Span<Real3 const> physicsNodeCoordinates) {
  auto& skinningData = reg.emplace<CContactSkinningData>(e);
  InitializeLinearContactSkinningJacobian(
      embedding, isize(physicsNodeCoordinates), contactSkin.GetActiveNodes(), skinningData);
  InitializeContactSkinningColumnCoalescingMap(surfaceDisc, skinningData);
  auto& deformedNodes = reg.emplace<CDeformedContactSkinNodes>(e);
  deformedNodes.referencePositions.resize_noinit(kSpaceDim3 * contactSkin.GetNumNodes());
  embedding.Update(
      physicsNodeCoordinates, Unflatten<Real3>(MakeSpan(deformedNodes.referencePositions)));
  deformedNodes.positions.resize(kSpaceDim3 * contactSkin.GetNumActiveNodes());
  reg.emplace<TagUseDeformableContactSkin>(e);
  reg.emplace<CSkinnedContactSnle>(e);
  reg.emplace<TagSkinnedContact>(e);
}

template <TimeStep kTimeStep>
void mochi::linear_contact_skin::UpdateSamples(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    CFemSurfaceDiscretization const& surfaceDisc,
    CDeformedContactSkinNodes const& deformedNodes,
    CContactSamples<kTimeStep>& outSamples) {
  MOCHI_PROFILE_SCOPE();
  UpdateCollisionSamplePositionsFromNodePositions(
      Unflatten<Real3 const>(MakeConstSpan(deformedNodes.positions)), surfaceDisc, outSamples);
}

template void mochi::linear_contact_skin::UpdateSamples<TimeStep::Current>(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    CFemSurfaceDiscretization const&,
    CDeformedContactSkinNodes const&,
    CContactSamples<TimeStep::Current>&);
template void mochi::linear_contact_skin::UpdateSamples<TimeStep::StageStart>(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    CFemSurfaceDiscretization const&,
    CDeformedContactSkinNodes const&,
    CContactSamples<TimeStep::StageStart>&);

template <TimeStep kTimeStep>
void mochi::linear_contact_skin::UpdatePositionsAndSamples(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    CContactSkinMesh const& contactSkin,
    CFinalDisplacementRef<kTimeStep> const& displacements,
    CFemSurfaceDiscretization const& surfaceDisc,
    CDeformedContactSkinNodes& deformedNodes,
    CContactSamples<kTimeStep>& outSamples) {
  MOCHI_PROFILE_SCOPE();
  ComputeNodePositions(
      contactSkin,
      deformedNodes,
      displacements.value,
      Unflatten<Real3>(MakeSpan(deformedNodes.positions)));
  UpdateSamples<kTimeStep>({}, {}, surfaceDisc, deformedNodes, outSamples);
}

template void mochi::linear_contact_skin::UpdatePositionsAndSamples<TimeStep::Current>(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    CContactSkinMesh const&,
    CFinalDisplacementRef<TimeStep::Current> const&,
    CFemSurfaceDiscretization const&,
    CDeformedContactSkinNodes&,
    CContactSamples<TimeStep::Current>&);
template void mochi::linear_contact_skin::UpdatePositionsAndSamples<TimeStep::StageStart>(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    CContactSkinMesh const&,
    CFinalDisplacementRef<TimeStep::StageStart> const&,
    CFemSurfaceDiscretization const&,
    CDeformedContactSkinNodes&,
    CContactSamples<TimeStep::StageStart>&);

template <bool kIsSync>
void mochi::linear_contact_skin::SetupActiveCollisionNormals(
    ecs::Excluded<TagShellActor, TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    ecs::CtxGlobal<CSimulationParams const> simParams,
    CContactSkinMesh const& contactSkin,
    CDeformedContactSkinNodes const& deformedNodes,
    CFemSurfaceDiscretization const& surfaceDisc,
    CFinalDisplacementRef<TimeStep::StageStart> const& stageStartDispl,
    CRootTransform const& transform,
    CActiveCollisions<kIsSync, TimeStep::Current>& activeCollisions) {
  MOCHI_PROFILE_SCOPE();
  if (activeCollisions.empty()) {
    return;
  }

  bool const explicitNormals = simParams->experimentalEval.explicitNormals;
  int constexpr kNumContactSkinNodesEstimate = 2048;
  MOCHI_FILO_STACK_ALLOCATOR(allocator, kNumContactSkinNodesEstimate * sizeof(Real3));
  DynamicArray<Real3> stageStartPositions(&allocator);
  Span<Real3 const> nodePositions = Unflatten<Real3 const>(MakeConstSpan(deformedNodes.positions));
  if (explicitNormals) {
    stageStartPositions.resize_noinit(contactSkin.mesh->GetNumActiveNodes());
    ComputeNodePositions(
        contactSkin, deformedNodes, stageStartDispl.value, MakeSpan(stageStartPositions));
    nodePositions = MakeConstSpan(stageStartPositions);
  }

  deformable::details::SetupActiveCollisionNormalsFromElements(
      explicitNormals,
      transform,
      surfaceDisc,
      activeCollisions,
      [&](auto const& disc, int elementIndex, int /*quadPointIndex*/) -> Vec4r {
        auto const nodes = disc.femElements[elementIndex].Nodes();
        Vec4r const p0 = ToSimd(nodePositions[nodes[0]]);
        Vec4r const p1 = ToSimd(nodePositions[nodes[1]]);
        Vec4r const p2 = ToSimd(nodePositions[nodes[2]]);
        return Normalize<3>(Cross3(p1 - p0, p2 - p0));
      });
}

template void mochi::linear_contact_skin::SetupActiveCollisionNormals</*kIsSync*/ false>(
    ecs::Excluded<TagShellActor, TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    ecs::CtxGlobal<CSimulationParams const>,
    CContactSkinMesh const&,
    CDeformedContactSkinNodes const&,
    CFemSurfaceDiscretization const&,
    CFinalDisplacementRef<TimeStep::StageStart> const&,
    CRootTransform const&,
    CActiveCollisions</*kIsSync*/ false, TimeStep::Current>&);
template void mochi::linear_contact_skin::SetupActiveCollisionNormals</*kIsSync*/ true>(
    ecs::Excluded<TagShellActor, TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    ecs::CtxGlobal<CSimulationParams const>,
    CContactSkinMesh const&,
    CDeformedContactSkinNodes const&,
    CFemSurfaceDiscretization const&,
    CFinalDisplacementRef<TimeStep::StageStart> const&,
    CRootTransform const&,
    CActiveCollisions</*kIsSync*/ true, TimeStep::Current>&);

template <TimeStep kTimeStep>
void mochi::linear_contact_skin::UpdateBounds(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    ecs::OptionalTag<TagSoftActor> isSoft,
    CSimplicialMesh const& physicsMesh,
    CContactSkinMesh const& contactSkin,
    CFinalDisplacementRef<kTimeStep> const& displacements,
    CColliderInfo const& collider,
    CPointCloudColliderParams const* pointCloudColliderParams,
    CDeformedContactSkinNodes& deformedNodes,
    CBoundingVolume& outBounds) {
  MOCHI_PROFILE_SCOPE();
  auto const positions = Unflatten<Real3>(MakeSpan(deformedNodes.positions));
  ComputeNodePositions(contactSkin, deformedNodes, displacements.value, positions);
  Aabb bounds = CalcAabb(positions);
  if (collider.type != ColliderType::None) {
    auto const nodeCoordinates = physicsMesh.mesh->GetNodeCoordinates();
    auto const nodeDisplacements = Unflatten<Real3 const>(displacements.value.GetConstSpan());
    // Interior tetrahedral nodes cannot extend the bounds. Every shell node is on the surface.
    Aabb physicsBounds = isSoft
        ? CalcAabbWithDisplacementsAndSortedIndices(
              nodeCoordinates, nodeDisplacements, physicsMesh.mesh->GetBoundaryNodes())
        : CalcAabbWithDisplacements(nodeCoordinates, nodeDisplacements);
    if (pointCloudColliderParams) {
      physicsBounds = GetAabb(ExpandShape(GetObb(physicsBounds), pointCloudColliderParams->radius));
    }
    bounds = GetAabb(bounds, physicsBounds);
  }
  outBounds.localShape = GetObb(bounds);
}

template void mochi::linear_contact_skin::UpdateBounds<TimeStep::Current>(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    ecs::OptionalTag<TagSoftActor>,
    CSimplicialMesh const&,
    CContactSkinMesh const&,
    CFinalDisplacementRef<TimeStep::Current> const&,
    CColliderInfo const&,
    CPointCloudColliderParams const*,
    CDeformedContactSkinNodes&,
    CBoundingVolume&);
template void mochi::linear_contact_skin::UpdateBounds<TimeStep::StageStart>(
    ecs::Excluded<TagRodActor>,
    ecs::RequiredTag<TagUseDeformableContactSkin>,
    ecs::OptionalTag<TagSoftActor>,
    CSimplicialMesh const&,
    CContactSkinMesh const&,
    CFinalDisplacementRef<TimeStep::StageStart> const&,
    CColliderInfo const&,
    CPointCloudColliderParams const*,
    CDeformedContactSkinNodes&,
    CBoundingVolume&);
