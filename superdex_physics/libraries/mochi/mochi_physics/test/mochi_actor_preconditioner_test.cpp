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

#include "mochi_physics_test_fixture.h"

#include <mochi_core/solvers/actor_preconditioner.h>
#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_core/utils/container_utils.h>
#include <mochi_physics/mochi_physics.h>
#include <mochi_physics/mochi_physics_experimental.h>
#include <mochi_physics/src/mochi_island.h>
#include <mochi_physics/src/mochi_snle.h>

#include <gtest/gtest.h>

#include <initializer_list>
#include <utility>

using namespace mochi;
using namespace mochi::experimental;

class ActorPreconditionerTest : public test::MochiSceneTestBase {
 protected:
  [[nodiscard]] ShapeHandle CreateTetShape() {
    return test::CreateUnitCubeTetMeshShape(_mochiContext);
  }

  Actor* CreateRigid(bool isStatic = false) {
    RigidActorParams params;
    params.shape = CreateTetShape();
    params.isStatic = isStatic;
    params.colliderType = ColliderType::None;
    return _scene->CreateRigidActor(params, test::ExpectOK{});
  }

  Actor* CreateSoft() {
    SoftActorParams params;
    params.shape = CreateTetShape();
    return _scene->CreateSoftActor(params, test::ExpectOK{});
  }

  Actor* CreateNestedSoft() {
    SoftSkinnedActorParams params;
    params.skeletonParams.joints = {{.type = ArticulatedJointType::Free}};
    params.skeletonParams.links = {
        {.parentLink = -1, .shape = CreateTetShape(), .colliderType = ColliderType::None}};
    auto& softParams = params.softParams.push_back();
    softParams.shape = test::CreateUnitCubeTetSoftShape(_mochiContext);
    softParams.hasGravity = false;
    auto* actor = _scene->CreateSoftSkinnedActor(params, test::ExpectOK{});
    return _scene->GetActor(actor->GetNestedSoftActors(test::ExpectOK{}).front());
  }

  Actor* CreateShell() {
    auto [coordinates, connectivity] = test::CreateMinimalTriMeshUnitCube();
    ShellActorParams params;
    params.shape = _mochiContext->CreateTriMeshShape(
        Flatten(MakeSpan(coordinates)), Flatten(MakeSpan(connectivity)), test::ExpectOK{});
    return CreateShellActor(_scene, params, test::ExpectOK{});
  }

  Actor* CreateRod() {
    DynamicArray<Real3> const nodes = {{0_r, 0_r, 0_r}, {1_r, 0_r, 0_r}};
    DynamicArray<Real3> const axes = {{0_r, 1_r, 0_r}};
    RodActorParams params;
    params.shape =
        CreatePolylineShape(_mochiContext, nodes, axes, /*isClosedLoop*/ false, test::ExpectOK{});
    return CreateRodActor(_scene, params, test::ExpectOK{});
  }

  Actor* CreateArticulated(ArticulatedJointType jointType) {
    ArticulatedActorParams params;
    params.joints = {{.type = jointType}};
    params.links = {
        {.parentLink = -1, .shape = CreateTetShape(), .colliderType = ColliderType::None}};
    return _scene->CreateArticulatedActor(params, test::ExpectOK{});
  }

  void ConfigurePerActorSolve() {
    auto solverParams = _scene->GetSolverParams();
    solverParams.linearSolver.solverType = LinearSolverType::CG;
    solverParams.linearSolver.preconditionerType = PreconditionerType::PerActor;
    _scene->SetSolverParams(solverParams, test::ExpectOK{});
  }

  static void ExpectSupportedTypes(
      std::initializer_list<Actor*> actors,
      std::initializer_list<PreconditionerType> supportedTypes) {
    for (auto* actor : actors) {
      for (int value = 0; value < static_cast<int>(PreconditionerType::Count); ++value) {
        auto const type = static_cast<PreconditionerType>(value);
        Error error;
        SetActorPreconditionerType(actor, type, error);
        EXPECT_EQ(Contains(supportedTypes, type), error.IsOK());
      }
    }
  }
};

TEST_F(ActorPreconditionerTest, SupportsOnlyCompatibleTypesAndWarnsForSymInverse) {
  auto* rigid = CreateRigid();
  auto* soft = CreateSoft();
  auto* shell = CreateShell();
  auto* rod = CreateRod();
  auto* articulated = CreateArticulated(ArticulatedJointType::Free);
  auto* reducedSoft = CreateSoft();
  GetRegistry().get<CActorSnle>(GetEntity(reducedSoft)).EnableReduced(Matrix<real>::Zero(1, 1));
  auto* nestedSoft = CreateNestedSoft();
  auto* reducedNestedSoft = CreateNestedSoft();
  GetRegistry()
      .get<CActorSnle>(GetEntity(reducedNestedSoft))
      .EnableReduced(Matrix<real>::Zero(1, 1));
  auto* staticRigid = CreateRigid(/*isStatic*/ true);
  auto* zeroDofArticulated = CreateArticulated(ArticulatedJointType::Hard);
  auto const links = articulated->GetNestedLinkActors(test::ExpectOK{});
  ASSERT_FALSE(links.empty());
  auto* articulatedLink = _scene->GetActor(links.front());

  ExpectSupportedTypes({nullptr, staticRigid, zeroDofArticulated, articulatedLink}, {});
  ExpectSupportedTypes(
      {rigid, articulated, reducedSoft, reducedNestedSoft},
      {PreconditionerType::Jacobi, PreconditionerType::SymInverse});

  // Warnings before this scope fail the test.
  test::ExpectLoggingInScope expectWarning(_mochiContext, LogChannel::Warning);
  ExpectSupportedTypes(
      {soft, nestedSoft, shell},
      {PreconditionerType::Jacobi,
       PreconditionerType::BlockJacobi,
       PreconditionerType::AMG,
       PreconditionerType::SymInverse});
  ExpectSupportedTypes(
      {rod},
      {PreconditionerType::Jacobi,
       PreconditionerType::BlockJacobi,
       PreconditionerType::SymInverse,
       PreconditionerType::ILU0});
  EXPECT_EQ(4, expectWarning._messagesReceived);
}

TEST_F(ActorPreconditionerTest, ChangesCachedPreconditionerAndPreservesStateOnRejection) {
  ConfigurePerActorSolve();

  auto* actor = CreateSoft();
  auto& reg = GetRegistry();
  auto const actorEntity = GetEntity(actor);
  SetActorPreconditionerType(actor, PreconditionerType::Jacobi, test::ExpectOK{});
  _scene->Step(0.01_r);

  auto& actorSnle = reg.get<CActorSnle>(actorEntity);
  ASSERT_NE(nullptr, actorSnle.fullPreconditioner);
  EXPECT_EQ(PreconditionerType::Jacobi, actorSnle.fullPreconditioner->GetType());
  auto const island = reg.get<CIslandMemberInfo const>(actorEntity).island;
  auto* const islandPreconditioner = reg.get<CIslandPreconditioner>(island).get();
  auto* const actorPreconditioner = actorSnle.fullPreconditioner.get();

  SetActorPreconditionerType(actor, PreconditionerType::Jacobi, test::ExpectOK{});
  SetActorPreconditionerType(actor, PreconditionerType::ColoredSSOR, test::ExpectNotOK{});
  EXPECT_EQ(islandPreconditioner, reg.get<CIslandPreconditioner>(island).get());
  EXPECT_EQ(actorPreconditioner, actorSnle.fullPreconditioner.get());

  SetActorPreconditionerType(actor, PreconditionerType::BlockJacobi, test::ExpectOK{});
  EXPECT_NE(islandPreconditioner, reg.get<CIslandPreconditioner>(island).get());
  EXPECT_EQ(nullptr, actorSnle.fullPreconditioner);
  EXPECT_EQ(PreconditionerType::BlockJacobi, actorSnle.fullPreconditionerType);

  // Each representation keeps its own selection across ROM switches.
  actorSnle.EnableReduced(Matrix<real>::Zero(1, 1));
  SetActorPreconditionerType(actor, PreconditionerType::Jacobi, test::ExpectOK{});
  actorSnle.DisableReduced();
  EXPECT_EQ(PreconditionerType::BlockJacobi, actorSnle.GetActivePreconditionerType());
  actorSnle.EnableReduced(Matrix<real>::Zero(1, 1));
  EXPECT_EQ(PreconditionerType::Jacobi, actorSnle.GetActivePreconditionerType());
}

TEST_F(ActorPreconditionerTest, ArticulatedSelectionsSurviveFirstStepInitialization) {
  ConfigurePerActorSolve();

  DynamicArray<std::pair<Actor*, PreconditionerType>> actors;
  for (auto const type : {PreconditionerType::Jacobi, PreconditionerType::SymInverse}) {
    auto* actor = CreateArticulated(ArticulatedJointType::Free);
    actors.emplace_back(actor, type);
    SetActorPreconditionerType(actor, type, test::ExpectOK{});
  }

  _scene->Step(0.01_r);

  for (auto const& [actor, type] : actors) {
    auto const& actorSnle = GetRegistry().get<CActorSnle const>(GetEntity(actor));
    EXPECT_EQ(type, actorSnle.reducedPreconditionerType);
    ASSERT_NE(nullptr, actorSnle.reducedPreconditioner);
    EXPECT_EQ(type, actorSnle.reducedPreconditioner->GetType());
  }

  auto* const jacobiActor = actors.front().first;
  SetActorPreconditionerType(jacobiActor, PreconditionerType::SymInverse, test::ExpectOK{});
  EXPECT_EQ(
      nullptr, GetRegistry().get<CActorSnle const>(GetEntity(jacobiActor)).reducedPreconditioner);
}
