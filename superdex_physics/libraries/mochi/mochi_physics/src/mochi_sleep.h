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

#include "mochi_ecs.h"

#include <mochi_core/geometry/aabb.h>
#include <mochi_core/solvers/newton_solver_status.h>
#include <mochi_core/utils/dynamic_array.h>
#include <mochi_core/utils/error.h>
#include <mochi_core/utils/no_copy.h>
#include <mochi_physics/mochi_physics_experimental.h>

/*
  Island sleeping.

  An island at rest can be put to sleep. Sleeping islands are skipped by the simulation step.
  Sleeping actors keep their positions and report zero velocity. Anything that could change a
  sleeping island must wake it via WakeUp or WakeAll.

  After each solve, an island records a rest value (see StageRestValue). It goes to sleep once it
  has recorded SleepParams::minStepsBeforeSleep consecutive values of at least
  SleepParams::sleepThreshold.
*/

namespace mochi {

// Scene-global island sleeping parameters. See experimental::SleepParams.
struct CSleepParams final : public experimental::SleepParams, NoCopy {};

// Present on islands that are awake. Islands without it are asleep and are not simulated.
struct TagIslandIsAwake {};

// Requests that an island wakes up during the next PreStep.
struct TagWakeUp {};

// Per-island rest tracking.
struct CIslandSleepState : NoCopy {
  // Number of consecutive solves whose rest value was at least SleepParams::sleepThreshold.
  int numConsecutiveRestSteps = 0;
};

// Scene-global state which may cause islands to wake up.
struct CWakeUpRequests : NoCopy {
  // Request that all islands wake before the next step.
  bool wakeAll = false;

  // World-space regions that changed (e.g. static actors that moved). Sleeping islands with actors
  // overlapping any region will wake up before the next step.
  DynamicArray<Aabb> regions;
};

} // namespace mochi

namespace mochi::sleep {

// Validates the sleep parameters against the scene, reporting problems through the error.
void ValidateParams(
    entt::registry const& reg,
    experimental::SleepParams const& params,
    Error& error);

// Adds the sleep components to a newly created island. New islands are awake.
void InitIsland(entt::registry& reg, entt::entity island);

// Rest value of one integration stage: 0 if the stage did not converge, 1 if it took zero
// iterations, and the ratio of final to initial residual norms otherwise.
[[nodiscard]] real StageRestValue(NewtonSolverStatus<real> const& status);

// Records the rest value of an island's latest solve (the minimum over its stages).
void RecordStep(entt::registry& reg, entt::entity island, real restValue);

// Requests waking whatever could be affected by a change to the entity: the actor's island, the
// region around a static actor (including an articulation's static links), or the islands of a
// constraint's actors. For changes that move static colliders, call this before and after.
void WakeUp(entt::registry& reg, entt::entity e);

// Requests waking all islands. They will wake on the next PreStep.
void WakeAll(entt::registry& reg);

// Applies pending wake requests, then puts islands to sleep if they have been at rest long enough.
// Must be called after islands have been merged and split for this step.
void PreStep(entt::registry& reg);

// Returns true if the island has TagIslandIsAwake.
[[nodiscard]] bool IsIslandAwake(entt::registry const& reg, entt::entity island);

// FOR UNIT TESTS ONLY: Put an island to sleep immediately.
void ForceSleep(entt::registry& reg, entt::entity island);

// Call once during scene initialization.
void InitializeOnce(entt::registry& reg);

} // namespace mochi::sleep
