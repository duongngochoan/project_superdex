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

#include "mochi_active_collisions.h"

#include <mochi_core/utils/container_utils.h>

using namespace mochi;

static void RemoveAbsentCollidersAndClear(
    Span<entt::entity const> colliders,
    DynamicArray<ActiveCollision>& collisions) {
  collisions.erase(
      std::remove_if(
          collisions.begin(),
          collisions.end(),
          [&](auto const& collision) { return !Contains(colliders, collision.colliderEntity); }),
      collisions.end());
  for (auto& collision : collisions) {
    collision.Clear();
  }
}

template <bool kIsSync, TimeStep kTimeStep>
void CActiveCollisions<kIsSync, kTimeStep>::SetUp(Span<entt::entity const> potentialColliders) {
  // Detects entries added or reordered outside this class since the previous pass.
  MOCHI_ASSERT_VERBOSE(IsSortedAndUnique(), "Expected sorted and unique active collisions.");

  if (_numPartitions == 1) {
    RemoveAbsentCollidersAndClear(potentialColliders, *this);
    bool appended = false;
    for (entt::entity const colliderEntity : potentialColliders) {
      auto const wasPotential = std::any_of(begin(), end(), [&](auto const& collision) {
        return collision.colliderEntity == colliderEntity;
      });
      if (!wasPotential) {
        appended = true;
        emplace_back(ActiveCollision{colliderEntity, ContactDetectionResult{}});
      }
    }
    if (appended) {
      FinalizeActiveCollisions();
    }
    MOCHI_ASSERT_VERBOSE(IsSortedAndUnique(), "Expected sorted and unique active collisions.");
    return;
  }

  RestoreAggregateResults();
  for (auto& collision : *this) {
    _inactiveCollisions.emplace_back(std::move(collision));
  }
  clear();
  RemoveAbsentCollidersAndClear(potentialColliders, _inactiveCollisions);
  RemoveAbsentCollidersAndClear(potentialColliders, _aggregateCollisions);

  size_t const numRetainedAggregates = _aggregateCollisions.size();
  for (entt::entity const colliderEntity : potentialColliders) {
    auto const it = std::lower_bound(
        _aggregateCollisions.begin(),
        _aggregateCollisions.begin() + numRetainedAggregates,
        colliderEntity,
        [](ActiveCollision const& aggregate, entt::entity entity) {
          return aggregate.colliderEntity < entity;
        });
    if (it == _aggregateCollisions.begin() + numRetainedAggregates ||
        it->colliderEntity != colliderEntity) {
      _aggregateCollisions.emplace_back(ActiveCollision{colliderEntity, ContactDetectionResult{}});
    }
  }
  auto const compareAggregates = [](ActiveCollision const& lhs, ActiveCollision const& rhs) {
    return lhs.colliderEntity < rhs.colliderEntity;
  };
  auto const newAggregatesBegin = _aggregateCollisions.begin() + numRetainedAggregates;
  std::sort(newAggregatesBegin, _aggregateCollisions.end(), compareAggregates);
  std::inplace_merge(
      _aggregateCollisions.begin(),
      newAggregatesBegin,
      _aggregateCollisions.end(),
      compareAggregates);
}

template <bool kIsSync, TimeStep kTimeStep>
ContactDetectionResult& CActiveCollisions<kIsSync, kTimeStep>::ActivatePartitionResult(
    entt::entity colliderEntity,
    int partitionId) {
  MOCHI_ASSERT_VERBOSE(
      partitionId >= 0 && partitionId < _numPartitions, "Invalid contact partition.");
  ColliderPartitionLess::Key const key{colliderEntity, partitionId};
  auto const hasKey = [&](ActiveCollision const& collision) {
    return ColliderPartitionLess::GetKey(collision) == key;
  };
  MOCHI_ASSERT_VERBOSE(
      std::none_of(begin(), end(), hasKey), "Contact partition is already active.");

  auto const inactiveIt =
      std::find_if(_inactiveCollisions.begin(), _inactiveCollisions.end(), hasKey);
  if (inactiveIt == _inactiveCollisions.end()) {
    emplace_back(
        ActiveCollision{
            colliderEntity, ContactDetectionResult{.collidingPartitionId = partitionId}});
  } else {
    emplace_back(std::move(*inactiveIt));
    EraseIndexUnordered(_inactiveCollisions, inactiveIt - _inactiveCollisions.begin());
  }
  return back().collisionResult;
}

template <bool kIsSync, TimeStep kTimeStep>
void CActiveCollisions<kIsSync, kTimeStep>::RestoreAggregateResults() {
  // May also swap partition-zero pairs activated after the narrow phase, which is harmless because
  // SetUp clears all results afterwards.
  for (auto& collision : *this) {
    if (collision.collisionResult.collidingPartitionId != 0) {
      continue;
    }
    auto const aggregateIt = FindAggregate(collision.colliderEntity);
    if (aggregateIt != _aggregateCollisions.end() &&
        aggregateIt->colliderEntity == collision.colliderEntity) {
      std::swap(aggregateIt->collisionResult, collision.collisionResult);
    }
  }
}

#define MOCHI_INSTANTIATE_ACTIVE_COLLISIONS(isSync, timeStep)                                    \
  template void CActiveCollisions<isSync, timeStep>::SetUp(Span<entt::entity const>);            \
  template ContactDetectionResult& CActiveCollisions<isSync, timeStep>::ActivatePartitionResult( \
      entt::entity, int);                                                                        \
  template void CActiveCollisions<isSync, timeStep>::RestoreAggregateResults()
MOCHI_INSTANTIATE_ACTIVE_COLLISIONS(/*kIsSync*/ false, TimeStep::Current);
MOCHI_INSTANTIATE_ACTIVE_COLLISIONS(/*kIsSync*/ false, TimeStep::StageStart);
MOCHI_INSTANTIATE_ACTIVE_COLLISIONS(/*kIsSync*/ true, TimeStep::Current);
MOCHI_INSTANTIATE_ACTIVE_COLLISIONS(/*kIsSync*/ true, TimeStep::StageStart);
#undef MOCHI_INSTANTIATE_ACTIVE_COLLISIONS
