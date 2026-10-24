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

#include <mochi_physics/src/mochi_contact.h>

#include <gtest/gtest.h>

#include <utility>

using namespace mochi;

namespace {

constexpr int kNumPartitions = 3;

template <TimeStep kTimeStep>
using CActiveCollisionsAsync = CActiveCollisions</*kIsSync*/ false, kTimeStep>;

entt::entity Entity(int value) {
  return static_cast<entt::entity>(value);
}

template <TimeStep kTimeStep>
CActiveCollisionsAsync<kTimeStep> MakeActiveCollisions(
    std::initializer_list<std::pair<int, int>> pairs) {
  CActiveCollisionsAsync<kTimeStep> activeCollisions(kNumPartitions);
  activeCollisions.SetUp({});
  for (auto const& [colliderEntity, partitionId] : pairs) {
    activeCollisions.ActivatePartitionResult(Entity(colliderEntity), partitionId);
  }
  activeCollisions.FinalizeActiveCollisions();
  return activeCollisions;
}

template <TimeStep kTimeStep>
auto EntriesOf(CActiveCollisionsAsync<kTimeStep> const& activeCollisions) {
  DynamicArray<std::pair<int, int>> entries;
  for (auto const& collision : activeCollisions) {
    entries.emplace_back(
        static_cast<int>(collision.colliderEntity), collision.collisionResult.collidingPartitionId);
  }
  return entries;
}

} // namespace

TEST(MochiStageStartContact, AddsOnlyNonEmptyMissingPairs) {
  auto stageStartCollisions =
      MakeActiveCollisions<TimeStep::StageStart>({{10, 0}, {20, 1}, {30, 2}});
  auto currentCollisions = MakeActiveCollisions<TimeStep::Current>({{10, 2}, {30, 2}});
  stageStartCollisions[1].collisionResult.sampleIndices.push_back(7);
  stageStartCollisions[2].collisionResult.sampleIndices.push_back(8);

  AddMissingStageStartCollisions(stageStartCollisions, currentCollisions);

  DynamicArray<std::pair<int, int>> const expected{{10, 2}, {20, 1}, {30, 2}};
  EXPECT_EQ(EntriesOf(currentCollisions), expected);
}

TEST(MochiStageStartContact, InsertsMissingPartitionBetweenExistingPartitions) {
  auto stageStartCollisions = MakeActiveCollisions<TimeStep::StageStart>({{10, 1}});
  auto currentCollisions = MakeActiveCollisions<TimeStep::Current>({{10, 0}, {10, 2}});
  stageStartCollisions[0].collisionResult.sampleIndices.push_back(7);
  stageStartCollisions[0].collisionResult.isSdfGradUnitary = false;

  AddMissingStageStartCollisions(stageStartCollisions, currentCollisions);

  DynamicArray<std::pair<int, int>> const expected{{10, 0}, {10, 1}, {10, 2}};
  ASSERT_EQ(EntriesOf(currentCollisions), expected);
  EXPECT_FALSE(currentCollisions[1].collisionResult.isSdfGradUnitary);
}

TEST(MochiStageStartContact, PreservesExistingCurrentPair) {
  auto stageStartCollisions = MakeActiveCollisions<TimeStep::StageStart>({{10, 1}});
  auto currentCollisions = MakeActiveCollisions<TimeStep::Current>({{10, 1}});
  stageStartCollisions[0].collisionResult.sampleIndices.push_back(7);
  stageStartCollisions[0].collisionResult.isSdfGradUnitary = false;

  AddMissingStageStartCollisions(stageStartCollisions, currentCollisions);

  ASSERT_EQ(currentCollisions.size(), 1);
  EXPECT_TRUE(currentCollisions[0].collisionResult.isSdfGradUnitary);
}
