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

#include <superdex_recorder/recorder.h>
#include <superdex_robotics/superdex_robotics.h>

#include <mochi_core/utils/nd_array.h>
#include <mochi_core/utils/span.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace superdex::recorder {

struct SkeletonShadowConfig {
  /* Identifies this shadow within one robot, among its robot and skeleton shadows. */
  std::string id;
  /* Labels the skeleton in the viewer; empty uses the ID. */
  std::string label;
  /* Names each keypoint, in the order Record takes them. */
  std::vector<std::string> keypointNames;
  /* Each keypoint's parent keypoint index, which must be lower than its own, or -1 for none. */
  std::vector<int32_t> parentIndices;
  std::array<uint8_t, 4> colorRgba{242, 140, 38, 255};
  float keypointRadiusMeters = 0.004f;
};

/* @brief Records a keypoint skeleton, such as an operator's tracked hand, as a robot's shadow.
 * The viewer draws each keypoint and a line to its parent. The episode must outlive this shadow.
 * Construction fails if the episode already records a robot or skeleton shadow with this ID for a
 * robot with the same articulated actor name. */
class SUPERDEX_RECORDER_API SkeletonShadow final {
  MOCHI_DECLARE_NO_COPY_NO_MOVE(SkeletonShadow)

 public:
  SkeletonShadow(
      Episode& episode,
      robotics::Bot& robot,
      SkeletonShadowConfig const& config,
      mochi::Error& error);
  ~SkeletonShadow();

  /* Records one finite world position per keypoint, in configuration order. */
  void Record(SampleTime time, mochi::Span<mochi::Real3 const> worldKeypoints, mochi::Error& error);
  /* Hides the skeleton until the next Record. */
  void RecordUntracked(SampleTime time, mochi::Error& error);

 private:
  void Append(SampleTime time, mochi::Span<mochi::Real3 const> keypoints, mochi::Error& error);

  size_t _numKeypoints = 0;
  /* Null unless construction succeeded. */
  Points3DSeries* _series = nullptr;
};

} // namespace superdex::recorder
