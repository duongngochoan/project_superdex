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

#include <superdex_recorder/link_kinematics.h>
#include <superdex_recorder/recorder.h>
#include <superdex_robotics/superdex_robotics.h>

#include <mochi_core/utils/span.h>
#include <mochi_core/utils/transform_rt.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace superdex::recorder {

namespace detail {

struct ShadowLink {
  size_t index;
  TransformSeries& transformSeries;
};

} // namespace detail

struct RobotShadowConfig {
  /* Identifies this shadow within one robot. */
  std::string id;
  /* Selects one link and its descendants; nullopt selects the full robot. */
  std::optional<size_t> subtreeRootLinkIndex;
  std::array<uint8_t, 4> colorRgba{77, 191, 242, 115};
};

/* @brief Records one named visualization pose for a robot.
 * The episode and robot must outlive this shadow. Construction fails if the episode already
 * records a shadow with this ID for a robot with the same articulated actor name. */
class SUPERDEX_RECORDER_API RobotShadow final {
  MOCHI_DECLARE_NO_COPY_NO_MOVE(RobotShadow)

 public:
  RobotShadow(
      Episode& episode,
      robotics::Bot& robot,
      RobotShadowConfig const& config,
      mochi::Error& error);
  ~RobotShadow();

  /*
   * Records jointPositions for every robot DOF in actor DOF order, even when the shadow covers
   * only a subtree. worldFromAnchor locates the robot root or configured subtree root link.
   */
  void Record(
      SampleTime time,
      mochi::TransformRT const& worldFromAnchor,
      mochi::Span<mochi::real const> jointPositions,
      mochi::Error& error);

 private:
  std::optional<size_t> _subtreeRootLinkIndex;
  std::optional<detail::LinkKinematics> _kinematics;
  /* Null unless construction succeeded. */
  TransformSeries* _anchorSeries = nullptr;
  std::vector<detail::ShadowLink> _links;
};

} // namespace superdex::recorder
