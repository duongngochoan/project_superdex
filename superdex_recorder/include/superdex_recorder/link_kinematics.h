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

#include <mochi_core/articulated_body/articulated_body.h>
#include <mochi_core/linear_algebra/matrix.h>
#include <mochi_core/utils/dynamic_array.h>
#include <mochi_core/utils/dynamic_string.h>
#include <mochi_core/utils/error.h>
#include <mochi_core/utils/span.h>
#include <mochi_core/utils/transform_rt.h>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace superdex::recorder::detail {

/* @brief Computes each link's transform relative to its parent link from an articulation's joint
 * DoFs, with Mochi's forward kinematics. A link on a fixed joint always gets the same transform. */
class LinkKinematics final {
 public:
  /* Every span describes the same links, in Mochi's order. */
  LinkKinematics(
      mochi::Span<int const> parents,
      mochi::Span<mochi::ArticulatedJointType const> jointTypes,
      mochi::Span<mochi::Real3 const> jointAxes,
      mochi::Span<mochi::TransformRT const> jointFromChildLink,
      mochi::Span<mochi::TransformRT const> parentLinkFromJoint,
      mochi::Error& error);

  /* Computes the link transforms from joint DoFs in actor DoF order. */
  void Compute(mochi::Span<mochi::real const> dofs, mochi::Error& error);

  /* Each link's transform relative to its parent link, or to the articulation root for a root
   * link. */
  [[nodiscard]] mochi::Span<mochi::TransformRT const> GetParentFromLinks() const;

 private:
  mochi::DynamicArray<int> _parents;
  mochi::DynamicArray<mochi::ArticulatedJointType> _jointTypes;
  mochi::DynamicArray<mochi::Real3> _jointAxes;
  mochi::DynamicArray<mochi::ArticulatedRestTransform> _restTransforms;
  mochi::DynamicArray<mochi::ArticulatedDofInfo> _dofInfo;
  mochi::DynamicArray<mochi::ArticulatedPoseInfo> _poseInfo;
  mochi::ColumnVector<mochi::real> _dofs;
  mochi::ColumnVector<mochi::real> _pose;
  mochi::DynamicArray<mochi::TransformRT> _jointTransforms;
  mochi::DynamicArray<mochi::TransformRT> _rootFromLinks;
  mochi::DynamicArray<mochi::TransformRT> _parentFromLinks;
};

/* @brief Returns, for each link, an entity path that nests it under its parent link, such as
 * "base/arm/hand", so that each link's transform can be recorded relative to its parent.
 *
 * With subtreeRootLinkIndex, the paths start at that link, and links outside its subtree get an
 * empty path. */
std::vector<std::string> NestedLinkPaths(
    mochi::Span<int const> parents,
    mochi::Span<mochi::DynamicString const> linkNames,
    std::optional<size_t> subtreeRootLinkIndex,
    mochi::Error& error);

} // namespace superdex::recorder::detail
