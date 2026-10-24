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

#include <superdex_recorder/link_kinematics.h>

#include <superdex_recorder/recorder.h>

#include <algorithm>

namespace superdex::recorder::detail {

LinkKinematics::LinkKinematics(
    mochi::Span<int const> parents,
    mochi::Span<mochi::ArticulatedJointType const> jointTypes,
    mochi::Span<mochi::Real3 const> jointAxes,
    mochi::Span<mochi::TransformRT const> jointFromChildLink,
    mochi::Span<mochi::TransformRT const> parentLinkFromJoint,
    mochi::Error& error) {
  size_t const numLinks = parents.size();
  MOCHI_ERROR_IF_NOT(
      jointTypes.size() == numLinks && jointAxes.size() == numLinks &&
          jointFromChildLink.size() == numLinks && parentLinkFromJoint.size() == numLinks,
      error,
      "Articulation metadata has inconsistent sizes.");
  MOCHI_ERROR_RETURN(error);
  _parents.reserve(numLinks);
  _jointTypes.reserve(numLinks);
  _jointAxes.reserve(numLinks);
  _restTransforms.reserve(numLinks);
  for (size_t i = 0; i < numLinks; ++i) {
    _parents.push_back(parents[i]);
    _jointTypes.push_back(jointTypes[i]);
    _jointAxes.push_back(jointAxes[i]);
    _restTransforms.emplace_back(jointFromChildLink[i], parentLinkFromJoint[i]);
  }
  _dofInfo = mochi::articulated::SetupJointDofs(_jointTypes);
  _poseInfo = mochi::articulated::SetupJointPose(_jointTypes);
  _dofs.Resize(mochi::articulated::GetReducedDofsSize(_dofInfo));
  _pose.Resize(mochi::articulated::GetReducedPoseSize(_poseInfo));
  _jointTransforms.resize(numLinks);
  _rootFromLinks.resize(numLinks);
  _parentFromLinks.resize(numLinks);
}

void LinkKinematics::Compute(mochi::Span<mochi::real const> dofs, mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  MOCHI_ERROR_IF(
      dofs.size() != static_cast<size_t>(_dofs.Rows()),
      error,
      "Joint DoF count does not match the articulation.");
  MOCHI_ERROR_RETURN(error);
  std::copy(dofs.begin(), dofs.end(), _dofs.Data());
  mochi::articulated::ConvertDofsToPose(
      _jointTypes, _dofInfo, _poseInfo, mochi::AsConstView(_dofs), mochi::AsView(_pose));
  mochi::articulated::ComputeTransformsFromReducedPose(
      _jointTypes,
      _jointAxes,
      _poseInfo,
      _parents,
      _restTransforms,
      mochi::TransformRT::Identity(),
      mochi::AsConstView(_pose),
      _jointTransforms,
      _rootFromLinks);
  // The same composition as Mochi's parent-from-bone transforms, which the public API does not
  // expose. A fixed joint's transform is the identity, so its link's transform never changes.
  for (size_t i = 0; i < _parentFromLinks.size(); ++i) {
    _parentFromLinks[i] =
        _restTransforms[i].parentFromOuter * _jointTransforms[i] * _restTransforms[i].innerFromBone;
  }
}

mochi::Span<mochi::TransformRT const> LinkKinematics::GetParentFromLinks() const {
  return _parentFromLinks;
}

std::vector<std::string> NestedLinkPaths(
    mochi::Span<int const> parents,
    mochi::Span<mochi::DynamicString const> linkNames,
    std::optional<size_t> subtreeRootLinkIndex,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error, {});
  MOCHI_ERROR_IF(
      linkNames.size() != parents.size(), error, "Articulation metadata has inconsistent sizes.");
  MOCHI_ERROR_RETURN(error, {});
  std::vector<std::string> paths(parents.size());
  for (size_t i = 0; i < parents.size(); ++i) {
    int const parent = parents[i];
    // Mochi orders links from the roots to the leaves.
    MOCHI_ERROR_IF(
        parent >= static_cast<int>(i), error, "Articulated links must follow their parent link.");
    MOCHI_ERROR_RETURN(error, {});
    std::string const segment = EncodeEntityPathSegment(linkNames[i]);
    bool const isRoot = subtreeRootLinkIndex.has_value() ? i == *subtreeRootLinkIndex : parent < 0;
    // `paths` has one entry per link, and each parent precedes its child (checked above).
    // NOLINTBEGIN(facebook-hte-LocalUncheckedArrayBounds)
    if (isRoot) {
      paths[i] = segment;
    } else if (parent >= 0 && !paths[parent].empty()) {
      paths[i] = paths[parent] + "/" + segment;
    }
    // NOLINTEND(facebook-hte-LocalUncheckedArrayBounds)
  }
  return paths;
}

} // namespace superdex::recorder::detail
