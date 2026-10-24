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

#include <superdex_recorder/robot_shadow.h>

#include <superdex_recorder/series.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace superdex::recorder {
namespace {

void WriteGeometry(
    Episode& episode,
    std::string_view entityPath,
    mochi::Actor const& actor,
    std::array<uint8_t, 4> const& color,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  auto const mesh = actor.GetSurfaceMesh();
  if (mesh.nodesPerElement != 3 || mesh.coordinates.empty() || mesh.connectivity.empty()) {
    return;
  }

  std::vector<float> vertices;
  vertices.reserve(mesh.coordinates.size());
  std::transform(
      mesh.coordinates.begin(),
      mesh.coordinates.end(),
      std::back_inserter(vertices),
      [](mochi::real value) { return static_cast<float>(value); });

  std::vector<uint32_t> triangles;
  triangles.reserve(mesh.connectivity.size());
  MOCHI_ERROR_IF_NOT(
      std::all_of(
          mesh.connectivity.begin(), mesh.connectivity.end(), [](int index) { return index >= 0; }),
      error,
      "Robot shadow mesh contains a negative vertex index.");
  MOCHI_ERROR_RETURN(error);
  std::transform(
      mesh.connectivity.begin(),
      mesh.connectivity.end(),
      std::back_inserter(triangles),
      [](int index) { return static_cast<uint32_t>(index); });
  episode.WriteMesh(entityPath, vertices, triangles, color, error);
}

} // namespace

RobotShadow::RobotShadow(
    Episode& episode,
    robotics::Bot& robot,
    RobotShadowConfig const& config,
    mochi::Error& error)
    : _subtreeRootLinkIndex{config.subtreeRootLinkIndex} {
  mochi::Scene* const scene = robot.GetScene();
  mochi::Actor* const actor = robot.GetArticulatedActor();
  if (scene == nullptr || actor == nullptr) {
    MOCHI_ERROR_SET(error, "Robot scene or actor is unavailable.");
    return;
  }
  std::string const shadowPath = detail::ShadowEntityPath(actor->GetName(), config.id, error);
  MOCHI_ERROR_RETURN(error);

  auto const linkHandles = actor->GetNestedLinkActors(error);
  auto const shapeInfo = actor->GetArticulatedShapeInfo(error);
  MOCHI_ERROR_RETURN(error);
  size_t const numLinks = linkHandles.size();
  MOCHI_ERROR_IF_NOT(
      shapeInfo.linkNames.size() >= numLinks && shapeInfo.parents.size() >= numLinks &&
          shapeInfo.jointTypes.size() >= numLinks && shapeInfo.jointAxes.size() >= numLinks &&
          shapeInfo.jointFromChildLink.size() >= numLinks &&
          shapeInfo.parentLinkFromJoint.size() >= numLinks,
      error,
      "Robot articulation metadata is incomplete.");
  MOCHI_ERROR_IF(
      _subtreeRootLinkIndex.has_value() && *_subtreeRootLinkIndex >= numLinks,
      error,
      "Robot shadow subtree root index is out of range.");
  MOCHI_ERROR_RETURN(error);

  // The shadow's root entity records the anchor, and each link is recorded relative to its parent
  // link, so a link on a fixed joint is recorded once.
  auto const parents = shapeInfo.parents.subspan(0, numLinks);
  auto const linkPaths = detail::NestedLinkPaths(
      parents, shapeInfo.linkNames.subspan(0, numLinks), _subtreeRootLinkIndex, error);
  _kinematics.emplace(
      parents,
      shapeInfo.jointTypes.subspan(0, numLinks),
      shapeInfo.jointAxes.subspan(0, numLinks),
      shapeInfo.jointFromChildLink.subspan(0, numLinks),
      shapeInfo.parentLinkFromJoint.subspan(0, numLinks),
      error);
  auto* const anchorSeries = episode.CreateTransformSeries(shadowPath, error);
  MOCHI_ERROR_RETURN(error);
  if (anchorSeries == nullptr) {
    MOCHI_ERROR_SET(error, "Failed to create a robot shadow transform series.");
    return;
  }
  _links.reserve(linkHandles.size());
  for (size_t i = 0; i < linkHandles.size(); ++i) {
    if (linkPaths[i].empty()) {
      continue;
    }
    mochi::Actor* const linkActor = scene->GetActor(linkHandles[i]);
    if (linkActor == nullptr) {
      MOCHI_ERROR_SET(error, "Robot shadow contains an invalid link.");
      return;
    }
    std::string const linkPath = shadowPath + "/links/" + linkPaths[i];
    WriteGeometry(episode, linkPath, *linkActor, config.colorRgba, error);
    auto* const series = episode.CreateTransformSeries(linkPath, error);
    MOCHI_ERROR_RETURN(error);
    if (series == nullptr) {
      MOCHI_ERROR_SET(error, "Failed to create a robot shadow transform series.");
      return;
    }
    _links.push_back({i, *series});
  }
  _anchorSeries = anchorSeries;
}

RobotShadow::~RobotShadow() = default;

void RobotShadow::Record(
    SampleTime time,
    mochi::TransformRT const& worldFromAnchor,
    mochi::Span<mochi::real const> jointPositions,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  if (_anchorSeries == nullptr) {
    MOCHI_ERROR_SET(error, "Robot shadow construction failed.");
    return;
  }
  _kinematics->Compute(jointPositions, error);
  _anchorSeries->Append(time, worldFromAnchor, error);
  MOCHI_ERROR_RETURN(error);
  auto const parentFromLinks = _kinematics->GetParentFromLinks();
  for (auto& link : _links) {
    // The anchor locates a subtree shadow's root link itself.
    mochi::TransformRT const parentFromLink = link.index == _subtreeRootLinkIndex
        ? mochi::TransformRT::Identity()
        : parentFromLinks[link.index];
    link.transformSeries.Append(time, parentFromLink, error);
    MOCHI_ERROR_RETURN(error);
  }
}

} // namespace superdex::recorder
