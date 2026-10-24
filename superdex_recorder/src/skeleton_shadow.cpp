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

#include <superdex_recorder/skeleton_shadow.h>

#include <superdex_recorder/series.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string_view>

namespace superdex::recorder {

SkeletonShadow::SkeletonShadow(
    Episode& episode,
    robotics::Bot& robot,
    SkeletonShadowConfig const& config,
    mochi::Error& error)
    : _numKeypoints{config.keypointNames.size()} {
  mochi::Actor const* const actor = robot.GetArticulatedActor();
  if (actor == nullptr) {
    MOCHI_ERROR_SET(error, "Robot actor is unavailable.");
    return;
  }
  std::string const shadowPath = detail::ShadowEntityPath(actor->GetName(), config.id, error);
  MOCHI_ERROR_RETURN(error);
  MOCHI_ERROR_IF(_numKeypoints == 0, error, "A skeleton shadow must have at least one keypoint.");
  // Rerun keypoint IDs are uint16.
  MOCHI_ERROR_IF(
      _numKeypoints > size_t{std::numeric_limits<uint16_t>::max()} + 1,
      error,
      "A skeleton shadow has too many keypoints.");
  MOCHI_ERROR_IF(
      config.parentIndices.size() != _numKeypoints,
      error,
      "Skeleton shadow parent indices and keypoint names differ in size.");
  std::set<std::string_view> const uniqueNames(
      config.keypointNames.begin(), config.keypointNames.end());
  MOCHI_ERROR_IF(
      uniqueNames.size() != _numKeypoints || uniqueNames.count("") != 0,
      error,
      "Skeleton shadow keypoint names must be unique and not empty.");
  MOCHI_ERROR_IF_NOT(
      std::isfinite(config.keypointRadiusMeters) && config.keypointRadiusMeters > 0.0f,
      error,
      "Skeleton shadow keypoint radius must be finite and positive.");
  if (!error.IsOK()) {
    MOCHI_LOG_ERROR(
        "Invalid skeleton shadow at '%s': %s", shadowPath.c_str(), error.GetDescription());
    return;
  }

  std::vector<std::array<uint16_t, 2>> connections;
  for (size_t i = 0; i < _numKeypoints; ++i) {
    int32_t const parent = config.parentIndices[i];
    if (parent < -1 || parent >= static_cast<int64_t>(i)) {
      MOCHI_LOG_ERROR(
          "Keypoint %zu of the skeleton shadow at '%s' has parent %d.",
          i,
          shadowPath.c_str(),
          parent);
      MOCHI_ERROR_SET(error, "A skeleton shadow keypoint's parent must precede it or be -1.");
      return;
    }
    if (parent >= 0) {
      connections.push_back({static_cast<uint16_t>(parent), static_cast<uint16_t>(i)});
    }
  }

  _series = episode.CreatePoints3DSeries(shadowPath, error);
  MOCHI_ERROR_RETURN(error);
  if (_series == nullptr) {
    MOCHI_ERROR_SET(error, "Failed to create a skeleton shadow point series.");
    return;
  }
  episode.WriteKeypointSkeleton(
      shadowPath,
      config.label.empty() ? config.id : config.label,
      config.keypointNames,
      connections,
      config.colorRgba,
      config.keypointRadiusMeters,
      error);
  // Without its skeleton, the series would record bare points.
  if (!error.IsOK()) {
    _series = nullptr;
  }
}

SkeletonShadow::~SkeletonShadow() = default;

void SkeletonShadow::Record(
    SampleTime time,
    mochi::Span<mochi::Real3 const> worldKeypoints,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  MOCHI_ERROR_IF(
      worldKeypoints.size() != _numKeypoints,
      error,
      "Skeleton shadow sample must have one position per keypoint.");
  MOCHI_ERROR_IF_NOT(
      std::all_of(
          worldKeypoints.begin(),
          worldKeypoints.end(),
          [](mochi::Real3 const& keypoint) {
            return std::all_of(keypoint.begin(), keypoint.end(), [](mochi::real value) {
              return std::isfinite(value);
            });
          }),
      error,
      "Skeleton shadow keypoints must be finite.");
  Append(time, worldKeypoints, error);
}

void SkeletonShadow::RecordUntracked(SampleTime time, mochi::Error& error) {
  Append(time, {}, error);
}

void SkeletonShadow::Append(
    SampleTime time,
    mochi::Span<mochi::Real3 const> keypoints,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  if (_series == nullptr) {
    MOCHI_ERROR_SET(error, "Skeleton shadow construction failed.");
    return;
  }
  _series->Append(time, keypoints, error);
}

} // namespace superdex::recorder
