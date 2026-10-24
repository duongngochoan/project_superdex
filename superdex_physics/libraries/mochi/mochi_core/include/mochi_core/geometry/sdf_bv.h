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

#include <mochi_core/geometry/batch_sphere.h>
#include <mochi_core/geometry/grid_sdf.h>
#include <mochi_core/geometry/scalar_field.h>
#include <mochi_core/mochi_config.h>
#include <mochi_core/mochi_platform.h>
#include <mochi_core/utils/batch_types.h>
#include <mochi_core/utils/vmatrix.h>

namespace mochi {

/**
 * @brief Represents a bounding volume defined by the level set of a signed distance field (SDF).
 *
 * @note Overlap tests assume that the interpolated SDF is 1-Lipschitz: a sphere overlaps iff the
 * SDF at its center is at most its radius plus @ref distanceThreshold. Trilinear interpolation can
 * be up to sqrt(3)-Lipschitz where the distance field bends within a cell (e.g. near sharp or thin
 * features). Culling may therefore rarely discard a point within the threshold, but only one whose
 * SDF exceeds the threshold minus (sqrt(3) - 1) times the sphere radius. Scaling radii by sqrt(3)
 * would make culling conservative, at the cost of more false positives.
 */
struct SdfBv {
  // Pointer to the SDF.
  GridSdf const* gridSdf = nullptr;

  // Distance threshold that defines the level-set of the SDF. The distance is expressed in Points
  // space (whose scale may be different than Grid space). Positive values expand the volume.
  // Negative values contract the volume.
  real distanceThreshold;

  // Transform matrix that converts from Points space to Grid space. It may encode rotation,
  // translation and uniform scaling.
  VMatrix4x4r gridFromPointsT;
};

/**
 * @brief Tests overlap between an @ref SdfBv and each of a batch of spheres.
 *
 * @tparam kBatchSize The number of spheres in the batch.
 * @param sdfBv The SDF bounding volume.
 * @param sphere The batch of spheres.
 * @return A per-lane SIMD mask; each sphere's lane is set if overlap was detected.
 */
template <int kBatchSize>
[[nodiscard]] MOCHI_FORCE_INLINE BatchReal<kBatchSize> HasOverlap(
    SdfBv const& sdfBv,
    BatchSphere<kBatchSize> const& sphere) {
  using V = BatchReal<kBatchSize>;
  auto const matT = Broadcast3x3<V>(sdfBv.gridFromPointsT);
  auto const trans = Broadcast3<V>(sdfBv.gridFromPointsT[3]);
  auto const centerInGrid = DotVecMat(sphere.center, matT) + trans;
  V dist MOCHI_NO_INIT;
  sdfBv.gridSdf->GetDistanceGrid()
      .template TrilinearSampleBatch<V::kSize, GridExtrapolation::LowerBound>(centerInGrid, &dist);
  real const actorFromGridScale = sdfBv.gridSdf->GetActorFromGridScale();
  return dist * actorFromGridScale <= sphere.radius + sdfBv.distanceThreshold;
}

} // namespace mochi
