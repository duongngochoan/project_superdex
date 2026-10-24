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

#include <mochi_core/utils/coordinate_space.h>

#include <mochi_core/utils/basic_utils.h>

using namespace mochi;

[[nodiscard]] static bool IsValidCoordinateSpace(CoordinateSpaceAxes axes) {
  if ((static_cast<uint16_t>(axes) >> 9) != 0) {
    return false; // Upper bits should not be set.
  }
  int constexpr kNumDirections = 6; // Right, Left, Up, Down, Forward, Backward
  int const dirs[3] = {
      details::GetAxisDirection(axes, 0),
      details::GetAxisDirection(axes, 1),
      details::GetAxisDirection(axes, 2)};
  int dims[3] = {};
  for (int i = 0; i < 3; ++i) {
    MOCHI_ASSERT_VERBOSE(dirs[i] >= 0, "GetAxisDirection should be positive no matter what");
    if (dirs[i] >= kNumDirections) {
      return false; // Invalid direction
    }
    dims[i] = dirs[i] >> 1; // 0 for +/-X, 1 for +/-Y, 2 for +/-Z
  }
  return (dims[0] != dims[1]) && (dims[1] != dims[2]) && (dims[0] != dims[2]);
}

void CoordinateSpace::Validate(Error& error) const {
  MOCHI_ERROR_RETURN(error);
  MOCHI_ERROR_IF(
      !IsValidCoordinateSpace(axes),
      error,
      "CoordinateSpaceAxes must map X, Y, and Z to three different orthogonal directions.");
  MOCHI_ERROR_IF(
      !IsFinite(unitsPerMeter) || unitsPerMeter <= 0_r,
      error,
      "CoordinateSpace::unitsPerMeter must be finite and greater than zero.");
}
