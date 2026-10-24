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

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

namespace superdex::studio {

// The widest vertical field of view a screenshot widens to.
inline constexpr double kMaxScreenshotFovDeg = 150.0;

// How a view framed for the viewport widens for an image of another shape.
struct ScreenshotFraming {
  double fovDeg = 0.0; // vertical field of view, degrees
  double orthoScale = 1.0; // factor on the orthographic height
};

// Widens the view so that a @p width x @p height image shows at least everything the
// @p viewportWidth x @p viewportHeight viewport shows: an image narrower than the viewport, for its
// height, widens until its width spans the viewport's, and an image of the viewport's shape or
// wider keeps the view. Up to kMaxScreenshotFovDeg: past it, a very tall image crops rather than
// degenerates. Nothing for an empty size or viewport, or a field of view outside (0, 180).
inline std::optional<ScreenshotFraming>
FrameScreenshot(double fovDeg, int width, int height, int viewportWidth, int viewportHeight) {
  if (width < 1 || height < 1 || viewportWidth < 1 || viewportHeight < 1 ||
      !std::isfinite(fovDeg) || fovDeg <= 0.0 || fovDeg >= 180.0) {
    return std::nullopt;
  }
  constexpr double kDeg2Rad = std::numbers::pi / 180.0;
  double const halfTan = std::tan(fovDeg * 0.5 * kDeg2Rad);
  double const maxScale = std::max(1.0, std::tan(kMaxScreenshotFovDeg * 0.5 * kDeg2Rad) / halfTan);
  // The viewport's aspect over the image's: above 1, the image is the narrower one.
  double const narrowness =
      (static_cast<double>(viewportWidth) * height) / (static_cast<double>(viewportHeight) * width);
  double const scale = std::clamp(narrowness, 1.0, maxScale);
  return ScreenshotFraming{
      .fovDeg = 2.0 * std::atan(halfTan * scale) / kDeg2Rad, .orthoScale = scale};
}

} // namespace superdex::studio
