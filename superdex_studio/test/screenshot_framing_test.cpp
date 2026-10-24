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

#include "rendering/screenshot_framing.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <tuple>
#include <utility>

using namespace superdex::studio;

namespace {

double HalfTan(double fovDeg) {
  return std::tan(fovDeg * std::numbers::pi / 360.0);
}

// Studio's viewport is landscape unless the window is made tall.
constexpr int kViewportWidth = 1280;
constexpr int kViewportHeight = 720;

TEST(ScreenshotFramingTest, AnImageAtLeastAsWideAsTheViewportKeepsTheView) {
  for (auto const& [width, height] :
       {std::pair{1280, 720}, std::pair{1920, 1080}, std::pair{8192, 1}}) {
    ScreenshotFraming const framing =
        FrameScreenshot(45.0, width, height, kViewportWidth, kViewportHeight).value();
    EXPECT_NEAR(framing.fovDeg, 45.0, 1e-9) << width << "x" << height;
    EXPECT_DOUBLE_EQ(framing.orthoScale, 1.0) << width << "x" << height;
  }
}

TEST(ScreenshotFramingTest, ANarrowerImageWidensUntilItSpansTheViewport) {
  for (auto const& [width, height] :
       {std::pair{800, 600}, std::pair{1024, 1024}, std::pair{480, 640}}) {
    ScreenshotFraming const framing =
        FrameScreenshot(45.0, width, height, kViewportWidth, kViewportHeight).value();
    // The image's horizontal half-angle is the viewport's, and its vertical one is wider.
    EXPECT_NEAR(
        HalfTan(framing.fovDeg) * width / height,
        HalfTan(45.0) * kViewportWidth / kViewportHeight,
        1e-12)
        << width << "x" << height;
    EXPECT_GT(framing.fovDeg, 45.0) << width << "x" << height;
    EXPECT_DOUBLE_EQ(framing.orthoScale, HalfTan(framing.fovDeg) / HalfTan(45.0))
        << width << "x" << height;
  }
}

TEST(ScreenshotFramingTest, AVeryTallImageStopsAtTheWidestView) {
  ScreenshotFraming const framing =
      FrameScreenshot(45.0, 1, 8192, kViewportWidth, kViewportHeight).value();
  EXPECT_NEAR(framing.fovDeg, kMaxScreenshotFovDeg, 1e-9);
  EXPECT_NEAR(framing.orthoScale, HalfTan(kMaxScreenshotFovDeg) / HalfTan(45.0), 1e-9);
}

TEST(ScreenshotFramingTest, AnEmptySizeOrViewportOrABadFieldOfViewGivesNoFraming) {
  double const inf = std::numeric_limits<double>::infinity();
  for (auto const& [fovDeg, width, height] :
       {std::tuple{45.0, 0, 640},
        std::tuple{45.0, 480, 0},
        std::tuple{45.0, -480, 640},
        std::tuple{45.0, 480, -640},
        std::tuple{0.0, 480, 640},
        std::tuple{-10.0, 480, 640},
        std::tuple{180.0, 480, 640},
        std::tuple{200.0, 480, 640},
        std::tuple{inf, 480, 640}}) {
    EXPECT_FALSE(FrameScreenshot(fovDeg, width, height, kViewportWidth, kViewportHeight))
        << fovDeg << " " << width << "x" << height;
  }
  EXPECT_FALSE(FrameScreenshot(
      std::numeric_limits<double>::quiet_NaN(), 480, 640, kViewportWidth, kViewportHeight));
  EXPECT_FALSE(FrameScreenshot(45.0, 480, 640, 0, kViewportHeight));
  EXPECT_FALSE(FrameScreenshot(45.0, 480, 640, kViewportWidth, 0));
}

TEST(ScreenshotFramingTest, AViewWiderThanTheCapIsLeftAlone) {
  ScreenshotFraming const framing =
      FrameScreenshot(160.0, 480, 640, kViewportWidth, kViewportHeight).value();
  EXPECT_NEAR(framing.fovDeg, 160.0, 1e-9);
  EXPECT_DOUBLE_EQ(framing.orthoScale, 1.0);
}

TEST(ScreenshotFramingTest, APortraitViewportWidensOnlyForATallerImage) {
  ScreenshotFraming const sameShape = FrameScreenshot(45.0, 480, 640, 480, 640).value();
  EXPECT_NEAR(sameShape.fovDeg, 45.0, 1e-9);
  EXPECT_DOUBLE_EQ(sameShape.orthoScale, 1.0);
  ScreenshotFraming const taller = FrameScreenshot(45.0, 480, 960, 480, 640).value();
  EXPECT_DOUBLE_EQ(taller.orthoScale, (960.0 / 480.0) / (640.0 / 480.0));
}

} // namespace
