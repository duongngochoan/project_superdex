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

// Saves a screenshot of a file without anyone driving the UI: Studio starts in its normal window,
// opens the file, waits until it has loaded, frames the view, writes the PNG and exits.

#include <array>
#include <chrono>
#include <optional>
#include <string>

namespace superdex::studio {

class SuperDexStudio;

struct CameraPlacement {
  std::array<double, 3> eye{};
  std::array<double, 3> target{};
};

struct ScreenshotOptions {
  std::string outPath;
  std::string openPath;
  // The image size; 0 x 0 keeps the window's size, or the viewport's with viewportOnly.
  int width = 0;
  int height = 0;
  // Frames this actor and everything under it. Empty frames the whole scene.
  std::string focus;
  // Places the camera instead of framing, in editor coordinates.
  std::optional<CameraPlacement> camera;
  // Saves the 3D viewport alone, rendered at the image size, instead of the whole window.
  bool viewportOnly = false;
};

// A screenshot run. The app calls Step each time a frame is on screen, so the window can be read
// back, and a camera change made in one step shows in the next frame.
class ScreenshotRun {
 public:
  explicit ScreenshotRun(ScreenshotOptions options);

  // Advances the run and returns false once it is over. ExitCode() is then 0 if the PNG was
  // written, and 1 if not, with the reason printed to stderr.
  bool Step(SuperDexStudio& studio);
  [[nodiscard]] int ExitCode() const {
    return _exitCode;
  }

 private:
  enum class Stage { Open, Load, Capture, Done };

  bool Open(SuperDexStudio& studio);
  bool Load(SuperDexStudio& studio);
  bool Capture(SuperDexStudio& studio);
  bool Finish(char const* error, int width, int height);

  ScreenshotOptions _options;
  Stage _stage = Stage::Open;
  // ImGui frame count when the stage began.
  int _stageFrame = 0;
  std::chrono::steady_clock::time_point _loadDeadline;
  std::chrono::steady_clock::time_point _resizeDeadline;
  std::string _cameraText;
  int _exitCode = 1;
};

} // namespace superdex::studio
