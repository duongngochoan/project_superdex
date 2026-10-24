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

#include "app/screenshot_command.h"

#include "app/app.h"
#include "editors/asset_editor.h"
#include "rendering/viewport.h"

#include <mochi_core/mochi_platform.h>
#include <mochi_core/utils/error.h>
#include <mochi_core/utils/log.h>

#include <imguios/common.h>
#include <imguios/imguios_window.h>

#include <cstdio>
#include <filesystem>
#include <utility>

namespace superdex::studio {

namespace {

// Frames a newly opened editor needs to size its viewport and frame its content.
constexpr int kSettleFrames = 2;
// Frames to wait after framing before reading the window, which may show a frame or two late.
constexpr int kCaptureFrames = 3;
// How long the window may take to reach the requested size; a window manager may apply it late.
constexpr std::chrono::seconds kResizeTimeout{5};
constexpr std::chrono::minutes kLoadTimeout{10};

mochi::Real3 ToReal3(std::array<double, 3> const& value) {
  return {
      static_cast<mochi::real>(value[0]),
      static_cast<mochi::real>(value[1]),
      static_cast<mochi::real>(value[2])};
}

std::string VectorText(mochi::Real3 const& value) {
  return mochi::Format(
      "%.6g,%.6g,%.6g",
      static_cast<double>(value[0]),
      static_cast<double>(value[1]),
      static_cast<double>(value[2]));
}

bool Report(std::string const& reason) {
  std::fprintf(stderr, "superdex_studio: %s\n", reason.c_str());
  return false;
}

} // namespace

ScreenshotRun::ScreenshotRun(ScreenshotOptions options) : _options(std::move(options)) {
  _options.outPath = std::filesystem::absolute(_options.outPath).string();
  _options.openPath = std::filesystem::absolute(_options.openPath).string();
#if MOCHI_PLATFORM_MACOS
  if (!_options.viewportOnly) {
    std::fputs(
        "superdex_studio: on macOS the viewport is saved (as with --viewport-only)\n", stderr);
    _options.viewportOnly = true;
  }
#endif
}

bool ScreenshotRun::Step(SuperDexStudio& studio) {
  switch (_stage) {
    case Stage::Open:
      return Open(studio);
    case Stage::Load:
      return Load(studio);
    case Stage::Capture:
      return Capture(studio);
    case Stage::Done:
      break;
  }
  return false;
}

bool ScreenshotRun::Open(SuperDexStudio& studio) {
  _stage = Stage::Done;
  if (studio.FindImporterForPath(mochi::Path{_options.openPath}) != nullptr) {
    return Report(
        _options.openPath +
        " opens in an import dialog, which needs someone at the window; open what the import "
        "writes instead");
  }
  if (!studio.OpenPath(mochi::Path{_options.openPath})) {
    return Report("could not open " + _options.openPath + "; the log above says why");
  }
  AssetEditor* const editor = studio.GetActiveAssetEditor();
  if (editor == nullptr || editor->GetViewport() == nullptr) {
    return Report(_options.openPath + " did not open in an editor with a viewport");
  }
  if (!_options.viewportOnly && _options.width > 0) {
    ImGuios::Window* const window = studio.GetMainWindow();
    if (window == nullptr) {
      return Report("there is no window to resize");
    }
    glfwRestoreWindow(window->GetGLFW());
    glfwSetWindowSize(window->GetGLFW(), _options.width, _options.height);
    _resizeDeadline = std::chrono::steady_clock::now() + kResizeTimeout;
  }
  _stage = Stage::Load;
  _stageFrame = ImGui::GetFrameCount();
  _loadDeadline = std::chrono::steady_clock::now() + kLoadTimeout;
  return true;
}

bool ScreenshotRun::Load(SuperDexStudio& studio) {
  _stage = Stage::Done;
  AssetEditor* const editor = studio.GetActiveAssetEditor();
  Viewport* const viewport = editor != nullptr ? editor->GetViewport() : nullptr;
  if (editor == nullptr || viewport == nullptr) {
    return Report("the editor closed while loading");
  }
  bool const loading = ImGui::GetFrameCount() < _stageFrame + kSettleFrames ||
      studio.IsAsyncTasksRunning() || viewport->IsCameraMoving();
  if (loading) {
    if (std::chrono::steady_clock::now() > _loadDeadline) {
      return Report("the file was still loading after 10 minutes");
    }
    _stage = Stage::Load;
    return true;
  }

  if (!viewport->HasVisibleGeometry()) {
    return Report(
        _options.openPath +
        " shows nothing to capture: no visible geometry loaded (the log above "
        "may say why)");
  }
  if (_options.camera.has_value()) {
    viewport->SetCameraLookAt(ToReal3(_options.camera->eye), ToReal3(_options.camera->target));
  } else if (_options.focus.empty()) {
    viewport->FocusCameraOnScene(std::nullopt, /*animate=*/false);
  } else {
    bool ambiguous = false;
    std::vector<mochi_renderer::SceneObject*> const actors =
        viewport->FindActors(_options.focus, &ambiguous);
    if (ambiguous) {
      return Report(
          "'" + _options.focus + "' names parts of more than one actor; give its full path");
    }
    if (actors.empty()) {
      return Report("no visible actor is named '" + _options.focus + "' or sits under it");
    }
    viewport->FocusCameraOnSceneObjects(actors, std::nullopt, /*animate=*/false);
  }
  auto const [eye, target] = viewport->GetCameraLookAt();
  _cameraText = "eye=" + VectorText(eye) + " target=" + VectorText(target);

  if (!_options.viewportOnly) {
    _stage = Stage::Capture;
    _stageFrame = ImGui::GetFrameCount();
    return true;
  }
  int width = _options.width;
  int height = _options.height;
  if (width == 0) {
    RenderTarget const* const renderTarget = viewport->GetRenderTarget();
    if (renderTarget == nullptr) {
      return Report("the viewport has no render target to capture");
    }
    renderTarget->GetSize(width, height);
    if (width < 1 || height < 1) {
      return Report("the viewport has no size to capture yet");
    }
  }
  mochi::Error error;
  studio.RenderViewportScreenshot(*editor, width, height, mochi::Path{_options.outPath}, error);
  return Finish(error.IsOK() ? nullptr : error.GetDescription(), width, height);
}

bool ScreenshotRun::Capture(SuperDexStudio& studio) {
  _stage = Stage::Done;
  ImGuios::Window* const window = studio.GetMainWindow();
  if (window == nullptr) {
    return Report("there is no window to capture");
  }
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window->GetGLFW(), &width, &height);
  if (ImGui::GetFrameCount() < _stageFrame + kCaptureFrames) {
    _stage = Stage::Capture;
    return true;
  }
  bool const resized =
      _options.width == 0 || (width == _options.width && height == _options.height);
  if (!resized) {
    if (std::chrono::steady_clock::now() < _resizeDeadline) {
      // Asked again each frame: restoring a maximized window can land after the first request.
      glfwSetWindowSize(window->GetGLFW(), _options.width, _options.height);
      _stage = Stage::Capture;
      return true;
    }
    return Report(
        mochi::Format(
            "the window stayed %dx%d instead of %dx%d; use a larger display or --viewport-only",
            width,
            height,
            _options.width,
            _options.height));
  }
  mochi::Error error;
  studio.SaveWindowScreenshot(mochi::Path{_options.outPath}, error);
  return Finish(error.IsOK() ? nullptr : error.GetDescription(), width, height);
}

bool ScreenshotRun::Finish(char const* error, int width, int height) {
  _stage = Stage::Done;
  if (error != nullptr) {
    return Report(error);
  }
  std::printf(
      "Saved %s (%dx%d), camera %s\n",
      _options.outPath.c_str(),
      width,
      height,
      _cameraText.c_str());
  std::fflush(stdout);
  _exitCode = 0;
  return false;
}

} // namespace superdex::studio
