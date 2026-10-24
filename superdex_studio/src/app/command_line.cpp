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

#include "app/command_line.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace superdex::studio {

namespace {

constexpr int kMaxImageSide = 8192;

// Parses "x,y,z".
bool ParseVector(std::string_view text, std::array<double, 3>& out) {
  for (std::size_t i = 0; i < out.size(); ++i) {
    std::size_t const comma = text.find(',');
    bool const last = i + 1 == out.size();
    if (last != (comma == std::string_view::npos)) {
      return false;
    }
    std::string const number(text.substr(0, comma));
    std::size_t used = 0;
    try {
      out[i] = std::stod(number, &used);
    } catch (std::logic_error const&) {
      return false;
    }
    // Far past any scene Studio shows, and within float range, so the camera stays finite.
    if (used != number.size() || !std::isfinite(out[i]) || std::abs(out[i]) > 1e6) {
      return false;
    }
    text = last ? std::string_view{} : text.substr(comma + 1);
  }
  return true;
}

// Whether @p a and @p b are different points once cast to float, the precision the camera uses.
bool DifferAsFloats(std::array<double, 3> const& a, std::array<double, 3> const& b) {
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (static_cast<float>(a[i]) != static_cast<float>(b[i])) {
      return true;
    }
  }
  return false;
}

// Parses "<width>x<height>".
bool ParseSize(std::string_view text, int& width, int& height) {
  std::size_t const x = text.find('x');
  if (x == std::string_view::npos) {
    return false;
  }
  auto const side = [](std::string_view digits, int& value) {
    if (digits.empty() || digits.size() > 4 ||
        digits.find_first_not_of("0123456789") != std::string_view::npos) {
      return false;
    }
    value = std::atoi(std::string(digits).c_str());
    return value >= 1 && value <= kMaxImageSide;
  };
  return side(text.substr(0, x), width) && side(text.substr(x + 1), height);
}

// Parses the words after --camera: "eye=x,y,z" and "target=x,y,z", in either order, as separate
// arguments or in one.
bool ParseCamera(std::vector<std::string> const& words, CameraPlacement& out) {
  bool haveEye = false;
  bool haveTarget = false;
  for (std::string const& word : words) {
    std::string_view rest = word;
    while (!rest.empty()) {
      std::size_t const space = rest.find(' ');
      std::string_view const part = rest.substr(0, space);
      rest = space == std::string_view::npos ? std::string_view{} : rest.substr(space + 1);
      if (part.empty()) {
        continue;
      }
      if (part.starts_with("eye=") && !haveEye) {
        haveEye = ParseVector(part.substr(4), out.eye);
        if (!haveEye) {
          return false;
        }
      } else if (part.starts_with("target=") && !haveTarget) {
        haveTarget = ParseVector(part.substr(7), out.target);
        if (!haveTarget) {
          return false;
        }
      } else {
        return false;
      }
    }
  }
  return haveEye && haveTarget && DifferAsFloats(out.eye, out.target);
}

} // namespace

bool ParseCommandLine(std::vector<std::string> const& args, CommandLine& out, std::string& error) {
  out = {};
  ProcessOptions process;
  ScreenshotOptions screenshot;
  std::string size;
  std::vector<std::string> cameraWords;
  bool viewportOnly = false;
  bool allowCollidingExports = false;
  struct ValueOption {
    std::string_view name;
    std::string& destination;
    // The option it only applies with, if any.
    std::string_view mode;
  };
  std::array<ValueOption, 9> const valueOptions{{
      {"--process", process.pipelinePath, {}},
      {"--out", process.outDir, "--process"},
      {"--cad", process.slots.cadPath, "--process"},
      {"--render", process.slots.renderPath, "--process"},
      {"--mochi", process.slots.mochiPath, "--process"},
      {"--screenshot", screenshot.outPath, {}},
      {"--open", screenshot.openPath, "--screenshot"},
      {"--size", size, "--screenshot"},
      {"--focus", screenshot.focus, "--screenshot"},
  }};
  std::vector<std::string> positional;
  std::vector<std::string> unknown;

  for (std::size_t i = 0; i < args.size(); ++i) {
    std::string_view const arg = args[i];
    if (arg == "-h" || arg == "--help") {
      out.help = true;
      continue;
    }
    if (arg == "--viewport-only") {
      viewportOnly = true;
      continue;
    }
    if (arg == "--override") {
      allowCollidingExports = true;
      continue;
    }
    if (!arg.starts_with("--")) {
      positional.push_back(args[i]);
      continue;
    }
    std::size_t const equals = arg.find('=');
    std::string_view const name = arg.substr(0, equals);
    if (name == "--camera") {
      if (!cameraWords.empty()) {
        error = "--camera is given more than once";
        return false;
      }
      if (equals != std::string_view::npos) {
        cameraWords.emplace_back(arg.substr(equals + 1));
      }
      while (cameraWords.size() < 2 && i + 1 < args.size() &&
             (args[i + 1].starts_with("eye=") || args[i + 1].starts_with("target="))) {
        cameraWords.push_back(args[++i]);
      }
      if (cameraWords.empty()) {
        error = "--camera needs eye=<x,y,z> target=<x,y,z>";
        return false;
      }
      continue;
    }
    std::string* target = nullptr;
    for (ValueOption const& option : valueOptions) {
      if (name == option.name) {
        target = &option.destination;
      }
    }
    if (target == nullptr) {
      unknown.emplace_back(name);
      continue;
    }
    std::string value;
    if (equals != std::string_view::npos) {
      value = arg.substr(equals + 1);
    } else if (i + 1 < args.size() && !args[i + 1].starts_with("--")) {
      value = args[++i];
    }
    if (value.empty()) {
      error = std::string(name) + " needs a value";
      return false;
    }
    if (!target->empty()) {
      error = std::string(name) + " is given more than once";
      return false;
    }
    *target = std::move(value);
  }

  bool const processing = !process.pipelinePath.empty();
  bool const screenshotting = !screenshot.outPath.empty();
  if (processing && screenshotting) {
    error = "--process and --screenshot cannot be combined";
    return false;
  }
  for (ValueOption const& option : valueOptions) {
    bool const modeGiven = (option.mode == "--process" && processing) ||
        (option.mode == "--screenshot" && screenshotting);
    if (!option.mode.empty() && !modeGiven && !option.destination.empty()) {
      error = std::string(option.name) + " only applies with " + std::string(option.mode);
      return false;
    }
  }
  if (!processing && allowCollidingExports) {
    error = "--override only applies with --process";
    return false;
  }
  if (!screenshotting && (viewportOnly || !cameraWords.empty())) {
    error = std::string(viewportOnly ? "--viewport-only" : "--camera") +
        " only applies with --screenshot";
    return false;
  }
  if (!processing && !screenshotting) {
    out.ignoredOptions = std::move(unknown);
    return true;
  }
  if (!unknown.empty()) {
    error = "unknown option '" + unknown.front() + "'";
    return false;
  }
  if (!positional.empty()) {
    error = "unexpected argument '" + positional.front() + "'";
    return false;
  }
  if (processing) {
    process.allowCollidingExports = allowCollidingExports;
    out.process = std::move(process);
    return true;
  }

  if (screenshot.openPath.empty()) {
    error = "--screenshot needs --open <file>";
    return false;
  }
  if (!size.empty() && !ParseSize(size, screenshot.width, screenshot.height)) {
    error = "--size must be <width>x<height>, each from 1 to 8192 pixels";
    return false;
  }
  if (!cameraWords.empty()) {
    if (!screenshot.focus.empty()) {
      error = "--focus and --camera cannot be combined";
      return false;
    }
    CameraPlacement camera;
    if (!ParseCamera(cameraWords, camera)) {
      error =
          "--camera needs eye=<x,y,z> target=<x,y,z>, two different points with each coordinate "
          "within 1e6";
      return false;
    }
    screenshot.camera = camera;
  }
  screenshot.viewportOnly = viewportOnly;
  out.screenshot = std::move(screenshot);
  return true;
}

std::string CommandLineUsage() {
  return R"(Usage:
  superdex_studio
      Start SuperDex Studio.

  superdex_studio --process <pipeline> [--out <dir>] [--cad <file>] [--render <file>] [--mochi <file>]
                  [--override]
      Run a Model Editor processing pipeline (a .StudioProcessing.json) without opening a window:
      every enabled modifier in order, each export writing its file as soon as its stage has
      run, as Build/Export All does. Prints one line
      per stage and exits with 0 when every stage and export succeeded, 1 otherwise.

      The models a "From Model Viewer" source reads are found next to the pipeline, the way the
      Model Editor finds them for the model that owns it: <asset>/intermediates/<name>.StudioProcessing.json
      pairs with <name>.step, <name>.glb and so on in <asset>. Exports on Auto are named after the
      model the source read.

      Give a value that begins with -- as --option=<value>.

      --out <dir>      Write every export into <dir>, keeping each file name.
      --cad <file>     The CAD model (STEP or STL) to use instead.
      --render <file>  The render model to use instead.
      --mochi <file>   The mochi model (.mochi.h5) to use instead.
      --override       Run even when two exports would write the same file; the later one wins,
                       as in the Model Editor. Without it, such a pipeline is refused.

  superdex_studio --screenshot <png> --open <file> [--size <W>x<H>]
                  [--focus <actor> | --camera eye=<x,y,z> target=<x,y,z>] [--viewport-only]
      Open <file> in SuperDex Studio, wait until it has loaded, save a screenshot and exit with 0
      when the PNG was written, 1 otherwise. The whole scene is framed unless --focus or --camera
      says otherwise. Studio runs in its normal window, so a host without a display needs a
      virtual one, for example:
        xvfb-run -a -s "-screen 0 1920x1080x24" superdex_studio --screenshot ...

      --size <W>x<H>   The image size. Default: the window's size, or the viewport's.
      --focus <actor>  Frame this actor and everything under it, e.g. "board/screw".
      --camera eye=<x,y,z> target=<x,y,z>
                       Place the camera at eye, looking at target, in editor coordinates.
      --viewport-only  Save the 3D viewport alone, rendered at the image size, rather than the
                       whole window. Always so on macOS.

  superdex_studio --help
      Show this help.
)";
}

} // namespace superdex::studio
