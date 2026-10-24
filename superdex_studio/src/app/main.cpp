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

#include <imguios/gpu_selector.h>

#include "app/app.h"
#include "app/command_line.h"
#include "app/process_command.h"

#include <mochi_core/mochi_platform.h>
#include <mochi_core/utils/console.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  // This is a GUI-subsystem binary on Windows, so it starts with no console and would otherwise
  // discard everything it prints -- including the reason it is about to fail.
  mochi::AttachParentConsole();

  using namespace superdex::studio;
  std::vector<std::string> const args(argv + 1, argv + argc);
  CommandLine commandLine;
  std::string error;
  if (!ParseCommandLine(args, commandLine, error)) {
    std::fprintf(stderr, "superdex_studio: %s\n\n%s", error.c_str(), CommandLineUsage().c_str());
    return 2;
  }
  if (commandLine.help) {
    std::fputs(CommandLineUsage().c_str(), stdout);
    return 0;
  }
  // Checked before the app is constructed: constructing it opens the window.
  if (commandLine.process.has_value()) {
    return RunProcessCommand(*commandLine.process);
  }
  for (std::string const& option : commandLine.ignoredOptions) {
    std::fprintf(stderr, "superdex_studio: ignoring unknown option '%s'\n", option.c_str());
  }

#if MOCHI_PLATFORM_LINUX
  if (commandLine.screenshot.has_value() && std::getenv("DISPLAY") == nullptr &&
      std::getenv("WAYLAND_DISPLAY") == nullptr) {
    std::fputs(
        "superdex_studio: --screenshot runs Studio's window and needs a display. Without one, use "
        "a virtual one:\n  xvfb-run -a -s \"-screen 0 1920x1080x24\" superdex_studio "
        "--screenshot ...\n",
        stderr);
    return 1;
  }
#endif

  SuperDexStudio app;
  if (commandLine.screenshot.has_value()) {
    app.StartScreenshotRun(*commandLine.screenshot);
  }
  app.Run();
  return app.GetExitCode();
}
