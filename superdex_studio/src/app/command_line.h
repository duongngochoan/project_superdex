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

// superdex_studio's command line. With no options it starts the GUI. `--process` runs a processing
// pipeline without a window instead (see process_command.h), and `--screenshot` saves a picture of
// a file and exits (see screenshot_command.h).

#include "app/process_command.h"
#include "app/screenshot_command.h"

#include <optional>
#include <string>
#include <vector>

namespace superdex::studio {

struct CommandLine {
  bool help = false;
  std::optional<ProcessOptions> process;
  std::optional<ScreenshotOptions> screenshot;
  std::vector<std::string> ignoredOptions; // unknown options, when the GUI starts anyway
};

// Parses @p args, the arguments after the program name. Returns false and sets @p error on the
// first problem. Without `--process` or `--screenshot`, arguments that are not options are ignored,
// as they always were, so a launcher that passes a file name still starts the GUI; unknown options
// are ignored too, and listed so the caller can warn about them.
bool ParseCommandLine(std::vector<std::string> const& args, CommandLine& out, std::string& error);

// The `--help` text.
std::string CommandLineUsage();

} // namespace superdex::studio
