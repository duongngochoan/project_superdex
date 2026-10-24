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

// Every pipeline file in the repository -- each asset's `*.StudioProcessing.json` under
// SUPERDEX_ASSETS_PATH and the presets under SUPERDEX_STUDIO_PRESETS_PATH -- loads with nothing
// left unread: no unknown field, enum value, modifier or method. Studio loads such a file anyway,
// falling back to defaults and placeholders, so a typo or a later rename would go unnoticed.

#include "meshing/processing_modifiers/processing_serialization.h"

#include <mochi_core/utils/path.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace superdex::studio;
namespace fs = std::filesystem;

namespace {

struct PipelineFile {
  std::string name; // unique, for the test's name
  fs::path path;
};

void PrintTo(PipelineFile const& file, std::ostream* out) {
  *out << file.name;
}

fs::path DirFromEnv(char const* variable) {
  char const* const dir = std::getenv(variable);
  return dir != nullptr ? fs::path(dir) : fs::path();
}

// The files under @p dir whose names end in @p suffix, named by @p prefix and their relative path.
std::vector<PipelineFile>
FindFiles(fs::path const& dir, char const* suffix, bool recursive, std::string const& prefix) {
  std::vector<PipelineFile> files;
  if (dir.empty() || !fs::is_directory(dir)) {
    return files;
  }
  mochi::Error error;
  for (fs::path const& file : mochi::path::ScanDirectoryForFiles(dir, suffix, recursive, error)) {
    files.push_back({prefix + file.generic_string(), dir / file});
  }
  return files;
}

std::vector<PipelineFile> FindPipelineFiles() {
  std::vector<PipelineFile> files = FindFiles(
      DirFromEnv("SUPERDEX_ASSETS_PATH"), ".StudioProcessing.json", /*recursive=*/true, "assets/");
  for (PipelineFile& preset : FindFiles(
           DirFromEnv("SUPERDEX_STUDIO_PRESETS_PATH"), ".json", /*recursive=*/false, "presets/")) {
    files.push_back(std::move(preset));
  }
  return files;
}

class PipelineLintTest : public ::testing::TestWithParam<PipelineFile> {};

} // namespace

TEST_P(PipelineLintTest, LoadsWithNothingLeftUnread) {
  fs::path const& path = GetParam().path;
  // The reader names each unknown field and enum value only on stdout.
  testing::internal::CaptureStdout();
  LoadedPipeline loaded;
  mochi::Error error;
  bool const read = LoadProcessingPipeline(path.string(), path, loaded, error);
  std::string const log = testing::internal::GetCapturedStdout();

  ASSERT_TRUE(read) << error.GetDescription();
  std::ostringstream issues;
  for (std::string const& issue : loaded.issues) {
    issues << "\n  " << issue;
  }
  EXPECT_TRUE(loaded.issues.empty())
      << path << " holds what Studio cannot read:" << issues.str() << "\n"
      << log << "Fix the file, or teach the loader the old name if a field or value was renamed.";
}

INSTANTIATE_TEST_SUITE_P(
    Files,
    PipelineLintTest,
    ::testing::ValuesIn(FindPipelineFiles()),
    [](::testing::TestParamInfo<PipelineFile> const& info) {
      std::string name = info.param.name;
      for (char& c : name) {
        if (!std::isalnum(static_cast<unsigned char>(c))) {
          c = '_';
        }
      }
      return name;
    });

TEST(PipelineLint, FindsAssetPipelinesAndPresets) {
  std::vector<PipelineFile> const files = FindPipelineFiles();
  auto const count = [&](std::string const& prefix) {
    return std::ranges::count_if(
        files, [&](PipelineFile const& file) { return file.name.starts_with(prefix); });
  };
  EXPECT_GT(count("assets/"), 0);
  EXPECT_GT(count("presets/"), 0);
}
