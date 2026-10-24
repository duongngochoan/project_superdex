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

#include "meshing/processing_modifiers/preset_discovery.h"

#include <gtest/gtest.h>
#include <mochi_core/mochi_platform.h>
#include <mochi_core/utils/file_utils.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace superdex::studio {
namespace {

constexpr char const* kOverrideVariable = "MOCHI_STUDIO_PRESETS_DIR";

// Sets the preset-folder override for this process, or clears it for nullptr.
void SetOverride(char const* value) {
#if MOCHI_PLATFORM_WINDOWS
  _putenv_s(kOverrideVariable, value == nullptr ? "" : value);
#else
  if (value == nullptr) {
    unsetenv(kOverrideVariable);
  } else {
    setenv(kOverrideVariable, value, 1);
  }
#endif
}

void WriteText(std::filesystem::path const& path, std::string_view text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream(path) << text;
}

std::vector<std::string> Names(std::vector<ProcessingPreset> const& presets) {
  std::vector<std::string> names;
  names.reserve(presets.size());
  for (ProcessingPreset const& preset : presets) {
    names.push_back(preset.name);
  }
  return names;
}

// Each test starts without the override and puts back whatever was set before.
class PresetDiscoveryTest : public testing::Test {
 protected:
  void SetUp() override {
    if (char const* const value = std::getenv(kOverrideVariable)) {
      _saved = value;
    }
    SetOverride(nullptr);
  }
  void TearDown() override {
    SetOverride(_saved.has_value() ? _saved->c_str() : nullptr);
  }

 private:
  std::optional<std::string> _saved;
};

TEST_F(PresetDiscoveryTest, AnEmptyExecutableFolderFindsNoPresets) {
  EXPECT_TRUE(DiscoverProcessingPresets({}).empty());
}

TEST_F(PresetDiscoveryTest, PresetsComeFromTheExecutableFoldersPresetsFolder) {
  mochi::Error error;
  mochi::TempDirCleanup const exeDir = mochi::CreateTempDirectory("preset_discovery_test", error);
  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  WriteText(exeDir.Path() / "processing_presets" / "Bake_SDF.json", R"({"description": "SDF"})");
  WriteText(exeDir.Path() / "processing_presets" / "Auto_Collision.json", "{}");
  WriteText(exeDir.Path() / "processing_presets" / "notes.txt", "not a preset");

  std::vector<ProcessingPreset> const presets = DiscoverProcessingPresets(exeDir.Path());

  EXPECT_EQ(Names(presets), (std::vector<std::string>{"Auto Collision", "Bake SDF"}));
  EXPECT_EQ(presets.at(1).description, "SDF");
}

TEST_F(PresetDiscoveryTest, TheOverrideFolderIsUsedEvenWithoutAnExecutableFolder) {
  mochi::Error error;
  mochi::TempDirCleanup const folder = mochi::CreateTempDirectory("preset_discovery_test", error);
  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  WriteText(folder.Path() / "Custom.json", "{}");
  SetOverride(folder.Path().string().c_str());

  EXPECT_EQ(Names(DiscoverProcessingPresets({})), (std::vector<std::string>{"Custom"}));
}

} // namespace
} // namespace superdex::studio
