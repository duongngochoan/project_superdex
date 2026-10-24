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

#include "assets/controller_params.h"

#include <superdex_robotics/controllers/controller_basic_jsc_pd.h>

#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_core/utils/file_utils.h>
#include <mochi_core/utils/reflection.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

using namespace superdex::studio;
using namespace mochi::test;
using superdex::robotics::ControllerBasicJscPd;

namespace {

constexpr std::string_view kJsc = ControllerBasicJscPd::TypeName();

class ControllerParamsTest : public testing::Test {
 protected:
  mochi::TempDirCleanup _tempDirCleanup =
      mochi::CreateTempDirectory("controller_params_test", ExpectOK{});
  std::filesystem::path _tempDir = _tempDirCleanup.Path();

  [[nodiscard]] std::string TempPath(std::string_view filename) const {
    return (_tempDir / filename).generic_string();
  }

  // Normalized JSC params whose only non-default value is a single Kp gain.
  static std::string JscWithKp(mochi::real kp) {
    superdex::robotics::ControllerBasicJscPdParams params;
    params.Kp = {kp};
    return SReflect::ToJsonString(params, true);
  }

  [[nodiscard]] std::string WriteJscFile(std::string_view filename, mochi::real kp) const {
    std::string const path = TempPath(filename);
    std::string error;
    EXPECT_TRUE(SaveParamsFile(kJsc, path, JscWithKp(kp), error)) << error;
    return path;
  }
};

TEST_F(ControllerParamsTest, DefaultsNormalizeToThemselves) {
  std::string const defaults = DefaultParamsJson(kJsc);
  std::string normalized;
  std::string error;
  ASSERT_TRUE(NormalizeParamsJson(kJsc, defaults, normalized, error)) << error;
  EXPECT_EQ(normalized, defaults);
}

TEST_F(ControllerParamsTest, LegacyTypeNamesUseTheirController) {
  EXPECT_EQ(CanonicalControllerType("joint_space_pd_v1"), kJsc);
  EXPECT_TRUE(IsBuiltinControllerType("joint_space_pd_v1"));
  EXPECT_EQ(DefaultParamsJson("joint_space_pd_v1"), DefaultParamsJson(kJsc));
}

TEST_F(ControllerParamsTest, NormalizeRejectsMalformedJson) {
  std::string normalized;
  std::string error;
  EXPECT_FALSE(NormalizeParamsJson(kJsc, "{", normalized, error));
  EXPECT_FALSE(error.empty());
}

TEST_F(ControllerParamsTest, CustomTypeNormalizesAnyJsonObject) {
  std::string normalized;
  std::string error;
  ASSERT_TRUE(NormalizeParamsJson("MyCustomController", R"({ "a":1 })", normalized, error))
      << error;
  std::string again;
  ASSERT_TRUE(NormalizeParamsJson("MyCustomController", normalized, again, error)) << error;
  EXPECT_EQ(again, normalized);
}

TEST_F(ControllerParamsTest, CustomTypeRejectsNonObject) {
  std::string normalized;
  std::string error;
  EXPECT_FALSE(NormalizeParamsJson("MyCustomController", "[1, 2]", normalized, error));
}

TEST_F(ControllerParamsTest, SaveThenLoadRoundTrips) {
  std::string const path = TempPath("jsc.superdex_controller");
  std::string const json = DefaultParamsJson(kJsc);
  std::string error;
  ASSERT_TRUE(SaveParamsFile(kJsc, path, json, error)) << error;
  std::string loaded;
  ASSERT_TRUE(LoadParamsFile(kJsc, path, loaded, error)) << error;
  EXPECT_EQ(loaded, json);
}

TEST_F(ControllerParamsTest, SaveCreatesParentDirectory) {
  std::string const path = TempPath("controllers/jsc.superdex_controller");
  std::string error;
  ASSERT_TRUE(SaveParamsFile(kJsc, path, DefaultParamsJson(kJsc), error)) << error;
  EXPECT_TRUE(std::filesystem::is_regular_file(path));
}

TEST_F(ControllerParamsTest, LoadMissingFileFails) {
  std::string loaded;
  std::string error;
  EXPECT_FALSE(LoadParamsFile(kJsc, TempPath("missing.superdex_controller"), loaded, error));
  EXPECT_FALSE(error.empty());
}

TEST_F(ControllerParamsTest, CacheEntryIsUnmodifiedUntilEdited) {
  std::string const path = WriteJscFile("jsc.superdex_controller", 1.0f);
  ControllerParamsCache cache;
  auto& entry = cache.GetOrLoad(path, kJsc);
  ASSERT_TRUE(entry.error.empty()) << entry.error;
  EXPECT_EQ(entry.json, JscWithKp(1.0f));
  EXPECT_FALSE(ControllerParamsCache::IsModified(entry));

  entry.json = JscWithKp(2.0f);
  EXPECT_TRUE(ControllerParamsCache::IsModified(entry));
  entry.json = JscWithKp(1.0f);
  EXPECT_FALSE(ControllerParamsCache::IsModified(entry));
}

TEST_F(ControllerParamsTest, CacheLoadsEachPathOnce) {
  std::string const path = WriteJscFile("jsc.superdex_controller", 1.0f);
  ControllerParamsCache cache;
  cache.GetOrLoad(path, kJsc).json = JscWithKp(2.0f);
  EXPECT_EQ(cache.GetOrLoad(path, kJsc).json, JscWithKp(2.0f));
}

TEST_F(ControllerParamsTest, CacheMissingFileRecordsErrorAndIsNotWritten) {
  std::string const path = TempPath("missing.superdex_controller");
  ControllerParamsCache cache;
  auto& entry = cache.GetOrLoad(path, kJsc);
  EXPECT_FALSE(entry.error.empty());
  EXPECT_FALSE(ControllerParamsCache::IsModified(entry));

  std::string error;
  EXPECT_TRUE(cache.SaveModified(error)) << error;
  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST_F(ControllerParamsTest, CacheWritesAdoptedEntry) {
  std::string const path = TempPath("new.superdex_controller");
  ControllerParamsCache cache;
  EXPECT_TRUE(ControllerParamsCache::IsModified(cache.Adopt(path, kJsc, JscWithKp(3.0f))));

  std::string error;
  ASSERT_TRUE(cache.SaveModified(error)) << error;
  EXPECT_FALSE(ControllerParamsCache::IsModified(*cache.Find(path)));
  std::string loaded;
  ASSERT_TRUE(LoadParamsFile(kJsc, path, loaded, error)) << error;
  EXPECT_EQ(loaded, JscWithKp(3.0f));
}

TEST_F(ControllerParamsTest, CacheSaveAsMovesEditsToTheNewFile) {
  std::string const original = WriteJscFile("jsc.superdex_controller", 1.0f);
  std::string const copy = TempPath("copy.superdex_controller");
  ControllerParamsCache cache;
  cache.GetOrLoad(original, kJsc).json = JscWithKp(2.0f);

  cache.SaveAs(original, copy);
  std::string error;
  ASSERT_TRUE(cache.SaveModified(error)) << error;

  std::string loaded;
  ASSERT_TRUE(LoadParamsFile(kJsc, copy, loaded, error)) << error;
  EXPECT_EQ(loaded, JscWithKp(2.0f));
  ASSERT_TRUE(LoadParamsFile(kJsc, original, loaded, error)) << error;
  EXPECT_EQ(loaded, JscWithKp(1.0f));
}

TEST_F(ControllerParamsTest, CacheRestoreValuesRevertsEdits) {
  std::string const path = WriteJscFile("jsc.superdex_controller", 1.0f);
  ControllerParamsCache cache;
  cache.GetOrLoad(path, kJsc);
  auto const captured = cache.CaptureValues();

  cache.Find(path)->json = JscWithKp(2.0f);
  cache.RestoreValues(captured);
  EXPECT_FALSE(ControllerParamsCache::IsModified(*cache.Find(path)));
}

TEST_F(ControllerParamsTest, CacheRestoreResetsEntriesMissingFromSnapshot) {
  std::string const path = WriteJscFile("jsc.superdex_controller", 1.0f);
  ControllerParamsCache cache;
  auto const beforeLoad = cache.CaptureValues();
  cache.GetOrLoad(path, kJsc).json = JscWithKp(2.0f);

  cache.RestoreValues(beforeLoad);
  EXPECT_EQ(cache.Find(path)->json, JscWithKp(1.0f));
}

// A one-bot scene whose controllers have the given params.
superdex::robotics::BotScenePrefab MakeScene(std::initializer_list<std::string> params) {
  superdex::robotics::BotScenePrefab prefab;
  superdex::robotics::BotEntry bot;
  bot.name = "robot";
  int index = 0;
  for (auto const& value : params) {
    superdex::robotics::ControllerEntry controller;
    controller.name = "c" + std::to_string(index++);
    controller.type = kJsc;
    controller.params = value;
    bot.controllers.push_back(std::move(controller));
  }
  prefab.bots.push_back(std::move(bot));
  return prefab;
}

TEST_F(ControllerParamsTest, ExternalizeClearsInlineDefaults) {
  auto prefab = MakeScene({DefaultParamsJson(kJsc)});
  ControllerParamsCache cache;
  int resolveCount = 0;
  std::string error;
  ASSERT_TRUE(ExternalizeInlineParams(
      prefab,
      cache,
      [&](auto const&, auto const&) {
        ++resolveCount;
        return std::string();
      },
      error))
      << error;
  EXPECT_TRUE(prefab.bots[0].controllers[0].params.empty());
  EXPECT_EQ(resolveCount, 0);
}

TEST_F(ControllerParamsTest, ExternalizeAdoptsInlineEditsAtResolvedPath) {
  auto prefab = MakeScene({JscWithKp(4.0f)});
  ControllerParamsCache cache;
  std::string const path = TempPath("robot_c0.superdex_controller");
  std::string error;
  ASSERT_TRUE(ExternalizeInlineParams(
      prefab, cache, [&](auto const&, auto const&) { return std::string(path); }, error))
      << error;
  EXPECT_EQ(std::string(prefab.bots[0].controllers[0].params), path);
  auto const* entry = cache.Find(path);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->json, JscWithKp(4.0f));
  EXPECT_TRUE(ControllerParamsCache::IsModified(*entry));
}

TEST_F(ControllerParamsTest, ExternalizeLeavesFilesAndDefaultsAlone) {
  std::string const path = WriteJscFile("jsc.superdex_controller", 1.0f);
  auto prefab = MakeScene({path, ""});
  ControllerParamsCache cache;
  std::string error;
  ASSERT_TRUE(ExternalizeInlineParams(
      prefab, cache, [&](auto const&, auto const&) { return std::string("unused"); }, error))
      << error;
  EXPECT_EQ(std::string(prefab.bots[0].controllers[0].params), path);
  EXPECT_TRUE(prefab.bots[0].controllers[1].params.empty());
  EXPECT_EQ(cache.Find("unused"), nullptr);
}

TEST_F(ControllerParamsTest, ExternalizeCancelLeavesSceneUnchanged) {
  auto prefab = MakeScene({JscWithKp(4.0f), JscWithKp(5.0f)});
  ControllerParamsCache cache;
  std::string const firstPath = TempPath("first.superdex_controller");
  std::string error;
  EXPECT_FALSE(ExternalizeInlineParams(
      prefab,
      cache,
      [&](auto const&, auto const& controller) {
        return controller.name == "c0" ? firstPath : std::string();
      },
      error));
  EXPECT_EQ(std::string(prefab.bots[0].controllers[0].params), JscWithKp(4.0f));
  EXPECT_EQ(std::string(prefab.bots[0].controllers[1].params), JscWithKp(5.0f));
  EXPECT_EQ(cache.Find(firstPath), nullptr);
}

TEST_F(ControllerParamsTest, ExternalizeRejectsInvalidInlineJson) {
  auto prefab = MakeScene({"{"});
  ControllerParamsCache cache;
  std::string error;
  EXPECT_FALSE(ExternalizeInlineParams(
      prefab, cache, [&](auto const&, auto const&) { return std::string("unused"); }, error));
  EXPECT_FALSE(error.empty());
}

auto const& JscParamsInfo() {
  return static_cast<SReflect::StructTypeInfo const&>(
      SReflect::GetTypeInfo<superdex::robotics::ControllerBasicJscPdParams>());
}

TEST_F(ControllerParamsTest, ResizeArrayFieldsExtendsWithLastValue) {
  superdex::robotics::ControllerBasicJscPdParams params;
  params.Kp = {2.0f};
  EXPECT_TRUE(ResizeArrayFields(JscParamsInfo(), &params, 3));
  using Reals = mochi::DynamicArray<mochi::real>;
  EXPECT_EQ(params.Kp, (Reals{2.0f, 2.0f, 2.0f}));
  EXPECT_EQ(params.Kd, (Reals{0.0f, 0.0f, 0.0f}));
}

TEST_F(ControllerParamsTest, ResizeArrayFieldsTruncatesThenReportsNoChange) {
  superdex::robotics::ControllerBasicJscPdParams params;
  params.Kp = {1.0f, 2.0f, 3.0f};
  params.Kd = {4.0f, 5.0f};
  params.saturation = {6.0f, 7.0f};
  params.deadband = {8.0f, 9.0f};
  EXPECT_TRUE(ResizeArrayFields(JscParamsInfo(), &params, 2));
  EXPECT_EQ(params.Kp, (mochi::DynamicArray<mochi::real>{1.0f, 2.0f}));
  EXPECT_FALSE(ResizeArrayFields(JscParamsInfo(), &params, 2));
}

} // namespace
