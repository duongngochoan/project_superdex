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

#include "mochi_debugger_test.h"

#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_core/test/wait_until.h>
#include <mochi_physics/mochi_physics.h>
#include <mochi_physics/mochi_physics_experimental.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <initializer_list>
#include <optional>

using namespace mochi;
using namespace mochi::dbg;
using experimental::SleepParams;

namespace {

class SleepParamsTest : public MochiDebuggerTest {
 protected:
  static constexpr SleepParams kParamsA{
      .canSleep = true,
      .sleepThreshold = 0.25,
      .minStepsBeforeSleep = 5};
  static constexpr SleepParams kParamsB{
      .canSleep = true,
      .sleepThreshold = 0.75,
      .minStepsBeforeSleep = 7};

  static SleepParams GetServerParams(Scene* scene) {
    return experimental::GetSleepParams(scene, test::ExpectOK{});
  }

  // Pump the (paused) scenes until the client reports the expected value.
  bool WaitForClientParams(
      std::initializer_list<Scene*> scenes,
      std::optional<SleepParams> const& expected) {
    return test::WaitUntil([&] {
      for (Scene* scene : scenes) {
        scene->UpdateDebugger();
      }
      return _client->GetSleepParams() == expected;
    });
  }

  bool ClientLoggedWarning() {
    return _clientLogs.Read([](auto const& logs) {
      return std::ranges::any_of(
          logs, [](auto const& log) { return log.channel == LogChannel::Warning; });
    });
  }
};

} // namespace

TEST_F(SleepParamsTest, ReadsServerValueOnSelect) {
  Scene* scene = _context->CreateScene("SleepParamsTest");
  experimental::SetSleepParams(scene, kParamsA, test::ExpectOK{});
  StartServer();
  ConnectClient();
  ClientSelectScene(scene->GetHandle());

  EXPECT_TRUE(WaitForClientParams({scene}, kParamsA));
  EXPECT_EQ(kParamsA, GetServerParams(scene)); // The client did not overwrite it
}

TEST_F(SleepParamsTest, SetUpdatesServer) {
  Scene* scene = _context->CreateScene("SleepParamsTest");
  StartServer();
  ConnectClient();
  ClientSelectScene(scene->GetHandle());
  ASSERT_TRUE(WaitForClientParams({scene}, SleepParams{}));

  _client->SetSleepParams(kParamsA);
  EXPECT_EQ(kParamsA, _client->GetSleepParams()); // Shown before the server confirms it
  EXPECT_TRUE(test::WaitUntil([&] {
    scene->UpdateDebugger();
    return GetServerParams(scene) == kParamsA;
  }));
  EXPECT_TRUE(WaitForClientParams({scene}, kParamsA));
}

TEST_F(SleepParamsTest, SetIsIgnoredBeforeServerValueArrives) {
  Scene* scene = _context->CreateScene("SleepParamsTest");
  StartServer();
  ConnectClient();
  ClientSelectScene(scene->GetHandle());

  // The scene has not been pumped, so it has not replied yet.
  _client->SetSleepParams(kParamsA);
  EXPECT_EQ(std::nullopt, _client->GetSleepParams());
  EXPECT_TRUE(WaitForClientParams({scene}, SleepParams{}));
  EXPECT_EQ(SleepParams{}, GetServerParams(scene));
}

TEST_F(SleepParamsTest, SceneSwitchShowsNewScene) {
  Scene* sceneA = _context->CreateScene("A");
  Scene* sceneB = _context->CreateScene("B");
  experimental::SetSleepParams(sceneA, kParamsA, test::ExpectOK{});
  experimental::SetSleepParams(sceneB, kParamsB, test::ExpectOK{});
  StartServer();
  ConnectClient();
  ClientSelectScene(sceneA->GetHandle());
  ASSERT_TRUE(WaitForClientParams({sceneA, sceneB}, kParamsA));

  ClientSelectScene(sceneB->GetHandle());
  EXPECT_EQ(std::nullopt, _client->GetSleepParams()); // Scene A's value is cleared
  EXPECT_TRUE(WaitForClientParams({sceneA, sceneB}, kParamsB));
}

TEST_F(SleepParamsTest, InvalidSetIsRejectedByServer) {
  Scene* scene = _context->CreateScene("SleepParamsTest");
  experimental::SetSleepParams(scene, kParamsA, test::ExpectOK{});
  StartServer();
  ConnectClient();
  ClientSelectScene(scene->GetHandle());
  ASSERT_TRUE(WaitForClientParams({scene}, kParamsA));

  SleepParams invalid = kParamsA;
  invalid.minStepsBeforeSleep = 0;
  _client->SetSleepParams(invalid);

  EXPECT_TRUE(test::WaitUntil([&] {
    scene->UpdateDebugger();
    return ClientLoggedWarning();
  }));
  EXPECT_EQ(kParamsA, GetServerParams(scene));
  EXPECT_EQ(invalid, _client->GetSleepParams()); // The client's copy is not reverted
}

TEST_F(SleepParamsTest, ClearedOnDisconnect) {
  Scene* scene = _context->CreateScene("SleepParamsTest");
  StartServer();
  ConnectClient();
  ClientSelectScene(scene->GetHandle());
  ASSERT_TRUE(WaitForClientParams({scene}, SleepParams{}));

  DisconnectClient();
  EXPECT_EQ(std::nullopt, _client->GetSleepParams());
}

TEST_F(SleepParamsTest, ReconnectReadsServerValue) {
  Scene* scene = _context->CreateScene("SleepParamsTest");
  StartServer();
  ConnectClient();
  ClientSelectScene(scene->GetHandle());
  ASSERT_TRUE(WaitForClientParams({scene}, SleepParams{}));
  _client->SetSleepParams(kParamsA);
  ASSERT_TRUE(test::WaitUntil([&] {
    scene->UpdateDebugger();
    return GetServerParams(scene) == kParamsA;
  }));
  DisconnectClient();

  // The host changes the value while disconnected. The client shows it after reconnecting.
  experimental::SetSleepParams(scene, kParamsB, test::ExpectOK{});
  ConnectClient();
  ClientSelectScene(scene->GetHandle());
  EXPECT_TRUE(WaitForClientParams({scene}, kParamsB));
  EXPECT_EQ(kParamsB, GetServerParams(scene));
}
