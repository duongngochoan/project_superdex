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

// Covers superdex_studio's command-line parsing (app/command_line.h).

#include "app/command_line.h"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

using namespace superdex::studio;

namespace {

// Parses @p args, expecting success.
CommandLine Parse(std::vector<std::string> const& args) {
  CommandLine commandLine;
  std::string error;
  EXPECT_TRUE(ParseCommandLine(args, commandLine, error)) << error;
  return commandLine;
}

// Parses @p args, expecting failure, and returns the error.
std::string ParseError(std::vector<std::string> const& args) {
  CommandLine commandLine;
  std::string error;
  EXPECT_FALSE(ParseCommandLine(args, commandLine, error));
  return error;
}

} // namespace

TEST(CommandLineTest, NoOptionsStartTheGui) {
  CommandLine const commandLine = Parse({});

  EXPECT_FALSE(commandLine.help);
  EXPECT_FALSE(commandLine.process.has_value());
}

TEST(CommandLineTest, TheGuiStillIgnoresFileArguments) {
  EXPECT_FALSE(Parse({"model.step"}).process.has_value());
}

TEST(CommandLineTest, TheGuiListsTheUnknownOptionsItIgnores) {
  CommandLine const commandLine = Parse({"--procss", "a.json", "--verbose"});

  EXPECT_FALSE(commandLine.process.has_value());
  EXPECT_EQ(commandLine.ignoredOptions, (std::vector<std::string>{"--procss", "--verbose"}));
}

TEST(CommandLineTest, ProcessTakesEveryOptionInEitherSpelling) {
  CommandLine const commandLine = Parse(
      {"--process",
       "intermediates/part.StudioProcessing.json",
       "--out=build",
       "--cad",
       "cad/part.step",
       "--render=render/part.glb",
       "--mochi",
       "collision/part.mochi.h5",
       "--override"});

  ASSERT_TRUE(commandLine.process.has_value());
  EXPECT_TRUE(commandLine.process->allowCollidingExports);
  EXPECT_EQ(commandLine.process->pipelinePath, "intermediates/part.StudioProcessing.json");
  EXPECT_EQ(commandLine.process->outDir, "build");
  EXPECT_EQ(commandLine.process->slots.cadPath, "cad/part.step");
  EXPECT_EQ(commandLine.process->slots.renderPath, "render/part.glb");
  EXPECT_EQ(commandLine.process->slots.mochiPath, "collision/part.mochi.h5");
}

TEST(CommandLineTest, HelpIsRecognizedInBothSpellings) {
  EXPECT_TRUE(Parse({"--help"}).help);
  EXPECT_TRUE(Parse({"-h"}).help);
}

TEST(CommandLineTest, AValueThatBeginsWithDashesTakesTheEqualsForm) {
  CommandLine const commandLine = Parse({"--process", "a.json", "--cad=--odd.step"});
  ASSERT_TRUE(commandLine.process.has_value());
  EXPECT_EQ(commandLine.process->slots.cadPath, "--odd.step");
}

TEST(CommandLineTest, MistakesAreReported) {
  EXPECT_EQ(ParseError({"--process", "a.json", "--procss"}), "unknown option '--procss'");
  EXPECT_EQ(ParseError({"--process"}), "--process needs a value");
  EXPECT_EQ(ParseError({"--process="}), "--process needs a value");
  EXPECT_EQ(ParseError({"--process", "--out", "x"}), "--process needs a value");
  EXPECT_EQ(
      ParseError({"--process", "a.json", "--out", "x", "--out", "y"}),
      "--out is given more than once");
  EXPECT_EQ(ParseError({"--out", "x"}), "--out only applies with --process");
  EXPECT_EQ(ParseError({"--override"}), "--override only applies with --process");
  EXPECT_EQ(ParseError({"--process", "a.json", "b.json"}), "unexpected argument 'b.json'");
}

TEST(CommandLineTest, ScreenshotTakesEveryOption) {
  CommandLine const commandLine = Parse(
      {"--screenshot",
       "out.png",
       "--open=board.mochi_prefab",
       "--size",
       "1280x720",
       "--focus",
       "board/screw",
       "--viewport-only"});

  ASSERT_TRUE(commandLine.screenshot.has_value());
  ScreenshotOptions const& screenshot = *commandLine.screenshot;
  EXPECT_EQ(screenshot.outPath, "out.png");
  EXPECT_EQ(screenshot.openPath, "board.mochi_prefab");
  EXPECT_EQ(screenshot.width, 1280);
  EXPECT_EQ(screenshot.height, 720);
  EXPECT_EQ(screenshot.focus, "board/screw");
  EXPECT_FALSE(screenshot.camera.has_value());
  EXPECT_TRUE(screenshot.viewportOnly);
  EXPECT_FALSE(commandLine.process.has_value());
}

TEST(CommandLineTest, ScreenshotDefaultsToTheWholeWindowAtItsSize) {
  ScreenshotOptions const screenshot =
      Parse({"--screenshot", "out.png", "--open", "bot.superdex_bot"}).screenshot.value();

  EXPECT_EQ(screenshot.width, 0);
  EXPECT_EQ(screenshot.height, 0);
  EXPECT_TRUE(screenshot.focus.empty());
  EXPECT_FALSE(screenshot.camera.has_value());
  EXPECT_FALSE(screenshot.viewportOnly);
}

TEST(CommandLineTest, CameraTakesEyeAndTargetAsTwoArgumentsOrOne) {
  std::vector<std::string> const base{"--screenshot", "out.png", "--open", "a.mochi_prefab"};
  auto parseCamera = [&](std::vector<std::string> const& cameraArgs) {
    std::vector<std::string> args = base;
    args.insert(args.end(), cameraArgs.begin(), cameraArgs.end());
    return Parse(args).screenshot.value().camera.value();
  };
  std::array<double, 3> const eye{0.5, -0.5, 0.25};
  std::array<double, 3> const target{0.0, 0.0, 0.1};

  for (auto const& cameraArgs : std::vector<std::vector<std::string>>{
           {"--camera", "eye=0.5,-0.5,0.25", "target=0,0,0.1"},
           {"--camera", "target=0,0,0.1", "eye=0.5,-0.5,0.25"},
           {"--camera", "eye=0.5,-0.5,0.25 target=0,0,0.1"},
           {"--camera=eye=0.5,-0.5,0.25", "target=0,0,0.1"},
       }) {
    CameraPlacement const camera = parseCamera(cameraArgs);
    EXPECT_EQ(camera.eye, eye) << cameraArgs[1];
    EXPECT_EQ(camera.target, target) << cameraArgs[1];
  }
}

TEST(CommandLineTest, ScreenshotMistakesAreReported) {
  EXPECT_EQ(ParseError({"--screenshot", "out.png"}), "--screenshot needs --open <file>");
  EXPECT_EQ(ParseError({"--open", "a.mochi_prefab"}), "--open only applies with --screenshot");
  EXPECT_EQ(ParseError({"--viewport-only"}), "--viewport-only only applies with --screenshot");
  EXPECT_EQ(
      ParseError({"--process", "p.json", "--screenshot", "out.png"}),
      "--process and --screenshot cannot be combined");

  std::vector<std::string> const base{"--screenshot", "out.png", "--open", "a.mochi_prefab"};
  auto error = [&](std::vector<std::string> const& extra) {
    std::vector<std::string> args = base;
    args.insert(args.end(), extra.begin(), extra.end());
    return ParseError(args);
  };
  std::string const sizeError = "--size must be <width>x<height>, each from 1 to 8192 pixels";
  EXPECT_EQ(error({"--size", "1280"}), sizeError);
  EXPECT_EQ(error({"--size", "0x720"}), sizeError);
  EXPECT_EQ(error({"--size", "9000x720"}), sizeError);
  std::string const cameraError =
      "--camera needs eye=<x,y,z> target=<x,y,z>, two different points with each coordinate "
      "within 1e6";
  EXPECT_EQ(error({"--camera", "eye=1,2,3"}), cameraError);
  EXPECT_EQ(error({"--camera", "eye=0,1e39,0", "target=0,0,0"}), cameraError);
  EXPECT_EQ(error({"--camera", "eye=1,2", "target=0,0,0"}), cameraError);
  EXPECT_EQ(error({"--camera", "eye=1,2,3", "target=1,2,3"}), cameraError);
  // Different as doubles, but the same point in the camera's float precision.
  EXPECT_EQ(error({"--camera", "eye=1,2,3", "target=1.00000001,2,3"}), cameraError);
  EXPECT_EQ(error({"--camera"}), "--camera needs eye=<x,y,z> target=<x,y,z>");
  EXPECT_EQ(
      error({"--focus", "box", "--camera", "eye=1,2,3", "target=0,0,0"}),
      "--focus and --camera cannot be combined");
  EXPECT_EQ(error({"extra.png"}), "unexpected argument 'extra.png'");
}
