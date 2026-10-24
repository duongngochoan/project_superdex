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

// Runs processing pipelines headlessly (app/process_command.h) on a cube written by the test, and
// checks the files they export. The SDF stages need the superdex_mesh_cli helper, which the test
// target points SUPERDEX_MESH_CLI_PATH at.

#include "app/process_command.h"

#include <mochi_core/geometry/model_data.h>
#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_core/utils/file_utils.h>
#include <mochi_physics/utils/mochi_model_utils.h>
#include <mochi_renderer/utils.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace superdex::studio;
namespace fs = std::filesystem;

namespace {

constexpr float kHalfSide = 0.05f; // a 10 cm cube centered on the origin
constexpr int kCubeTriangles = 12;

// Writes a binary STL of the cube, outward-wound.
void WriteCubeStl(fs::path const& path) {
  fs::create_directories(path.parent_path());
  float const h = kHalfSide;
  std::array<std::array<float, 3>, 8> const corner{{
      {-h, -h, -h},
      {h, -h, -h},
      {-h, h, -h},
      {h, h, -h},
      {-h, -h, h},
      {h, -h, h},
      {-h, h, h},
      {h, h, h},
  }};
  // Two triangles per side (-x, +x, -y, +y, -z, +z), corners indexed as bit 0 = x, 1 = y, 2 = z.
  std::array<std::array<int, 4>, 6> const side{{
      {0, 4, 6, 2},
      {1, 3, 7, 5},
      {0, 1, 5, 4},
      {2, 6, 7, 3},
      {0, 2, 3, 1},
      {4, 5, 7, 6},
  }};
  std::ofstream out(path, std::ios::binary);
  std::array<char, 80> const header{};
  out.write(header.data(), header.size());
  uint32_t const count = kCubeTriangles;
  out.write(reinterpret_cast<char const*>(&count), sizeof(count));
  auto writeTriangle = [&](int a, int b, int c) {
    std::array<float, 3> const normal{};
    out.write(reinterpret_cast<char const*>(normal.data()), sizeof(normal));
    for (int const i : {a, b, c}) {
      out.write(reinterpret_cast<char const*>(corner[i].data()), sizeof(corner[i]));
    }
    uint16_t const attribute = 0;
    out.write(reinterpret_cast<char const*>(&attribute), sizeof(attribute));
  };
  for (auto const& quad : side) {
    writeTriangle(quad[0], quad[1], quad[2]);
    writeTriangle(quad[0], quad[2], quad[3]);
  }
}

// A pipeline: a CAD source, a mesh export, and an SDF export with 1 cm voxels. The mesh export
// passes its input on; the SDF export's own output is the SDF's reconstructed surface.
void WritePipeline(fs::path const& path, std::string const& source) {
  fs::create_directories(path.parent_path());
  std::ofstream(path) << R"({
  "version": 1,
  "modifiers": [
    )" << source << R"(,
    {"modifier": "Export Mesh File", "method": "Mesh File", "enabled": true, "collapsed": true,
     "properties": {"color": [0.2, 0.4, 0.6]}},
    {"modifier": "Export Mochi Model", "method": "SDF", "enabled": true, "collapsed": true,
     "properties": {"sdf": {"resolutionMode": "Explicit", "resolutionDelta": [0.01, 0.01, 0.01],
                            "boundaryPaddingDist": 0.01, "minGridResolution": [6, 6, 6]}}}
  ]
})";
}

constexpr char const* kSourceFromModelViewer =
    R"({"modifier": "Source from CAD Model", "method": "From Model Viewer", "enabled": true,
        "collapsed": true, "properties": {}})";

class ProcessCommandTest : public ::testing::Test {
 protected:
  // Checks the .mochi.h5 holds the cube and a grid SDF that is negative inside it.
  static void ExpectCubeModel(fs::path const& path, mochi::Int3 const& expectedDims) {
    mochi::Error error;
    mochi::ModelData const model = mochi::model_utils::LoadFromFile(path.string(), error);
    ASSERT_TRUE(error.IsOK()) << error.GetDescription();
    ASSERT_TRUE(model.mesh.has_value());
    EXPECT_EQ(model.mesh->GetNumElements(), kCubeTriangles);
    ASSERT_TRUE(model.sdf.has_value());
    EXPECT_EQ(model.sdf->dims, expectedDims);
    auto const [lowest, highest] =
        std::minmax_element(model.sdf->values.begin(), model.sdf->values.end());
    EXPECT_LT(*lowest, -0.8f * kHalfSide); // deep inside, near the center
    EXPECT_GT(*highest, 0.0f); // the padding outside
  }

  static int GlbTriangles(fs::path const& path) {
    int triangles = 0;
    for (mochi_renderer::MeshSection const& section :
         mochi_renderer::ReadGlbFromFile(path.string().c_str())) {
      triangles += static_cast<int>(section.indices.size() / 3);
    }
    return triangles;
  }

  mochi::TempDirCleanup _rootCleanup =
      mochi::CreateTempDirectory("process_command_test", mochi::test::ExpectOK{});
  fs::path _root = _rootCleanup.Path();
};

} // namespace

TEST_F(ProcessCommandTest, ExportsNextToTheAssetThatOwnsThePipeline) {
  WriteCubeStl(_root / "cad" / "cube.stl");
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  WritePipeline(pipeline, kSourceFromModelViewer);

  mochi::Error error;
  ProcessReport const report = RunProcessingPipeline({.pipelinePath = pipeline.string()}, error);

  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  EXPECT_TRUE(report.succeeded);
  EXPECT_TRUE(fs::equivalent(report.slots.cadPath, _root / "cad" / "cube.stl"));
  ASSERT_EQ(report.stages.size(), 3u);
  EXPECT_EQ(report.stages[0].triangles, kCubeTriangles);
  EXPECT_TRUE(fs::equivalent(report.stages[1].exportedPath, _root / "render" / "cube.glb"));
  ASSERT_TRUE(report.stages[2].sdfGrid.has_value());
  EXPECT_TRUE(fs::equivalent(report.stages[2].exportedPath, _root / "collision" / "cube.mochi.h5"));
  ExpectCubeModel(_root / "collision" / "cube.mochi.h5", *report.stages[2].sdfGrid);
  EXPECT_EQ(GlbTriangles(_root / "render" / "cube.glb"), kCubeTriangles);
}

TEST_F(ProcessCommandTest, OutDirCollectsTheExports) {
  WriteCubeStl(_root / "asset" / "cad" / "cube.stl");
  fs::path const pipeline = _root / "asset" / "intermediates" / "cube.StudioProcessing.json";
  WritePipeline(pipeline, kSourceFromModelViewer);
  fs::path const out = _root / "out";

  mochi::Error error;
  ProcessReport const report =
      RunProcessingPipeline({.pipelinePath = pipeline.string(), .outDir = out.string()}, error);

  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  EXPECT_TRUE(report.succeeded);
  EXPECT_TRUE(fs::is_regular_file(out / "cube.mochi.h5"));
  EXPECT_TRUE(fs::is_regular_file(out / "cube.glb"));
  ASSERT_EQ(report.stages.size(), 3u);
  EXPECT_EQ(
      report.stages[1].pipelineExportPath, (_root / "asset" / "render" / "cube.glb").string());
  EXPECT_EQ(
      report.stages[2].pipelineExportPath,
      (_root / "asset" / "collision" / "cube.mochi.h5").string());
  EXPECT_FALSE(fs::exists(_root / "asset" / "collision"));
  EXPECT_FALSE(fs::exists(_root / "asset" / "render"));
}

TEST_F(ProcessCommandTest, ExportsThatWouldWriteTheSameFileAreRefused) {
  WriteCubeStl(_root / "cad" / "cube.stl");
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  // Two mesh exports with different folders but the same file name.
  fs::create_directories(pipeline.parent_path());
  std::ofstream(pipeline) << R"({
  "version": 1,
  "modifiers": [
    )" << kSourceFromModelViewer
                          << R"(,
    {"modifier": "Export Mesh File", "method": "Mesh File", "enabled": true, "collapsed": true,
     "properties": {"color": [0.2, 0.4, 0.6]}},
    {"modifier": "Export Mesh File", "method": "Mesh File", "enabled": true, "collapsed": true,
     "properties": {"path": "../preview/cube.glb", "color": [0.2, 0.4, 0.6]}}
  ]
})";
  fs::path const out = _root / "out";

  mochi::Error separateError;
  ProcessReport const separate =
      RunProcessingPipeline({.pipelinePath = pipeline.string()}, separateError);
  mochi::Error collectedError;
  ProcessReport const collected = RunProcessingPipeline(
      {.pipelinePath = pipeline.string(), .outDir = out.string()}, collectedError);

  ASSERT_TRUE(separateError.IsOK()) << separateError.GetDescription();
  EXPECT_TRUE(separate.succeeded);
  EXPECT_TRUE(separate.collidingExportPaths.empty());
  EXPECT_TRUE(fs::is_regular_file(_root / "render" / "cube.glb"));
  EXPECT_TRUE(fs::is_regular_file(_root / "preview" / "cube.glb"));
  EXPECT_FALSE(collectedError.IsOK());
  EXPECT_TRUE(collected.stages.empty());
  EXPECT_EQ(collected.collidingExportPaths, std::vector<std::string>{(out / "cube.glb").string()});
  EXPECT_FALSE(fs::exists(out));
}

TEST_F(ProcessCommandTest, OverrideLetsTheLaterOfTwoCollidingExportsWin) {
  WriteCubeStl(_root / "cad" / "cube.stl");
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  // Two mesh exports to the same file, the second with a color the first does not use.
  fs::create_directories(pipeline.parent_path());
  std::ofstream(pipeline) << R"({
  "version": 1,
  "modifiers": [
    )" << kSourceFromModelViewer
                          << R"(,
    {"modifier": "Export Mesh File", "method": "Mesh File", "enabled": true, "collapsed": true,
     "properties": {"path": "../render/cube.glb", "color": [0.2, 0.4, 0.6]}},
    {"modifier": "Export Mesh File", "method": "Mesh File", "enabled": true, "collapsed": true,
     "properties": {"path": "../render/cube.glb", "color": [0.9, 0.1, 0.1]}}
  ]
})";

  mochi::Error error;
  ProcessReport const report = RunProcessingPipeline(
      {.pipelinePath = pipeline.string(), .allowCollidingExports = true}, error);

  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  EXPECT_TRUE(report.succeeded);
  ASSERT_EQ(report.collidingExportPaths.size(), 1u);
  EXPECT_TRUE(fs::equivalent(report.collidingExportPaths[0], _root / "render" / "cube.glb"));
  std::vector<mochi_renderer::MeshSection> const sections =
      mochi_renderer::ReadGlbFromFile((_root / "render" / "cube.glb").string().c_str());
  ASSERT_EQ(sections.size(), 1u);
  EXPECT_NEAR(sections[0].baseColor[0], 0.9f, 1e-3f);
}

TEST_F(ProcessCommandTest, CadOptionSuppliesAModelNamedDifferentlyFromThePipeline) {
  WriteCubeStl(_root / "cad" / "part_link.stl");
  fs::path const pipeline = _root / "intermediates" / "link.StudioProcessing.json";
  WritePipeline(pipeline, kSourceFromModelViewer);

  mochi::Error unresolved;
  ProcessReport const withoutCad =
      RunProcessingPipeline({.pipelinePath = pipeline.string()}, unresolved);
  EXPECT_FALSE(unresolved.IsOK());
  EXPECT_TRUE(withoutCad.stages.empty());

  mochi::Error error;
  ProcessReport const withCad = RunProcessingPipeline(
      {.pipelinePath = pipeline.string(),
       .slots = {.cadPath = (_root / "cad" / "part_link.stl").string()}},
      error);
  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  EXPECT_TRUE(withCad.succeeded);
  // Auto exports are named after the model the source read, not after the pipeline.
  EXPECT_TRUE(fs::is_regular_file(_root / "collision" / "part_link.mochi.h5"));
}

TEST_F(ProcessCommandTest, AFailedSourceFailsEveryStageAndWritesNothing) {
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  WritePipeline(
      pipeline,
      R"({"modifier": "Source from CAD Model", "method": "From File", "enabled": true,
          "collapsed": true, "properties": {"path": "../cad/missing.stl"}})");

  mochi::Error error;
  ProcessReport const report = RunProcessingPipeline({.pipelinePath = pipeline.string()}, error);

  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  EXPECT_FALSE(report.succeeded);
  ASSERT_EQ(report.stages.size(), 3u);
  for (ProcessStageReport const& stage : report.stages) {
    EXPECT_FALSE(stage.succeeded) << stage.label;
    EXPECT_FALSE(stage.error.empty()) << stage.label;
  }
  EXPECT_FALSE(fs::exists(_root / "collision"));
  EXPECT_FALSE(fs::exists(_root / "render"));
}

TEST_F(ProcessCommandTest, AnExportThatCannotBeWrittenFailsOnlyItsStage) {
  WriteCubeStl(_root / "cad" / "cube.stl");
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  WritePipeline(pipeline, kSourceFromModelViewer);
  std::ofstream(_root / "render") << "a file where the mesh export's folder should be";

  mochi::Error error;
  ProcessReport const report = RunProcessingPipeline({.pipelinePath = pipeline.string()}, error);

  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  EXPECT_FALSE(report.succeeded);
  ASSERT_EQ(report.stages.size(), 3u);
  EXPECT_TRUE(report.stages[0].succeeded);
  EXPECT_FALSE(report.stages[1].succeeded);
  EXPECT_TRUE(report.stages[1].exportedPath.empty());
  EXPECT_FALSE(report.stages[1].error.empty());
  EXPECT_NE(report.stages[1].error, "produced no mesh");
  EXPECT_TRUE(report.stages[2].succeeded);
  EXPECT_TRUE(fs::is_regular_file(_root / "collision" / "cube.mochi.h5"));
}

// Writes a pipeline of @p modifiers, each a JSON object, in order.
void WriteModifiers(fs::path const& path, std::vector<std::string> const& modifiers) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << R"({"version": 1, "modifiers": [)";
  for (std::size_t i = 0; i < modifiers.size(); ++i) {
    out << (i > 0 ? ",\n" : "\n") << modifiers[i];
  }
  out << "]}";
}

constexpr char const* kMeshExportOnAuto =
    R"({"modifier": "Export Mesh File", "method": "Mesh File", "enabled": true,
        "collapsed": true, "properties": {"color": [0.2, 0.4, 0.6]}})";

TEST_F(ProcessCommandTest, AnUnrecognizedModifierStopsTheRun) {
  WriteCubeStl(_root / "cad" / "cube.stl");
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  WriteModifiers(
      pipeline,
      {kSourceFromModelViewer,
       R"({"modifier": "Frobnicate Mesh", "method": "Wobble", "enabled": true,
           "collapsed": true, "properties": {"amount": 3}})",
       kMeshExportOnAuto});

  mochi::Error error;
  ProcessReport const report = RunProcessingPipeline({.pipelinePath = pipeline.string()}, error);

  EXPECT_FALSE(error.IsOK());
  EXPECT_EQ(report.unrecognizedModifiers, std::vector<std::string>{"Frobnicate Mesh / Wobble"});
  EXPECT_TRUE(report.stages.empty());
  EXPECT_FALSE(fs::exists(_root / "render"));
}

TEST_F(ProcessCommandTest, AnExportWithNoFilePathFailsItsStage) {
  WriteCubeStl(_root / "cad" / "cube.stl");
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  // Auto exports are named after the first modifier's model; this asset has no mochi model.
  WriteModifiers(
      pipeline,
      {R"({"modifier": "Source from Mochi Model", "method": "From Model Viewer",
           "enabled": false, "collapsed": true, "properties": {}})",
       kSourceFromModelViewer,
       kMeshExportOnAuto});

  mochi::Error error;
  ProcessReport const report = RunProcessingPipeline({.pipelinePath = pipeline.string()}, error);

  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  EXPECT_FALSE(report.succeeded);
  ASSERT_EQ(report.stages.size(), 2u);
  EXPECT_TRUE(report.stages[0].succeeded);
  EXPECT_FALSE(report.stages[1].succeeded);
  EXPECT_NE(report.stages[1].error.find("no file path"), std::string::npos)
      << report.stages[1].error;
}

TEST_F(ProcessCommandTest, ExportsOutsideTheChainDoNotCollide) {
  WriteCubeStl(_root / "cad" / "cube.stl");
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  // The first segment's source is off, so its export never runs.
  WriteModifiers(
      pipeline,
      {R"({"modifier": "Source from CAD Model", "method": "From Model Viewer", "enabled": false,
           "collapsed": true, "properties": {}})",
       kMeshExportOnAuto,
       kSourceFromModelViewer,
       kMeshExportOnAuto});

  mochi::Error error;
  ProcessReport const report = RunProcessingPipeline({.pipelinePath = pipeline.string()}, error);

  ASSERT_TRUE(error.IsOK()) << error.GetDescription();
  EXPECT_TRUE(report.succeeded);
  ASSERT_EQ(report.stages.size(), 2u);
  EXPECT_TRUE(fs::equivalent(report.stages[1].exportedPath, _root / "render" / "cube.glb"));
}

TEST_F(ProcessCommandTest, OutRefusesAPipelineWhoseSourceReadsAnEarlierExport) {
  WriteCubeStl(_root / "cad" / "cube.stl");
  fs::path const pipeline = _root / "intermediates" / "cube.StudioProcessing.json";
  // A first run leaves render/cube.glb, which the second segment's source then reads.
  WriteModifiers(pipeline, {kSourceFromModelViewer, kMeshExportOnAuto});
  mochi::Error firstError;
  ASSERT_TRUE(RunProcessingPipeline({.pipelinePath = pipeline.string()}, firstError).succeeded)
      << firstError.GetDescription();
  WriteModifiers(
      pipeline,
      {kSourceFromModelViewer,
       kMeshExportOnAuto,
       R"({"modifier": "Source from Render Model", "method": "From Model Viewer",
           "enabled": true, "collapsed": true, "properties": {}})",
       R"({"modifier": "Export Mesh File", "method": "Mesh File", "enabled": true,
           "collapsed": true, "properties": {"path": "../preview/cube.glb"}})"});
  fs::path const out = _root / "out";

  mochi::Error collectedError;
  ProcessReport const collected = RunProcessingPipeline(
      {.pipelinePath = pipeline.string(), .outDir = out.string()}, collectedError);
  mochi::Error inPlaceError;
  ProcessReport const inPlace =
      RunProcessingPipeline({.pipelinePath = pipeline.string()}, inPlaceError);

  EXPECT_FALSE(collectedError.IsOK());
  EXPECT_TRUE(fs::equivalent(collected.outConflictPath, _root / "render" / "cube.glb"));
  EXPECT_FALSE(fs::exists(out));
  ASSERT_TRUE(inPlaceError.IsOK()) << inPlaceError.GetDescription();
  EXPECT_TRUE(inPlace.succeeded);
  EXPECT_TRUE(fs::is_regular_file(_root / "preview" / "cube.glb"));
}

TEST_F(ProcessCommandTest, AnAssetInAFolderNamedLikeARoleFolderFindsItsOwnModels) {
  // The asset sits in a folder named "cad"; a stray model beside its role folders must not win.
  fs::path const asset = _root / "cad" / "part";
  WriteCubeStl(asset / "cad" / "cube.stl");
  WriteCubeStl(asset / "cube.stl");
  fs::path const pipeline = asset / "intermediates" / "cube.StudioProcessing.json";
  WritePipeline(pipeline, kSourceFromModelViewer);

  PipelineModelSlots const slots = FindPipelineModelSlots(pipeline);

  EXPECT_TRUE(fs::equivalent(slots.cadPath, asset / "cad" / "cube.stl")) << slots.cadPath;
}

TEST_F(ProcessCommandTest, ShadowedModelPipelineFillsOnlyThatModelsSlot) {
  fs::path const model = _root / "render" / "internal" / "part.glb";
  fs::create_directories(model.parent_path());
  std::ofstream(model) << "glb";
  fs::create_directories(_root / "cad");
  std::ofstream(_root / "cad" / "part.step") << "step";

  PipelineModelSlots const slots = FindPipelineModelSlots(
      _root / "intermediates" / "render" / "internal" / "part.glb.StudioProcessing.json");

  EXPECT_TRUE(fs::equivalent(slots.renderPath, model));
  EXPECT_EQ(slots.cadPath, "");
  EXPECT_EQ(slots.mochiPath, "");
}
