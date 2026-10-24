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

#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_mesh/step_face_colors.h>
#include <mochi_mesh/step_mesh_body.h>
#include <mochi_mesh/step_tessellation.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace mochi;
using namespace mochi::mesh;
using namespace mochi::test;

namespace {

// multi_face_colors.stp: one filleted solid of 16 faces. The solid is styled #CAD1EE and 11 faces
// carry their own STYLED_ITEM, read off the file's style chains.
constexpr uint32_t kBodyColor = 0xCAD1EEu;
constexpr uint32_t kRed = 0xFF0000u;
constexpr uint32_t kUnusedColor = 0x123456u;

std::string AssetPath(char const* name) {
  char const* const dir = std::getenv("MOCHI_MESH_TEST_ASSETS");
#ifdef MOCHI_MESH_TEST_ASSETS_DIR
  std::string const base = dir != nullptr ? dir : MOCHI_MESH_TEST_ASSETS_DIR;
#else
  std::string const base = dir != nullptr ? dir : "";
#endif
  return base + name;
}

std::string FixturePath() {
  return AssetPath("multi_face_colors.stp");
}

StepMeshBodyParams BaseParams() {
  StepMeshBodyParams params;
  params.targetEdgeLength = 4.0; // mm
  return params;
}

MeshData Mesh(StepMeshBodyParams const& params) {
  ExpectOK expectOK;
  return MeshStepBody(FixturePath(), params, expectOK);
}

// Edges used by exactly one triangle once coincident vertices are merged. MeshStepBody leaves
// shared-edge vertices unwelded but bit-identical across faces, so exact matching suffices; any
// open edge means two faces were sampled differently along the edge they share.
int CountOpenEdges(MeshData const& mesh) {
  std::map<std::array<real, 3>, int> welded;
  std::vector<int> remap(static_cast<size_t>(mesh.GetNumNodes()));
  for (int node = 0; node < mesh.GetNumNodes(); ++node) {
    std::array<real, 3> const key{
        mesh.coordinates[3 * node], mesh.coordinates[3 * node + 1], mesh.coordinates[3 * node + 2]};
    remap[static_cast<size_t>(node)] =
        welded.emplace(key, static_cast<int>(welded.size())).first->second;
  }

  std::map<std::pair<int, int>, int> edgeUses;
  for (int tri = 0; tri < mesh.GetNumElements(); ++tri) {
    for (int corner = 0; corner < 3; ++corner) {
      int const a = remap[static_cast<size_t>(mesh.connectivity[3 * tri + corner])];
      int const b = remap[static_cast<size_t>(mesh.connectivity[3 * tri + (corner + 1) % 3])];
      ++edgeUses[{std::min(a, b), std::max(a, b)}];
    }
  }
  int open = 0;
  for (auto const& [edge, uses] : edgeUses) {
    open += uses == 1 ? 1 : 0;
  }
  return open;
}

} // namespace

TEST(StepFaceColors, ReadsEveryDistinctColorWithItsFaceCount) {
  ExpectOK expectOK;
  std::vector<StepFaceColorInfo> colors;
  ASSERT_TRUE(ReadStepFaceColors(FixturePath(), colors, expectOK));

  std::map<uint32_t, uint32_t> faceCounts;
  for (StepFaceColorInfo const& info : colors) {
    EXPECT_GT(info.area, 0.0);
    faceCounts[info.rgb] = info.faceCount;
  }
  std::map<uint32_t, uint32_t> const expected{
      {0x0000FFu, 5},
      {0x00FFFFu, 1},
      {0x60C500u, 2},
      {0xC0FFFFu, 1},
      {kBodyColor, 5},
      {0xE87108u, 1},
      {kRed, 1},
  };
  EXPECT_EQ(faceCounts, expected);
}

TEST(StepFaceColors, ColorsAreOrderedByArea) {
  ExpectOK expectOK;
  std::vector<StepFaceColorInfo> colors;
  ASSERT_TRUE(ReadStepFaceColors(FixturePath(), colors, expectOK));
  for (size_t i = 1; i < colors.size(); ++i) {
    EXPECT_GE(colors[i - 1].area, colors[i].area);
  }
}

TEST(StepFaceColors, ReadRejectsMissingFile) {
  ExpectNotOK expectNotOK;
  std::vector<StepFaceColorInfo> colors;
  EXPECT_FALSE(ReadStepFaceColors(AssetPath("does_not_exist.stp"), colors, expectNotOK));
  EXPECT_TRUE(colors.empty());
}

// A color picked off the per-color preview must be a valid sizing key, so the preview is split by
// exactly the colors ReadStepFaceColors lists. Every face of the fixture is colored.
TEST(StepFaceColors, TessellateByColorSplitsByTheListedColors) {
  ExpectOK expectOK;
  std::vector<StepFaceColorInfo> colors;
  ASSERT_TRUE(ReadStepFaceColors(FixturePath(), colors, expectOK));
  std::vector<StepColorMesh> const meshes =
      TessellateStepByColor(FixturePath(), StepTessellationParams{}, expectOK);

  std::set<uint32_t> listed;
  for (StepFaceColorInfo const& info : colors) {
    listed.insert(info.rgb);
  }
  std::set<uint32_t> tessellated;
  for (StepColorMesh const& mesh : meshes) {
    ASSERT_TRUE(mesh.rgb.has_value());
    EXPECT_GT(mesh.mesh.GetNumElements(), 0);
    tessellated.insert(*mesh.rgb);
  }
  EXPECT_EQ(tessellated, listed);
}

TEST(StepFaceColors, TessellateByColorRejectsMissingFile) {
  ExpectNotOK expectNotOK;
  EXPECT_TRUE(
      TessellateStepByColor(AssetPath("does_not_exist.stp"), StepTessellationParams{}, expectNotOK)
          .empty());
}

TEST(StepFaceColorSizing, ColorAbsentFromFileChangesNothing) {
  MeshData const baseline = Mesh(BaseParams());

  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kUnusedColor, 0.5, 0.0, 0.0}};
  MeshData const sized = Mesh(params);

  EXPECT_EQ(sized.connectivity, baseline.connectivity);
  EXPECT_EQ(sized.coordinates, baseline.coordinates);
}

// Every face matched to its color but sized exactly as globally: exercises the whole color path
// while pinning that it has no side effect of its own.
TEST(StepFaceColorSizing, InheritedSizingMatchesGlobalSizing) {
  MeshData const baseline = Mesh(BaseParams());

  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kBodyColor, 0.0, 0.0, 0.0}, {kRed, 4.0, 0.0, 0.0}};
  MeshData const sized = Mesh(params);

  EXPECT_EQ(sized.connectivity, baseline.connectivity);
  EXPECT_EQ(sized.coordinates, baseline.coordinates);
}

TEST(StepFaceColorSizing, FinerColorRefinesAndStaysWatertight) {
  MeshData const baseline = Mesh(BaseParams());
  ASSERT_EQ(CountOpenEdges(baseline), 0);

  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kRed, 0.5, 0.0, 0.0}};
  MeshData const sized = Mesh(params);

  EXPECT_GT(sized.GetNumElements(), baseline.GetNumElements());
  EXPECT_EQ(CountOpenEdges(sized), 0);
}

TEST(StepFaceColorSizing, CoarserBodyColorCoarsensTheMesh) {
  MeshData const baseline = Mesh(BaseParams());

  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kBodyColor, 12.0, 0.0, 0.0}};
  MeshData const sized = Mesh(params);

  EXPECT_LT(sized.GetNumElements(), baseline.GetNumElements());
  EXPECT_EQ(CountOpenEdges(sized), 0);
}

TEST(StepFaceColorSizing, RejectsDuplicateColor) {
  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kRed, 0.5, 0.0, 0.0}, {kRed, 1.0, 0.0, 0.0}};
  ExpectNotOK expectNotOK;
  EXPECT_EQ(MeshStepBody(FixturePath(), params, expectNotOK).GetNumElements(), 0);
}

TEST(StepFaceColorSizing, RejectsNegativeSize) {
  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kRed, -1.0, 0.0, 0.0}};
  ExpectNotOK expectNotOK;
  EXPECT_EQ(MeshStepBody(FixturePath(), params, expectNotOK).GetNumElements(), 0);
}

// A color's fraction is of the same whole-model diagonal as the global fraction, so matching the
// global fraction reproduces the global mesh exactly.
TEST(StepFaceColorSizing, FractionMatchingGlobalFractionChangesNothing) {
  StepMeshBodyParams base;
  base.targetEdgeLengthFraction = 0.05;
  MeshData const baseline = Mesh(base);

  StepMeshBodyParams params = base;
  params.faceColorSizing = {{kRed, 0.0, 0.0, 0.0, 0.05}};
  MeshData const sized = Mesh(params);

  EXPECT_EQ(sized.connectivity, baseline.connectivity);
  EXPECT_EQ(sized.coordinates, baseline.coordinates);
}

TEST(StepFaceColorSizing, FinerFractionRefinesAndStaysWatertight) {
  MeshData const baseline = Mesh(BaseParams());

  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kRed, 0.0, 0.0, 0.0, 0.005}}; // ~0.5 mm on this ~95 mm model
  MeshData const sized = Mesh(params);

  EXPECT_GT(sized.GetNumElements(), baseline.GetNumElements());
  EXPECT_EQ(CountOpenEdges(sized), 0);
}

TEST(StepFaceColorSizing, ExplicitLengthOverridesFraction) {
  MeshData const baseline = Mesh(BaseParams());

  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kRed, BaseParams().targetEdgeLength, 0.0, 0.0, 0.005}};
  MeshData const sized = Mesh(params);

  EXPECT_EQ(sized.connectivity, baseline.connectivity);
  EXPECT_EQ(sized.coordinates, baseline.coordinates);
}

TEST(StepFaceColorSizing, RejectsNegativeFraction) {
  StepMeshBodyParams params = BaseParams();
  params.faceColorSizing = {{kRed, 0.0, 0.0, 0.0, -0.01}};
  ExpectNotOK expectNotOK;
  EXPECT_EQ(MeshStepBody(FixturePath(), params, expectNotOK).GetNumElements(), 0);
}
