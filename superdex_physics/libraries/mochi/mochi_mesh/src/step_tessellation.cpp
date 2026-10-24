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

#include <mochi_mesh/step_tessellation.h>

#include "mesh_cli_adapter.h"
#include "mesh_cli_client.h"

#include <string_view>
#include <vector>

// TessellateStep marshals the request and routes it to the superdex_mesh_cli helper process, which
// runs the OpenCascade tessellation. Input validation lives in the helper, so an invalid request
// round-trips a clean error rather than failing here.

using namespace mochi;
using namespace mochi::mesh;

MeshData mochi::mesh::TessellateStep(
    std::string_view stepFilePath,
    StepTessellationParams const& params,
    Error& error) {
  MOCHI_ERROR_RETURN(error, {});

  cli::PayloadWriter writer;
  writer.WriteByteArray(stepFilePath);
  writer.WriteStepTessellationParams(params);
  return InvokeMeshOp(cli::GeometryOp::TessellateStep, writer, error);
}

std::vector<StepColorMesh> mochi::mesh::TessellateStepByColor(
    std::string_view stepFilePath,
    StepTessellationParams const& params,
    Error& error) {
  MOCHI_ERROR_RETURN(error, {});

  cli::PayloadWriter writer;
  writer.WriteByteArray(stepFilePath);
  writer.WriteStepTessellationParams(params);
  std::vector<char> const response =
      InvokeMeshCli(cli::GeometryOp::TessellateStepByColor, writer.Bytes(), error);
  MOCHI_ERROR_RETURN(error, {});

  cli::PayloadReader reader(response);
  std::vector<cli::ColorMeshData> cliMeshes;
  MOCHI_ERROR_IF(
      !reader.ReadColorMeshes(cliMeshes) || !reader.AtEnd(),
      error,
      "Malformed TessellateStepByColor response from superdex_mesh_cli.");
  MOCHI_ERROR_RETURN(error, {});

  std::vector<StepColorMesh> meshes;
  meshes.reserve(cliMeshes.size());
  for (cli::ColorMeshData const& cliMesh : cliMeshes) {
    StepColorMesh& mesh = meshes.emplace_back();
    if (cliMesh.hasColor) {
      mesh.rgb = cliMesh.rgb;
    }
    mesh.mesh = cli_adapter::FromCliMeshData(cliMesh.mesh, error);
    MOCHI_ERROR_RETURN(error, {});
  }
  return meshes;
}
