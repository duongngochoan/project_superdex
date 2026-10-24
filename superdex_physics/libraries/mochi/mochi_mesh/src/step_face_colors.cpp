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

#include <mochi_mesh/step_face_colors.h>

#include "mesh_cli_client.h"

#include <mochi_mesh/mochi_mesh_cli_encoding.h>

#include <string_view>
#include <vector>

using namespace mochi;
using namespace mochi::mesh;

bool mochi::mesh::ReadStepFaceColors(
    std::string_view stepFilePath,
    std::vector<StepFaceColorInfo>& outColors,
    Error& error) {
  outColors.clear();
  MOCHI_ERROR_RETURN(error, false);

  cli::PayloadWriter writer;
  writer.WriteByteArray(stepFilePath);

  std::vector<char> const response =
      InvokeMeshCli(cli::GeometryOp::ReadStepFaceColors, writer.Bytes(), error);
  MOCHI_ERROR_RETURN(error, false);

  cli::PayloadReader reader(response);
  bool const decoded = reader.ReadStepFaceColorInfos(outColors) && reader.AtEnd();
  if (!decoded) {
    outColors.clear();
  }
  MOCHI_ERROR_IF(!decoded, error, "Malformed ReadStepFaceColors response from superdex_mesh_cli.");
  MOCHI_ERROR_RETURN(error, false);
  return true;
}
