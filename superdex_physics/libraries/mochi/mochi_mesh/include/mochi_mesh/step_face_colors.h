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

#include <mochi_core/utils/error.h>
#include <mochi_mesh/mochi_mesh_cli_types.h>

#include <string_view>
#include <vector>

namespace mochi::mesh {

/// @brief List the distinct face colors of a STEP CAD file, for keying
/// @ref StepMeshBodyParams::faceColorSizing.
///
/// Each face's color resolves as in the CAD file's glTF export: its own surface style, else that of
/// its nearest styled ancestor (shell, solid, part), else the assembly instance's.
///
/// @param[in] stepFilePath Filesystem path to the STEP file (UTF-8).
/// @param[out] outColors One entry per distinct color, largest total face area first. Faces with
///                       no color are not reported. Cleared on failure.
/// @param[in,out] error Error status.
/// @return True on success.
[[nodiscard]] bool ReadStepFaceColors(
    std::string_view stepFilePath,
    std::vector<StepFaceColorInfo>& outColors,
    Error& error);

} // namespace mochi::mesh
