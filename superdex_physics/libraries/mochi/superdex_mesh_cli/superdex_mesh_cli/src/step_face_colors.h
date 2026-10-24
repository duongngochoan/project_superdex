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

// XCAF-based STEP document loading and face color resolution. Only compiled where OpenCascade is
// available (MOCHI_USE_OCCT); elsewhere this header is an empty translation unit.
#if MOCHI_USE_OCCT

#include "mesh_cli_compat.h"

#include <NCollection_DataMap.hxx>
#include <TDocStd_Document.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace mochi::mesh::cli::occ {

/// Reads a STEP file into an XCAF document, preserving colors, materials, names and layers.
/// Geometry is normalized to millimeters, matching STEPControl_Reader in the other STEP paths.
[[nodiscard]] bool
LoadStepDocument(std::string const& path, Handle(TDocStd_Document) & outDocument, CliError& error);

/// Releases a document from @ref LoadStepDocument.
void CloseStepDocument(Handle(TDocStd_Document) const& document);

/// A face of the document, placed in world coordinates, with its resolved color as packed
/// `0xRRGGBB` sRGB. Resolution matches OpenCascade's glTF writer: the face's own style, else its
/// nearest styled ancestor, else the part/instance style.
struct StyledFace {
  TopoDS_Face face;
  std::optional<uint32_t> rgb;
};

/// Every face of every part instance in @p document.
[[nodiscard]] std::vector<StyledFace> CollectStyledFaces(Handle(TDocStd_Document) const& document);

using FaceColorMap = NCollection_DataMap<TopoDS_Shape, uint32_t, TopTools_ShapeMapHasher>;

/// Assigns each face of @p shape the color of the source face it lies on, found geometrically so it
/// survives topology rebuilt by booleans or a STEP round-trip. A face lying on no source face, or
/// on an uncolored one, is left out.
[[nodiscard]] FaceColorMap MatchFaceColors(
    TopoDS_Shape const& shape,
    std::span<StyledFace const> sources);

} // namespace mochi::mesh::cli::occ

#endif // MOCHI_USE_OCCT
