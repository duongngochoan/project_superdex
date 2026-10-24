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

// Render-side STEP export. Reads via XCAF to preserve per-body colors/materials.
// Skips topology-modifying steps (CombineSolids/NormalizeShape) to keep styles intact.
// Normals are taken from the real CAD surface at each vertex rather than averaged from the
// triangles. They are stored on the mesh before the UVs are rewritten into the 0-1 range, so
// changing the UVs does not change the shading. The mesher is chosen by
// StepVisualExportParams::backend.

#include "mesh_cli_geometry.h"

#if MOCHI_USE_OCCT

#include "occ_shape_meshing.h"
#include "step_face_colors.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <IMeshData_Status.hxx>
#include <Message_ProgressRange.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <RWGltf_CafWriter.hxx>
#include <RWGltf_DracoParameters.hxx>
#include <RWMesh_CoordinateSystem.hxx>
#include <RWMesh_CoordinateSystemConverter.hxx>
#include <RWObj_CafWriter.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Integer.hxx>
#include <StlAPI_Writer.hxx>
#include <TColStd_IndexedDataMapOfStringString.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <span>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

using namespace mochi::mesh::cli;

// Triangles below this area [mm^2] are dropped; see RemoveZeroAreaTriangles.
constexpr double kDegenerateTriangleArea = 1e-10;

// OpenCascade emits geometry in millimeters (see LoadStepDocument).
constexpr double kMillimetersToMeters = 0.001;

// Tessellates one shape in place, using the backend the caller asked for.
IMeshData_Status MeshShapeForVisualExport(
    TopoDS_Shape const& shape,
    StepVisualExportParams const& params,
    double targetEdgeLength,
    occ::FaceMeshFallbackStats& fallbackStats) {
  occ::ShapeMeshingParams meshingParams;
  meshingParams.faceMesher = params.backend == CadMeshingBackend::Isotropic
      ? occ::FaceMesher::Isotropic
      : occ::FaceMesher::Delabella;
  meshingParams.linearDeflection = params.linearDeflection;
  meshingParams.angularDeflection = params.angularDeflection;
  meshingParams.targetEdgeLength = targetEdgeLength;
  meshingParams.edgeSampling = params.edgeSampling;
  return occ::MeshShape(shape, meshingParams, &fallbackStats);
}

double TriangleArea(gp_Pnt const& a, gp_Pnt const& b, gp_Pnt const& c) {
  return 0.5 * gp_Vec(a, b).Crossed(gp_Vec(a, c)).Magnitude();
}

// Drops zero-area triangles from every face's triangulation.
// The rebuilt triangulation must carry over UV nodes and normals.
int RemoveZeroAreaTriangles(TopoDS_Shape const& shape) {
  TopTools_IndexedMapOfShape faceMap;
  TopExp::MapShapes(shape, TopAbs_FACE, faceMap);

  BRep_Builder builder;
  int totalRemoved = 0;
  for (Standard_Integer fi = 1; fi <= faceMap.Extent(); ++fi) {
    TopoDS_Face const face = TopoDS::Face(faceMap(fi));
    TopLoc_Location location;
    Handle(Poly_Triangulation) const triangulation = BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull() || triangulation->NbTriangles() == 0) {
      continue;
    }
    gp_Trsf const transform = location.Transformation();

    std::vector<Poly_Triangle> validTriangles;
    validTriangles.reserve(triangulation->NbTriangles());
    for (Standard_Integer i = 1; i <= triangulation->NbTriangles(); ++i) {
      Poly_Triangle const& triangle = triangulation->Triangle(i);
      Standard_Integer n1 = 0;
      Standard_Integer n2 = 0;
      Standard_Integer n3 = 0;
      triangle.Get(n1, n2, n3);
      if (n1 == n2 || n2 == n3 || n1 == n3) {
        continue;
      }
      double const area = TriangleArea(
          triangulation->Node(n1).Transformed(transform),
          triangulation->Node(n2).Transformed(transform),
          triangulation->Node(n3).Transformed(transform));
      if (area >= kDegenerateTriangleArea) {
        validTriangles.push_back(triangle);
      }
    }

    int const removed = triangulation->NbTriangles() - static_cast<int>(validTriangles.size());
    if (removed == 0) {
      continue;
    }
    totalRemoved += removed;

    if (validTriangles.empty()) {
      builder.UpdateFace(face, Handle(Poly_Triangulation)());
      continue;
    }

    Standard_Integer const numNodes = triangulation->NbNodes();
    Handle(Poly_Triangulation) const rebuilt = new Poly_Triangulation(
        numNodes,
        static_cast<Standard_Integer>(validTriangles.size()),
        triangulation->HasUVNodes(),
        triangulation->HasNormals());
    for (Standard_Integer i = 1; i <= numNodes; ++i) {
      rebuilt->SetNode(i, triangulation->Node(i));
    }
    if (triangulation->HasUVNodes()) {
      for (Standard_Integer i = 1; i <= numNodes; ++i) {
        rebuilt->SetUVNode(i, triangulation->UVNode(i));
      }
    }
    if (triangulation->HasNormals()) {
      for (Standard_Integer i = 1; i <= numNodes; ++i) {
        rebuilt->SetNormal(i, triangulation->Normal(i));
      }
    }
    for (size_t i = 0; i < validTriangles.size(); ++i) {
      rebuilt->SetTriangle(static_cast<Standard_Integer>(i + 1), validTriangles[i]);
    }
    builder.UpdateFace(face, rebuilt);
  }
  return totalRemoved;
}

// Space left between neighboring faces in the UV layout, relative to the overall size of the
// layout.
constexpr double kUvChartPadding = 0.01;

// The UV layout of one face. It records the face's UV range and how much real-world distance one
// unit of U or V covers, so every face gets the same texture resolution.
struct UvChart {
  Handle(Poly_Triangulation) triangulation;
  double uMin = 0.0;
  double vMin = 0.0;
  double scaleU = 0.0;
  double scaleV = 0.0;
  double width = 0.0;
  double height = 0.0;
  double offsetU = 0.0;
  double offsetV = 0.0;
};

// If the mesh has no stored normals, the GLB/OBJ writers look up the CAD surface at each vertex's
// UV to get one, which would give wrong normals once the UVs are rewritten. So this stores the
// normals first, computed the same way the writers would. While visiting each vertex it also
// measures how stretched U and V are, to size the face's layout.
UvChart BakeNormalsAndMeasureChart(
    TopoDS_Face const& face,
    Handle(Poly_Triangulation) const& triangulation) {
  TopoDS_Face forwardFace = TopoDS::Face(face.Oriented(TopAbs_FORWARD));
  forwardFace.Location(TopLoc_Location());
  BRepLProp_SLProps props(BRepAdaptor_Surface(forwardFace, false), 1, 1e-12);

  double uMin = std::numeric_limits<double>::max();
  double vMin = std::numeric_limits<double>::max();
  double uMax = std::numeric_limits<double>::lowest();
  double vMax = std::numeric_limits<double>::lowest();
  double sumScaleU = 0.0;
  double sumScaleV = 0.0;

  triangulation->AddNormals();
  Standard_Integer const numNodes = triangulation->NbNodes();
  for (Standard_Integer i = 1; i <= numNodes; ++i) {
    gp_Pnt2d const uv = triangulation->UVNode(i);
    props.SetParameters(uv.X(), uv.Y());
    triangulation->SetNormal(i, props.IsNormalDefined() ? props.Normal() : gp::DZ());
    sumScaleU += props.D1U().Magnitude();
    sumScaleV += props.D1V().Magnitude();
    uMin = std::min(uMin, uv.X());
    vMin = std::min(vMin, uv.Y());
    uMax = std::max(uMax, uv.X());
    vMax = std::max(vMax, uv.Y());
  }

  double const scaleU = sumScaleU / numNodes;
  double const scaleV = sumScaleV / numNodes;
  return UvChart{
      .triangulation = triangulation,
      .uMin = uMin,
      .vMin = vMin,
      .scaleU = scaleU,
      .scaleV = scaleV,
      .width = (uMax - uMin) * scaleU,
      .height = (vMax - vMin) * scaleV,
  };
}

// The CAD mesher's UVs are the surface's own coordinates (an angle on a cylinder, millimeters on a
// plane), so they fall far outside 0-1 and faces overlap. This rescales each face to its real-world
// size, lays the faces out in rows without overlapping, and then shrinks the whole layout to fit
// inside 0-1.
void PackFaceUvs(TopTools_IndexedMapOfShape const& faces) {
  std::vector<UvChart> charts;
  // Copies of the same part share one mesh, so each mesh is laid out only once and the copies keep
  // the same UVs.
  std::unordered_set<Poly_Triangulation const*> seen;
  for (Standard_Integer fi = 1; fi <= faces.Extent(); ++fi) {
    TopoDS_Face const face = TopoDS::Face(faces(fi));
    TopLoc_Location location;
    Handle(Poly_Triangulation) const triangulation = BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull() || !triangulation->HasUVNodes() || triangulation->NbNodes() == 0 ||
        !seen.insert(triangulation.get()).second) {
      continue;
    }
    charts.push_back(BakeNormalsAndMeasureChart(face, triangulation));
  }

  double area = 0.0;
  double maxWidth = 0.0;
  for (UvChart const& chart : charts) {
    area += chart.width * chart.height;
    maxWidth = std::max(maxWidth, chart.width);
  }
  if (!(area > 0.0)) {
    return;
  }
  double const padding = kUvChartPadding * std::sqrt(area);
  double paddedArea = 0.0;
  for (UvChart const& chart : charts) {
    paddedArea += (chart.width + padding) * (chart.height + padding);
  }
  double const shelfWidth = std::max(std::sqrt(paddedArea), maxWidth + 2.0 * padding);

  std::ranges::stable_sort(charts, std::ranges::greater{}, &UvChart::height);
  double x = padding;
  double y = padding;
  double shelfHeight = 0.0;
  double usedWidth = 0.0;
  for (UvChart& chart : charts) {
    if (x > padding && x + chart.width + padding > shelfWidth) {
      x = padding;
      y += shelfHeight + padding;
      shelfHeight = 0.0;
    }
    chart.offsetU = x;
    chart.offsetV = y;
    x += chart.width + padding;
    shelfHeight = std::max(shelfHeight, chart.height);
    usedWidth = std::max(usedWidth, x);
  }
  double const invAtlasSize = 1.0 / std::max(usedWidth, y + shelfHeight + padding);

  for (UvChart const& chart : charts) {
    Standard_Integer const numNodes = chart.triangulation->NbNodes();
    for (Standard_Integer i = 1; i <= numNodes; ++i) {
      gp_Pnt2d const uv = chart.triangulation->UVNode(i);
      chart.triangulation->SetUVNode(
          i,
          gp_Pnt2d(
              (chart.offsetU + (uv.X() - chart.uMin) * chart.scaleU) * invAtlasSize,
              (chart.offsetV + (uv.Y() - chart.vMin) * chart.scaleV) * invAtlasSize));
    }
  }
}

// Tessellates every free shape in the document. Returns false when a face failed and the caller
// asked not to tolerate that; sets @p outHadPartialFailure when a failure was tolerated.
bool MeshDocument(
    Handle(TDocStd_Document) const& document,
    StepVisualExportParams const& params,
    bool& outHadPartialFailure,
    CliError& error) {
  Handle(XCAFDoc_ShapeTool) const shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
  TDF_LabelSequence freeShapes;
  shapeTool->GetFreeShapes(freeShapes);

  // One target edge length for the whole file, derived from a bounding box covering every body.
  double targetEdgeLength = 0.0;
  if (params.backend == CadMeshingBackend::Isotropic) {
    Bnd_Box bbox;
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
      TopoDS_Shape const shape = shapeTool->GetShape(freeShapes.Value(i));
      if (!shape.IsNull()) {
        BRepBndLib::Add(shape, bbox);
      }
    }
    targetEdgeLength = occ::ResolveTargetEdgeLength(
        bbox, params.targetEdgeLength, params.targetEdgeLengthFraction);
  }

  // Shared across every shape so the warning below reports one total for the file.
  occ::FaceMeshFallbackStats fallbackStats;

  int totalDegenerate = 0;
  TopTools_IndexedMapOfShape faces;
  for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
    TopoDS_Shape const shape = shapeTool->GetShape(freeShapes.Value(i));
    if (shape.IsNull()) {
      continue;
    }
    IMeshData_Status const status =
        MeshShapeForVisualExport(shape, params, targetEdgeLength, fallbackStats);
    if (status & IMeshData_Failure) {
      if (!params.allowPartialFailure) {
        MOCHI_MESH_CLI_ERROR_SET(
            error,
            "STEP tessellation failed on one or more faces (enable partial failure to keep a partial mesh).");
        return false;
      }
      outHadPartialFailure = true;
    }
    totalDegenerate += RemoveZeroAreaTriangles(shape);
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
  }
  PackFaceUvs(faces);

  if (fallbackStats.FailedCount() > 0) {
    MOCHI_MESH_CLI_LOG_WARNING(
        "%d face(s) defeated the CGAL mesher; %d recovered with the fallback mesher",
        fallbackStats.FailedCount(),
        fallbackStats.RescuedCount());
  }

  if (totalDegenerate > 0) {
    MOCHI_MESH_CLI_LOG_WARNING("visual export: removed %d degenerate triangle(s)", totalDegenerate);
  }

  return true;
}

// Fixes up the material block of the glTF just written, and optionally renames each material to
// the RRGGBBAA hex of its base color so downstream tools can key off the color without parsing the
// material. Handles both the JSON chunk of a GLB and a plain .gltf document.
bool PostProcessGltfMaterials(
    std::string const& path,
    bool isBinary,
    bool renameToRgba,
    CliError& error) {
  // Semi-glossy plastic: a neutral, flattering default for untextured CAD.
  constexpr double kMetallicFactor = 0.0;
  constexpr double kRoughnessFactor = 0.3;

  // GLB layout: [u32 magic][u32 version][u32 totalLength] then chunks of
  // [u32 chunkLength][u32 chunkType][payload]. The JSON chunk is always first.
  constexpr size_t kGlbHeaderSize = 12;
  constexpr size_t kGlbChunkHeaderSize = 8;

  std::vector<char> fileBytes;
  {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
      MOCHI_MESH_CLI_ERROR_SET(
          error, "Failed to reopen the exported glTF file to fix up its materials.");
      return false;
    }
    fileBytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }

  size_t jsonOffset = 0;
  size_t jsonLength = fileBytes.size();
  if (isBinary) {
    if (fileBytes.size() < kGlbHeaderSize + kGlbChunkHeaderSize) {
      MOCHI_MESH_CLI_ERROR_SET(error, "The exported GLB is too small to contain a JSON chunk.");
      return false;
    }
    uint32_t chunkLength = 0;
    std::memcpy(&chunkLength, fileBytes.data() + kGlbHeaderSize, sizeof(chunkLength));
    jsonOffset = kGlbHeaderSize + kGlbChunkHeaderSize;
    jsonLength = chunkLength;
    if (jsonOffset + jsonLength > fileBytes.size()) {
      MOCHI_MESH_CLI_ERROR_SET(error, "The exported GLB declares a truncated JSON chunk.");
      return false;
    }
  }

  rapidjson::Document json;
  json.Parse(fileBytes.data() + jsonOffset, jsonLength);
  if (json.HasParseError() || !json.IsObject()) {
    MOCHI_MESH_CLI_ERROR_SET(error, "Failed to parse the exported glTF JSON.");
    return false;
  }

  auto const materials = json.FindMember("materials");
  if (materials == json.MemberEnd() || !materials->value.IsArray()) {
    return true;
  }
  for (rapidjson::Value& material : materials->value.GetArray()) {
    if (!material.IsObject()) {
      continue;
    }
    auto const pbr = material.FindMember("pbrMetallicRoughness");
    if (pbr == material.MemberEnd() || !pbr->value.IsObject()) {
      continue;
    }

    pbr->value.RemoveMember("metallicFactor");
    pbr->value.AddMember("metallicFactor", kMetallicFactor, json.GetAllocator());
    pbr->value.RemoveMember("roughnessFactor");
    pbr->value.AddMember("roughnessFactor", kRoughnessFactor, json.GetAllocator());

    if (!renameToRgba) {
      continue;
    }
    auto const baseColor = pbr->value.FindMember("baseColorFactor");
    if (baseColor == pbr->value.MemberEnd() || !baseColor->value.IsArray() ||
        baseColor->value.Size() != 4) {
      continue;
    }
    int channels[4] = {};
    for (rapidjson::SizeType c = 0; c < 4; ++c) {
      long const value = std::lround(baseColor->value[c].GetDouble() * 255.0);
      channels[c] = static_cast<int>(std::clamp<long>(value, 0, 255));
    }
    char name[9] = {};
    std::snprintf(
        name, sizeof(name), "%02X%02X%02X%02X", channels[0], channels[1], channels[2], channels[3]);
    material.RemoveMember("name");
    material.AddMember("name", rapidjson::Value(name, json.GetAllocator()), json.GetAllocator());
  }

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  json.Accept(writer);
  std::string newJson(buffer.GetString(), buffer.GetSize());

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    MOCHI_MESH_CLI_ERROR_SET(error, "Failed to rewrite the exported glTF file.");
    return false;
  }
  if (!isBinary) {
    out.write(newJson.data(), static_cast<std::streamsize>(newJson.size()));
    return out.good();
  }

  // Chunk payloads must be 4-byte aligned; the JSON chunk pads with spaces.
  newJson.resize((newJson.size() + 3u) & ~size_t{3u}, ' ');
  auto const newJsonLength = static_cast<uint32_t>(newJson.size());
  size_t const tailOffset = jsonOffset + jsonLength;
  auto const newTotalLength = static_cast<uint32_t>(
      kGlbHeaderSize + kGlbChunkHeaderSize + newJson.size() + (fileBytes.size() - tailOffset));

  out.write(fileBytes.data(), 8); // magic + version
  out.write(reinterpret_cast<char const*>(&newTotalLength), sizeof(newTotalLength));
  out.write(reinterpret_cast<char const*>(&newJsonLength), sizeof(newJsonLength));
  out.write(fileBytes.data() + kGlbHeaderSize + 4, 4); // JSON chunk type
  out.write(newJson.data(), static_cast<std::streamsize>(newJson.size()));
  out.write(
      fileBytes.data() + tailOffset, static_cast<std::streamsize>(fileBytes.size() - tailOffset));
  if (!out.good()) {
    MOCHI_MESH_CLI_ERROR_SET(error, "Failed to rewrite the exported GLB file.");
    return false;
  }
  return true;
}

// glTF is defined Y-up, so the writer rotates OpenCascade's native Z-up frame.
// OBJ has no such convention, so it stays Z-up.
void ConfigureCoordinateSystem(
    RWMesh_CoordinateSystemConverter& converter,
    double scale,
    RWMesh_CoordinateSystem outputSystem) {
  converter.SetOutputLengthUnit(kMillimetersToMeters / scale);
  converter.SetInputCoordinateSystem(RWMesh_CoordinateSystem_Zup);
  converter.SetOutputCoordinateSystem(outputSystem);
}

bool WriteGltf(
    Handle(TDocStd_Document) const& document,
    std::string const& path,
    VisualMeshFormat format,
    StepVisualExportParams const& params,
    CliError& error) {
  bool const isBinary = format == VisualMeshFormat::Glb;

  RWGltf_CafWriter writer(path.c_str(), isBinary);
  ConfigureCoordinateSystem(
      writer.ChangeCoordinateSystemConverter(), params.scale, RWMesh_CoordinateSystem_Yup);

  // Draco needs a decompressor on the reading side; plain buffers keep the output universally
  // readable, and these models are small.
  RWGltf_DracoParameters draco;
  draco.DracoCompression = false;
  writer.SetCompressionParameters(draco);

  writer.SetToEmbedTexturesInGlb(true);
  writer.SetMergeFaces(true);
  writer.SetForcedUVExport(true);

  if (!writer.Perform(document, TColStd_IndexedDataMapOfStringString(), Message_ProgressRange())) {
    MOCHI_MESH_CLI_ERROR_SET(error, "Failed to write the glTF/GLB file.");
    return false;
  }
  return PostProcessGltfMaterials(path, isBinary, params.rgbaMaterialNames, error);
}

bool WriteObj(
    Handle(TDocStd_Document) const& document,
    std::string const& path,
    StepVisualExportParams const& params,
    CliError& error) {
  RWObj_CafWriter writer(path.c_str());
  ConfigureCoordinateSystem(
      writer.ChangeCoordinateSystemConverter(), params.scale, RWMesh_CoordinateSystem_Zup);

  if (!writer.Perform(document, TColStd_IndexedDataMapOfStringString(), Message_ProgressRange())) {
    MOCHI_MESH_CLI_ERROR_SET(error, "Failed to write the OBJ file.");
    return false;
  }
  return true;
}

// STL carries neither materials nor normals, and its writer takes a shape rather than a document,
// so this path applies the scale itself instead of going through the coordinate converter.
bool WriteStl(
    Handle(TDocStd_Document) const& document,
    std::string const& path,
    StepVisualExportParams const& params,
    CliError& error) {
  Handle(XCAFDoc_ShapeTool) const shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
  TDF_LabelSequence freeShapes;
  shapeTool->GetFreeShapes(freeShapes);

  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);

  for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
    TopoDS_Shape const shape = shapeTool->GetShape(freeShapes.Value(i));
    if (shape.IsNull()) {
      continue;
    }
    builder.Add(compound, shape);
  }

  gp_Trsf scaling;
  scaling.SetScale(gp_Pnt(0.0, 0.0, 0.0), params.scale);

  StlAPI_Writer writer;
  writer.ASCIIMode() = Standard_False;
  if (!writer.Write(
          compound.Moved(TopLoc_Location(scaling)), path.c_str(), Message_ProgressRange())) {
    MOCHI_MESH_CLI_ERROR_SET(error, "Failed to write the STL file.");
    return false;
  }
  return true;
}

} // namespace

bool mochi::mesh::cli::ExportStepVisual(
    std::string_view stepFilePath,
    std::span<VisualExportOutput const> outputs,
    StepVisualExportParams const& params,
    std::vector<VisualExportStatus>& outStatuses,
    CliError& error) {
  MOCHI_MESH_CLI_ERROR_RETURN(error, false);

  MOCHI_MESH_CLI_ERROR_IF(stepFilePath.empty(), error, "STEP file path is empty.");
  MOCHI_MESH_CLI_ERROR_IF(outputs.empty(), error, "No output files were requested.");
  MOCHI_MESH_CLI_ERROR_IF(
      !(params.linearDeflection > 0.0) || !(params.angularDeflection > 0.0),
      error,
      "STEP tessellation deflections must be positive.");
  MOCHI_MESH_CLI_ERROR_IF(!(params.scale > 0.0), error, "Output scale must be positive.");
  MOCHI_MESH_CLI_ERROR_RETURN(error, false);

  for (VisualExportOutput const& output : outputs) {
    MOCHI_MESH_CLI_ERROR_IF(output.path.empty(), error, "Output file path is empty.");
  }
  MOCHI_MESH_CLI_ERROR_RETURN(error, false);

  Handle(TDocStd_Document) document;
  if (!occ::LoadStepDocument(std::string(stepFilePath), document, error)) {
    return false;
  }

  outStatuses.assign(outputs.size(), VisualExportStatus::Failed);
  bool hadPartialFailure = false;
  bool meshed = false;

  // OpenCascade reports failures by raising Standard_Failure; catch it so an export failure becomes
  // a clean error rather than crashing the helper (which would corrupt the framed response).
  try {
    // Tessellate once then multi-format export.
    meshed = MeshDocument(document, params, hadPartialFailure, error);
    if (meshed) {
      for (size_t i = 0; i < outputs.size(); ++i) {
        VisualExportOutput const& output = outputs[i];
        // A write failure is reported through the output's status rather than through `error`: the
        // outputs are independent files, and one unwritable format must not discard the others.
        CliError writeError;
        bool written = false;
        switch (output.format) {
          case VisualMeshFormat::Glb:
          case VisualMeshFormat::Gltf:
            written = WriteGltf(document, output.path, output.format, params, writeError);
            break;
          case VisualMeshFormat::Obj:
            written = WriteObj(document, output.path, params, writeError);
            break;
          case VisualMeshFormat::Stl:
            written = WriteStl(document, output.path, params, writeError);
            break;
          case VisualMeshFormat::Count:
            MOCHI_MESH_CLI_ERROR_SET(writeError, "Invalid visual mesh format.");
            break;
        }
        if (!written) {
          MOCHI_MESH_CLI_LOG_WARNING(
              "Failed to write '%s': %s", output.path.c_str(), writeError.ToString().c_str());
        }
        outStatuses[i] = !written ? VisualExportStatus::Failed
            : hadPartialFailure   ? VisualExportStatus::WrittenPartial
                                  : VisualExportStatus::Written;
      }
    }
  } catch (Standard_Failure const& failure) {
    char const* const message = failure.GetMessageString();
    std::fprintf(
        stderr, "[step_visual_export] OpenCascade failure: %s\n", message ? message : "(none)");
    MOCHI_MESH_CLI_ERROR_SET(error, "OpenCascade failed during the STEP visual export.");
  } catch (...) {
    MOCHI_MESH_CLI_ERROR_SET(error, "OpenCascade threw during the STEP visual export.");
  }

  occ::CloseStepDocument(document);
  MOCHI_MESH_CLI_ERROR_RETURN(error, false);
  return meshed;
}

#else // !MOCHI_USE_OCCT

bool mochi::mesh::cli::ExportStepVisual(
    std::string_view /*stepFilePath*/,
    std::span<VisualExportOutput const> /*outputs*/,
    StepVisualExportParams const& /*params*/,
    std::vector<VisualExportStatus>& /*outStatuses*/,
    CliError& error) {
  MOCHI_MESH_CLI_ERROR_RETURN(error, false);
  MOCHI_MESH_CLI_ERROR_SET(
      error, "STEP visual export is not supported on this platform (OpenCascade unavailable).");
  return false;
}

#endif // MOCHI_USE_OCCT
