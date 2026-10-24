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

#include "meshing/processing_modifiers/source_cad_model.h"

#include "meshing/processing_modifiers/processing_mesh_utils.h" // ApplyTransform, LoadRenderMesh, EndsWithNoCase

#include <imgui.h>
#include <imguios/fonts/icons_font_awesome5.h>

#include <picojson/picojson.h>

#include <mochi_core/utils/dynamic_string.h>
#include <mochi_core/utils/log.h>

#include <mochi_mesh/step_face_colors.h> // ReadStepFaceColors (face color sizing's Read button)
#include <mochi_mesh/step_mesh_body.h> // MeshStepBody (CAD Mesher path)
#include <mochi_mesh/step_tessellation.h> // TessellateStep (Delabella path)

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace superdex::studio {

// Tessellation backend + edge-sampling mode, reflected as string names for readable JSON.
enum class CadTessellationBackend { Isotropic, Delabella };
enum class CadEdgeSampling { Uniform, Adaptive };

} // namespace superdex::studio

// Enum reflection specializes the global SReflectTypeTraits template, so it must be declared at
// global scope (the fully-qualified names refer back into superdex::studio).
MOCHI_ENUM_BEGIN(superdex::studio::CadTessellationBackend);
MOCHI_ENUM_ITEM(Isotropic) MOCHI_ATTRIBUTE(PreviouslyKnownAs("CadMesher"));
MOCHI_ENUM_ITEM(Delabella);
MOCHI_ENUM_END();

MOCHI_ENUM_BEGIN(superdex::studio::CadEdgeSampling)
MOCHI_ENUM_ITEM(Uniform)
MOCHI_ENUM_ITEM(Adaptive)
MOCHI_ENUM_END()

namespace superdex::studio {

namespace {

// Sizing for the STEP faces of one color, as authored in the CAD tool. 0 inherits the global value.
struct CadFaceColorSizingProps {
  std::string color = "#FFFFFF"; // sRGB hex
  double targetEdgeLength = 0.0; // mm
  double linearDeflection = 0.0; // mm
  double angularDeflection = 0.0; // rad
  double targetEdgeLengthFraction = 0.0; // of the model's bbox diagonal; used when length is 0

  MOCHI_STRUCT_BEGIN(superdex::studio::CadFaceColorSizingProps)
  MOCHI_FIELD(color)
  MOCHI_FIELD(targetEdgeLength)
  MOCHI_FIELD(linearDeflection)
  MOCHI_FIELD(angularDeflection)
  MOCHI_FIELD(targetEdgeLengthFraction)
  MOCHI_STRUCT_END()
};

// STEP tessellation parameters. Only consulted when the slotted CAD file is a STEP (.step/.stp); an
// STL is already a triangle mesh and is read directly, ignoring these.
struct CadStepToMeshProps {
  CadTessellationBackend backend = CadTessellationBackend::Isotropic;
  double linearDeflection = 0.05; // mm
  double angularDeflection = 0.25; // rad
  bool combineTouchingSolids = true;
  CadEdgeSampling edgeSampling = CadEdgeSampling::Uniform;
  double targetEdgeLength = 0.0; // mm, 0 = auto
  double targetEdgeLengthFraction = 0.02; // fraction of bbox diagonal
  bool allowPartialFailure = true;
  std::vector<CadFaceColorSizingProps> faceColorSizing; // Isotropic only

  MOCHI_STRUCT_BEGIN(superdex::studio::CadStepToMeshProps)
  MOCHI_FIELD(backend)
  MOCHI_FIELD(linearDeflection)
  MOCHI_FIELD(angularDeflection)
  MOCHI_FIELD(combineTouchingSolids)
  MOCHI_FIELD(edgeSampling)
  MOCHI_FIELD(targetEdgeLength)
  MOCHI_FIELD(targetEdgeLengthFraction)
  MOCHI_FIELD(allowPartialFailure)
  MOCHI_FIELD(faceColorSizing)
  MOCHI_STRUCT_END()
};

// "From File" parameters: the modifier's own CAD-model slot (independent of the editor's slots)
// plus the STEP tessellation params applied when that slot holds a STEP.
struct CadFileSourceProps {
  CadStepToMeshProps step;
  mochi::DynamicString path;

  MOCHI_STRUCT_BEGIN(superdex::studio::CadFileSourceProps)
  MOCHI_FIELD(step)
  MOCHI_FIELD(path)
  MOCHI_STRUCT_END()
};

bool IsStlPath(std::string const& path) {
  return processing::EndsWithNoCase(path, ".stl");
}

// Pipelines saved before combineTouchingSolids replaced `bodySelection` still carry it. Its only
// value, 0 for all bodies, combined touching solids as combineTouchingSolids does by default, so
// it is dropped rather than read as an unknown field.
void DropRetiredStepFields(picojson::value& step) {
  if (step.is<picojson::object>()) {
    step.get<picojson::object>().erase("bodySelection");
  }
}

// Packed 0xRRGGBB from "#RRGGBB" (the '#' is optional); nullopt for anything else.
std::optional<uint32_t> ParseHexRgb(std::string_view text) {
  if (!text.empty() && text.front() == '#') {
    text.remove_prefix(1);
  }
  if (text.size() != 6) {
    return std::nullopt;
  }
  uint32_t rgb = 0;
  for (char const c : text) {
    int digit = -1;
    if (c >= '0' && c <= '9') {
      digit = c - '0';
    } else if (c >= 'a' && c <= 'f') {
      digit = c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
      digit = c - 'A' + 10;
    } else {
      return std::nullopt;
    }
    rgb = (rgb << 4) | static_cast<uint32_t>(digit);
  }
  return rgb;
}

std::string FormatHexRgb(uint32_t rgb) {
  std::array<char, 8> text{};
  std::snprintf(text.data(), text.size(), "#%06X", static_cast<unsigned>(rgb & 0xFFFFFFu));
  return text.data();
}

// Produces the source mesh from a CAD file: an STL is read directly as a triangle mesh; a STEP is
// tessellated with @p props (CGAL per-face remesher or OpenCascade's BRepMesh).
//
// Only the STL path converts to renderer space (inside LoadRenderMesh); the STEP calls below need
// no conversion because the superdex_mesh_cli helper already returns renderer-space geometry.
mochi::MeshData
CadSourceMesh(std::string const& path, CadStepToMeshProps const& props, mochi::Error& error) {
  if (IsStlPath(path)) {
    return processing::LoadRenderMesh(path, error);
  }
  if (props.backend == CadTessellationBackend::Delabella) {
    // OpenCascade's standard BRepMesh (Delabella) -- meshes the whole file; deflections only.
    mochi::mesh::StepTessellationParams params;
    params.linearDeflection = props.linearDeflection;
    params.angularDeflection = props.angularDeflection;
    return mochi::mesh::TessellateStep(path, params, error);
  }
  mochi::mesh::StepMeshBodyParams params;
  params.linearDeflection = props.linearDeflection;
  params.angularDeflection = props.angularDeflection;
  params.targetEdgeLength = props.targetEdgeLength;
  params.targetEdgeLengthFraction = props.targetEdgeLengthFraction;
  params.edgeSampling = static_cast<mochi::mesh::StepMeshBodyParams::EdgeSampling>(
      static_cast<int>(props.edgeSampling));
  params.allowPartialFailure = props.allowPartialFailure;
  params.combineTouchingSolids = props.combineTouchingSolids;
  for (CadFaceColorSizingProps const& row : props.faceColorSizing) {
    std::optional<uint32_t> const rgb = ParseHexRgb(row.color);
    MOCHI_ERROR_IF(!rgb, error, "Source from CAD Model: a face color sizing color is not #RRGGBB.");
    MOCHI_ERROR_RETURN(error, {});
    params.faceColorSizing.push_back(
        {*rgb,
         row.targetEdgeLength,
         row.linearDeflection,
         row.angularDeflection,
         row.targetEdgeLengthFraction});
  }
  return mochi::mesh::MeshStepBody(path, params, error);
}

// 0 (inherit) stays 0; anything else is clamped to [lo, hi].
double ClampOrInherit(double value, double lo, double hi) {
  return value <= 0.0 ? 0.0 : std::clamp(value, lo, hi);
}

// The per-color sizing list of the Isotropic path, plus the runtime-only state behind its Read
// button: the in-flight read and the face statistics of the last one, shown beside each row.
class FaceColorSizingPanel {
 public:
  void Show(
      std::vector<CadFaceColorSizingProps>& rows,
      std::string const& stepPath,
      bool adaptiveSampling,
      ModifierGuiContext const& gui) {
    ConsumeRead(rows, stepPath);
    if (_statsPath != stepPath) {
      _stats.clear();
    }

    // A borderless child with window padding insets the header, which ImGui otherwise stretches
    // past the panel's edges, and spaces the section from the controls around it.
    ImGui::Spacing();
    if (ImGui::BeginChild(
            "FaceColorSizing",
            ImVec2(0.0f, 0.0f),
            ImGuiChildFlags_AlwaysUseWindowPadding | ImGuiChildFlags_AutoResizeY)) {
      ShowContents(rows, stepPath, adaptiveSampling, gui);
    }
    ImGui::EndChild();
  }

 private:
  void ShowContents(
      std::vector<CadFaceColorSizingProps>& rows,
      std::string const& stepPath,
      bool adaptiveSampling,
      ModifierGuiContext const& gui) {
    ModifierTooltip const& tooltip = gui.tooltip;

    // "###" pins the ID so the open state survives the row count changing in the label.
    std::array<char, 64> header{};
    std::snprintf(
        header.data(),
        header.size(),
        "Color-based Face Sizing (%zu)###FaceColorSizing",
        rows.size());
    bool const open = ImGui::CollapsingHeader(header.data());
    tooltip(
        "Set face sizing based on STEP file's face colors. Read Colors then set face sizing parameters "
        "for each color.");
    if (!open) {
      return;
    }

    if (gui.host) {
      bool const busy = gui.host->IsBackgroundTaskRunning() || _pending;
      ImGui::BeginDisabled(busy || stepPath.empty());
      if (ImGui::Button("Read Colors")) {
        StartRead(stepPath, *gui.host);
      }
      ImGui::EndDisabled();
      tooltip("List the face colors used in the STEP file, adds new colors for those unlisted.");
      ImGui::SameLine();
    }
    if (ImGui::Button("Add Color")) {
      rows.push_back({});
    }
    tooltip("Add a face color");

    if (rows.empty()) {
      ImGui::TextDisabled("No colors listed: every face uses the global sizing.");
      return;
    }
    ImGui::TextDisabled("A value of 0 uses the global setting.");

    std::unordered_set<uint32_t> seen;
    bool hasDuplicate = false;
    bool hasInvalid = false;
    std::optional<size_t> removeIndex;
    for (size_t i = 0; i < rows.size(); ++i) {
      CadFaceColorSizingProps& row = rows[i];
      std::optional<uint32_t> const rgb = ParseHexRgb(row.color);
      hasInvalid = hasInvalid || !rgb;
      hasDuplicate = hasDuplicate || (rgb && !seen.insert(*rgb).second);

      ImGui::PushID(static_cast<int>(i));
      uint32_t const packed = rgb.value_or(0);
      std::array<float, 3> color{
          static_cast<float>((packed >> 16) & 0xFFu) / 255.0f,
          static_cast<float>((packed >> 8) & 0xFFu) / 255.0f,
          static_cast<float>(packed & 0xFFu) / 255.0f};
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12.0f);
      if (ImGui::ColorEdit3(
              "##color",
              color.data(),
              ImGuiColorEditFlags_DisplayHex | ImGuiColorEditFlags_InputRGB)) {
        uint32_t edited = 0;
        for (float const channel : color) {
          edited = (edited << 8) |
              static_cast<uint32_t>(std::clamp(std::lround(channel * 255.0f), 0L, 255L));
        }
        row.color = FormatHexRgb(edited);
      }
      tooltip("Face color to match (8-bit sRGB).");
      ImGui::SameLine();
      auto const stats = rgb ? _stats.find(*rgb) : _stats.end();
      if (stats != _stats.end()) {
        ImGui::Text("%u faces, %.1f mm^2", stats->second.faceCount, stats->second.area);
      } else if (!_stats.empty()) {
        ImGui::TextDisabled("not in file");
      }
      ImGui::SameLine();
      if (ImGui::Button(ICON_FA_TRASH)) {
        removeIndex = i;
      }
      tooltip("Remove this color.");

      ImGui::Indent();
      ImGui::BeginDisabled(!adaptiveSampling);
      if (ImGui::InputDouble("Linear Deflection (mm)", &row.linearDeflection, 0.0, 0.0, "%.3f")) {
        row.linearDeflection = ClampOrInherit(row.linearDeflection, 0.001, 10.0);
      }
      tooltip(
          "Used only for Adaptive edge sampling, where angular deflection usually decides how "
          "finely curves are sampled.");
      if (ImGui::InputDouble(
              "Angular Deflection (rad)", &row.angularDeflection, 0.0, 0.0, "%.3f")) {
        row.angularDeflection = ClampOrInherit(row.angularDeflection, 0.01, 1.57);
      }
      tooltip("Used only for Adaptive edge sampling");
      ImGui::EndDisabled();
      if (ImGui::InputDouble("Target Edge Length (mm)", &row.targetEdgeLength, 0.0, 0.0, "%.4f")) {
        row.targetEdgeLength = std::max(row.targetEdgeLength, 0.0);
      }
      ImGui::BeginDisabled(row.targetEdgeLength != 0.0);
      if (ImGui::InputDouble(
              "Edge Length Fraction", &row.targetEdgeLengthFraction, 0.0, 0.0, "%.4f")) {
        row.targetEdgeLengthFraction = ClampOrInherit(row.targetEdgeLengthFraction, 0.001, 0.5);
      }
      tooltip("Target edge length as a fraction of the whole model's bounding-box diagonal");
      ImGui::EndDisabled();
      ImGui::Unindent();
      ImGui::PopID();
    }
    if (removeIndex) {
      rows.erase(rows.begin() + static_cast<std::ptrdiff_t>(*removeIndex));
    }

    ImVec4 const kWarningColor(1.0f, 0.4f, 0.3f, 1.0f);
    if (hasDuplicate) {
      ImGui::TextColored(kWarningColor, "A color is listed twice; generating will fail.");
    }
    if (hasInvalid) {
      ImGui::TextColored(kWarningColor, "A color is not #RRGGBB; generating will fail.");
    }
  }

  // Shared with the worker (which fills it) and the completion callback (which marks it done), so
  // neither has to outlive or reference the panel.
  struct ReadResult {
    std::string path;
    std::vector<mochi::mesh::StepFaceColorInfo> colors;
    bool ok = false;
    bool done = false; // main thread only
  };

  void StartRead(std::string const& stepPath, ModifierHost& host) {
    auto result = std::make_shared<ReadResult>();
    result->path = stepPath;
    // CLI-backed: a cancel aborts it by killing the helper subprocess, so it needs no cancel check.
    if (host.RunBackgroundTask(
            "Reading STEP Face Colors",
            "Read face colors",
            [result] {
              mochi::ErrorLog error;
              result->ok = mochi::mesh::ReadStepFaceColors(result->path, result->colors, error);
              return result->ok;
            },
            [result] { result->done = true; })) {
      _pending = std::move(result);
    }
  }

  // Merges a finished read of @p stepPath into @p rows: colors not yet listed are appended,
  // inheriting every value, and listed rows are left untouched. A read of another file, finished
  // while the panel was hidden and the slot then changed, is dropped.
  void ConsumeRead(std::vector<CadFaceColorSizingProps>& rows, std::string const& stepPath) {
    if (!_pending || !_pending->done) {
      return;
    }
    std::shared_ptr<ReadResult> const result = std::move(_pending);
    if (!result->ok || result->path != stepPath) {
      return;
    }
    std::unordered_set<uint32_t> listed;
    for (CadFaceColorSizingProps const& row : rows) {
      if (std::optional<uint32_t> const rgb = ParseHexRgb(row.color)) {
        listed.insert(*rgb);
      }
    }
    _stats.clear();
    _statsPath = result->path;
    for (mochi::mesh::StepFaceColorInfo const& info : result->colors) {
      _stats[info.rgb] = info;
      if (listed.insert(info.rgb).second) {
        CadFaceColorSizingProps row;
        row.color = FormatHexRgb(info.rgb);
        rows.push_back(std::move(row));
      }
    }
    if (result->colors.empty()) {
      MOCHI_LOG_WARNING("No face colors found in %s.", result->path.c_str());
    }
  }

  std::shared_ptr<ReadResult> _pending;
  std::unordered_map<uint32_t, mochi::mesh::StepFaceColorInfo> _stats;
  std::string _statsPath;
};

// Draws the STEP tessellation controls. Only shown when the slotted file is a STEP; an STL has no
// options.
void ShowStepTessellationParams(
    CadStepToMeshProps& props,
    std::string const& stepPath,
    FaceColorSizingPanel& faceColorPanel,
    ModifierGuiContext const& gui) {
  ModifierTooltip const& tooltip = gui.tooltip;
  int backend = static_cast<int>(props.backend);
  std::array<char const*, 2> const backendItems{"Isotropic (CGAL)", "Delabella (OCCT)"};
  if (ImGui::Combo(
          "Backend", &backend, backendItems.data(), static_cast<int>(backendItems.size()))) {
    props.backend = static_cast<CadTessellationBackend>(backend);
  }
  tooltip(
      "Tessellation backend:\n\n"
      "Isotropic = Uses Constrained Delaunay Triangulation based on uniformly "
      "sampled points along CAD faces. Provides highest quality for simple CAD geometry. Prioritizes "
      "uniform edge length. \n\n"
      "Delabella = OpenCascade's fast BRepMesh which aims for visual fidelity. Remeshing is highly "
      "recommended for physics simulation.");

  // Isotropic with Uniform sampling sizes everything by edge length, so the deflections don't
  // apply.
  ImGui::BeginDisabled(
      props.backend == CadTessellationBackend::Isotropic &&
      props.edgeSampling == CadEdgeSampling::Uniform);

  if (ImGui::InputDouble("Linear Deflection (mm)", &props.linearDeflection, 0.0, 0.0, "%.3f")) {
    props.linearDeflection = std::clamp(props.linearDeflection, 0.001, 10.0);
  }
  tooltip(
      "Maximum chord deviation between the mesh and the true surface (mm). Smaller = finer.\n\n"
      "Isotropic: used only with Adaptive edge sampling, where angular deflection usually decides "
      "how finely curves are sampled.");

  if (ImGui::InputDouble("Angular Deflection (rad)", &props.angularDeflection, 0.0, 0.0, "%.3f")) {
    props.angularDeflection = std::clamp(props.angularDeflection, 0.01, 1.57);
  }
  tooltip(
      "Maximum angle between adjacent facets (rad). Smaller = finer on curved surfaces.\n\n"
      "Isotropic: used only with Adaptive edge sampling.");
  ImGui::EndDisabled();

  // The remaining controls only affect the Isotropic (CGAL) path; Delabella meshes the whole file
  // using only the deflections above.
  if (props.backend != CadTessellationBackend::Isotropic) {
    return;
  }

  ImGui::Checkbox("Combine Touching Solids", &props.combineTouchingSolids);
  tooltip(
      "Attempts to fuse solid bodies that touch into one body before meshing. "
      "Not recommended for complex assemblies with high part counts.");

  int edgeSampling = static_cast<int>(props.edgeSampling);
  std::array<char const*, 2> const edgeSamplingItems{"Uniform", "Adaptive"};
  if (ImGui::Combo(
          "Edge Sampling",
          &edgeSampling,
          edgeSamplingItems.data(),
          static_cast<int>(edgeSamplingItems.size()))) {
    props.edgeSampling = static_cast<CadEdgeSampling>(edgeSampling);
  }
  tooltip(
      "How face boundary curves are sampled before triangulation. Both space their samples evenly; "
      "Uniform takes the count from the target edge length alone, Adaptive respects angular deflection "
      "at the cost of shorter edge lengths along curves.\n\n"
      "Use Adaptive to preserve round holes or fillets.");

  if (ImGui::InputDouble(
          "Target Edge Length (mm), 0=auto", &props.targetEdgeLength, 0.0, 0.0, "%.4f")) {
    props.targetEdgeLength = std::max(props.targetEdgeLength, 0.0);
  }
  tooltip("Desired triangle edge length (mm). 0 = auto from the bounding-box fraction below.");

  ImGui::BeginDisabled(props.targetEdgeLength != 0.0);
  if (ImGui::InputDouble(
          "Edge Length Fraction", &props.targetEdgeLengthFraction, 0.0, 0.0, "%.4f")) {
    props.targetEdgeLengthFraction = std::clamp(props.targetEdgeLengthFraction, 0.001, 0.5);
  }
  tooltip(
      "Edge length is determined from the fraction of diagonal length of bounding-box of "
      "the input geometry.");
  ImGui::EndDisabled();

  ImGui::Checkbox("Allow Partial Failure", &props.allowPartialFailure);
  tooltip("Output a mesh even if some faces fail to tessellate, instead of aborting.");

  faceColorPanel.Show(
      props.faceColorSizing, stepPath, props.edgeSampling == CadEdgeSampling::Adaptive, gui);
}

// ---- From Model Viewer --------------------------------------------------------------------------
class CadFromViewerMethod : public ReflectedMethod<CadStepToMeshProps> {
 public:
  char const* Name() const override {
    return "From Model Viewer";
  }
  char const* Description() const override {
    return "Uses the editor's current CAD Model (its Model Viewer slot) as the source. A STEP file "
           "is tessellated with the options below; an STL is read directly.";
  }
  bool CanGenerate(ModifierRunContext const& ctx) const override {
    return !ctx.cadFilePath.empty();
  }
  std::string SourceFilePath(ModifierRunContext const& ctx) const override {
    return ctx.cadFilePath;
  }
  // Fold the slotted file + baked CAD transform into the signature so changing either re-generates.
  std::string PropsSignature(ModifierRunContext const& ctx) const override {
    mochi::Real4 const r = ctx.cadRotation.ToReal4();
    std::string sig = SReflect::ToJsonString(_props, /*pretty=*/false);
    sig += '|';
    sig += ctx.cadFilePath;
    sig += '|';
    for (int i = 0; i < 3; ++i) {
      sig += std::to_string(ctx.cadScale[i]) + ',';
    }
    for (int i = 0; i < 4; ++i) {
      sig += std::to_string(r[i]) + ',';
    }
    for (int i = 0; i < 3; ++i) {
      sig += std::to_string(ctx.cadTranslation[i]) + ',';
    }
    return sig;
  }
  int DeserializeProps(picojson::value const& in) override {
    picojson::value props = in;
    DropRetiredStepFields(props);
    return ReflectedMethod::DeserializeProps(props);
  }

  void ShowParams(ModifierGuiContext const& gui) override {
    ImGui::TextUnformatted("Uses the editor's current CAD Model as the source.");
    if (gui.cadFilePath.empty()) {
      ImGui::TextDisabled("Slot a CAD Model (STEP or STL) in the Model Viewer.");
      return;
    }
    if (IsStlPath(gui.cadFilePath)) {
      ImGui::TextDisabled("STL: used directly; no tessellation options.");
      return;
    }
    ShowStepTessellationParams(_props, gui.cadFilePath, _faceColorPanel, gui);
  }

  mochi::MeshData Run(
      mochi::MeshData const& /*input*/,
      ModifierRunContext const& ctx,
      mochi::Error& error) const override {
    MOCHI_ERROR_IF(
        ctx.cadFilePath.empty(), error, "Source from CAD Model: no CAD file is slotted.");
    MOCHI_ERROR_RETURN(error, {});
    mochi::MeshData mesh = CadSourceMesh(ctx.cadFilePath, _props, error);
    if (!error.IsOK()) {
      return {};
    }
    processing::ApplyTransform(mesh, ctx.cadScale, ctx.cadRotation, ctx.cadTranslation);
    return mesh;
  }

 private:
  FaceColorSizingPanel _faceColorPanel;
};

// ---- From File ----------------------------------------------------------------------------------
class CadFromFileMethod : public ReflectedMethod<CadFileSourceProps> {
 public:
  char const* Name() const override {
    return "From File";
  }
  char const* Description() const override {
    return "Uses a CAD Model from this modifier's own slot, independent of the editor's Model "
           "Viewer. A STEP file is tessellated with the options below; an STL is read directly.";
  }
  bool CanGenerate(ModifierRunContext const& /*ctx*/) const override {
    return !_props.path.empty();
  }
  std::string SourceFilePath(ModifierRunContext const& /*ctx*/) const override {
    return std::string{_props.path.data(), _props.path.size()};
  }
  std::vector<std::string_view> PathPropKeys() const override {
    return {"path"};
  }
  int DeserializeProps(picojson::value const& in) override {
    picojson::value props = in;
    if (props.is<picojson::object>()) {
      auto const step = props.get<picojson::object>().find("step");
      if (step != props.get<picojson::object>().end()) {
        DropRetiredStepFields(step->second);
      }
    }
    return ReflectedMethod::DeserializeProps(props);
  }
  void ShowParams(ModifierGuiContext const& gui) override {
    if (!gui.host) {
      return;
    }
    gui.host->ModelSlot("##cadsocket", _props.path, ModelSlotType::CadModel);
    gui.tooltip(
        "Drop a CAD Model here (a STEP/STL in the 'cad' folder). This slot is independent of the "
        "editor's own Model Viewer slots.");
    std::string const path{_props.path.data(), _props.path.size()};
    if (path.empty()) {
      return;
    }
    if (IsStlPath(path)) {
      ImGui::TextDisabled("STL: used directly; no tessellation options.");
      return;
    }
    ShowStepTessellationParams(_props.step, path, _faceColorPanel, gui);
  }
  mochi::MeshData Run(
      mochi::MeshData const& /*input*/,
      ModifierRunContext const& /*ctx*/,
      mochi::Error& error) const override {
    std::string const path{_props.path.data(), _props.path.size()};
    MOCHI_ERROR_IF(path.empty(), error, "Source from CAD Model: no CAD file is set.");
    MOCHI_ERROR_RETURN(error, {});
    return CadSourceMesh(path, _props.step, error);
  }

 private:
  FaceColorSizingPanel _faceColorPanel;
};

} // namespace

ModifierRegistryEntry MakeCadModelSourceEntry() {
  return ModifierRegistryEntry{"Source from CAD Model", ModifierKind::Source, []() {
                                 std::vector<std::unique_ptr<MeshProcessingMethod>> methods;
                                 methods.push_back(std::make_unique<CadFromViewerMethod>());
                                 methods.push_back(std::make_unique<CadFromFileMethod>());
                                 return methods;
                               }};
}

} // namespace superdex::studio
