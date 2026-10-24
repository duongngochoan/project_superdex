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

// Part of the OCCT BRepMesh isotropic tessellation pipeline. Only compiled where OpenCascade
// is available (MOCHI_USE_OCCT); elsewhere this header is an empty translation unit.
#if MOCHI_USE_OCCT

#include <NCollection_DataMap.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Shape.hxx>

namespace mochi::mesh::cli::occ {

/// Resolved meshing sizes for a face; every value is positive.
struct FaceSizing {
  double targetEdgeLength{}; ///< [mm]
  double linearDeflection{}; ///< [mm]
  double angularDeflection{}; ///< [rad]
};

/// Component-wise minimum, the sizing an edge shared by @p a and @p b must honor.
[[nodiscard]] FaceSizing Min(FaceSizing const& a, FaceSizing const& b);

/// Per-face sizing keyed by face (location-aware, orientation-agnostic).
using FaceSizingMap = NCollection_DataMap<TopoDS_Shape, FaceSizing, TopTools_ShapeMapHasher>;

/// Global sizing plus optional per-face overrides for one shape. Immutable once built, so
/// BRepMesh's parallel edge and face workers may share it.
class FaceSizingField {
 public:
  /// @param overrides Sizing keyed by faces of @p shape as TopExp explores them; may be empty.
  FaceSizingField(FaceSizing const& defaults, FaceSizingMap overrides, TopoDS_Shape const& shape);

  [[nodiscard]] FaceSizing const& Defaults() const {
    return _defaults;
  }

  [[nodiscard]] bool HasOverrides() const {
    return !_overrides.IsEmpty();
  }

  /// Override for @p face, or null when it uses @ref Defaults.
  [[nodiscard]] FaceSizing const* Find(TopoDS_Shape const& face) const {
    return _overrides.Seek(face);
  }

  [[nodiscard]] FaceSizing const& At(TopoDS_Shape const& face) const;

  /// True when @p face shares an edge with a face that has a smaller target edge length, so its
  /// interior must grade toward that finer boundary instead of fanning into it.
  [[nodiscard]] bool BordersFinerFace(TopoDS_Shape const& face) const {
    return _bordersFiner.Contains(face);
  }

 private:
  FaceSizing _defaults;
  FaceSizingMap _overrides;
  TopTools_MapOfShape _bordersFiner;
};

} // namespace mochi::mesh::cli::occ

#endif // MOCHI_USE_OCCT
