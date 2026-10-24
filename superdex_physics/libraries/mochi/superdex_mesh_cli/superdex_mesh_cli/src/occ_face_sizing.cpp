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

#include "occ_face_sizing.h"

#if MOCHI_USE_OCCT

#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

#include <algorithm>
#include <limits>
#include <utility>

namespace mochi::mesh::cli::occ {

FaceSizing Min(FaceSizing const& a, FaceSizing const& b) {
  return {
      std::min(a.targetEdgeLength, b.targetEdgeLength),
      std::min(a.linearDeflection, b.linearDeflection),
      std::min(a.angularDeflection, b.angularDeflection)};
}

FaceSizingField::FaceSizingField(
    FaceSizing const& defaults,
    FaceSizingMap overrides,
    TopoDS_Shape const& shape)
    : _defaults(defaults), _overrides(std::move(overrides)) {
  if (_overrides.IsEmpty()) {
    return;
  }

  TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);
  for (int i = 1; i <= edgeToFaces.Extent(); ++i) {
    TopTools_ListOfShape const& faces = edgeToFaces(i);
    double finest = std::numeric_limits<double>::max();
    for (TopoDS_Shape const& face : faces) {
      finest = std::min(finest, At(face).targetEdgeLength);
    }
    for (TopoDS_Shape const& face : faces) {
      if (At(face).targetEdgeLength > finest) {
        _bordersFiner.Add(face);
      }
    }
  }
}

FaceSizing const& FaceSizingField::At(TopoDS_Shape const& face) const {
  FaceSizing const* const sizing = Find(face);
  return sizing != nullptr ? *sizing : _defaults;
}

} // namespace mochi::mesh::cli::occ

#endif // MOCHI_USE_OCCT
