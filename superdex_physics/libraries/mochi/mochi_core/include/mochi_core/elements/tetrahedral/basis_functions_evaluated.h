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

#include <mochi_core/elements/tetrahedral/basis_functions.h>
#include <mochi_core/elements/tetrahedral/simplex_quadrature.h>
#include <mochi_core/utils/nd_array.h>

namespace mochi::tetrahedral {

/// @brief The basis functions evaluated at the points of @p quadrature.
template <int kPolyOrder, int kNumQuadPoints>
constexpr auto EvaluateBasis(TetrahedralQuadrature<kNumQuadPoints> const& quadrature) {
  using Basis = BarycentricBasisTetrahedra<kPolyOrder>;
  NdArray<real, kNumQuadPoints, Basis::kNumDofs> result = {};
  for (int q = 0; q < kNumQuadPoints; ++q) {
    for (int f = 0; f < Basis::kNumDofs; ++f) {
      result[q][f] = Basis::GetValue(f, quadrature.points[q]);
    }
  }
  return result;
}

/// @brief The basis function gradients with respect to parametric coordinates, evaluated at the
/// points of @p quadrature.
template <int kPolyOrder, int kNumQuadPoints>
constexpr auto EvaluateBasisGradients(TetrahedralQuadrature<kNumQuadPoints> const& quadrature) {
  using Basis = BarycentricBasisTetrahedra<kPolyOrder>;
  NdArray<real, kNumQuadPoints, Basis::kNumDofs, 3> result = {};
  for (int q = 0; q < kNumQuadPoints; ++q) {
    for (int f = 0; f < Basis::kNumDofs; ++f) {
      result[q][f] = Basis::GetDValue(f, quadrature.points[q]);
    }
  }
  return result;
}

/// @brief @ref EvaluateBasis for each of the four face quadratures of a tetrahedron.
template <int kPolyOrder, int kNumQuadPoints>
constexpr auto EvaluateBasis(
    NdArray<TetrahedralQuadrature<kNumQuadPoints>, 4> const& faceQuadratures) {
  NdArray<decltype(EvaluateBasis<kPolyOrder>(faceQuadratures[0])), 4> result = {};
  for (int face = 0; face < 4; ++face) {
    result[face] = EvaluateBasis<kPolyOrder>(faceQuadratures[face]);
  }
  return result;
}

/// @brief @ref EvaluateBasisGradients for each of the four face quadratures of a tetrahedron.
template <int kPolyOrder, int kNumQuadPoints>
constexpr auto EvaluateBasisGradients(
    NdArray<TetrahedralQuadrature<kNumQuadPoints>, 4> const& faceQuadratures) {
  NdArray<decltype(EvaluateBasisGradients<kPolyOrder>(faceQuadratures[0])), 4> result = {};
  for (int face = 0; face < 4; ++face) {
    result[face] = EvaluateBasisGradients<kPolyOrder>(faceQuadratures[face]);
  }
  return result;
}

} // namespace mochi::tetrahedral
