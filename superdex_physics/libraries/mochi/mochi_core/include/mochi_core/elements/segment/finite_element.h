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

#include <mochi_core/elements/segment/basis_functions_evaluated.h>
#include <mochi_core/utils/math_utils.h>
#include <mochi_core/utils/matrix_utils.h>

namespace mochi::segment {

/**
 * @brief 1D scalar Lagrange segment element with equal-order field interpolation.
 */
template <int kPolyOrder_, int kNumQuadPoints_>
class Pk1DElement final {
  static_assert(kPolyOrder_ == 1, "Higher order polynomials not yet implemented");

 public:
  static constexpr int kPolyOrder = kPolyOrder_;
  static constexpr int kNumQuadPoints = kNumQuadPoints_;
  static constexpr int kSpaceDim = 3;
  static constexpr int kSpaceDimParam = 1;
  using Basis = BasisSegment<kPolyOrder>;
  using BasisEvaluated = BasisFunctionsEvaluatedSegment<kPolyOrder, kNumQuadPoints>;
  static constexpr int kNumDofs = Basis::kNumDofs;

  /**
   * @brief Construct a Lagrange segment element.
   * @param elementIndex Index of the element.
   * @param coordinates Reference to the whole set of coordinates of the mesh nodes. The second
   *        node index is computed as (elementIndex + 1) % isize(coordinates), which handles both
   *        open and closed-loop topologies.
   * @note Connectivity is inferred from the order of the nodal coordinates.
   */
  Pk1DElement(int elementIndex, Span<Real3 const> coordinates)
      : elementIndex(elementIndex), connectivity{isize(coordinates)} {
    QuadratureEvaluateMap(InterpolateInteriorNodes(coordinates, Nodes()));
  }

 private:
  /** @brief Tabulate the isoparametric map and the quadrature weights at the quad points. */
  void QuadratureEvaluateMap(NdArray<real, kNumDofs, kSpaceDim> const& nodesCrdsPhys) {
    for (int q = 0; q < kNumQuadPoints; ++q) {
      // Here we are using an isoparametric map
      mapEvaluated[q] = {};
      Real3 tangent = {};
      for (int f = 0; f < kNumDofs; ++f) {
        // Get the map of the quadrature point
        mapEvaluated[q] += kBasisEvaluatedParametric.kBasisEvaluated[q][f] * nodesCrdsPhys[f];

        // Get the tangent map at the quadrature point, which for parametric dimension 1 is a
        // single vector
        tangent += kBasisEvaluatedParametric.kDBasisEvaluated[q][f][0] * nodesCrdsPhys[f];
      }

      // Compute quadrature weight. The Jacobian determinant of the mapping of a 1D element
      // embedded in 3D is the norm of the tangent vector.
      quadWeights[q] = kQuadrature.weights[q] * Norm(tangent);
    }
  }

  /** @brief Compute nodal positions in physical space. */
  static NdArray<real, kNumDofs, kSpaceDim> InterpolateInteriorNodes(
      Span<Real3 const> coordinates,
      Int2 nodes) {
    NdArray<real, kNumDofs, kSpaceDim> nodesCrdsPhys;
    constexpr int kNumVertex = 2;
    for (int i = 0; i < kNumVertex; ++i) {
      nodesCrdsPhys[i] = coordinates[nodes[i]];
    }
    // TODO: Interpolate interior nodes if implementing higher order elements
    static_assert(kPolyOrder == 1, "Higher order polynomials not yet implemented");
    return nodesCrdsPhys;
  }

 public:
  static constexpr int kNumNodes = 2;
  static constexpr SegmentQuadrature<kNumQuadPoints> kQuadrature = {};
  static constexpr BasisEvaluated kBasisEvaluatedParametric = {};
  static constexpr Basis kBasis = {};
  static constexpr NdArray<real, kNumQuadPoints, kNumDofs> kBasisEvaluated =
      kBasisEvaluatedParametric.kBasisEvaluated;
  static constexpr NdArray<real, kNumQuadPoints, kNumDofs> basisEvaluated = kBasisEvaluated;

  // NOTE: These seemingly trivial accessor methods are for compatibility with templated
  // code that also supports other element types.
  inline auto const& GetBaseElement() const {
    return *this;
  }
  inline Int2 Nodes() const {
    return connectivity[elementIndex];
  }
  inline constexpr Int2 LocalNodes() const {
    return Int2{0, 1};
  }
  inline int GetElementIndex() const {
    return elementIndex;
  }

  // For the segment case, the "element" maps to two consecutive nodes. The node pair is
  // derived directly from the (outer) element index, with the second node wrapped modulo
  // `numNodes` to support closed-loop topologies.
  struct SegmentConnectivity {
    int numNodes = 0;
    Int2 operator[](int i) const {
      MOCHI_ASSERT_VERBOSE(i >= 0 && i < numNodes, "Invalid element index");
      return {i, (i + 1) % numNodes};
    }
  };

  // Set by constructor
  int const elementIndex;
  SegmentConnectivity const connectivity;

  // The array of mapped quadrature points into physical space
  NdArray<real, kNumQuadPoints, kSpaceDim> mapEvaluated;

  // The quadrature weights for arc length integration
  NdArray<real, kNumQuadPoints> quadWeights = {};
};

} // namespace mochi::segment
