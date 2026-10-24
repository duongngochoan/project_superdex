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

#include <mochi_core/elements/finite_element_utils.h>
#include <mochi_core/elements/tetrahedral/basis_functions_evaluated.h>
#include <mochi_core/elements/tetrahedral/simplex_quadrature.h>
#include <mochi_core/geometry/geometry_utils.h>
#include <mochi_core/utils/debug.h>
#include <mochi_core/utils/nd_array.h>
#include <mochi_core/utils/span.h>

namespace mochi::tetrahedral {

/**
The lagrange Tetrahedral element.
  In the mindset of keeping memory demand as low as possible
  and high efficiency, the element assumes the same interpolant
  for each of the fields; a reasonable assumption in the case of
  elasticity. This code is not intended to be as general as
  possible rather striking a balance between somewhat
  efficient and versatile.
*/
template <int kPolyOrder_, int kNumQuadPoints_ = 1>
class Pk3DElement final {
  static_assert(kPolyOrder_ == 1, "Higher order polynomials not yet implemented");

 public:
  using Basis = BarycentricBasisTetrahedra<kPolyOrder_>;

  static constexpr int kPolyOrder = kPolyOrder_;
  static constexpr int kNumQuadPoints = kNumQuadPoints_;
  static constexpr int kSpaceDimParam = 3;
  static constexpr int kSpaceDim = 3;
  static constexpr int kNumDofs = Basis::kNumDofs;
  static constexpr Basis kBasis = {};
  static constexpr TetrahedralQuadrature<kNumQuadPoints> kQuadrature =
      GetTetrahedralQuadrature<kNumQuadPoints>();

  // The basis functions, and their gradients with respect to parametric coordinates, evaluated at
  // the quad points. They are the same for every element.
  static constexpr NdArray<real, kNumQuadPoints, kNumDofs> kBasisEvaluated =
      EvaluateBasis<kPolyOrder>(kQuadrature);
  static constexpr NdArray<real, kNumQuadPoints, kNumDofs, kSpaceDimParam>
      kDBasisEvaluatedParametric = EvaluateBasisGradients<kPolyOrder>(kQuadrature);
  static constexpr NdArray<real, kNumQuadPoints, kNumDofs> basisEvaluated = kBasisEvaluated;

  /**
    The constructor for a lagrange tet element

    Args:
      elementIndex (int): the index of the element
      coordinates (ndarray): reference to the whole set of coordinates of the mesh nodes (passed by
        reference)

      connectivity (ndarray): reference to the connectivity of the whole mesh (passed by reference)
  */
  Pk3DElement(int elementIndex, Span<Real3 const> coordinates, Span<Int4 const> connectivity)
      : elementIndex(elementIndex), coordinates(coordinates), connectivity(connectivity) {
    // Construct the array containing the physical coordinates corresponding to the coordinates of
    // each degree of freedom
    InterpolateInteriorNodes();

    Initialize();
  }

 private:
  /**
    Tabulate the isoparametric map, the quadrature weights and the basis derivatives with respect
    to physical coordinates at the quad points
  */
  void Initialize() {
    for (int q = 0; q < kNumQuadPoints; ++q) {
      // Here we are using an isoparametric map
      mapEvaluated[q] = {};
      EvaluateField<kSpaceDim, kNumDofs>(kBasisEvaluated[q], nodesCrdsPhys, &mapEvaluated[q]);

      // Note this is slight abuse of this function as technically should be called with the
      // derivative of the basis wrt to reference coordinates not parametric coordinates.
      // We need the induced basis from the isoparametric mapping (dMap) to compute the element
      // basis derivative wrt reference coordinates (dBasisEvaluated).
      NdArray<real, kSpaceDim, kSpaceDimParam> dMap;
      EvaluateFieldGradient<kSpaceDim, kNumDofs, kSpaceDimParam>(
          kDBasisEvaluatedParametric[q], nodesCrdsPhys, &dMap);
      real const dMapDet = Det(dMap);
      NdArray<real, kSpaceDimParam, kSpaceDim> const dMapInv = Invert(dMap, dMapDet);

      // Compute quadrature weight
      quadWeights[q] = kQuadrature.weights[q] * dMapDet;

      // Compute gradient with respect to physical coordinates
      for (int f = 0; f < kNumDofs; ++f) {
        dBasisEvaluated[q][f] = DotMatVec(Transpose(dMapInv), kDBasisEvaluatedParametric[q][f]);
      }
    }
  }

  /**
    Computes the nodal position in physical space of interior nodes.

    Only relevant for higher order elements this method interpolates linearly the mid and interior
    nodes on the physical domain.
  */
  void InterpolateInteriorNodes() {
    constexpr int kNumVertex = 4;

    // Here we are simply interpolating linearly the physical coordinates of the interior nodes this
    // could be more general
    for (int i = 0; i < kNumVertex; ++i) {
      nodesCrdsPhys[i] = coordinates[connectivity[elementIndex][i]];
    }
    if constexpr (kPolyOrder > 1) {
      auto nodeCrdsParams = PrecomputeNodesCrdsParam();
      for (int i = kNumVertex; i < kNumDofs; ++i) {
        nodesCrdsPhys[i] = nodeCrdsParams[0] * nodeCrdsParams[0] +
            nodeCrdsParams[1] * nodeCrdsParams[1] + nodeCrdsParams[2] * nodeCrdsParams[2] +
            (1_r - nodeCrdsParams[0] - nodeCrdsParams[1] - nodeCrdsParams[2]) * nodeCrdsParams[3];
      }
    }
  }

  // Compile-time algorithm to compute kNodesCrdsParam
  static constexpr auto PrecomputeNodesCrdsParam() {
    // Get the coordinates of the nodes in the parametric domain this are the nodes of the exterior
    // vertices, edge nodes, as well as interior nodes. Here we remove the last column that
    // correspond to the additional barycentric coordinate
    NdArray<real, kNumDofs, kSpaceDimParam> nodesCrdsParam = {};
    for (size_t i = 0; i < kBasis.kDofNodes.size(); ++i) { // for each row
      auto const& row = kBasis.kDofNodes[i];
      for (size_t j = 0; j < row.size() - 1; ++j) { // for each column except the last one
        nodesCrdsParam[i][j] = kBasis.kDofNodes[i][j];
      }
    }
    return nodesCrdsParam;
  }

 public:
  // Coordinates of the nodes in the parametric domain
  static constexpr NdArray<real, kNumDofs, 3> kNodesCrdsParam = PrecomputeNodesCrdsParam();

  // Set by constructor
  int elementIndex;
  Span<Real3 const> coordinates;
  Span<Int4 const> connectivity;

  // The nodal position in physical space of interior nodes.
  NdArray<real, kNumDofs, kSpaceDim> nodesCrdsPhys;

  // The array of mapped quadrature points into physical space
  NdArray<real, kNumQuadPoints, kSpaceDim> mapEvaluated;

  // dbasis evaluated at quad pts
  NdArray<real, kNumQuadPoints, kNumDofs, kSpaceDim> dBasisEvaluated;

  // The quadrature weights for volume integration
  NdArray<real, kNumQuadPoints> quadWeights = {};

  inline int GetElementIndex() const {
    return elementIndex;
  }

  /// @brief Get the node indices of the element.
  Int4 const& Nodes() const {
    return connectivity[elementIndex];
  }

  // Evaluate the inverse isoparametric map for a point x in physical space.
  inline NdArray<real, 3> GetInvMap(NdArray<real, 3> const& x) const {
    Int4 corners = connectivity[elementIndex];
    auto coords = BarycentricCoords4(
        coordinates[corners[0]],
        coordinates[corners[1]],
        coordinates[corners[2]],
        coordinates[corners[3]],
        x);
    return {coords[1], coords[2], coords[3]}; // Drop the first coordinate
  }
};

} // namespace mochi::tetrahedral
