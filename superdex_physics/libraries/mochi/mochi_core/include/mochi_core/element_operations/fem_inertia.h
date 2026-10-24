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

#include <mochi_core/element_operations/batched_element_utils.h>
#include <mochi_core/mochi_platform.h>
#include <mochi_core/utils/array_utils.h>
#include <mochi_core/utils/batch_types.h>
#include <mochi_core/utils/math_utils.h>
#include <mochi_core/utils/nd_array.h>
#include <mochi_core/utils/span.h>

#include <utility>

namespace mochi::fem {

namespace details {

// Returns 1/(N·(N + 1)) for a linear simplex with N nodes: its consistent mass matrix is
// ρ·|e|/(N·(N + 1))·(1 + δᵢⱼ) per displacement component, with |e| its measure and ρ the density.
template <class SimplexT>
[[nodiscard]] MOCHI_FORCE_INLINE constexpr real SimplexMassFactor() {
  static_assert(
      SimplexT::kPolyOrder == 1 && SimplexT::kNumDofs == SimplexT::kSpaceDimParam + 1,
      "The closed-form mass matrix holds only for linear simplices.");
  constexpr int kNumNodes = SimplexT::kNumDofs;
  return 1_r / static_cast<real>(kNumNodes * (kNumNodes + 1));
}

// Returns, for each lane, the off-diagonal entry of its simplex's consistent mass matrix at unit
// density, times scale and the lane's extra weight. Lanes past the batch get zero.
template <int kBatchSize, class SimplexT>
[[nodiscard]] MOCHI_FORCE_INLINE BatchReal<kBatchSize> SimplexMassOffDiagonal(
    NdArray<int, kBatchSize> const& elementIndices,
    Span<SimplexT const> elements,
    real scale,
    Span<real const> perElementExtraWeight) {
  using V = BatchReal<kBatchSize>;
  MOCHI_ASSERT_VERBOSE(
      Min(elementIndices) >= 0 && Max(elementIndices) < isize(elements),
      "Element index out of range.");
  MOCHI_ASSERT_VERBOSE(
      perElementExtraWeight.empty() || (perElementExtraWeight.size() == elements.size()),
      "Inconsistent sizes.");
  // Lanes expanded at compile time: a buffer written per lane and read back as one vector defeats
  // store forwarding.
  auto const perLane = [&]<size_t... kLanes>(
                           std::index_sequence<kLanes...>,
                           auto const& value) MOCHI_FORCE_INLINE_LAMBDA {
    return V{(static_cast<int>(kLanes) < kBatchSize ? value(elementIndices[kLanes]) : 0_r)...};
  };
  constexpr auto kAllLanes = std::make_index_sequence<V::kSize>{};
  // The quadrature weights include the Jacobian determinant, so they add up to the measure.
  V measure = perLane(
      kAllLanes, [&](int e) MOCHI_FORCE_INLINE_LAMBDA { return Sum(elements[e].quadWeights); });
  if (!perElementExtraWeight.empty()) {
    measure *= perLane(
        kAllLanes, [&](int e) MOCHI_FORCE_INLINE_LAMBDA { return perElementExtraWeight[e]; });
  }
  return V{scale * SimplexMassFactor<SimplexT>()} * measure;
}

} // namespace details

/// @brief Compute inertia work for a batch of linear simplices.
///
/// Implements the inertia term from Brown et al. 2018, "Accurate Dissipative Forces in
/// Optimization Integrators": (1/(h²α²))·M·(x − x̃), where x̃ = xₚ + α·h·vₚ is the stage-start
/// position (integration-method-dependent). M is the consistent mass matrix in closed form, c·(1 +
/// δᵢⱼ) per displacement component, with c = ρ·|e|/(N·(N + 1)) for an N-node simplex of measure |e|
/// (see @ref AddMassToDRes). With d = x − x̃ and s = Σᵢ dᵢ, node i of M·d is c·(dᵢ + s), and
/// dᵀMd = c·(Σᵢ |dᵢ|² + |s|²).
///
/// @param[in] elementIndices  Indices into @p elements for each batch lane.
/// @param[in] elements  Linear simplices, e.g. tetrahedra or triangles.
/// @param[in] disp  Batched displacements. Only the first ElementT::kNumDofs * ElementT::kSpaceDim
/// entries are read, so it may be a prefix span into a larger element vector.
/// @param[in] stageStartDispl  Batched stage-start displacements (x̃ = xₚ + α·h·vₚ). Only the first
/// ElementT::kNumDofs * ElementT::kSpaceDim entries are read.
/// @param[in,out] outEnergy  If non-null, accumulates per-element energy.
/// @param[in,out] outRes  If non-empty, accumulates per-element residual into the first
/// ElementT::kNumDofs * ElementT::kSpaceDim entries.
/// @param[in] density  Density: [kg/m³] for tetrahedra, [kg/m²] for triangles.
/// @param[in] dtfi2  Inverse squared time step factor: 1 / (Δt·α)².
/// @param[in] perElementExtraWeight  Optional per-element weight multiplier.
/// @return true if outputs were written.
///
/// @note The dresidual is added by @ref AddMassToDRes.
/// @note This is a spatial-only operation. It uses kNumFields == kSpaceDim implicitly. For element
/// types with extra DoFs per node (e.g., rod twist), use dedicated inertia functions.
template <int kBatchSize, class ElementT>
bool InertiaWork(
    NdArray<int, kBatchSize> const& elementIndices,
    Span<ElementT const> elements,
    Span<BatchReal<kBatchSize> const> disp,
    Span<BatchReal<kBatchSize> const> stageStartDispl,
    BatchDouble<kBatchSize>* outEnergy,
    Span<BatchReal<kBatchSize>> outRes,
    real density,
    real dtfi2,
    Span<real const> perElementExtraWeight = {}) {
  using V = BatchReal<kBatchSize>;
  using V3 = BatchReal3<kBatchSize>;
  using Vd = BatchDouble<kBatchSize>;
  constexpr auto kSpaceDim = ElementT::kSpaceDim;
  constexpr auto kNumNodes = ElementT::kNumDofs;
  constexpr auto kNumDofs = kNumNodes * kSpaceDim;
  static_assert(kSpaceDim == 3, "Only 3D elements are supported");
  MOCHI_ASSERT_VERBOSE(disp.size() >= kNumDofs, "Displacement span is too small.");
  MOCHI_ASSERT_VERBOSE(stageStartDispl.size() >= kNumDofs, "Stage-start span is too small.");
  MOCHI_ASSERT_VERBOSE(outRes.empty() || outRes.size() >= kNumDofs, "Residual span is too small.");

  bool const evalObj = (outEnergy != nullptr);
  bool const evalRes = !outRes.empty();
  if (!evalObj && !evalRes) {
    return false;
  }

  V const c = details::SimplexMassOffDiagonal<kBatchSize>(
      elementIndices, elements, density * dtfi2, perElementExtraWeight);
  // Recomputed in each pass: on some compilers (e.g. MSVC), a local array of d would live on the
  // stack.
  auto const delta = [&](int k) MOCHI_FORCE_INLINE_LAMBDA { return disp[k] - stageStartDispl[k]; };
  V3 sum = {};
  for (int f = 0; f < kNumNodes; ++f) {
    sum[0] += delta(kSpaceDim * f + 0);
    sum[1] += delta(kSpaceDim * f + 1);
    sum[2] += delta(kSpaceDim * f + 2);
  }

  if (evalObj) {
    // |s|² is added last, so that the multiply-add chain over d does not wait for s.
    V deltaNormSqr = {};
    for (int k = 0; k < kNumDofs; ++k) {
      V const dk = delta(k);
      deltaNormSqr += dk * dk;
    }
    *outEnergy += StaticCast<Vd>(0.5_r * c * (deltaNormSqr + NormSqr(sum)));
  }

  if (evalRes) {
    for (int f = 0; f < kNumNodes; ++f) {
      outRes[kSpaceDim * f + 0] += c * (delta(kSpaceDim * f + 0) + sum[0]);
      outRes[kSpaceDim * f + 1] += c * (delta(kSpaceDim * f + 1) + sum[1]);
      outRes[kSpaceDim * f + 2] += c * (delta(kSpaceDim * f + 2) + sum[2]);
    }
  }

  return true;
}

template <int kBatchSize, class ElementT>
MOCHI_FORCE_INLINE bool InertiaWork(
    NdArray<int, kBatchSize> const& elementIndices,
    Span<ElementT const> elements,
    BatchElementVector<kBatchSize, ElementT> const& disp,
    BatchElementVector<kBatchSize, ElementT> const& stageStartDispl,
    BatchDouble<kBatchSize>* outEnergy,
    BatchElementVector<kBatchSize, ElementT>* outRes,
    real density,
    real dtfi2,
    Span<real const> perElementExtraWeight = {}) {
  return InertiaWork<kBatchSize, ElementT>(
      elementIndices,
      elements,
      MakeConstSpan(disp),
      MakeConstSpan(stageStartDispl),
      outEnergy,
      outRes ? MakeSpan(*outRes) : Span<BatchReal<kBatchSize>>{},
      density,
      dtfi2,
      perElementExtraWeight);
}

/// @brief Add the scaled consistent mass matrices of linear simplices to batched element
/// dresiduals.
///
/// @details The consistent mass matrix ∫φᵢ·φⱼ of a linear simplex with N nodes and measure |e| is
/// |e|/(N·(N + 1))·(1 + δᵢⱼ) per displacement component, exactly. The first N nodes of
/// @p ElementT are the simplex nodes; the remaining entries of the element dresidual are left
/// unchanged.
///
/// @param[in] elementIndices  Indices into @p elements for each batch lane.
/// @param[in] elements  Linear simplices, e.g. tetrahedra or triangles.
/// @param[in] scale  Factor applied to the mass matrix, e.g. ρ·(1/(Δt·α)² + mass-damping scale).
/// @param[in,out] outDRes  Accumulates the scaled mass matrix for each lane.
/// @param[in] perElementExtraWeight  Optional per-element weight multiplier.
/// @return true.
template <int kBatchSize, class ElementT, class SimplexT>
bool AddMassToDRes(
    NdArray<int, kBatchSize> const& elementIndices,
    Span<SimplexT const> elements,
    real scale,
    BatchElementMatrix<kBatchSize, ElementT>& outDRes,
    Span<real const> perElementExtraWeight = {}) {
  static_assert(ElementT::kNumDofs >= SimplexT::kNumDofs);
  using V = BatchReal<kBatchSize>;
  constexpr int kSpaceDim = ElementT::kSpaceDim;
  constexpr int kNumDof = ElementT::kNumDofs * kSpaceDim;

  V const offDiagonal = details::SimplexMassOffDiagonal<kBatchSize>(
      elementIndices, elements, scale, perElementExtraWeight);
  V const diagonal = offDiagonal + offDiagonal;
  // c·(1 + δᵢⱼ), with the diagonal written apart rather than selected per entry.
  for (int i = 0; i < SimplexT::kNumDofs; ++i) {
    for (int d = 0; d < kSpaceDim; ++d) {
      int const row = (kSpaceDim * i + d) * kNumDof + d;
      for (int j = 0; j < i; ++j) {
        outDRes[row + kSpaceDim * j] += offDiagonal;
      }
      outDRes[row + kSpaceDim * i] += diagonal;
      for (int j = i + 1; j < SimplexT::kNumDofs; ++j) {
        outDRes[row + kSpaceDim * j] += offDiagonal;
      }
    }
  }
  return true;
}

/// @brief Computes the lumped mass of each displacement DoF: the row sums of the consistent mass
/// matrices of linear simplices (see @ref AddMassToDRes), which give each node ρ·|e|/N of each
/// N-node simplex e it belongs to.
///
/// @param[in] elements  Linear simplices, e.g. tetrahedra or triangles.
/// @param[in] density  Density: [kg/m³] for tetrahedra, [kg/m²] for triangles.
/// @param[in] activeElementIndices  Elements to include. Empty for all.
/// @param[in] perElementExtraWeight  Optional per-element weight multiplier.
/// @param[out] outLumpedMass  Lumped mass [kg] of each DoF, kSpaceDim per node. Nodes of no
/// included element get zero.
template <class SimplexT>
void ComputeLumpedMass(
    Span<SimplexT const> elements,
    real density,
    Span<int const> activeElementIndices,
    Span<real const> perElementExtraWeight,
    Span<real> outLumpedMass) {
  constexpr int kNumNodes = SimplexT::kNumDofs;
  constexpr int kSpaceDim = SimplexT::kSpaceDim;
  MOCHI_ASSERT_VERBOSE(
      perElementExtraWeight.empty() || (perElementExtraWeight.size() == elements.size()),
      "Inconsistent sizes.");
  // Each row of (1 + δᵢⱼ) adds up to N + 1.
  constexpr real kNodeShare =
      static_cast<real>(kNumNodes + 1) * details::SimplexMassFactor<SimplexT>();
  Fill(outLumpedMass, 0_r);
  bool const useSubset = !activeElementIndices.empty();
  int const numElements = useSubset ? isize(activeElementIndices) : isize(elements);
  for (int i = 0; i < numElements; ++i) {
    int const e = useSubset ? activeElementIndices[i] : i;
    real const weight = perElementExtraWeight.empty() ? 1_r : perElementExtraWeight[e];
    real const nodeMass = density * Sum(elements[e].quadWeights) * weight * kNodeShare;
    for (int const node : elements[e].Nodes()) {
      MOCHI_ASSERT_VERBOSE(
          kSpaceDim * node + kSpaceDim <= isize(outLumpedMass), "Node index out of range.");
      for (int d = 0; d < kSpaceDim; ++d) {
        outLumpedMass[kSpaceDim * node + d] += nodeMass;
      }
    }
  }
}

} // namespace mochi::fem
