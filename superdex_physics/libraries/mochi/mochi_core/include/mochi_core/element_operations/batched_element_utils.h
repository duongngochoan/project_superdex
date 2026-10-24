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

#include <mochi_core/utils/batch_types.h>
#include <mochi_core/utils/nd_array.h>
#include <mochi_core/utils/simd.h>
#include <mochi_core/utils/span.h>

#include <utility>

namespace mochi::fem {

// Batched element vector (displacement, residual).
template <int kBatchSize, class ElementT, int kNumFields = ElementT::kSpaceDim>
using BatchElementVector = NdArray<BatchReal<kBatchSize>, ElementT::kNumDofs * kNumFields>;

// Batched element matrix (dresidual).
template <int kBatchSize, class ElementT, int kNumFields = ElementT::kSpaceDim>
using BatchElementMatrix = NdArray<
    BatchReal<kBatchSize>,
    ElementT::kNumDofs * kNumFields * ElementT::kNumDofs * kNumFields>;

// Compute per-element extra weights.
template <size_t kBatchSize>
[[nodiscard]] MOCHI_FORCE_INLINE NdArray<real, kBatchSize> ComputeExtraWeights(
    NdArray<int, kBatchSize> const& elementIndices,
    Span<real const> perElementExtraWeight) {
  NdArray<real, kBatchSize> extraWeight MOCHI_NO_INIT;
  for (size_t b = 0; b < kBatchSize; ++b) {
    extraWeight[b] = perElementExtraWeight.empty() ? 1_r : perElementExtraWeight[elementIndices[b]];
  }
  return extraWeight;
}

// Pack basis functions and quadrature weights.
template <class ElementT, size_t kBatchSize>
MOCHI_FORCE_INLINE void PackBasisAndQuadWeight(
    NdArray<int, kBatchSize> const& elementIndices,
    Span<ElementT const> elements,
    NdArray<real, kBatchSize> const& extraWeight,
    int q,
    NdArray<BatchReal<kBatchSize>, ElementT::kNumDofs>& batchedBasis,
    BatchReal<kBatchSize>& batchedQuadWeight) {
  using V = BatchReal<kBatchSize>;
  constexpr int kNumNodes = ElementT::kNumDofs;
  alignas(alignof(V)) real staging[V::kSize]{};
  for (int f = 0; f < kNumNodes; ++f) {
    if constexpr (requires { ElementT::kBasisEvaluated; }) {
      // Shared by all elements of the type: broadcast it rather than gather identical values.
      batchedBasis[f] = V(ElementT::kBasisEvaluated[q][f]);
    } else {
      for (size_t b = 0; b < kBatchSize; ++b) {
        staging[b] = elements[elementIndices[b]].basisEvaluated[q][f];
      }
      batchedBasis[f] = Load<V>(staging);
    }
  }
  for (size_t b = 0; b < kBatchSize; ++b) {
    staging[b] = elements[elementIndices[b]].quadWeights[q] * extraWeight[b];
  }
  batchedQuadWeight = Load<V>(staging);
}

/**
 * @brief Gather a batch element vector from a global vector, e.g. the solution or the predicted
 * target.
 *
 * @tparam kBatchSize  Number of elements per batch
 * @tparam ElementT    Element type
 * @tparam kNumFields  Number of fields per node
 * @tparam kStride     Stride (in DoFs) between consecutive elements in @p indicesFlat. Defaults to
 *                     ElementT::kNumDofs * kNumFields (matching a uniform-stride L2G). Set to a
 *                     larger value when gathering a sub-stencil from a padded L2G with wider stride
 *                     (e.g., 3-node triangle inertia from a 6-node shell bending stencil with
 *                     stride 18).
 * @param[in]  indicesFlat       Flat local-to-global DoF indices
 * @param[in]  globalElemIndices Per-batch-lane element indices (padded to kBatchSize)
 * @param[in]  globalVec         Global vector
 * @param[out] outBatchElemVec   Batch element vector
 */
template <
    int kBatchSize,
    class ElementT,
    int kNumFields = ElementT::kSpaceDim,
    int kStride = ElementT::kNumDofs * kNumFields>
MOCHI_FORCE_INLINE void GatherBatchElementVector(
    Span<int const> indicesFlat,
    NdArray<int, kBatchSize> const& globalElemIndices,
    Span<real const> globalVec,
    BatchElementVector<kBatchSize, ElementT, kNumFields>& outBatchElemVec) {
  using V = BatchReal<kBatchSize>;
  static_assert(ElementT::kSpaceDim == 3, "GatherBatchElementVector requires kSpaceDim == 3");
  static_assert(kNumFields <= 4, "GatherBatchElementVector requires kNumFields <= 4");
  constexpr int kNumNodes = ElementT::kNumDofs;
  constexpr int kNumEleDofs = kNumNodes * kNumFields;
  static_assert(kStride >= kNumEleDofs, "Stride must be >= element DoF count.");

#if MOCHI_ASSERT_VERBOSE_ENABLED
  for (int b = 0; b < kBatchSize; ++b) {
    int const* const elemDofs = &indicesFlat[globalElemIndices[b] * kStride];
    for (int d = 0; d < kNumEleDofs; ++d) {
      MOCHI_ASSERT_VERBOSE(
          elemDofs[d] == elemDofs[d - d % kNumFields] + d % kNumFields,
          "Per-node DoFs must be contiguous in the global vector.");
    }
  }
#endif // MOCHI_ASSERT_VERBOSE_ENABLED

  // Per-node DoFs are contiguous in the global vector, so each node needs one index load per lane.
  // The lanes are expanded at compile time: some compilers (e.g. MSVC) otherwise do not unroll a
  // loop over them and keep the node pointers in memory.
  // TODO: Consider loading each lane's node values with one vector load and transpose them in
  // registers. Needs a LoadTransposed variant that takes one pointer per lane. See D122821232.
  auto const gatherNode =
      [&]<size_t... kLanes>(int nodeDof, std::index_sequence<kLanes...>) MOCHI_FORCE_INLINE_LAMBDA {
        int const elemDof[] = {
            globalElemIndices[Min(static_cast<int>(kLanes), kBatchSize - 1)] * kStride...};
        real const* const nodeValues[] = {&globalVec[indicesFlat[elemDof[kLanes] + nodeDof]]...};
        for (int f = 0; f < kNumFields; ++f) {
          outBatchElemVec[nodeDof + f] = V{nodeValues[kLanes][f]...};
        }
      };
  for (int nI = 0; nI < kNumNodes; ++nI) {
    gatherNode(nI * kNumFields, std::make_index_sequence<V::kSize>{});
  }
}

} // namespace mochi::fem
