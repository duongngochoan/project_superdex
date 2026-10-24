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

#include <mochi_core/element_operations/fem_inertia.h>
#include <mochi_core/element_operations/fem_shell.h>
#include <mochi_core/elements/tetrahedral/finite_element.h>
#include <mochi_core/elements/triangular/finite_element.h>
#include <mochi_core/geometry/tetrahedral_mesh.h>
#include <mochi_core/test/batched_work_test_utils.h>
#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_core/utils/dynamic_array.h>

#include <gtest/gtest.h>

#include <array>

using namespace mochi;
using namespace mochi::fem;

// The inertia term is (1/(h²α²))·M·(x − x̃) with M the consistent mass matrix, here integrated by a
// quadrature that is exact for it (the oracle). With a = disp − x̃, the closed-form kernels must
// satisfy:
//   res    = dtfi2 · M · a
//   energy = ½ · dtfi2 · aᵀ M a
//   dRes   = scale · M                (AddMassToDRes)
//   lumped = row sums of M            (ComputeLumpedMass)
// Tetrahedra fill a 4-node element. Triangles fill the leading 3 nodes of the 6-node shell
// stencil, whose other entries must stay untouched. x̃ is gathered from nodal data by
// GatherBatchElementVector with the element's stride. Each lane gets distinct displacement data,
// with element indices cycling through the mesh, so every node is gathered. Larger batch sizes
// also exercise duplicate element lanes while preserving per-lane independence (mixed-lane).

// Outputs start here, so the batched kernels must accumulate and ComputeLumpedMass overwrite.
static constexpr real kInitialValue = -1000_r;

namespace {

// Closed-form element, quadrature element on the same mesh, and assembly element.
struct TetCase {
  using Simplex = tetrahedral::Pk3DElement<1, 1>;
  using Quadrature = tetrahedral::Pk3DElement<1, 4>;
  using Stencil = Simplex;
  template <class ElementT>
  using MeshData = TestTetMeshData<ElementT>;
};

struct TriangleCase {
  using Simplex = triangular::Pk2DElement<1, 1>;
  using Quadrature = triangular::Pk2DElement<1, 3>;
  using Stencil = ShellStencilElement;
  template <class ElementT>
  using MeshData = TestTriMeshData<ElementT>;
};

} // namespace

template <class Fn>
static void RunSimplexCases(Fn const& fn) {
  fn.template operator()<TetCase>();
  fn.template operator()<TriangleCase>();
}

template <class ElementT>
static constexpr int kDisplDim = ElementT::kSpaceDim * ElementT::kNumDofs;

// Consistent mass matrix ∫ρ·φᵢ·φⱼ per displacement component, by quadrature.
template <class ElementT>
static NdArray<real, kDisplDim<ElementT>, kDisplDim<ElementT>> QuadratureMass(
    ElementT const& element,
    real density) {
  constexpr int kSpaceDim = ElementT::kSpaceDim;
  NdArray<real, kDisplDim<ElementT>, kDisplDim<ElementT>> mass{};
  for (int q = 0; q < ElementT::kNumQuadPoints; ++q) {
    for (int f = 0; f < ElementT::kNumDofs; ++f) {
      for (int g = 0; g < ElementT::kNumDofs; ++g) {
        real const m = density * element.basisEvaluated[q][f] * element.basisEvaluated[q][g] *
            element.quadWeights[q];
        for (int d = 0; d < kSpaceDim; ++d) {
          mass[f * kSpaceDim + d][g * kSpaceDim + d] += m;
        }
      }
    }
  }
  return mass;
}

template <int kBS, size_t kDim>
static NdArray<BatchReal<kBS>, kDim> PackBatch(NdArray<NdArray<real, kDim>, kBS> const& perLane) {
  using V = BatchReal<kBS>;
  NdArray<V, kDim> out{};
  alignas(alignof(V)) real staging[V::kSize]{};
  for (size_t k = 0; k < kDim; ++k) {
    for (int b = 0; b < kBS; ++b) {
      staging[b] = perLane[b][k];
    }
    out[k] = Load<V>(staging);
  }
  return out;
}

template <class Case, int kBS>
static void VerifyBatchedInertia(OutputConfig cfg) {
  using Simplex = typename Case::Simplex;
  using Stencil = typename Case::Stencil;
  constexpr int kDim = kDisplDim<Simplex>;
  constexpr int kStencilDim = kDisplDim<Stencil>;
  auto const data = Case::template MeshData<Simplex>::CreateMinimalCube();
  auto const quadData = Case::template MeshData<typename Case::Quadrature>::CreateMinimalCube();
  int const numElements = isize(data.elements);

  real const density = 1000_r;
  real const dtFactor = 0.01_r;
  real const dtfi2 = 1_r / (dtFactor * dtFactor);
  real const dresScale = 3_r * dtfi2;

  DynamicArray<real> extraWeights(numElements);
  for (int i = 0; i < numElements; ++i) {
    extraWeights[i] = 1_r + 0.1_r * static_cast<real>(i);
  }

  // The simplex leads each stencil; the gather must not read the other stencil entries.
  auto const connectivity = data.mesh.GetElementConnectivity();
  DynamicArray<int> indicesFlat(kStencilDim * numElements, 0);
  for (int e = 0; e < numElements; ++e) {
    for (int k = 0; k < kDim; ++k) {
      indicesFlat[e * kStencilDim + k] = 3 * connectivity[e][k / 3] + k % 3;
    }
  }
  // Exactly sized, so ASAN catches a read past the last node.
  DynamicArray<real> target(3 * data.mesh.GetNumNodes());
  auto gen = RandomGenerator(7919);
  SetRandom(gen, -0.3_r, 0.3_r, MakeSpan(target));

  for (int trial = 0; trial < kNumTrials; ++trial) {
    NdArray<int, kBS> idx;
    for (int b = 0; b < kBS; ++b) {
      idx[b] = (trial + b) % numElements;
    }
    NdArray<NdArray<real, kStencilDim>, kBS> disp;
    for (int b = 0; b < kBS; ++b) {
      disp[b] = MakeRandomArray<kStencilDim>(static_cast<unsigned int>(trial * kBS + b));
    }

    auto const dispBatch = PackBatch<kBS>(disp);
    fem::BatchElementVector<kBS, Simplex> targetBatch;
    GatherBatchElementVector<kBS, Simplex, Simplex::kSpaceDim, kStencilDim>(
        MakeConstSpan(indicesFlat), idx, MakeConstSpan(target), targetBatch);
    auto const elemSpan = MakeConstSpan(data.elements);
    // Every other trial passes no weights.
    Span<real const> const weights =
        trial % 2 == 0 ? Span<real const>{} : MakeConstSpan(extraWeights);

    BatchDouble<kBS> energy{static_cast<double>(kInitialValue)};
    fem::BatchElementVector<kBS, Stencil> res;
    fem::BatchElementMatrix<kBS, Stencil> dres;
    Fill(MakeSpan(res), BatchReal<kBS>{kInitialValue});
    Fill(MakeSpan(dres), BatchReal<kBS>{kInitialValue});
    if (cfg.energy || cfg.residual) {
      EXPECT_TRUE(
          InertiaWork<kBS>(
              idx,
              elemSpan,
              MakeConstSpan(dispBatch),
              MakeConstSpan(targetBatch),
              cfg.energy ? &energy : nullptr,
              cfg.residual ? MakeSpan(res) : Span<BatchReal<kBS>>{},
              density,
              dtfi2,
              weights));
    }
    if (cfg.dresidual) {
      EXPECT_TRUE((AddMassToDRes<kBS, Stencil>(idx, elemSpan, density * dresScale, dres, weights)));
    }

    for (int b = 0; b < kBS; ++b) {
      auto const mm = QuadratureMass(quadData.elements[idx[b]], density) *
          (weights.empty() ? 1_r : extraWeights[idx[b]]);
      std::array<real, kDim> a{};
      for (int k = 0; k < kDim; ++k) {
        a[k] = disp[b][k] - target[indicesFlat[idx[b] * kStencilDim + k]];
      }

      if (cfg.residual) {
        NdArray<real, kStencilDim> refRes{};
        NdArray<real, kStencilDim> actualRes{};
        for (int i = 0; i < kStencilDim; ++i) {
          real massTimesA = 0_r;
          if (i < kDim) {
            for (int j = 0; j < kDim; ++j) {
              massTimesA += mm[i][j] * a[j];
            }
          }
          refRes[i] = kInitialValue + dtfi2 * massTimesA;
          actualRes[i] = res[i][b];
        }
        ExpectNearL2(refRes, actualRes);
      }

      if (cfg.energy) {
        double refEnergy = 0.0;
        for (int i = 0; i < kDim; ++i) {
          for (int j = 0; j < kDim; ++j) {
            refEnergy += static_cast<double>(a[i]) * static_cast<double>(mm[i][j]) *
                static_cast<double>(a[j]);
          }
        }
        refEnergy = 0.5 * static_cast<double>(dtfi2) * refEnergy + kInitialValue;
        ExpectNearEnergy(refEnergy, energy[b]);
      }

      if (cfg.dresidual) {
        NdArray<real, kStencilDim * kStencilDim> refDRes{};
        NdArray<real, kStencilDim * kStencilDim> actualDRes{};
        for (int i = 0; i < kStencilDim; ++i) {
          for (int j = 0; j < kStencilDim; ++j) {
            real const mass = (i < kDim && j < kDim) ? mm[i][j] : 0_r;
            refDRes[i * kStencilDim + j] = kInitialValue + dresScale * mass;
            actualDRes[i * kStencilDim + j] = dres[i * kStencilDim + j][b];
          }
        }
        ExpectNearL2(refDRes, actualDRes);
      }
    }
  }
}

// The lumped mass sums the rows of the mass matrices of all elements, or of a weighted subset.
template <class Case>
static void VerifyLumpedMass() {
  using Simplex = typename Case::Simplex;
  constexpr int kDim = kDisplDim<Simplex>;
  auto const data = Case::template MeshData<Simplex>::CreateMinimalCube();
  auto const quadData = Case::template MeshData<typename Case::Quadrature>::CreateMinimalCube();
  int const numElements = isize(data.elements);
  real const density = 1000_r;
  DynamicArray<int> subset;
  DynamicArray<real> weights(numElements);
  for (int e = 0; e < numElements; ++e) {
    if (e % 2 == 1) {
      subset.push_back(e);
    }
    weights[e] = 1_r + 0.1_r * static_cast<real>(e);
  }

  for (bool const useSubset : {false, true}) {
    Span<int const> const activeIndices = useSubset ? MakeConstSpan(subset) : Span<int const>{};
    Span<real const> const activeWeights = useSubset ? MakeConstSpan(weights) : Span<real const>{};
    DynamicArray<real> lumpedMass(3 * data.mesh.GetNumNodes(), kInitialValue);
    ComputeLumpedMass(
        MakeConstSpan(data.elements), density, activeIndices, activeWeights, MakeSpan(lumpedMass));

    DynamicArray<real> expected(3 * data.mesh.GetNumNodes(), 0_r);
    for (int e = 0; e < numElements; ++e) {
      if (useSubset && e % 2 == 0) {
        continue;
      }
      auto const mm = QuadratureMass(quadData.elements[e], density);
      auto const nodes = data.mesh.GetElementConnectivity()[e];
      for (int i = 0; i < kDim; ++i) {
        expected[3 * nodes[i / 3] + i % 3] += (useSubset ? weights[e] : 1_r) * Sum(mm[i]);
      }
    }
    real errorSqr = 0_r;
    real expectedSqr = 0_r;
    for (int i = 0; i < isize(expected); ++i) {
      errorSqr += Sqr(lumpedMass[i] - expected[i]);
      expectedSqr += Sqr(expected[i]);
    }
    EXPECT_LE(Sqrt(errorSqr), kRelTol * Sqrt(expectedSqr));
  }
}

TEST(Inertia, MatchesReference) {
  RunSimplexCases([&]<class Case>() {
    for (auto cfg : kAllOutputConfigs) {
      RunAllBatchSizes([&]<int kBS>() { VerifyBatchedInertia<Case, kBS>(cfg); });
    }
    VerifyLumpedMass<Case>();
  });
}
