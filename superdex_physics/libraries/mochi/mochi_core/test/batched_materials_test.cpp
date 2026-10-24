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

#include "batched_material_invariant_test_helpers.h"

#include "data/material_test_data.h"

#include <mochi_core/materials/batched_materials.h>
#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_core/utils/container_utils.h>
#include <mochi_core/utils/math_utils.h>
#include <mochi_core/utils/span.h>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

using namespace mochi;
using namespace mochi::materials;
using namespace mochi::materials::test;

template <typename Spec, int kBS>
static void RunCommonForBatchSize() {
  auto const permutedIndices = MakePermutedElementIndices<kBS>();
  VerifyBatchedMatchesSingleLaneBatched<Spec, kBS, BatchedParamLayout::Homogeneous>(
      permutedIndices);
  VerifyBatchedMatchesSingleLaneBatched<Spec, kBS, BatchedParamLayout::PerElementHomogeneous>(
      permutedIndices);
  VerifyBatchedMatchesSingleLaneBatched<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>(
      permutedIndices);

  VerifyBatchedOutputMasks<Spec, kBS, BatchedParamLayout::Homogeneous>();
  VerifyBatchedOutputMasks<Spec, kBS, BatchedParamLayout::PerElementHomogeneous>();
  VerifyBatchedOutputMasks<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();

  if constexpr (ShouldRunBatchedFiniteDifference<Spec>()) {
    VerifyBatchedPk1FiniteDifference<Spec, kBS, BatchedParamLayout::Homogeneous>();
    VerifyBatchedTangentFiniteDifference<Spec, kBS, BatchedParamLayout::Homogeneous>();
    VerifyBatchedPk1FiniteDifference<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();
    VerifyBatchedTangentFiniteDifference<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();
  }

  VerifyBatchedPsdStrategyDoesNotAffectEnergyOrPk1<Spec, kBS, BatchedParamLayout::Homogeneous>();
  VerifyBatchedPsdStrategyDoesNotAffectEnergyOrPk1<
      Spec,
      kBS,
      BatchedParamLayout::PerElementHomogeneous>();
  VerifyBatchedPsdStrategyDoesNotAffectEnergyOrPk1<
      Spec,
      kBS,
      BatchedParamLayout::PerElementHeterogeneous>();

#if MOCHI_USE_EIGEN
  VerifyBatchedPsdProjectionProducesPsdTangent<Spec, kBS, BatchedParamLayout::Homogeneous>();
  VerifyBatchedPsdProjectionProducesPsdTangent<
      Spec,
      kBS,
      BatchedParamLayout::PerElementHomogeneous>();
  VerifyBatchedPsdProjectionProducesPsdTangent<
      Spec,
      kBS,
      BatchedParamLayout::PerElementHeterogeneous>();
  if constexpr (Spec::kHasAnalyticPsdProjectionTests) {
    VerifyBatchedAnalyticPsdProjection<Spec, kBS, BatchedParamLayout::Homogeneous>(
        Spec::kAnalyticPsdProjectionRelTol);
  }
#endif
}

template <typename Spec>
static void RunCommonMaterialTests() {
  RunCommonForBatchSize<Spec, 1>();
  RunCommonForBatchSize<Spec, 4>();
  RunCommonForBatchSize<Spec, 8>();
}

template <typename Spec, int kBS>
static void RunInvariantsForBatchSize() {
  if constexpr (requires { Spec::kHasDefaultFrameInvariantRestState; }) {
    if constexpr (Spec::kHasDefaultFrameInvariantRestState) {
      VerifyBatchedDefaultUndeformedTet<Spec, kBS, BatchedParamLayout::Homogeneous>();
      VerifyBatchedDefaultUndeformedTet<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();
    }
  }
  if constexpr (Spec::kHasFrameInvariantRestState) {
    VerifyBatchedUndeformedTet<Spec, kBS, BatchedParamLayout::Homogeneous>();
    VerifyBatchedUndeformedTet<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();
  }
  if constexpr (Spec::kHasRotationInvariantRestState) {
    VerifyBatchedRotatedTet<Spec, kBS, BatchedParamLayout::Homogeneous>();
    VerifyBatchedRotatedTet<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();
  }
  if constexpr (Spec::kHasMinimumEnergy) {
    VerifyBatchedMinimumEnergy<Spec, kBS, BatchedParamLayout::Homogeneous>();
    VerifyBatchedMinimumEnergy<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();
  }
  if constexpr (Spec::kHasMatchingTarget) {
    VerifyBatchedMatchingShapeTarget<Spec, kBS, BatchedParamLayout::Homogeneous>();
    VerifyBatchedMatchingShapeTarget<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();
  }
  if constexpr (Spec::kHasLinearElasticLimit) {
    VerifyBatchedLinearElasticLimit<Spec, kBS, BatchedParamLayout::Homogeneous>();
    VerifyBatchedLinearElasticLimit<Spec, kBS, BatchedParamLayout::PerElementHeterogeneous>();
  }
}

template <typename Spec>
static void RunInvariantMaterialTests() {
  RunInvariantsForBatchSize<Spec, 1>();
  RunInvariantsForBatchSize<Spec, 4>();
  RunInvariantsForBatchSize<Spec, 8>();
}

template <typename Spec>
static void RunMaterialTests() {
  RunCommonMaterialTests<Spec>();
  RunInvariantMaterialTests<Spec>();
}

static void RunSmithNeoHookeanOracleMaterialTests(MaterialPsdOracle oracle) {
  static_assert(
      static_cast<int>(MaterialPsdOracle::Count) == 3,
      "Please update switch statement below if MaterialPsdOracle enum changes");
  switch (oracle) {
    case MaterialPsdOracle::None:
      RunMaterialTests<SmithNeoHookeanOracleSpec<MaterialPsdOracle::None>>();
      break;
    case MaterialPsdOracle::Correct:
      RunMaterialTests<SmithNeoHookeanOracleSpec<MaterialPsdOracle::Correct>>();
      break;
    case MaterialPsdOracle::Conservative:
      RunMaterialTests<SmithNeoHookeanOracleSpec<MaterialPsdOracle::Conservative>>();
      break;
    case MaterialPsdOracle::Count:
      MOCHI_ASSERT(false, "Unexpected PSD oracle.");
      break;
  }
}

static constexpr real kGoldenDataDenominatorEpsilon = 1e-8_r;
static constexpr real kGoldenDataRelTol = 1e-4_r;

static void ExpectGoldenDataTensorNear(
    Tensor3x3x3x3r const& expected,
    Tensor3x3x3x3r const& actual) {
  auto const& expectedFlat = reinterpret_cast<NdArray<real, 81> const&>(expected);
  auto const& actualFlat = reinterpret_cast<NdArray<real, 81> const&>(actual);
  EXPECT_NEAR(
      0_r,
      Norm(expectedFlat - actualFlat),
      kGoldenDataRelTol * (Norm(expectedFlat) + kGoldenDataDenominatorEpsilon));
}

template <int kBS, typename GoldenData>
static void ExpectGoldenDataMatch(
    GoldenData const& expected,
    BatchDouble<kBS> const& energy,
    BatchReal3x3<kBS> const& pk1,
    NdArray<BatchReal3x3<kBS>, 3, 3> const& tangent,
    int lane) {
  EXPECT_NEAR(
      Abs(expected.strainEnergy - energy[lane]) /
          (Abs(expected.strainEnergy) + kGoldenDataDenominatorEpsilon),
      0_r,
      kGoldenDataRelTol);
  auto const pk1Lane = GetPk1Lane(pk1, lane);
  EXPECT_NEAR(
      Norm3x3(ToSimdMatrix(pk1Lane - expected.piolaKirchhoffStress)) /
          (Norm3x3(ToSimdMatrix(expected.piolaKirchhoffStress)) + kGoldenDataDenominatorEpsilon),
      0_r,
      kGoldenDataRelTol);
  auto const tangentLane = GetTangentLane(tangent, lane);
  ExpectGoldenDataTensorNear(expected.tangent, tangentLane);
}

template <int kBS, typename EvalFn>
[[nodiscard]] static BatchedMaterialOutputs<kBS> EvalBatchedMaterialFull(EvalFn eval) {
  BatchDouble<kBS> energy MOCHI_NO_INIT;
  BatchReal3x3<kBS> pk1 MOCHI_NO_INIT;
  NdArray<BatchReal3x3<kBS>, 3, 3> tangent MOCHI_NO_INIT;
  eval(&energy, &pk1, &tangent);
  return {.energy = energy, .pk1 = pk1, .tangent = tangent};
}

template <int kBS>
struct ActiveNeoHookeanComponentOutputs {
  BatchedMaterialOutputs<kBS> composite;
  BatchedMaterialOutputs<kBS> smith;
  BatchedMaterialOutputs<kBS> aniso;
};

template <int kBS>
[[nodiscard]] static ActiveNeoHookeanComponentOutputs<kBS> EvalActiveNeoHookeanComponents(
    ActiveNeoHookeanMaterialParams const& params,
    BatchReal3x3<kBS> const& batchedF,
    bool projectPsd) {
  return {
      .composite = EvalBatchedMaterialFull<kBS>([&](auto* energy, auto* pk1, auto* tangent) {
        BatchedActiveNeoHookeanConstitutiveResponse<kBS>(
            BuildBatchParams<kBS>(params), batchedF, energy, pk1, tangent, projectPsd);
      }),
      .smith = EvalBatchedMaterialFull<kBS>([&](auto* energy, auto* pk1, auto* tangent) {
        BatchedSmithNeoHookeanConstitutiveResponse<kBS>(
            BuildBatchParams<kBS>(params.passiveIsotropic),
            batchedF,
            energy,
            pk1,
            tangent,
            projectPsd);
      }),
      .aniso = EvalBatchedMaterialFull<kBS>([&](auto* energy, auto* pk1, auto* tangent) {
        BatchedActiveAnisoArapConstitutiveResponse<kBS>(
            BuildBatchParams<kBS>(params.activeAnisotropic),
            batchedF,
            energy,
            pk1,
            tangent,
            projectPsd);
      }),
  };
}

template <int kBS>
static void VerifyStVenantKirchhoffGoldenData() {
  // The golden data compares against independently-generated first Piola-Kirchhoff stress and
  // tangent values for randomly generated deformation gradients.
  auto const testData = mochi::materials::test::st_venant_kirchhoff_test_data::kTestData;
  StVenantKirchhoffMaterialParams params;
  params.youngsModulus = 1.0_r;
  params.poissonRatio = 0.4_r;

  for (bool projectPsd : {false, true}) {
    for (auto psdStrategy :
         {MaterialPsdStrategy::None,
          MaterialPsdStrategy::Projection,
          MaterialPsdStrategy::AbsEigenProjection}) {
      params.psdStrategy = psdStrategy;
      auto const batchedParams = BuildBatchParams<kBS>(params);

      for (int start = 0; start < isize(testData); start += kBS) {
        std::array<Matrix3x3r, kBS> Fs{};
        for (int lane = 0; lane < kBS; ++lane) {
          Fs[lane] = testData[(start + lane) % isize(testData)].deformationGradient;
        }
        auto const batchedF = mochi::test::LoadBatchMatrix3x3<kBS>(Fs);
        BatchDouble<kBS> energy MOCHI_NO_INIT;
        BatchReal3x3<kBS> pk1 MOCHI_NO_INIT;
        NdArray<BatchReal3x3<kBS>, 3, 3> tangent MOCHI_NO_INIT;
        BatchedStVenantKirchhoffConstitutiveResponse<kBS>(
            batchedParams, batchedF, &energy, &pk1, &tangent, projectPsd);

        for (int lane = 0; lane < kBS; ++lane) {
          auto const& ref = testData[(start + lane) % isize(testData)];
          ExpectGoldenDataMatch<kBS>(ref, energy, pk1, tangent, lane);
        }
      }
    }
  }
}

template <int kBS>
static void VerifySmithNeoHookeanGoldenData(MaterialPsdOracle oracle) {
  // The golden data compares against independently-generated first Piola-Kirchhoff stress and
  // tangent values for randomly generated deformation gradients.
  auto const testData = mochi::materials::test::smith_neo_hookean_test_data::kTestData;
  SmithNeoHookeanMaterialParams params;
  params.youngsModulus = 1.0_r;
  params.poissonRatio = 0.4_r;
  params.psdStrategy = MaterialPsdStrategy::None;
  auto const batchedParams = BuildBatchParams<kBS>(params);

  for (int start = 0; start < isize(testData); start += kBS) {
    std::array<Matrix3x3r, kBS> Fs{};
    for (int lane = 0; lane < kBS; ++lane) {
      Fs[lane] = testData[(start + lane) % isize(testData)].deformationGradient;
    }
    auto const batchedF = mochi::test::LoadBatchMatrix3x3<kBS>(Fs);
    BatchDouble<kBS> energy MOCHI_NO_INIT;
    BatchReal3x3<kBS> pk1 MOCHI_NO_INIT;
    NdArray<BatchReal3x3<kBS>, 3, 3> tangent MOCHI_NO_INIT;
    BatchedSmithNeoHookeanConstitutiveResponse<kBS>(
        batchedParams, batchedF, &energy, &pk1, &tangent, false, oracle);

    for (int lane = 0; lane < kBS; ++lane) {
      auto const& ref = testData[(start + lane) % isize(testData)];
      ExpectGoldenDataMatch<kBS>(ref, energy, pk1, tangent, lane);
    }
  }
}

// Lanes F = U * diag(sigma) * V^T on either side of a PSD threshold, at relative margins of 512 eps
// to sqrt(eps), scaled like rounding errors, which grow with jmaCondition:
//   - twist and flip, for poissonRatio > -0.9: lambdaHat^2 * (J - alpha)^2 * max_i(sigma_i^2)
//     against muHatK^2, with margins scaled by jmaCondition over its value 17/3 at
//     poissonRatio = 0.3, and the two largest singular values nearly equal, where values-only
//     singular values lose up to half their digits;
//   - scaling block, with twist and flip positive: its smallest eigenvalue over muHatK against
//     zero, with margins scaled by max(1, jmaCondition / 16), since near incompressibility that
//     ratio only reaches about -0.03; expansions for poissonRatio > -0.9, otherwise compressions
//     below J = 2/5 * alpha.
// About a quarter of the lanes are at or near rest, and none may count as indefinite: F = I or a
// rotation, on the twist and flip threshold, or, in odd batches, F = s * I instead of I, with s
// placing its margin at 12 eps * jmaCondition, 3/4 of the largest margin Correct exempts, unless
// Correct's cap binds, where s = 1. Fast drops the d^2J term in exactly the lanes the oracle counts
// as indefinite.
template <int kBS>
static void VerifySmithNeoHookeanCorrectOracleNearPsdBoundary(
    real youngsModulus,
    real poissonRatio) {
  SmithNeoHookeanMaterialParams params;
  params.youngsModulus = youngsModulus;
  params.poissonRatio = poissonRatio;
  params.psdStrategy = MaterialPsdStrategy::Fast;
  auto const batchedParams = BuildBatchParams<kBS>(params);
  auto const [lambda, mu] = utils::ComputeLameConstants(params.youngsModulus, params.poissonRatio);
  double const muHat = 4.0 / 3.0 * mu;
  double const lambdaHat = lambda + 5.0 / 6.0 * mu;
  double const alpha = 1.0 + 0.75 * muHat / lambdaHat;
  double const eps = std::numeric_limits<real>::epsilon();
  double const jmaCondition = (alpha + 1.0) / (alpha - 1.0);
  bool const expansion = poissonRatio > -0.9_r;
  // F = s * I has the margin (1 - s) * (6 / (alpha - 1) - 1) to first order.
  double const restMargin = 16.0 * jmaCondition < 8192.0 ? 12.0 * eps * jmaCondition : 0.0;
  real const restScale = static_cast<real>(1.0 - restMargin / (6.0 / (alpha - 1.0) - 1.0));

  // The smallest twist and flip eigenvalue and the smallest scaling block eigenvalue, over muHatK,
  // at sigma = t * (1, 1, beta) with t > 0 and 0 < beta <= 1. The scaling block has the eigenvector
  // (1, -1, 0); its other eigenvalues are those of its restriction to (1, 1, 0) / sqrt(2) and
  // (0, 0, 1).
  struct MinEigenvalues {
    double twistFlip;
    double scaling;
  };
  auto const minEigenvalues = [&](double t, double beta) {
    double const t3 = t * beta;
    double const J = t * t * t3;
    double const Ic = 2.0 * t * t + t3 * t3;
    double const muHatK = muHat * Ic / (Ic + 1.0);
    double const c3 = 2.0 * muHat / Sqr(Ic + 1.0);
    double const J2alphaLambda = (2.0 * J - alpha) * lambdaHat;
    double const a11 = muHatK + c3 * t * t + lambdaHat * Sqr(t * t3);
    double const a12 = c3 * t * t + J2alphaLambda * t3;
    double const a13 = c3 * t * t3 + J2alphaLambda * t;
    double const a33 = muHatK + c3 * t3 * t3 + lambdaHat * Sqr(t * t);
    double const p = a11 + a12;
    return MinEigenvalues{
        .twistFlip = 1.0 - lambdaHat * Abs(J - alpha) * t / muHatK,
        .scaling = Min(a11 - a12, 0.5 * (p + a33) - std::hypot(0.5 * (p - a33), Sqrt(2.0) * a13)) /
            muHatK};
  };

  auto generator = RandomGenerator(42);
  for (int batch = 0; batch < 64; ++batch) {
    std::array<Matrix3x3r, kBS> Fs{};
    std::array<bool, kBS> indefinite{};
    for (int lane = 0; lane < kBS; ++lane) {
      if (RandomUniformValue(generator, 0, 3) == 0) {
        Fs[lane] = RandomUniformValue(generator, 0, 1) == 0
            ? (batch % 2 == 0 ? 1_r : restScale) * Eye<3, real>()
            : ToNdArray3x3(ToSimdMatrix(GetRandomRotationMatrix(generator)));
        continue;
      }
      indefinite[lane] = RandomUniformValue(generator, 0, 1) == 1;
      double const margin =
          512.0 * eps * Pow(1.0 / (512.0 * Sqrt(eps)), RandomUniformValue(generator, 0.0, 1.0));
      std::array<double, 3> sigma{};
      if (expansion && (batch * kBS + lane) % 2 == 0) {
        // Bisect t for the target: twistFlipMargin tends to +inf as t -> 0 and is -1 at J = alpha.
        double const eta = Pow(10.0, RandomUniformValue(generator, -7.0, -2.0));
        double const beta = RandomUniformValue(generator, 0.3, 0.7);
        double const target = (indefinite[lane] ? margin : -margin) * 3.0 / 17.0 * jmaCondition;
        auto const twistFlipMargin = [&](double t) {
          double const J = t * t * t * (1.0 - eta) * beta;
          double const Ic = t * t * (1.0 + (1.0 - eta) * (1.0 - eta) + beta * beta);
          return Sqr(lambdaHat * (J - alpha) * t * (Ic + 1.0) / (muHat * Ic)) - 1.0;
        };

        double hi = std::cbrt(alpha / ((1.0 - eta) * beta));
        double lo = 1e-3 * hi;
        for (int i = 0; i < 100; ++i) {
          double const mid = 0.5 * (lo + hi);
          (twistFlipMargin(mid) > target ? lo : hi) = mid;
        }
        sigma = {lo, lo * (1.0 - eta), lo * beta};
      } else {
        // Walk from J = alpha or 2/5 * alpha, where the scaling block is positive definite, to the
        // first crossing of the target with twist and flip above 1e-3 * muHatK, clear of their
        // threshold so that the lane tests the scaling block alone, then bisect. Expansions cross
        // within a relative distance of about (alpha - 1) / alpha of J = alpha, hence the step.
        double const target = (indefinite[lane] ? -margin : margin) * Max(1.0, jmaCondition / 16.0);
        double const step = 1.0 + 1e-3 * (alpha - 1.0) / alpha;
        bool found = false;

        for (int attempt = 0; attempt < 100 && !found; ++attempt) {
          double const beta = expansion ? RandomUniformValue(generator, 0.6, 0.95)
                                        : RandomUniformValue(generator, 0.985, 1.0);
          double t0 = std::cbrt((expansion ? alpha : 0.4 * alpha) / beta);
          auto e0 = minEigenvalues(t0, beta);
          for (int i = 0; i < 1000 && !found; ++i) {
            double const t1 = expansion ? t0 * step : t0 / step;
            auto const e1 = minEigenvalues(t1, beta);
            if (e0.twistFlip > 1e-3 && e1.twistFlip > 1e-3 &&
                (e0.scaling > target) != (e1.scaling > target)) {
              double lo = t0;
              double hi = t1;
              for (int k = 0; k < 100; ++k) {
                double const mid = 0.5 * (lo + hi);
                double const scaling = minEigenvalues(mid, beta).scaling;
                ((scaling > target) == (e0.scaling > target) ? lo : hi) = mid;
              }
              sigma = {lo, lo, lo * beta};
              found = true;
            }
            t0 = t1;
            e0 = e1;
          }
        }
        ASSERT_TRUE(found);
      }

      auto const U = GetRandomRotationMatrix(generator);
      auto const V = GetRandomRotationMatrix(generator);
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          double entry = 0.0;
          for (int k = 0; k < 3; ++k) {
            entry += static_cast<double>(U(i, k)) * sigma[k] * static_cast<double>(V(j, k));
          }
          Fs[lane][i][j] = static_cast<real>(entry);
        }
      }
    }
    auto const batchedF = mochi::test::LoadBatchMatrix3x3<kBS>(Fs);
    NdArray<BatchReal3x3<kBS>, 3, 3> correct MOCHI_NO_INIT;
    NdArray<BatchReal3x3<kBS>, 3, 3> dropped MOCHI_NO_INIT;
    NdArray<BatchReal3x3<kBS>, 3, 3> kept MOCHI_NO_INIT;
    BatchedSmithNeoHookeanConstitutiveResponse<kBS>(
        batchedParams, batchedF, nullptr, nullptr, &correct, true, MaterialPsdOracle::Correct);
    BatchedSmithNeoHookeanConstitutiveResponse<kBS>(
        batchedParams, batchedF, nullptr, nullptr, &dropped, true, MaterialPsdOracle::None);
    BatchedSmithNeoHookeanConstitutiveResponse<kBS>(
        batchedParams, batchedF, nullptr, nullptr, &kept, false);
    for (int lane = 0; lane < kBS; ++lane) {
      // The alternatives differ only by the d^2J term, which can be much smaller than the tangent.
      auto const flat = [lane](NdArray<BatchReal3x3<kBS>, 3, 3> const& tangent) {
        Tensor3x3x3x3r const laneTangent = GetTangentLane(tangent, lane);
        return reinterpret_cast<NdArray<real, 81> const&>(laneTangent);
      };
      EXPECT_LE(
          Norm(flat(correct) - flat(indefinite[lane] ? dropped : kept)),
          0.25_r * Norm(flat(dropped) - flat(kept)));
    }
  }
}

template <int kBS>
static void VerifyActiveNeoHookeanComposite() {
  for (int testIdx = 0; testIdx < kBatchedMaterialTestCases; ++testIdx) {
    auto const params = MakeRandomMaterialParams<ActiveNeoHookeanMaterialParams>(
        testIdx, MaterialPsdStrategy::None);

    std::array<Matrix3x3r, kBS> Fs = MakeBatchFs<kBS>(testIdx);
    auto const batchedF = mochi::test::LoadBatchMatrix3x3<kBS>(Fs);

    for (bool projectPsd : {false, true}) {
      auto const outputs = EvalActiveNeoHookeanComponents<kBS>(params, batchedF, projectPsd);

      for (int lane = 0; lane < kBS; ++lane) {
        ExpectEnergyNear(
            outputs.smith.energy[lane] + outputs.aniso.energy[lane],
            outputs.composite.energy[lane]);
        ExpectMatrixNear(
            GetPk1Lane(outputs.smith.pk1, lane) + GetPk1Lane(outputs.aniso.pk1, lane),
            GetPk1Lane(outputs.composite.pk1, lane));
        ExpectTensorNear(
            GetTangentLane(outputs.smith.tangent, lane) +
                GetTangentLane(outputs.aniso.tangent, lane),
            GetTangentLane(outputs.composite.tangent, lane));
      }
    }
  }
}

static void VerifyActiveNeoHookeanDefaultParamsComposite() {
  constexpr int kBS = 1;
  ActiveNeoHookeanMaterialParams params;
  EXPECT_GT(params.passiveIsotropic.youngsModulus, 0_r);
  EXPECT_GT(params.activeAnisotropic.alpha, 0_r);
  EXPECT_GT(params.activeAnisotropic.length, 0_r);

  for (int testIdx = 0; testIdx < kBatchedMaterialTestCases; ++testIdx) {
    std::array<Matrix3x3r, kBS> Fs{
        ToNdArray3x3(ToSimdMatrix(Matrix<real, 3, 3>::Random(testIdx, -1_r, 1_r)))};
    auto const batchedF = mochi::test::LoadBatchMatrix3x3<kBS>(Fs);

    for (bool projectPsd : {false, true}) {
      auto const outputs = EvalActiveNeoHookeanComponents<kBS>(params, batchedF, projectPsd);

      ExpectEnergyNear(
          outputs.smith.energy[0] + outputs.aniso.energy[0], outputs.composite.energy[0]);
      ExpectMatrixNear(
          GetPk1Lane(outputs.smith.pk1, 0) + GetPk1Lane(outputs.aniso.pk1, 0),
          GetPk1Lane(outputs.composite.pk1, 0));
      ExpectTensorNear(
          GetTangentLane(outputs.smith.tangent, 0) + GetTangentLane(outputs.aniso.tangent, 0),
          GetTangentLane(outputs.composite.tangent, 0));
    }
  }
}

// Runs generic batched checks for the linear elastic material.
TEST(BatchedMaterials, LinearElastic) {
  RunMaterialTests<LinearElasticSpec>();
}

// Runs generic batched checks for St. Venant-Kirchhoff, including PSD projections.
TEST(BatchedMaterials, StVenantKirchhoff) {
  RunMaterialTests<StVenantKirchhoffSpec>();
}

// Runs generic Smith neo-Hookean checks for the default oracle and each explicit oracle.
TEST(BatchedMaterials, SmithNeoHookean) {
  RunMaterialTests<SmithNeoHookeanSpec>();
  for (int i = 0; i < static_cast<int>(MaterialPsdOracle::Count); ++i) {
    RunSmithNeoHookeanOracleMaterialTests(static_cast<MaterialPsdOracle>(i));
  }
}

// At the largest Poisson's ratio below 0.5, F = nextafter(1, 0) * I compresses enough to make the
// twist eigenvalues a few times mu below zero: the oracle must count every lane as indefinite, so
// Fast drops the d^2J term.
template <int kBS>
static void VerifySmithNeoHookeanCorrectOracleNearIncompressible() {
  SmithNeoHookeanMaterialParams params;
  params.youngsModulus = 1_r;
  params.poissonRatio = std::nextafter(0.5_r, 0_r);
  params.psdStrategy = MaterialPsdStrategy::Fast;
  auto const batchedParams = BuildBatchParams<kBS>(params);
  std::array<Matrix3x3r, kBS> Fs{};
  Fs.fill(std::nextafter(1_r, 0_r) * Eye<3, real>());
  auto const batchedF = mochi::test::LoadBatchMatrix3x3<kBS>(Fs);
  NdArray<BatchReal3x3<kBS>, 3, 3> correct MOCHI_NO_INIT;
  NdArray<BatchReal3x3<kBS>, 3, 3> dropped MOCHI_NO_INIT;
  BatchedSmithNeoHookeanConstitutiveResponse<kBS>(
      batchedParams, batchedF, nullptr, nullptr, &correct, true, MaterialPsdOracle::Correct);
  BatchedSmithNeoHookeanConstitutiveResponse<kBS>(
      batchedParams, batchedF, nullptr, nullptr, &dropped, true, MaterialPsdOracle::None);
  for (int lane = 0; lane < kBS; ++lane) {
    EXPECT_EQ(GetTangentLane(dropped, lane), GetTangentLane(correct, lane));
  }
}

// Checks the Smith neo-Hookean Correct oracle's decisions at rest and near the PSD boundary, for
// strongly auxetic to nearly incompressible materials whose squared Young's moduli are the smallest
// normal number or whose fourth powers overflow float, and slightly compressed at the largest
// Poisson's ratio below 0.5.
TEST(BatchedMaterials, SmithNeoHookeanCorrectOracleDecisions) {
  for (real const youngsModulus : {Sqrt(std::numeric_limits<real>::min()), 1e11_r}) {
    for (real const poissonRatio : {-0.95_r, -0.5_r, 0_r, 0.3_r, 0.45_r, 0.49_r, 0.499_r}) {
      VerifySmithNeoHookeanCorrectOracleNearPsdBoundary<1>(youngsModulus, poissonRatio);
      VerifySmithNeoHookeanCorrectOracleNearPsdBoundary<4>(youngsModulus, poissonRatio);
      VerifySmithNeoHookeanCorrectOracleNearPsdBoundary<8>(youngsModulus, poissonRatio);
    }
  }
  VerifySmithNeoHookeanCorrectOracleNearIncompressible<1>();
  VerifySmithNeoHookeanCorrectOracleNearIncompressible<4>();
  VerifySmithNeoHookeanCorrectOracleNearIncompressible<8>();
}

// Runs generic batched checks for Kim neo-Hookean, including rest, rigid-motion, and linear-limit
// invariants.
TEST(BatchedMaterials, KimNeoHookean) {
  RunMaterialTests<KimNeoHookeanSpec>();
}

// Runs generic batched checks for ARAP, including rigid-motion and minimum-energy invariants.
TEST(BatchedMaterials, Arap) {
  RunMaterialTests<ArapSpec>();
}

// Runs generic batched checks for active anisotropic ARAP, including rest, rigid-motion, and
// minimum-energy invariants.
TEST(BatchedMaterials, ActiveAnisoArap) {
  RunMaterialTests<ActiveAnisoArapSpec>();
}

// Runs generic batched checks for active shape-targeting ARAP, including matching-target states.
TEST(BatchedMaterials, ActiveShapeTargetingArap) {
  RunMaterialTests<ActiveShapeTargetingArapSpec>();
}

// Runs generic batched checks for active neo-Hookean, including rest, rigid-motion, and
// minimum-energy invariants.
TEST(BatchedMaterials, ActiveNeoHookean) {
  RunMaterialTests<ActiveNeoHookeanSpec>();
}

// Compares St. Venant-Kirchhoff batched outputs against golden data.
TEST(BatchedMaterials, StVenantKirchhoffGoldenData) {
  VerifyStVenantKirchhoffGoldenData<1>();
  VerifyStVenantKirchhoffGoldenData<4>();
  VerifyStVenantKirchhoffGoldenData<8>();
}

// Compares Smith neo-Hookean batched outputs against golden data for every oracle.
TEST(BatchedMaterials, SmithNeoHookeanGoldenData) {
  for (int i = 0; i < static_cast<int>(MaterialPsdOracle::Count); ++i) {
    auto const oracle = static_cast<MaterialPsdOracle>(i);
    VerifySmithNeoHookeanGoldenData<1>(oracle);
    VerifySmithNeoHookeanGoldenData<4>(oracle);
    VerifySmithNeoHookeanGoldenData<8>(oracle);
  }
}

// Verifies active neo-Hookean equals the sum of its Smith and active anisotropic ARAP parts.
TEST(BatchedMaterials, ActiveNeoHookeanComposite) {
  VerifyActiveNeoHookeanDefaultParamsComposite();
  VerifyActiveNeoHookeanComposite<1>();
  VerifyActiveNeoHookeanComposite<4>();
  VerifyActiveNeoHookeanComposite<8>();
}

// Verifies an active anisotropic ARAP degenerate lane returns zero while other lanes remain valid.
TEST(BatchedMaterials, ActiveAnisoArapDegenerateLane) {
  constexpr int kBS = 4;

  auto params =
      MakeRandomMaterialParams<ActiveAnisoArapMaterialParams>(0, MaterialPsdStrategy::Projection);
  // Use axis-aligned fiber direction to guarantee F * a = 0 exactly in floating point.
  params.anisoDir = {1_r, 0_r, 0_r};

  // F with zero first-column entries gives F * (1, 0, 0) = (0, 0, 0) exactly.
  Matrix3x3r const Fdeg = {Real3{0_r, 1_r, 0_r}, Real3{0_r, 0_r, 1_r}, Real3{0_r, 1_r, 1_r}};
  Matrix3x3r const Fid = Eye<3, real>();
  std::array<Matrix3x3r, kBS> Fs = {Fdeg, Fid, Fid, Fid};
  auto const batchedF = mochi::test::LoadBatchMatrix3x3<kBS>(Fs);

  for (bool projectPsd : {false, true}) {
    BatchDouble<kBS> energy MOCHI_NO_INIT;
    BatchReal3x3<kBS> pk1 MOCHI_NO_INIT;
    NdArray<BatchReal3x3<kBS>, 3, 3> tangent MOCHI_NO_INIT;
    auto const batchParams = BuildBatchParams<kBS>(params);
    BatchedActiveAnisoArapConstitutiveResponse<kBS>(
        batchParams, batchedF, &energy, &pk1, &tangent, projectPsd);

    std::array<Matrix3x3r, kBS> identityFs = {Fid, Fid, Fid, Fid};
    auto const identityF = mochi::test::LoadBatchMatrix3x3<kBS>(identityFs);
    BatchDouble<kBS> identityEnergy MOCHI_NO_INIT;
    BatchReal3x3<kBS> identityPk1 MOCHI_NO_INIT;
    NdArray<BatchReal3x3<kBS>, 3, 3> identityTangent MOCHI_NO_INIT;
    BatchedActiveAnisoArapConstitutiveResponse<kBS>(
        batchParams, identityF, &identityEnergy, &identityPk1, &identityTangent, projectPsd);

    EXPECT_EQ(0.0, energy[0]);
    Matrix3x3r const zero{};
    EXPECT_EQ(zero, GetPk1Lane(pk1, 0));
    EXPECT_EQ(Tensor3x3x3x3r{}, GetTangentLane(tangent, 0));
    for (int lane = 1; lane < kBS; ++lane) {
      ExpectEnergyNear(identityEnergy[lane], energy[lane]);
      ExpectMatrixNear(GetPk1Lane(identityPk1, lane), GetPk1Lane(pk1, lane));
      ExpectTensorNear(GetTangentLane(identityTangent, lane), GetTangentLane(tangent, lane));
    }
  }
}

// Verifies per-element Lame parameters built from E/nu arrays match direct conversion.
TEST(BatchedMaterials, BuildPerElementLameParamsFromArrays) {
  // Multi-element heterogeneous case: distinct E and nu per element, including negative nu.
  DynamicArray<real> E = {1e3_r, 1e4_r, 5e5_r, 2e6_r};
  DynamicArray<real> nu = {-0.4_r, 0_r, 0.25_r, 0.45_r};
  auto const peDefault = BuildPerElementLameParams(MakeConstSpan(E), MakeConstSpan(nu));
  ASSERT_EQ(isize(E), isize(peDefault.lambda));
  ASSERT_EQ(isize(E), isize(peDefault.mu));
  EXPECT_EQ(MaterialPsdStrategy::MaterialDefault, peDefault.psdStrategy);
  for (int i = 0; i < isize(E); ++i) {
    auto const [lambda, mu] = utils::ComputeLameConstants(E[i], nu[i]);
    EXPECT_NEAR_RTOL(lambda, peDefault.lambda[i], kBatchedMaterialRelTol);
    EXPECT_NEAR_RTOL(mu, peDefault.mu[i], kBatchedMaterialRelTol);
  }

  // Custom PSD strategy is propagated.
  auto const pePsd = BuildPerElementLameParams(
      MakeConstSpan(E), MakeConstSpan(nu), MaterialPsdStrategy::Projection);
  EXPECT_EQ(MaterialPsdStrategy::Projection, pePsd.psdStrategy);

  // Single-element case.
  DynamicArray<real> Eone = {1e5_r};
  DynamicArray<real> nuOne = {0.3_r};
  auto const peOne = BuildPerElementLameParams(MakeConstSpan(Eone), MakeConstSpan(nuOne));
  EXPECT_EQ(1, isize(peOne.lambda));
  EXPECT_EQ(1, isize(peOne.mu));
  auto const [lambdaOne, muOne] = utils::ComputeLameConstants(Eone[0], nuOne[0]);
  EXPECT_NEAR_RTOL(lambdaOne, peOne.lambda[0], kBatchedMaterialRelTol);
  EXPECT_NEAR_RTOL(muOne, peOne.mu[0], kBatchedMaterialRelTol);
}

// Verifies heterogeneous per-element gather honors permuted and repeated element indices.
TEST(BatchedMaterials, PerElementGatherUsesPermutedAndRepeatedIndices) {
  constexpr int kBS = 4;
  VerifyBatchedMatchesSingleLaneBatched<
      KimNeoHookeanSpec,
      kBS,
      BatchedParamLayout::PerElementHeterogeneous>(MakePermutedElementIndices<kBS>());
  VerifyBatchedMatchesSingleLaneBatched<
      KimNeoHookeanSpec,
      kBS,
      BatchedParamLayout::PerElementHeterogeneous>(MakeRepeatedElementIndices<kBS>());
}
