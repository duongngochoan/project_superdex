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

#include <mochi_core/materials/batched_smith_neo_hookean.h>

#include <mochi_core/materials/material_params_utils.h>
#include <mochi_core/materials/material_utils.h>
#include <mochi_core/utils/batch_types.h>
#include <mochi_core/utils/decomposition_utils.h>
#include <mochi_core/utils/matrix_utils.h>
#include <mochi_core/utils/simd.h>

#include <limits>

namespace mochi::materials {

// Lanes where shift·I − weight·G has a negative eigenvalue, for G = FᵀF, i.e. weight·max σᵢ² >
// shift, decided to rounding accuracy. A symmetric matrix M is positive semidefinite iff its
// largest diagonal entry d is positive and the Schur complement of d is positive semidefinite, or
// M = 0, which counts as indefinite here. Eliminating the largest diagonal entry first, as pivoted
// Cholesky does, keeps the decision accurate when M has two eigenvalues near zero, as when the two
// largest singular values of F are close, where det(M) cancels to rounding noise.
template <int kBatchSize>
[[nodiscard]] static MOCHI_FORCE_INLINE BatchReal<kBatchSize> IsShiftedGramIndefinite(
    BatchReal<kBatchSize> const& shift,
    BatchReal<kBatchSize> const& weight,
    BatchSymMatrix3x3<kBatchSize> const& G) {
  using V = BatchReal<kBatchSize>;
  V const m00 = shift - weight * G[0];
  V const m11 = shift - weight * G[1];
  V const m22 = shift - weight * G[2];
  V const m01 = -weight * G[3];
  V const m02 = -weight * G[4];
  V const m12 = -weight * G[5];

  // Order the indices as (p, i, j) = (0, 1, 2), (1, 0, 2) or (2, 0, 1), with d = m_pp the largest.
  auto const p1 = m11 > m00;
  V const d01 = Select(p1, m11, m00);
  auto const p2 = m22 > d01;
  auto const p12 = p1 | p2;
  V const d = Select(p2, m22, d01);
  V const mpi = Select(p2, m02, m01);
  V const mpj = Select(p12, m12, m02);
  V const mii = Select(p12, m00, m11);
  V const mjj = Select(p2, m11, m22);
  V const mij = Select(p2, m01, Select(p1, m02, m12));

  // The Schur complement of d, divided by d, which bounds its entries by 1 when M is positive
  // semidefinite, whatever M's scale.
  V const zero = V{0_r};
  V const one = V{1_r};
  auto const positiveD = d > zero;
  V const invD = one / Select(positiveD, d, one);
  V const ri = mpi * invD;
  V const rj = mpj * invD;
  V const sii = mii * invD - ri * ri;
  V const sjj = mjj * invD - rj * rj;
  V const sij = mij * invD - ri * rj;
  return ~(positiveD & (sii >= zero) & (sjj >= zero) & (sii * sjj >= sij * sij));
}

// Lanes where the Smith tangent's scaling block A (Asym below) may be indefinite, among those with
// J > alpha and nonnegative twist and flip eigenvalues or with 0 <= J < 2/5 * alpha. With
// s = (sigma_i^2) and 1 = (1, 1, 1),
//
//   A' = diag(sigma) * A * diag(sigma) = diag(a * s_i + c) + kappa * 1 * 1^T + c3 * s * s^T,
//
// with a = muHatK, c = lambdaHat * (alpha - J) * J, kappa = lambdaHat * J * (2 * J - alpha) and
// c3 = 2 * muHat / (Ic + 1)^2, all passed divided by 4/3 * lambdaHat. For J > 0, A' is congruent
// to A, so A is positive definite iff A' is. For 2/5 * alpha <= J <= alpha, A' is positive
// definite: diag(a * s_i) is positive definite, c3 * s * s^T is positive semidefinite, and so is
// c * I + kappa * 1 * 1^T, whose eigenvalues are c and
// c + 3 * kappa = lambdaHat * J * (5 * J - 2 * alpha). In the tested lanes, A' has at most one
// negative eigenvalue: for J > alpha, nonnegative twist and flip keep a * s_i + c >= 0 for the two
// largest sigma_i, and the other terms are positive semidefinite; for J < 2/5 * alpha, only the
// rank-one kappa * 1 * 1^T can have a negative eigenvalue. So A' is positive definite iff
// det(A') > 0, a symmetric polynomial in s_i that follows from the invariants Ic,
// IIc = sum_{i<j} s_i * s_j and J^2 of F^T * F, without an SVD. Within the twist and flip test's
// tolerance, A' can have two slightly negative eigenvalues that det(A') misses. Rounding decides
// near-isotropic expansions whose smallest scaling eigenvalue is within a few sqrt(eps) * muHatK of
// zero. det(A') = 0, as for J = 0, counts as indefinite.
template <int kBatchSize>
[[nodiscard]] static MOCHI_FORCE_INLINE BatchReal<kBatchSize> IsScalingBlockIndefinite(
    BatchReal<kBatchSize> const& a,
    BatchReal<kBatchSize> const& c,
    BatchReal<kBatchSize> const& kappa,
    BatchReal<kBatchSize> const& c3,
    BatchReal<kBatchSize> const& Ic,
    BatchReal<kBatchSize> const& IIc,
    BatchReal<kBatchSize> const& Jsq) {
  using V = BatchReal<kBatchSize>;

  // With d_i = a * s_i + c and {i, j, k} = {0, 1, 2}, the matrix determinant lemma gives
  //   det(A') = e3 + kappa * e2 + c3 * sum_i s_i^2 * d_j * d_k
  //             + kappa * c3 * sum_k (s_i - s_j)^2 * d_k
  // for the elementary symmetric functions e2 and e3 of d_i. Below, p2 = sum_i s_i^2,
  // q = sum_{i != j} s_i^2 * s_j, r = sum_k (s_i - s_j)^2 / 2 and u = sum_k (s_i - s_j)^2 * s_k.
  V const p2 = Ic * Ic - V{2_r} * IIc;
  V const q = Ic * IIc - V{3_r} * Jsq;
  V const r = Ic * Ic - V{3_r} * IIc;
  V const u = Ic * IIc - V{9_r} * Jsq;

  V const e2 = a * a * IIc + V{2_r} * a * c * Ic + V{3_r} * c * c;
  V const e3 = a * a * a * Jsq + a * a * c * IIc + a * c * c * Ic + c * c * c;

  V const det = e3 + kappa * e2 + c3 * (a * a * Jsq * Ic + a * c * q + c * c * p2) +
      kappa * c3 * (a * u + V{2_r} * c * r);
  return ~(det > V{0_r});
}

template <int kBatchSize>
void BatchedSmithNeoHookeanConstitutiveResponse(
    BatchLameParams<kBatchSize> const& params,
    BatchReal3x3<kBatchSize> const& F,
    BatchDouble<kBatchSize>* outEnergy,
    BatchReal3x3<kBatchSize>* outPK1,
    NdArray<BatchReal3x3<kBatchSize>, 3, 3>* outTangent,
    bool projectPsd,
    MaterialPsdOracle oracle) {
  using V = BatchReal<kBatchSize>;
  using Vd = BatchDouble<kBatchSize>;
  using V3 = BatchReal3<kBatchSize>;
  using V9 = BatchReal9<kBatchSize>;
  using V3x3 = BatchReal3x3<kBatchSize>;
  MOCHI_ASSERT_VERBOSE(
      IsFinite(params.mu) && AllTrue<kBatchSize>(params.mu > V{0_r}) && IsFinite(params.lambda),
      "Invalid Lame parameters.");
  MOCHI_ASSERT_VERBOSE(
      IsResolvedPsdStrategySupported<SmithNeoHookeanMaterialParams>(params.psdStrategy),
      "Batched material response requires a supported, resolved PSD strategy.");
  MaterialPsdStrategy const psdStrategy = params.psdStrategy;

  // Reparameterized Lamé constants (Sec. 3.4 of Smith et al. 2018).
  V const one = V{1_r};
  V const muHat = params.mu * V{4_r / 3_r};
  V const lambdaHat = params.lambda + params.mu * V{5_r / 6_r};
  V const alphaMinusOne = V{3_r / 4_r} * muHat / lambdaHat;
  V const alpha = one + alphaMinusOne;

  // Invariants.
  V const Ic = NormSqr(F);
  V const IcPlus1 = Ic + one;
  V const IcPlus1Inv = one / IcPlus1;
  V const J = Det(F);
  V const Jma = J - alpha;

  if (outEnergy) {
    *outEnergy = StaticCast<Vd>(
        V{0.5_r} * muHat * (Ic - V{3_r}) + V{0.5_r} * lambdaHat * Jma * Jma -
        V{0.5_r} * muHat * Ln(IcPlus1));
  }

  bool projectingPsd = projectPsd && (psdStrategy != MaterialPsdStrategy::None);
  V isIndefiniteMask = ~SimdZero<V>(); // All lanes indefinite unless oracle proves otherwise.

  if (outTangent && projectingPsd && oracle != MaterialPsdOracle::None) {
    // PSD oracle: evaluate indefiniteness per lane, skipping projection if no lane requires it. The
    // tangent's eigenvalues are those of the 3x3 scaling block and the twist and flip eigenvalues
    //
    //   e_twist_i =  lambdaHat * (J - alpha) * sigma_i + muHatK
    //   e_flip_i  = -lambdaHat * (J - alpha) * sigma_i + muHatK
    //
    // where muHatK = muHat * (1 - 1 / (Ic + 1)), Ic = sum_i sigma_i^2, and
    // J = prod_i sigma_i. Inverted elements (J < 0) are indefinite. For J >= 0, the sign cases for
    // (J - alpha) and sigma_i reduce the twist and flip condition to checking:
    //
    //   lambdaHat * abs(J - alpha) * abs(sigma_i) > muHatK.
    //
    // Squaring and taking the largest singular value gives the exact condition
    //
    //   lambdaHat^2 * (J - alpha)^2 * max_i(sigma_i^2) > muHatK^2,
    //
    // evaluated divided by (4/3 * lambdaHat)^2, which keeps its terms O(1) whatever the stiffness:
    // lhsBase * max_i(sigma_i^2) > rhs, with lhsBase = (3/4 * (J - alpha))^2 and rhs = a^2 for
    // a = (alpha - 1) * (1 - 1 / (Ic + 1)), since alpha - 1 = 3/4 * muHat / lambdaHat.
    //
    // max_i(sigma_i^2) is the largest eigenvalue of F^T * F, so Correct decides this without an
    // SVD: the condition holds iff rhs * I - lhsBase * F^T * F is not positive semidefinite.
    // Conservative uses Ic, an upper bound on max_i(sigma_i^2), so it can project more often than
    // necessary but does not miss required projections.
    //
    // At rest and at rotations, J = 1, Ic = 3 and max_i(sigma_i^2) = 1, so the two sides are equal
    // and rounding decides the comparison: (alpha + 1) / (alpha - 1), the condition number of
    // J - alpha, amplifies relative errors in J, such as those from rounding in F, and in alpha.
    // So that Correct counts such lanes as positive semidefinite, its rhs is scaled by 1 + delta,
    // with delta = min(16 * eps * (alpha + 1) / (alpha - 1), 8192 * eps). A lane this exempts can
    // keep a twist or flip eigenvalue negative by up to about delta / 2 * muHatK, which at rest is
    // at most eps * (16 * lambdaHat + 6 * muHat), under 6 eps times the tangent's Frobenius norm
    // there, and in any case at most 4096 * eps * muHatK. The cap binds for Poisson's ratios
    // above about 0.498; above about 0.499, the rounding error at rest and at rotations can exceed
    // delta, so rounding decides such lanes. Conservative flags rest states anyway, since Ic = 3
    // there, so it keeps the unscaled rhs.
    //
    // The scaling block can be indefinite while twist and flip are not, but only for J > alpha, as
    // in anisotropic expansions, or J < 2/5 * alpha, as in compressions of strongly auxetic
    // materials. Correct tests it there with IsScalingBlockIndefinite. Conservative needs no such
    // test: the lanes it skips have lambdaHat * abs(J - alpha) * sqrt(Ic) <= muHatK, which keeps
    // the scaling block positive definite for J > alpha and otherwise implies
    // J >= (alpha + 2) / 3, above 2/5 * alpha for every Poisson's ratio above -1, unless F = 0,
    // where the tangent is zero.
    V const a = alphaMinusOne * (one - IcPlus1Inv);
    V const lhsBase = Sqr(V{3_r / 4_r} * Jma);

    if (oracle == MaterialPsdOracle::Correct) {
      constexpr real kEps = std::numeric_limits<real>::epsilon();
      V const rhs = alphaMinusOne *
          (alphaMinusOne + Min(V{16_r * kEps} * (alpha + one), V{8192_r * kEps} * alphaMinusOne)) *
          Sqr(one - IcPlus1Inv);
      // max_i(sigma_i^2) lies between the largest diagonal entry of F^T * F and the smaller of its
      // trace and largest absolute row sum. Lanes these bounds leave open, and lanes with J > alpha
      // or J < 2/5 * alpha not proven indefinite, count as indefinite unless the exact tests settle
      // them. The Projection strategies project the whole batch once any lane is indefinite, so
      // they skip the tests once a lane is proven indefinite; Fast drops the term per lane, so it
      // always runs them.
      BatchSymMatrix3x3<kBatchSize> const G = {
          F[0][0] * F[0][0] + F[1][0] * F[1][0] + F[2][0] * F[2][0],
          F[0][1] * F[0][1] + F[1][1] * F[1][1] + F[2][1] * F[2][1],
          F[0][2] * F[0][2] + F[1][2] * F[1][2] + F[2][2] * F[2][2],
          F[0][0] * F[0][1] + F[1][0] * F[1][1] + F[2][0] * F[2][1],
          F[0][0] * F[0][2] + F[1][0] * F[1][2] + F[2][0] * F[2][2],
          F[0][1] * F[0][2] + F[1][1] * F[1][2] + F[2][1] * F[2][2]};
      V const lowerBound = Max(G[0], G[1], G[2]);
      V const upperBound =
          Min(Ic,
              Max(G[0] + Abs(G[3]) + Abs(G[4]),
                  G[1] + Abs(G[3]) + Abs(G[5]),
                  G[2] + Abs(G[4]) + Abs(G[5])));

      V const provenIndefinite = (lhsBase * lowerBound > rhs) | (J < V{0_r});
      V const twistFlipUndecided = ~provenIndefinite & (lhsBase * upperBound > rhs);
      V const scalingUndecided = ~provenIndefinite & ((J > alpha) | (V{2.5_r} * J < alpha));
      isIndefiniteMask = provenIndefinite | twistFlipUndecided | scalingUndecided;

      if (AnyTrue<kBatchSize>(twistFlipUndecided | scalingUndecided) &&
          (psdStrategy == MaterialPsdStrategy::Fast || !AnyTrue<kBatchSize>(provenIndefinite))) {
        isIndefiniteMask = provenIndefinite;
        if (AnyTrue<kBatchSize>(twistFlipUndecided)) {
          isIndefiniteMask |=
              twistFlipUndecided & IsShiftedGramIndefinite<kBatchSize>(rhs, lhsBase, G);
        }

        if (AnyTrue<kBatchSize>(scalingUndecided)) {
          // IIc from G cancels only for needle-like F, whose tangent is indefinite iff twist or
          // flip is negative, so the cancellation can only add flags to the twist and flip test's.
          // For such F, the rounding of J can already add flags.
          V const IIc =
              G[0] * G[1] + G[0] * G[2] + G[1] * G[2] - G[3] * G[3] - G[4] * G[4] - G[5] * G[5];
          V const scalingIndefinite = IsScalingBlockIndefinite<kBatchSize>(
              a,
              V{-3_r / 4_r} * Jma * J,
              V{3_r / 4_r} * J * (V{2_r} * J - alpha),
              V{2_r} * alphaMinusOne * IcPlus1Inv * IcPlus1Inv,
              Ic,
              IIc,
              J * J);
          isIndefiniteMask |= scalingUndecided & scalingIndefinite;
        }
      }
    } else {
      MOCHI_ASSERT_VERBOSE(oracle == MaterialPsdOracle::Conservative, "Unexpected PSD oracle.");
      isIndefiniteMask = (lhsBase * Ic > Sqr(a)) | (J < V{0_r});
    }

    projectingPsd = AnyTrue<kBatchSize>(isIndefiniteMask);
  }

  bool const useEigensystemPath = outTangent && projectingPsd &&
      (psdStrategy == MaterialPsdStrategy::Projection ||
       psdStrategy == MaterialPsdStrategy::AbsEigenProjection);
  bool const needCofF = outPK1 || (outTangent && !useEigensystemPath);

  // The cofactor of the deformation gradient.
  V3x3 cofF MOCHI_NO_INIT;
  if (needCofF) {
    cofF = Cofactor(F);
  }

  // The first Piola--Kirchhoff.
  if (outPK1) {
    V const c0 = muHat * (one - IcPlus1Inv);
    V const c1 = lambdaHat * Jma;
    // NOTE: *outPK1 = c0 * F + c1 * cofF is cleaner but may cause register spilling.
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        (*outPK1)[r][c] = c0 * F[r][c] + c1 * cofF[r][c];
      }
    }
  }

  if (outTangent) {
    auto& C = *outTangent;

    if (useEigensystemPath) {
      // Eigensystem decomposition path (Smith et al. 2018).
      // Note: BatchedRotationVariantSvd3x3 returns U, V^T and sigma such that det(U) >= 0, det(V^T)
      // >= 0, and sigma[2] < 0 iff det(F) < 0.
      V3x3 U MOCHI_NO_INIT, VT MOCHI_NO_INIT;
      V3 sigma MOCHI_NO_INIT;
      BatchedRotationVariantSvd3x3<kBatchSize>(F, U, sigma, VT);

      V const coeff0 = muHat - muHat * IcPlus1Inv;
      V const coeff1 = lambdaHat * Jma;

      // 9 eigenvalues of the Hessian in layout [scaling_0..2, twist_0..2, flip_0..2].
      V9 eigLambda MOCHI_NO_INIT;

      // Twist: coeff1·σₙ + coeff0.
      eigLambda[3] = coeff1 * sigma[0] + coeff0;
      eigLambda[4] = coeff1 * sigma[1] + coeff0;
      eigLambda[5] = coeff1 * sigma[2] + coeff0;

      // Flip: −coeff1·σₙ + coeff0.
      eigLambda[6] = -coeff1 * sigma[0] + coeff0;
      eigLambda[7] = -coeff1 * sigma[1] + coeff0;
      eigLambda[8] = -coeff1 * sigma[2] + coeff0;

      // Scaling mode matrix A (3×3 symmetric).
      V const IcPlus1SqrInv = IcPlus1Inv * IcPlus1Inv;
      V const mu2 = V{2_r} * muHat;
      V const J2alphaLambda = (V{2_r} * J - alpha) * lambdaHat;

      V3 const sigmaSqr = Sqr(sigma);
      // clang-format off
      BatchSymMatrix3x3<kBatchSize> const Asym = {
          (mu2 * sigmaSqr[0] - muHat * IcPlus1) * IcPlus1SqrInv + lambdaHat * sigmaSqr[1] * sigmaSqr[2] + muHat,
          (mu2 * sigmaSqr[1] - muHat * IcPlus1) * IcPlus1SqrInv + lambdaHat * sigmaSqr[0] * sigmaSqr[2] + muHat,
          (mu2 * sigmaSqr[2] - muHat * IcPlus1) * IcPlus1SqrInv + lambdaHat * sigmaSqr[0] * sigmaSqr[1] + muHat,
          J2alphaLambda * sigma[2] + mu2 * sigma[0] * sigma[1] * IcPlus1SqrInv,
          J2alphaLambda * sigma[1] + mu2 * sigma[0] * sigma[2] * IcPlus1SqrInv,
          J2alphaLambda * sigma[0] + mu2 * sigma[1] * sigma[2] * IcPlus1SqrInv};
      // clang-format on

      V3 scalingEigvals MOCHI_NO_INIT;
      V3x3 AqT MOCHI_NO_INIT;
      BatchedAnalyticalEigendecompSym3x3<kBatchSize>(Asym, scalingEigvals, &AqT);

      eigLambda[0] = scalingEigvals[0];
      eigLambda[1] = scalingEigvals[1];
      eigLambda[2] = scalingEigvals[2];

      V const minEig = V{kMinProjectedEigenvalue};
      if (psdStrategy == MaterialPsdStrategy::Projection) {
        for (int i = 0; i < 9; ++i) {
          eigLambda[i] = Max(eigLambda[i], minEig);
        }
      } else {
        for (int i = 0; i < 9; ++i) {
          eigLambda[i] = Max(Abs(eigLambda[i]), minEig);
        }
      }

      utils::BatchedAssembleTangentFromEigensystem<kBatchSize>(eigLambda, U, VT, AqT, C);

    } else {
      // Direct tangent: C[ij][kl] = c₃·F[ij]·F[kl] + λ̂·cofF[ij]·cofF[kl]
      //                            + λ̂(J−α)·d²J/dF[ij]dF[kl] + c₂·δᵢₖδⱼₗ
      //
      // Coefficients:
      //   c₂ = μ̂(1 − 1/(Ic+1))   (identity term)
      //   c₃ = 2μ̂ / (Ic+1)²      (Ic² second derivative)
      V const c2 = muHat * (one - IcPlus1Inv);
      V const c3 = V{2_r} * muHat * IcPlus1Inv * IcPlus1Inv;
      bool const projectingFast = projectingPsd && (psdStrategy == MaterialPsdStrategy::Fast);

      if (projectingFast && AllTrue<kBatchSize>(isIndefiniteMask)) {
        // Fast PSD path - Drop offending second derivatives in the volume preservation term
        // altogether to prevent negative eigenvalues.
        // C[ij][kl] = c₃·F[ij]·F[kl] + λ̂·cofF[ij]·cofF[kl] + c₂·δᵢₖδⱼₗ
        // Note the omitted λ̂(J−α)·d²J/dF² term.
        // Contraction strategy: hoist both temporary products, mirror in a separate pass. Fastest
        // for 2-rank contractions on x86-64 (AVX2) and ARM (NEON).
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            int const ij = i * 3 + j;
            V const c3Fij = c3 * F[i][j];
            V const lambdaHatCofFij = lambdaHat * cofF[i][j];
            for (int k = 0; k < 3; ++k) {
              for (int l = 0; l < 3; ++l) {
                int const kl = k * 3 + l;
                if (kl >= ij) {
                  C[i][j][k][l] = c3Fij * F[k][l] + lambdaHatCofFij * cofF[k][l];
                }
              }
            }
            C[i][j][i][j] += c2;
          }
        }

      } else {
        V d2JCoeff = lambdaHat * Jma;
        if (projectingFast) {
          d2JCoeff = Select(isIndefiniteMask, V{0_r}, d2JCoeff);
        }

        // Contraction strategy for separable terms: hoist temporary products, mirror in a separate
        // pass.
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            int const ij = i * 3 + j;
            V const c3Fij = c3 * F[i][j];
            V const lambdaHatCofFij = lambdaHat * cofF[i][j];
            for (int k = 0; k < 3; ++k) {
              for (int l = 0; l < 3; ++l) {
                int const kl = k * 3 + l;
                if (kl >= ij) {
                  C[i][j][k][l] = c3Fij * F[k][l] + lambdaHatCofFij * cofF[k][l];
                }
              }
            }
            C[i][j][i][j] += c2;
          }
        }
        utils::BatchAddD2JdF2UpperTriangle<kBatchSize>(d2JCoeff, F, C);
      }
      utils::BatchMirrorTangentUpperToLower<kBatchSize>(C);
    }
  }
}

MOCHI_INSTANTIATE_BATCHED_MATERIAL(
    BatchedSmithNeoHookeanConstitutiveResponse,
    BatchLameParams,
    MaterialPsdOracle);

} // namespace mochi::materials
