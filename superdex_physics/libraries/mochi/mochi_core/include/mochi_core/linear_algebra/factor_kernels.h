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

#include <mochi_core/linear_algebra/matrix.h>
#include <mochi_core/utils/simd.h>

#include <type_traits>

namespace mochi {

using krylov::Direction;
using krylov::kDynamic;
using krylov::Ownership;

struct AlwaysFalseFtor {
  template <typename... A>
  MOCHI_FORCE_INLINE bool operator()(A&&... /*unused*/) {
    return false;
  }
};

namespace kernel {

// Scalar loops here that compute x -= sum(a * b) accumulate x in a local and store it once: the
// output may alias the factor, so updating x in place would force a store on every iteration. They
// also work on two rows per pass and add any term that couples the two rows last, so two
// independent chains share the loads and roughly double the work in flight on these latency-bound
// loops.

template <
    typename Scalar,
    int kRowsAtCT = krylov::kDynamic,
    int kColsAtCT = krylov::kDynamic,
    Direction kMajorDirection = Direction::ColMajor>
using MatrixViewDynLD = Matrix<
    Scalar,
    kRowsAtCT,
    kColsAtCT,
    kMajorDirection,
    krylov::Ownership::View,
    krylov::kDynamic>;

/**
 * @brief Compute \f$ P <- P L^{-T} \f$ where the diagonal of \f$ L \f$ is 1 everywhere.
 * @details The diagonal of L is not read and can contain any data.
 * @details It performs no runtime checks. The correct input matrix sizes are ensured at compile
 * time.
 *
 * @tparam Scalar
 * @tparam kTrSize
 * @param L
 * @param P
 */
template <typename Scalar, int kTrSize>
MOCHI_FORCE_INLINE void RightTriangularUnitTransposeSolve(
    MatrixViewDynLD<Scalar const, kTrSize, kTrSize, Direction::ColMajor> L,
    MatrixViewDynLD<Scalar, kDynamic, kTrSize, Direction::ColMajor> P) {
  using VecT = Simd<Scalar>;
  constexpr int kVecSize = VecT::kSize;
  int const rows = P.Rows();
  int blRow = 0;
  if constexpr (VecT::kIsSupported) {
    for (; blRow + kVecSize <= rows; blRow += kVecSize) {
      VecT Pv[kTrSize];
      for (int c = 0; c < kTrSize; ++c) {
        Pv[c] = Load<VecT>(&P(blRow, c));
      }
      for (int c = 1; c < kTrSize; ++c) {
        for (int uc = c; uc < kTrSize; ++uc) {
          Pv[uc] -= L(uc, c - 1) * Pv[c - 1];
        }
        Store(&P(blRow, c), Pv[c]);
      }
    }
  }
  int r = blRow;
  for (; r + 1 < rows; r += 2) {
    for (int c = 1; c < kTrSize; ++c) { // c: Updated column
      auto sum0 = P(r, c);
      auto sum1 = P(r + 1, c);
      for (int lc = 0; lc < c; ++lc) { // lc: left column
        auto const l = L(c, lc);
        sum0 -= l * P(r, lc);
        sum1 -= l * P(r + 1, lc);
      }
      P(r, c) = sum0;
      P(r + 1, c) = sum1;
    }
  }
  if (r < rows) {
    for (int c = 1; c < kTrSize; ++c) {
      auto sum = P(r, c);
      for (int lc = 0; lc < c; ++lc) {
        sum -= L(c, lc) * P(r, lc);
      }
      P(r, c) = sum;
    }
  }
}

MOCHI_FORCE_INLINE void RightTriangularUnitSolve(IsMatrix auto&& l, IsMatrix auto&& m) {
  // Performance note: The case with small 'n' and/or 'mRows' could be optimized further.
  int const n = l.Rows();
  int const mRows = m.Rows();
  if (mRows == 1) {
    int i = n - 2; // Last entry is unchanged.
    if (n >= 2 && n % 2 == 0) {
      // Entries go in pairs. With an odd count, this one, which has a single term, goes alone.
      m(0, i) -= m(0, i + 1) * l(i + 1, i);
      --i;
    }
    for (; i >= 1; i -= 2) {
      auto sum0 = m(0, i);
      auto sum1 = m(0, i - 1);
      for (int k = i + 1; k < n; ++k) {
        auto const mk = m(0, k);
        sum0 -= mk * l(k, i);
        sum1 -= mk * l(k, i - 1);
      }
      m(0, i) = sum0;
      m(0, i - 1) = sum1 - sum0 * l(i, i - 1);
    }
  } else {
    for (int i = n - 1; i-- > 0;) {
      int const len = n - (i + 1);
      m.Col(i) -= m.MiddleCols(i + 1, len) * l.template Block<kDynamic, 1>(i + 1, i, len, 1);
    }
  }
}

/**
 * @brief Same as RightTriangularUnitTransposeSolve but in blocks.
 * @details It performs no runtime checks. The correct input matrix sizes are ensured at compile
 * time.
 *
 * @tparam Scalar
 * @tparam kTrSize
 * @tparam kBlockSize
 * @param L
 * @param M
 */
template <typename Scalar, int kTrSize, int kBlockSize>
MOCHI_FORCE_INLINE void ApplyLmtOnRight(
    MatrixViewDynLD<Scalar const, kBlockSize, kBlockSize> L,
    MatrixViewDynLD<Scalar, kDynamic, kBlockSize> M) {
  static_assert(kBlockSize % kTrSize == 0, "Block size must be multiple of kTrSize");
  for (int i = 0; i < kBlockSize; i += kTrSize) {
    MatrixViewDynLD<Scalar, kDynamic, kTrSize> blockM = M.template MiddleCols<kTrSize>(i, kTrSize);
    if (i != 0) { // Left looking elimination
      auto leftM = M.LeftCols(i);
      auto factors = L.template Block<kTrSize, kDynamic>(i, 0, kTrSize, i);
      blockM -= leftM * factors.Transpose();
    }
    auto miniLDLt = L.template Block<kTrSize, kTrSize>(i, i, kTrSize, kTrSize);
    RightTriangularUnitTransposeSolve(miniLDLt, blockM);
  }
}

/**
 * @brief Same as Right Triangular Unit Solve in blocks.
 * @details It performs no runtime checks. The correct input matrix sizes are ensured at compile
 * time.
 *
 * @tparam Scalar
 * @tparam kTrSize
 * @tparam kBlockSize
 * @param L
 * @param M
 */
template <int kTrSize, int kBlockSize, typename LT, typename MT>
MOCHI_FORCE_INLINE void ApplyLm1OnRight(LT&& L, MT&& M) {
  MOCHI_ASSERT_VERBOSE(L.Rows() == L.Cols(), "L matrix must be square.");
  static_assert(kBlockSize % kTrSize == 0, "Block size must be multiple of kTrSize");
  int n = L.Rows();
  int i = n;
  for (; i >= kTrSize;) {
    auto blockM = M.template MiddleCols<kTrSize>(i - kTrSize, kTrSize);
    if (i != n) { // right looking elimination
      auto rightM = M.RightCols(n - i);
      auto factors = L.template Block<kDynamic, kTrSize>(i, i - kTrSize, n - i, kTrSize);
      blockM -= rightM * factors;
    }
    i -= kTrSize;
    auto miniL = L.template Block<kTrSize, kTrSize>(i, i, kTrSize, kTrSize);
    RightTriangularUnitSolve(miniL, blockM);
  }
  if (i != 0) {
    auto blockM = M.LeftCols(i);
    if (i != n) {
      auto rightM = M.RightCols(n - i);
      auto factors = L.Block(i, 0, n - i, i);
      blockM -= rightM * factors;
    }
    auto miniL = L.Block(0, 0, i, i);
    RightTriangularUnitSolve(miniL, blockM);
  }
}

template <typename LT, typename XT>
MOCHI_FORCE_INLINE void ApplyLm1OnLeft(LT&& L, XT&& X) {
  // Performance note: The case with small 'n' and/or 'xCols' could be optimized further. This case
  // is important for (1) the factorization of small matrices and (2) the inverse of matrices
  // computed via Small{LU,LDLt} specializations.
  MOCHI_ASSERT_VERBOSE(L.Rows() == L.Cols(), "L matrix must be square.");
  MOCHI_ASSERT_VERBOSE(X.Rows() == L.Cols(), "Inconsistent matrix sizes.");
  int const n = L.Rows();
  int const xCols = X.Cols();
  if (xCols == 1) {
    // Dedicated path for solve on a column vector. Some compilers (e.g. MSVC) introduce too much
    // overhead in the temporary views and function calls in the regular path.
    int i = 1; // First row is unchanged.
    if (n >= 2 && n % 2 == 0) {
      // Rows go in pairs. With an odd count, this one, which has a single term, goes alone.
      X(1, 0) -= L(1, 0) * X(0, 0);
      i = 2;
    }
    for (; i + 1 < n; i += 2) {
      auto sum0 = X(i, 0);
      auto sum1 = X(i + 1, 0);
      for (int j = 0; j < i; ++j) {
        auto const xj = X(j, 0);
        sum0 -= L(i, j) * xj;
        sum1 -= L(i + 1, j) * xj;
      }
      X(i, 0) = sum0;
      X(i + 1, 0) = sum1 - L(i + 1, i) * sum0;
    }
  } else {
    for (int i = 1; i < n; ++i) {
      X.Row(i) -= L.template Block<1, kDynamic>(i, 0, 1, i) * X.TopRows(i);
    }
  }
}

template <typename LT, typename XT>
MOCHI_FORCE_INLINE void ApplyLmtOnLeft(LT&& L, XT&& X) {
  // Performance note: The case with small 'n' and/or 'xCols' could be optimized further. This case
  // is important for (1) the factorization of small matrices and (2) the inverse of matrices
  // computed via Small{LU,LDLt} specializations.
  MOCHI_ASSERT_VERBOSE(L.Rows() == L.Cols(), "L matrix must be square.");
  MOCHI_ASSERT_VERBOSE(X.Rows() == L.Cols(), "Inconsistent matrix sizes.");
  int const n = L.Rows();
  int const xCols = X.Cols();
  if (xCols == 1) {
    // Dedicated path for solve on a column vector. Some compilers (e.g. MSVC) introduce too much
    // overhead in the temporary views and function calls in the regular path.
    int i = n - 2; // Last row is unchanged.
    if (n >= 2 && n % 2 == 0) {
      // Rows go in pairs. With an odd count, this one, which has a single term, goes alone.
      X(i, 0) -= L(i + 1, i) * X(i + 1, 0);
      --i;
    }
    for (; i >= 1; i -= 2) {
      auto sum0 = X(i, 0);
      auto sum1 = X(i - 1, 0);
      for (int j = i + 1; j < n; ++j) {
        auto const xj = X(j, 0);
        sum0 -= L(j, i) * xj;
        sum1 -= L(j, i - 1) * xj;
      }
      X(i, 0) = sum0;
      X(i - 1, 0) = sum1 - L(i, i - 1) * sum0;
    }
  } else {
    for (int i = n - 2; i >= 0; --i) { // Last row is unchanged.
      int const len = n - (i + 1);
      X.Row(i) -=
          L.template Block<kDynamic, 1>(i + 1, i, len, 1).Transpose() * X.MiddleRows(i + 1, len);
    }
  }
}

template <typename UT, typename XT>
MOCHI_FORCE_INLINE void ApplyUm1OnLeft(UT&& U, XT&& X) {
  // Performance note: The case with small 'n' and/or 'xCols' could be optimized further. This case
  // is important for (1) the factorization of small matrices and (2) the inverse of matrices
  // computed via Small{LU,LDLt} specializations.
  MOCHI_ASSERT_VERBOSE(U.Rows() == U.Cols(), "U matrix must be square.");
  MOCHI_ASSERT_VERBOSE(X.Rows() == U.Cols(), "Inconsistent matrix sizes.");
  int const n = U.Rows();
  int const xCols = X.Cols();
  if (xCols == 1) {
    // Dedicated path for solve on a column vector. Some compilers (e.g. MSVC) introduce too much
    // overhead in the temporary views and function calls in the regular path.
    int i = n - 1;
    if (n % 2 == 1) {
      // Rows go in pairs. With an odd count, this one, which has no terms, goes alone.
      X(i, 0) *= U(i, i);
      --i;
    }
    for (; i >= 1; i -= 2) {
      auto sum0 = X(i, 0);
      auto sum1 = X(i - 1, 0);
      for (int j = i + 1; j < n; ++j) {
        auto const xj = X(j, 0);
        sum0 -= U(i, j) * xj;
        sum1 -= U(i - 1, j) * xj;
      }
      auto const xi = sum0 * U(i, i);
      auto const xim1 = (sum1 - U(i - 1, i) * xi) * U(i - 1, i - 1);
      X(i, 0) = xi;
      X(i - 1, 0) = xim1;
    }
  } else {
    for (int i = n; i-- > 0;) {
      int const len = n - (i + 1);
      X.Row(i) -= U.template Block<1, kDynamic>(i, i + 1, 1, len) * X.MiddleRows(i + 1, len);
      for (int k = 0; k < xCols; ++k) {
        X(i, k) *= U(i, i);
      }
    }
  }
}

template <typename UT, typename XT>
MOCHI_FORCE_INLINE void ApplyUm1OnRight(UT&& U, XT&& X) {
  // Performance note: The case with small 'n' and/or 'xRows' could be optimized further. This case
  // is important for (1) the factorization of small matrices and (2) the inverse of matrices
  // computed via Small{LU,LDLt} specializations.
  MOCHI_ASSERT_VERBOSE(U.Rows() == U.Cols(), "U matrix must be square.");
  MOCHI_ASSERT_VERBOSE(X.Cols() == U.Cols(), "Inconsistent matrix sizes.");
  int const n = U.Rows();
  int const xRows = X.Rows();
  if (xRows == 1) {
    // Dedicated path for solve on a row vector. Some compilers (e.g. MSVC) introduce too much
    // overhead in the temporary views and function calls in the regular path.
    int i = 0;
    if (n % 2 == 1) {
      // Entries go in pairs. With an odd count, this one, which has no terms, goes alone.
      X(0, 0) *= U(0, 0);
      i = 1;
    }
    for (; i + 1 < n; i += 2) {
      auto sum0 = X(0, i);
      auto sum1 = X(0, i + 1);
      for (int j = 0; j < i; ++j) {
        auto const xj = X(0, j);
        sum0 -= xj * U(j, i);
        sum1 -= xj * U(j, i + 1);
      }
      auto const xi = sum0 * U(i, i);
      auto const xip1 = (sum1 - xi * U(i, i + 1)) * U(i + 1, i + 1);
      X(0, i) = xi;
      X(0, i + 1) = xip1;
    }
  } else {
    for (int i = 0; i < n; ++i) {
      X.Col(i) -= X.LeftCols(i) * U.template Block<kDynamic, 1>(0, i, i, 1);
      for (int k = 0; k < xRows; ++k) {
        X(k, i) *= U(i, i);
      }
    }
  }
}

template <typename Scalar, typename CheckFtor = AlwaysFalseFtor, int kRowsAtCT, int kColsAtCT>
MOCHI_FORCE_INLINE int Factor(
    MatrixViewDynLD<Scalar, kRowsAtCT, kColsAtCT, Direction::ColMajor> A,
    CheckFtor&& singularDetection = {}) {
  static_assert(
      kRowsAtCT == krylov::kDynamic || kColsAtCT == krylov::kDynamic || kRowsAtCT == kColsAtCT,
      "A fixed-sized input matrix must be square.");
  MOCHI_ASSERT_VERBOSE(A.Rows() == A.Cols(), "Input matrix must be square.");
  using VType = Simd<Scalar>;
  int singularities = 0;
  int const n = A.Rows();
  for (int er = 0; er < n; ++er) {
    // Left looking update. rv: Row of the updated vector.
    int rv = er;
    if constexpr (VType::kIsSupported) {
      for (; rv + VType::kSize <= n; rv += VType::kSize) {
        auto v = Load<VType>(&A(rv, er));
        for (int lc = 0; lc < er; ++lc) {
          v -= Load<VType>(&A(rv, lc)) * A(lc, er);
        }
        Store(&A(rv, er), v);
      }
    }
    for (; rv + 1 < n; rv += 2) {
      auto sum0 = A(rv, er);
      auto sum1 = A(rv + 1, er);
      for (int lc = 0; lc < er; ++lc) {
        auto const u = A(lc, er);
        sum0 -= A(rv, lc) * u;
        sum1 -= A(rv + 1, lc) * u;
      }
      A(rv, er) = sum0;
      A(rv + 1, er) = sum1;
    }
    if (rv < n) {
      auto sum = A(rv, er);
      for (int lc = 0; lc < er; ++lc) {
        sum -= A(rv, lc) * A(lc, er);
      }
      A(rv, er) = sum;
    }
    Scalar& d = A(er, er);
    if (singularDetection(er, d, A.Block(er, er, n - er, n - er))) MOCHI_UNLIKELY {
      ++singularities;
      d = Scalar{0};
      for (int r = er + 1; r < n; ++r) {
        A(er, r) = Scalar{0};
      }
      continue;
    }
    auto const dInv = Scalar(1) / d;
    // Copy to the upper part and multiply by the inverse.
    for (int r = er + 1; r < n; ++r) {
      A(er, r) = A(r, er);
      A(r, er) *= dInv;
    }
    d = dInv;
  }
  return singularities;
}

/**
 * @brief Factor a block \f$ A \f$ of limited size into \f$ A = L D L^T \f$.
 * @details Typically the block size should be lower than then number of SIMD registers of the CPU.
 * @tparam Scalar
 * @tparam kBlockSize
 * @tparam CheckFtor
 * @param A
 * @param singularities
 * @param singularDetection
 * @return U matrix of the LU decomposition of \f$ A \f$. Only the upper triangular part is
 * populated.
 */
template <typename Scalar, int kBlockSize, typename CheckFtor = AlwaysFalseFtor>
MOCHI_FORCE_INLINE Matrix<Scalar, kBlockSize, kBlockSize> FactorBlock(
    MatrixViewDynLD<Scalar, kBlockSize, kBlockSize, Direction::ColMajor> A,
    int& singularities,
    CheckFtor&& singularDetection = {}) {
  // Use the native SIMD size unless the block requires a smaller supported divisor.
  constexpr int kNativeVecSize = Simd<Scalar>::kSize;
  constexpr int kVecSize = Simd<Scalar, kBlockSize>::kIsSupported && kBlockSize < kNativeVecSize
      ? kBlockSize
      : (kBlockSize % kNativeVecSize == 0 ? kNativeVecSize : kNativeVecSize / 2);
  using VecT = Simd<Scalar, kVecSize>;
  static_assert(kBlockSize % kVecSize == 0, "Block size must be a multiple of SIMD size");
  static_assert(kVecSize % 2 == 0);
  Matrix<Scalar, kBlockSize, kBlockSize> U;
  for (int ir = 0; ir + kVecSize <= kBlockSize; ir += kVecSize) {
    for (int sr = 0; sr < kVecSize; ++sr) {
      int r = ir + sr;
      // Left looking update. rv: row of the updated vector
      if constexpr (VecT::kIsSupported) {
        for (int rv = ir; rv + kVecSize <= kBlockSize; rv += kVecSize) {
          auto v = Load<VecT>(&A(rv, r));
          for (int c = 0; c < r; ++c) {
            v -= Load<VecT>(&A(rv, c)) * U(c, r);
          }
          Store(&A(rv, r), v);
        }
      } else {
        int rr = r;
        for (; rr + 1 < kBlockSize; rr += 2) {
          auto sum0 = A(rr, r);
          auto sum1 = A(rr + 1, r);
          for (int c = 0; c < r; ++c) {
            auto const u = U(c, r);
            sum0 -= A(rr, c) * u;
            sum1 -= A(rr + 1, c) * u;
          }
          A(rr, r) = sum0;
          A(rr + 1, r) = sum1;
        }
        if (rr < kBlockSize) {
          auto sum = A(rr, r);
          for (int c = 0; c < r; ++c) {
            sum -= A(rr, c) * U(c, r);
          }
          A(rr, r) = sum;
        }
      }
      auto& d = A(r, r);
      if (singularDetection(r, d, A.Block(r, r, kBlockSize - r, kBlockSize - r))) MOCHI_UNLIKELY {
        d = Scalar{0};
        ++singularities;
        continue;
      }
      auto const invD = Scalar{1} / d;
      // Copy lower part to U and multiply by the inverse.
      for (int c = r + 1; c < kBlockSize; ++c) {
        U(r, c) = A(c, r);
        A(c, r) *= invD; // Final result for a column of L
      }
      d = invD;
    }
  }
  return U;
}

/**
 * @brief Factor a matrix into LU form without blocking but with pivoting.
 * @details This function is flexible in how the pivoting is implemented via the use of a
 * pivoting functor. The functor is called with `pivot(lu, k)` where k is the current equation
 * index to eliminate from the rest.
 *  - Chose the pivot
 *  - Swap rows and/or columns as required by the strategy, bringing the pivot to lu(k,k)
 *  - Return true if factorization should continue, false otherwise.
 *
 * If the pivot functor determines that the factorization is becoming unstable but could
 * be stabilized by using a pivot outside of the current block, it can return false and proceed
 * with some recovery algorithm outside of the scope of this function.
 *
 * The pivot functor can also be used to accumulate the determinant.
 *
 * @tparam M Matrix type for lu
 * @tparam Pvt Pivoting strategy functor type
 * @param lu The matrix to factor
 * @param pivot The pivoter functor
 */
template <typename M, typename Pvt>
MOCHI_FORCE_INLINE void FlexibleUnblockedPivotFactor(M&& lu, Pvt&& pivot) {
  using Scalar = std::decay_t<decltype(lu(0, 0))>;
  int nRow = lu.Rows();
  for (int k = 0; k < nRow - 1; ++k) {
    if (!pivot(lu, k)) MOCHI_UNLIKELY {
      return;
    }
    int rRow = nRow - k - 1;
    auto diag = lu(k, k);
    if (diag != Scalar{0}) MOCHI_LIKELY {
      auto f = Scalar{1} / diag;
      auto l = lu.Block(k + 1, k, rRow, 1);
      auto u = lu.Block(k, k + 1, 1, rRow);
      auto r = lu.Block(k + 1, k + 1, rRow, rRow);
      for (int i = 0; i < l.Rows(); ++i) {
        l(i, 0) /= diag;
      }
      lu(k, k) = f;
      r -= l * u;
    }
  }
  if (pivot(lu, nRow - 1)) {
    auto diag = lu(nRow - 1, nRow - 1);
    if (diag != Scalar{0}) MOCHI_LIKELY {
      lu(nRow - 1, nRow - 1) = Scalar{1} / diag;
    }
  }
}

template <typename RT, typename XT>
inline void BackSubstitutionInPlace(RT const& R, XT&& X) {
  MOCHI_ASSERT_VERBOSE(R.Rows() == R.Cols(), "R matrix must be square.");
  MOCHI_ASSERT_VERBOSE(X.Rows() == R.Cols(), "Inconsistent matrix sizes.");
  using Scalar = std::decay_t<decltype(X(0, 0))>;
  int const n = R.Rows();
  int const xCols = X.Cols();
  if (xCols == 1) {
    // Dedicated path for a column vector. Some compilers (e.g. MSVC) introduce too much overhead
    // in the temporary views and function calls in the regular path.
    auto const divide = [&](int i, Scalar x) MOCHI_FORCE_INLINE_LAMBDA {
      Scalar const RiiInv = (R(i, i) == Scalar(0)) ? Scalar(0) : Scalar(1) / R(i, i);
      MOCHI_ASSERT_VERBOSE(R(i, i) != Scalar(0) || x == Scalar(0), "Singular matrix.");
      return x * RiiInv;
    };
    int i = n - 1;
    if (n % 2 == 1) {
      // Rows go in pairs. With an odd count, this one, which has no terms, goes alone.
      X(i, 0) = divide(i, X(i, 0));
      --i;
    }
    for (; i >= 1; i -= 2) {
      auto sum0 = X(i, 0);
      auto sum1 = X(i - 1, 0);
      for (int j = i + 1; j < n; ++j) {
        auto const xj = X(j, 0);
        sum0 -= R(i, j) * xj;
        sum1 -= R(i - 1, j) * xj;
      }
      auto const xi = divide(i, sum0);
      auto const xim1 = divide(i - 1, sum1 - R(i - 1, i) * xi);
      X(i, 0) = xi;
      X(i - 1, 0) = xim1;
    }
  } else {
    for (int i = n - 1; i >= 0; --i) {
      int const len = n - (i + 1);
      X.Row(i) -= R.template Block<1, kDynamic>(i, i + 1, 1, len) * X.BottomRows(len);
      Scalar const RiiInv = (R(i, i) == Scalar(0)) ? Scalar(0) : Scalar(1) / R(i, i);
      for (int k = 0; k < xCols; ++k) {
        MOCHI_ASSERT_VERBOSE(R(i, i) != Scalar(0) || X(i, k) == Scalar(0), "Singular matrix.");
        X(i, k) *= RiiInv;
      }
    }
  }
}

} // namespace kernel

} // namespace mochi
