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

#include <mochi_core/linear_algebra/any_matrix.h>
#include <mochi_core/linear_algebra/block_sparse_matrix.h>
#include <mochi_core/linear_algebra/matrix.h>
#include <mochi_core/linear_algebra/sparse_matrix.h>
#include <mochi_core/mochi_config.h>
#include <mochi_core/utils/debug.h>
#include <mochi_core/utils/dynamic_array.h>
#include <mochi_core/utils/graph.h>
#include <mochi_core/utils/graph_views.h>
#include <mochi_core/utils/nd_array.h>
#include <mochi_core/utils/span.h>

#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace mochi {

namespace details {

template <
    typename Scalar,
    int kBlockSize,
    typename CRIdx,
    typename Ptr,
    template <typename, typename...> typename Storage>
void ConvertToFillLevel(
    int p,
    BlockSparseMatrix<Scalar, kBlockSize, CRIdx, Ptr, Storage> const& A,
    BlockSparseMatrix<
        std::remove_const_t<Scalar>,
        kBlockSize,
        std::remove_const_t<CRIdx>,
        std::remove_const_t<Ptr>>& R) {
  MOCHI_ASSERT_VERBOSE(p >= 0, "Fill-in level must not be negative.");
  if (p == 0) {
    R.Reset(A);
    return;
  }
  //
  // We are using the sparsity pattern of A^{p + 1}.
  //
  using NonConstIdx = std::remove_const_t<CRIdx>;
  using NonConstPtr = std::remove_const_t<Ptr>;
  auto const gA = AsGraphView(A);
  DynamicArray<NonConstPtr> pointers(gA.GetPointers().begin(), gA.GetPointers().end());
  DynamicArray<NonConstIdx> targets(gA.GetTargets().begin(), gA.GetTargets().end());
  Graph<NonConstIdx, NonConstPtr> gAk(std::move(pointers), std::move(targets));
  for (int k = 0; k < p; ++k) {
    gAk = Traverse(gAk, gA).SortTargets();
  }
  R.Reset(A.BlockCols(), gAk);
  R.SetZero();
  R += A;
}

template <
    typename Scalar,
    int kRows,
    int kCols,
    krylov::Direction kDir,
    krylov::Ownership kOwner,
    int kLead>
void ConvertToFillLevel(
    int p,
    Matrix<Scalar, kRows, kCols, kDir, kOwner, kLead> const& A,
    Matrix<std::remove_const_t<Scalar>, kRows, kCols, krylov::Direction::RowMajor>& R) {
  MOCHI_ASSERT_VERBOSE(p >= 0, "Fill-in level must not be negative.");
  MOCHI_ASSERT(p == 0, "Only zero fill-in level is supported for dense matrices.");
  R.Reset(A);
}

template <
    typename Scalar,
    typename CRIdx,
    typename Ptr,
    template <typename, typename...> typename Storage>
void ConvertToFillLevel(
    int p,
    SparseMatrix<Scalar, CRIdx, Ptr, Storage> const& A,
    SparseMatrix<std::remove_const_t<Scalar>, std::remove_const_t<CRIdx>, std::remove_const_t<Ptr>>&
        R) {
  MOCHI_ASSERT_VERBOSE(p >= 0, "Fill-in level must not be negative.");
  if (p == 0) {
    R.Reset(A);
    return;
  }
  //
  // We are using the sparsity pattern of A^{p + 1}.
  //
  using NonConstIdx = std::remove_const_t<CRIdx>;
  using NonConstPtr = std::remove_const_t<Ptr>;
  auto const gA = AsGraphView(A);
  DynamicArray<NonConstPtr> pointers(gA.GetPointers().begin(), gA.GetPointers().end());
  DynamicArray<NonConstIdx> targets(gA.GetTargets().begin(), gA.GetTargets().end());
  Graph<NonConstIdx, NonConstPtr> gAk(std::move(pointers), std::move(targets));
  for (int k = 0; k < p; ++k) {
    gAk = Traverse(gAk, gA).SortTargets();
  }
  R.Reset(A.Cols(), gAk);
  R.SetZero();
  R += A;
}

} // namespace details

// Build a sparsity pattern from an associative collection of non-zero coordinates,
// represented as a vector of vectors: each entry in the vector contains all
// column indices of a given row.
// The method can take an optional number of rows to support empty rows.
Graph<int, int> MakeSparsityGraph(
    std::vector<std::vector<int>>&& M,
    std::optional<int> numRowsOpt = {});

// Build a sparsity pattern from a collection non-zero coordinates, represented as (row, col) pairs.
// The method can take an optional number of rows to support empty rows.
Graph<int, int> MakeSparsityGraph(
    std::vector<Int2> coordinates,
    std::optional<int> numRowsOpt = {});

// Set the values on the specified rows of a matrix to zero, except for the diagonal, which will be
// set the specified value (usually 1). The rowOffset is added to each of the rowIndices and may be
// negative. The matrix may be smaller than 'rowIndices[i] + rowOffset', e.g. interaction matrices
// may not be full-size.
void SetZeroOnRows(
    SparseMatrixView<real> mat,
    Span<int const> rowIndices,
    int rowOffset,
    real diagonalValue);
void SetZeroOnRows(
    BlockSparseMatrixView<real, 3> mat,
    Span<int const> rowIndices,
    int rowOffset,
    real diagonalValue);
void SetZeroOnRows(
    BlockSparseMatrixView<real, 4> mat,
    Span<int const> rowIndices,
    int rowOffset,
    real diagonalValue);
void SetZeroOnRows(
    MatrixView<real> mat,
    Span<int const> rowIndices,
    int rowOffset,
    real diagonalValue);
void SetZeroOnRows(ColumnVectorView<real> col, Span<int const> rowIndices, int rowOffset);
void SetZeroOnRows(
    AnyMatrixView<real> mat,
    Span<int const> rowIndices,
    int rowOffset,
    real diagonalValue);

/**
 * @brief Set the values on the specified columns of a matrix to zero.
 *
 * @param[in,out] mat The matrix to modify.
 * @param[in] colIndices The column indices to zero out.
 * @param[in] colOffset An offset added to each column index (may be negative). The actual column
 * zeroed is `colIndices[i] + colOffset`. Out-of-bounds columns (negative or >= mat.Cols()) are
 * possible for interaction matrices that do not span the full system and therefore are ignored
 * without error.
 * @param[in] symmetricPair A matrix whose sparsity pattern is the transpose of `mat`. For a
 * symmetric sparsity pattern, pass `AsConstView(mat)`. For an off-diagonal interaction matrix from
 * actor A to actor B, the symmetric pair is the matrix from B to A. The symmetric pair's row c
 * contains the column indices of row c in the transpose, which are exactly the row indices that
 * have non-zeros in column c of `mat`.
 * @param[in] valueIndicesCache Optional precomputed cache of value array indices to zero. When
 * provided, the function simply zeros out all cached value indices, bypassing any row/column
 * lookups and significantly improving performance.
 *
 * @note For dense matrices, the symmetric pair and cache are ignored.
 */
void SetZeroOnCols(
    SparseMatrixView<real> mat,
    Span<int const> colIndices,
    int colOffset,
    AnyMatrixView<real const> symmetricPair,
    Span<int const> valueIndicesCache = {});
void SetZeroOnCols(
    BlockSparseMatrixView<real, 3> mat,
    Span<int const> colIndices,
    int colOffset,
    AnyMatrixView<real const> symmetricPair,
    Span<int const> valueIndicesCache = {});
void SetZeroOnCols(
    BlockSparseMatrixView<real, 4> mat,
    Span<int const> colIndices,
    int colOffset,
    AnyMatrixView<real const> symmetricPair,
    Span<int const> valueIndicesCache = {});
void SetZeroOnCols(
    MatrixView<real> mat,
    Span<int const> colIndices,
    int colOffset,
    AnyMatrixView<real const> symmetricPair,
    Span<int const> valueIndicesCache = {});
void SetZeroOnCols(
    AnyMatrixView<real> mat,
    Span<int const> colIndices,
    int colOffset,
    AnyMatrixView<real const> symmetricPair,
    Span<int const> valueIndicesCache = {});

/**
 * @brief For each column index in newColIndices, computes which indices in the matrix's value array
 * correspond to non-zeros in that column, and appends them to valueIndicesCache. No-op for dense
 * matrices.
 *
 * @param[in] mat The square matrix, whose (block) sparsity pattern must be symmetric (i.e., if (r,
 * c) is in the pattern, then (c, r) must also be). Results are undefined otherwise.
 * @param[in] newColIndices The column indices whose value array indices should be appended to the
 * cache.
 * @param[in,out] valueIndicesCache The cache to append to (new indices are added at the end).
 *
 * @note The appended indices are sorted to improve memory efficiency when the cache is used.
 */
void AppendColValueIndexCache(
    AnyMatrixView<real const> mat,
    Span<int const> newColIndices,
    DynamicArray<int>& valueIndicesCache);

// Build a SparseMatrix which is a copy of the original, except that all zeros have been pruned from
// the sparsity pattern. If (blockSize > 1) then we will only blocks if all values within the block
// are zero.
SparseMatrix<real> DuplicateAndPrune(SparseMatrixView<real const> mat, int blockSize);

} // namespace mochi
