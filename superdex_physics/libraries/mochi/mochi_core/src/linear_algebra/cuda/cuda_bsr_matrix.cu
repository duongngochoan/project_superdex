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

#include <mochi_core/linear_algebra/base_enums.h>
#include <mochi_core/linear_algebra/cuda/cuda_bsr_matrix.h>
#include <mochi_core/linear_algebra/cuda/cuda_lib.h>
#include <mochi_core/utils/basic_utils.h>

namespace mochi::krylov::details {

template <typename Scalar, krylov::Direction kMajorDir>
MOCHI_GPU_KERNEL void Convert(
    int blockSize,
    int nBlockRows,
    int const* MOCHI_RESTRICT rowPtr,
    Scalar const* MOCHI_RESTRICT csrVal,
    Scalar* MOCHI_RESTRICT bsrVal) {
  for (int rb = blockIdx.x; rb < nBlockRows; rb += gridDim.x) {
    // Number of block columns in this block row
    int const numBlockCols = rowPtr[rb + 1] - rowPtr[rb];
    int const totalCols = numBlockCols * blockSize;
    // Base pointers for this block row
    auto* bsrBase = bsrVal + blockSize * blockSize * rowPtr[rb];
    auto const* csrBase = csrVal + blockSize * blockSize * rowPtr[rb];
    for (int iy = threadIdx.y; iy < blockSize; iy += blockDim.y) {
      for (int jx = threadIdx.x; jx < totalCols; jx += blockDim.x) {
        int const localBlock = jx / blockSize;
        int const localCol = jx % blockSize;
        if constexpr (kMajorDir == mochi::krylov::Direction::ColMajor) {
          bsrBase[iy + localCol * blockSize + localBlock * blockSize * blockSize] =
              csrBase[jx + iy * totalCols];
        } else {
          bsrBase[localCol + iy * blockSize + localBlock * blockSize * blockSize] =
              csrBase[jx + iy * totalCols];
        }
      }
    }
  }
}

template <typename Scalar, krylov::Direction kMajorDir>
void ConvertCsrToBsr(
    int blockSize,
    int nBlockRows,
    int maxNnzPerRow,
    int const* d_bsrRowPtr,
    Scalar const* d_csrVal,
    Scalar* d_bsrVal) {
  MOCHI_ASSERT_VERBOSE(blockSize > 0, "Invalid block size (%d)", blockSize);
  if (nBlockRows == 0 || maxNnzPerRow == 0) {
    return;
  }
  int device = 0;
  MOCHI_CUDA_CHECK(cudaGetDevice(&device));
  int maxThreadsPerBlock = 0;
  MOCHI_CUDA_CHECK(
      cudaDeviceGetAttribute(&maxThreadsPerBlock, cudaDevAttrMaxThreadsPerBlock, device));
  MOCHI_ASSERT_VERBOSE(maxThreadsPerBlock > 0, "Invalid thread limit (%d)", maxThreadsPerBlock);
  // Convert strides in both dimensions, so capping at CUDA's thread limit is safe.
  int const nRowsY = Min(blockSize, maxThreadsPerBlock);
  auto const nColsX = (unsigned int)(Min(maxNnzPerRow, maxThreadsPerBlock / nRowsY));
  dim3 blockDim{nColsX, (unsigned int)(nRowsY)};
  Convert<Scalar, kMajorDir>
      <<<nBlockRows, blockDim>>>(blockSize, nBlockRows, d_bsrRowPtr, d_csrVal, d_bsrVal);
  MOCHI_CUDA_CHECK_LAST();
}

template void ConvertCsrToBsr<double, krylov::Direction::ColMajor>(
    int blockSize,
    int nBlockRows,
    int maxNnzPerRow,
    int const* d_bsrRowPtr,
    double const* d_csrVal,
    double* d_bsrVal);

template void ConvertCsrToBsr<double, krylov::Direction::RowMajor>(
    int blockSize,
    int nBlockRows,
    int maxNnzPerRow,
    int const* d_bsrRowPtr,
    double const* d_csrVal,
    double* d_bsrVal);

template void ConvertCsrToBsr<float, krylov::Direction::ColMajor>(
    int blockSize,
    int nBlockRows,
    int maxNnzPerRow,
    int const* d_bsrRowPtr,
    float const* d_csrVal,
    float* d_bsrVal);

template void ConvertCsrToBsr<float, krylov::Direction::RowMajor>(
    int blockSize,
    int nBlockRows,
    int maxNnzPerRow,
    int const* d_bsrRowPtr,
    float const* d_csrVal,
    float* d_bsrVal);

} // namespace mochi::krylov::details
