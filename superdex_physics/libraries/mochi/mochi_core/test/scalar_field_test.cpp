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

#include <mochi_core/geometry/aabb.h>
#include <mochi_core/geometry/geometry_utils.h>
#include <mochi_core/geometry/plane.h>
#include <mochi_core/geometry/scalar_field.h>
#include <mochi_core/geometry/sphere.h>
#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_core/utils/dynamic_array.h>
#include <mochi_core/utils/nd_array_utils.h>
#include <mochi_core/utils/simd.h>

#include <gtest/gtest.h>

#include <array>
#include <functional>

using namespace mochi;

using GetValueAndGradient = std::function<void(Real3 const& ptr, real& outValue, Real3& outGrad)>;

static_assert(
    static_cast<int>(GridExtrapolation::Count) == 4,
    "Please update the unit tests if GridExtrapolation enum changes");

static constexpr real kFdStep = 1e-5_r;

using RealVec = Simd<real>;
using RealVec3 = NdArray<RealVec, 3>;

// Samples the value at a single point by broadcasting it to every lane.
template <GridExtrapolation kMode>
static real SampleValue(DenseGrid3D<real> const& grid, Real3 const& point) {
  RealVec value MOCHI_NO_INIT;
  grid.template TrilinearSampleBatch<RealVec::kSize, kMode>(BroadcastEach<RealVec>(point), &value);
  return value[0];
}

// Samples the gradient at a single point by broadcasting it to every lane.
template <GridExtrapolation kMode>
static Real3 SampleGradient(DenseGrid3D<real> const& grid, Real3 const& point) {
  RealVec3 grad MOCHI_NO_INIT;
  grid.template TrilinearSampleBatch<
      RealVec::kSize,
      kMode,
      /*kComputeValues*/ false,
      /*kComputeGradients*/ true>(BroadcastEach<RealVec>(point), nullptr, &grad);
  return Real3{grad[0][0], grad[1][0], grad[2][0]};
}

// Verifies that each lane of a batch of distinct points sampled with
// @ref DenseGrid3D::TrilinearSampleBatch matches sampling that point alone. Also checks that
// sampling the value and gradient simultaneously matches sampling them separately.
template <GridExtrapolation kMode>
static void TestTrilinearSampleBatchEquivalence(
    DenseGrid3D<real> const& grid,
    Span<Real3 const> samplePoints) {
  constexpr int kBatchSize = RealVec::kSize;
  constexpr bool kIsGradientSupported = (kMode != GridExtrapolation::LowerBound);

  MOCHI_ASSERT_VERBOSE(!samplePoints.empty(), "Need at least one sample point.");

  // Assemble exactly one SIMD batch from the sample set, repeating points if there are fewer than
  // kBatchSize available.
  std::array<Real3, kBatchSize> batchPoints MOCHI_NO_INIT;
  for (int i = 0; i < kBatchSize; ++i) {
    batchPoints[i] = samplePoints[i % isize(samplePoints)];
  }
  RealVec3 simdPoints;
  LoadTransposed(&batchPoints[0][0], simdPoints[0], simdPoints[1], simdPoints[2]);

  RealVec batchValues;
  grid.template TrilinearSampleBatch<kBatchSize, kMode>(simdPoints, &batchValues);
  for (int i = 0; i < kBatchSize; ++i) {
    EXPECT_NEAR_EQ(SampleValue<kMode>(grid, batchPoints[i]), batchValues[i]);
  }

  if constexpr (kIsGradientSupported) {
    // Optimized emulated SIMD can round broadcast and distinct-lane gradients differently.
    constexpr real kGradientComparisonTolerance = 4e-6_r;
    RealVec3 batchGradients;
    grid.template TrilinearSampleBatch<
        kBatchSize,
        kMode,
        /*kComputeValues*/ false,
        /*kComputeGradients*/ true>(simdPoints, nullptr, &batchGradients);

    RealVec bothValues;
    RealVec3 bothGradients;
    grid.template TrilinearSampleBatch<
        kBatchSize,
        kMode,
        /*kComputeValues*/ true,
        /*kComputeGradients*/ true>(simdPoints, &bothValues, &bothGradients);

    for (int i = 0; i < kBatchSize; ++i) {
      Real3 const refGrad = SampleGradient<kMode>(grid, batchPoints[i]);
      Real3 const batchGrad{batchGradients[0][i], batchGradients[1][i], batchGradients[2][i]};
      Real3 const bothGrad{bothGradients[0][i], bothGradients[1][i], bothGradients[2][i]};
      EXPECT_NEAR_TOL(refGrad, batchGrad, kGradientComparisonTolerance);
      EXPECT_NEAR_EQ(batchValues[i], bothValues[i]);
      EXPECT_NEAR_TOL(refGrad, bothGrad, kGradientComparisonTolerance);
    }
  }
}

template <GridExtrapolation kMode>
static void TestSignedDistanceScalarField(
    DenseGrid3D<real> const& grid,
    GetValueAndGradient const& fn,
    Span<Real3 const> samplePoints,
    real absTol,
    bool expectAccurateGradientDirection,
    bool expectConsistentGradient) {
  constexpr bool kIsGradientSupported = (kMode != GridExtrapolation::LowerBound);

  for (Real3 const& point : samplePoints) {
    bool const isInteriorPoint = grid.Contains(point);
    real const sd = SampleValue<kMode>(grid, point);

    real trueSd = 0_r;
    Real3 trueGrad = {};
    fn(point, trueSd, trueGrad);

    // Check distance.
    if (isInteriorPoint) {
      EXPECT_NEAR_TOL(trueSd, sd, absTol);
    } else {
      if constexpr (kMode == GridExtrapolation::Clamp) {
        EXPECT_LE(sd, trueSd + absTol);
      } else if constexpr (kMode == GridExtrapolation::LowerBound) {
        EXPECT_LE(sd, trueSd + absTol);
      } else {
        EXPECT_EQ(GridExtrapolation::UpperBound, kMode);
        EXPECT_GE(sd, trueSd - absTol);
      }
    }

    // Check gradient.
    if constexpr (kIsGradientSupported) {
      Real3 const grad = SampleGradient<kMode>(grid, point);
      if (expectAccurateGradientDirection && isInteriorPoint) {
        // Check the computed closest point matches the true closest point. Only for interior
        // points (the gradient of exterior points is an approximation of the true gradient).
        Real3 trueClosestPt = point - trueSd * trueGrad;
        Real3 closestPt = point - sd * grad;
        EXPECT_LE(Norm(trueClosestPt - closestPt), absTol);
      }

      if (expectConsistentGradient) {
        // Check the computed gradient is consistent with the computed distance.
        for (int axis = 0; axis < 3; ++axis) {
          Real3 pointPlus = point;
          pointPlus[axis] += kFdStep;
          real const fdGrad = (SampleValue<kMode>(grid, pointPlus) - sd) / kFdStep;
          EXPECT_NEAR_RTOL(fdGrad, grad[axis], 1e-2_r);
        }
      }
    }
  }

  for (int base = 0; base < isize(samplePoints); base += RealVec::kSize) {
    TestTrilinearSampleBatchEquivalence<kMode>(
        grid, samplePoints.subspan(base, Min(RealVec::kSize, isize(samplePoints) - base)));
  }
}

// Create a DenseGridField which approximates a function
static DenseGrid3D<real>
CreateDenseGridField(GetValueAndGradient const& fn, Int3 gridSize, Aabb gridBounds) {
  DenseGrid3D<real> field{gridSize, gridBounds, gridBounds};
  Real3 const delta = gridBounds.GetSize() / StaticCast<Real3>(gridSize - 1);
  for (int x = 0; x < gridSize[0]; ++x) {
    for (int y = 0; y < gridSize[1]; ++y) {
      for (int z = 0; z < gridSize[2]; ++z) {
        Int3 index{x, y, z};
        Real3 pt = (StaticCast<Real3>(index) * delta) + gridBounds.GetMin();
        real dist = 0_r;
        Real3 gradIgnored;
        fn(pt, dist, gradIgnored);
        field(index) = dist;
      }
    }
  }
  return field;
}

TEST(DenseGrid3D, SphereSdf) {
  // Define an arbitrary sphere
  Real3 const center{1_r, 2_r, 3_r};
  real const radius = 0.1_r;

  // Function to compute the true signed distance and gradient
  auto fn = [&](Real3 const& pt, real& outSd, Real3& outGrad) {
    outSd = Norm(pt - center) - radius;
    outGrad = Normalize(pt - center);
  };

  // Function to generate points relative to the surface of the sphere
  auto getSamplePoints = [&](real offset) {
    DynamicArray<Real3> points;
    for (real a = 0_r; a < 2 * kPI; a += 0.1_r) {
      for (real b = 0_r; b < 2 * kPI; b += 0.1_r) {
        Real3 dir = //
            Quaternion::FromAxisAngle(Real3{0_r, 1_r, 0_r}, a) * //
            Quaternion::FromAxisAngle(Real3{0_r, 0_r, 1_r}, b) * //
            Real3{1_r, 0_r, 0_r};
        points.push_back(center + dir * (radius + offset));
      }
    }
    return points;
  };

  // Repeat these tests with different values
  constexpr real kTestGridPadding[] = {0_r, 0.05_r};
  constexpr Int3 kTestGridSizes[] = {{18, 18, 18}, {16, 17, 18}};
  constexpr real kTestSampleOffsets[] = {
      0_r, -0.08_r, 0.05_r, /* Tests points with 1, 2 and 3 axes outside the grid region */ 0.5_r};
  for (real gridPadding : kTestGridPadding) {
    for (Int3 gridSize : kTestGridSizes) {
      // Create a DenseGridField larger than the sphere
      Aabb const bounds{center - radius - gridPadding, center + radius + gridPadding};
      Real3 const delta = bounds.GetSize() / StaticCast<Real3>(gridSize - 1);
      auto sdf = CreateDenseGridField(fn, gridSize, bounds);

      // Check DenseGridField accessors
      EXPECT_NEAR_EQ(bounds, GetAabb(sdf.GetBounds()));
      EXPECT_EQ(gridSize, sdf.GetDimensions());
      EXPECT_NEAR_EQ(bounds.GetMin(), sdf.GetGridLowerCorner());
      EXPECT_NEAR_EQ(bounds.GetMax(), sdf.GetGridUpperCorner());
      EXPECT_NEAR_EQ(delta, sdf.GetVoxelSize());

      real const voxelDiagDist = Norm(delta);
      real const absTol = voxelDiagDist + 1e-5_r;
      for (real sampleOffset : kTestSampleOffsets) {
        // The direction of the gradient is inaccurate near the center of the sphere.
        bool const accurateGradDir = (Abs(radius + sampleOffset) >= 0.025_r);
        // FD gradients are not consistent since the SDF is not continuously differentiable.
        // Consistency is tested in DenseGrid3D.PlaneSdf, which is continuously differentiable.
        bool const consistentGrad = false;
        // Clamped offset to enforce points to be inside the grid region.
        real const clampedOffset = Min(sampleOffset, gridPadding);

        TestSignedDistanceScalarField<GridExtrapolation::Unsupported>(
            sdf, fn, getSamplePoints(clampedOffset), absTol, accurateGradDir, consistentGrad);
        TestSignedDistanceScalarField<GridExtrapolation::Clamp>(
            sdf, fn, getSamplePoints(sampleOffset), absTol, accurateGradDir, consistentGrad);
        TestSignedDistanceScalarField<GridExtrapolation::UpperBound>(
            sdf, fn, getSamplePoints(sampleOffset), absTol, accurateGradDir, consistentGrad);
        TestSignedDistanceScalarField<GridExtrapolation::LowerBound>(
            sdf, fn, getSamplePoints(sampleOffset), absTol, accurateGradDir, consistentGrad);
      }
    }
  }
}

static void TestPlaneSdf(Plane plane) {
  // Function to compute the true signed distance and gradient
  auto fn = [&](Real3 const& pt, real& outSd, Real3& outGrad) {
    outSd = Dot(pt, plane.GetNormal()) - plane.GetDistanceFromOrigin();
    outGrad = plane.GetNormal();
  };

  // Function to generate points relative to the surface of the plane
  auto getSamplePoints = [&](real offset) {
    // Test only points along the plane normal through the origin to ensure (a) no points outside
    // the grid region have negative distance (illegal) and (b) the SDF is locally continuously
    // differentiable (needed for consistency checks). Points with extrapolation along >1 axis are
    // tested in DenseGrid3D.SphereSdf.
    DynamicArray<Real3> points;
    points.push_back(plane.GetNormal() * (offset + plane.GetDistanceFromOrigin()));
    return points;
  };

  // Repeat these tests with different values
  constexpr Int3 kTestGridSizes[] = {{2, 2, 2}, {3, 4, 5}, {2, 20, 4}};
  constexpr real kTestSampleOffsets[] = {
      0_r, -0.05_r, 0.05_r, /* Tests points outside the grid region */ 1_r};
  for (Int3 gridSize : kTestGridSizes) {
    // Create a DenseGridField
    Aabb const bounds{Real3{-1_r, -1_r, -1_r}, Real3{1_r, 1_r, 1_r}};
    Real3 const delta = bounds.GetSize() / StaticCast<Real3>(gridSize - 1);
    auto sdf = CreateDenseGridField(fn, gridSize, bounds);

    // Check DenseGridField accessors
    EXPECT_NEAR_EQ(bounds, GetAabb(sdf.GetBounds()));
    EXPECT_EQ(gridSize, sdf.GetDimensions());
    EXPECT_NEAR_EQ(bounds.GetMin(), sdf.GetGridLowerCorner());
    EXPECT_NEAR_EQ(bounds.GetMax(), sdf.GetGridUpperCorner());
    EXPECT_NEAR_EQ(delta, sdf.GetVoxelSize());

    constexpr real kAbsTol = 1e-6_r; // Should be quite accurate
    for (real sampleOffset : kTestSampleOffsets) {
      // Clamped offset to enforce points (including FD differences) to be inside the grid region.
      real const clampedSampleOffset =
          Min(sampleOffset, 0.99999_r - plane.GetDistanceFromOrigin() - kFdStep);

      TestSignedDistanceScalarField<GridExtrapolation::Unsupported>(
          sdf, fn, getSamplePoints(clampedSampleOffset), kAbsTol, true, true);
      TestSignedDistanceScalarField<GridExtrapolation::Clamp>(
          sdf, fn, getSamplePoints(sampleOffset), kAbsTol, true, true);
      TestSignedDistanceScalarField<GridExtrapolation::UpperBound>(
          sdf, fn, getSamplePoints(sampleOffset), kAbsTol, true, true);
      TestSignedDistanceScalarField<GridExtrapolation::LowerBound>(
          sdf, fn, getSamplePoints(sampleOffset), kAbsTol, true, true);
    }
  }
}

TEST(DenseGrid3D, PlaneSdf) {
  // Test a few different planes
  Plane const kTestPlanes[] = {
      {Real3{1_r, 0_r, 0_r}, 0.1_r},
      {Real3{0_r, 1_r, 0_r}, 0.2_r},
      {Real3{0_r, 0_r, 1_r}, 0.3_r},
      {Real3{-1_r, 0_r, 0_r}, 0.4_r},
      {Real3{0_r, -1_r, 0_r}, 0.5_r},
      {Real3{0_r, 0_r, -1_r}, 0.6_r},
  };
  for (auto plane : kTestPlanes) {
    TestPlaneSdf(plane);
  }
}
