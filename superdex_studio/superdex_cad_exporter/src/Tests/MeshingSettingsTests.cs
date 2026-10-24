/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using CADRobotExporter.UI.ViewModels;
using Meshing;
using Xunit;

namespace CADRobotExporter.Tests
{
    /// <summary>
    /// Covers the meshing column presentation rules: combo index mapping, enable state, and the
    /// range clamping that keeps a stale configuration from throwing inside the CAD host.
    /// </summary>
    public class MeshingSettingsTests
    {
        [Theory]
        [InlineData(MeshingBackend.Isotropic)]
        [InlineData(MeshingBackend.Delabella)]
        public void Backend_RoundTripsThroughItsComboIndex(MeshingBackend backend)
        {
            int index = MeshingSettings.IndexFromBackend(backend);

            Assert.Equal(backend, MeshingSettings.BackendFromIndex(index));
        }

        [Theory]
        [InlineData(EdgeSampling.Adaptive)]
        [InlineData(EdgeSampling.Uniform)]
        public void EdgeSampling_RoundTripsThroughItsComboIndex(EdgeSampling sampling)
        {
            int index = MeshingSettings.IndexFromEdgeSampling(sampling);

            Assert.Equal(sampling, MeshingSettings.EdgeSamplingFromIndex(index));
        }

        /// <summary>
        /// An index outside the known range must fall back rather than throw, since it can come
        /// from a combo box that has not been populated yet.
        /// </summary>
        [Fact]
        public void UnknownIndicesFallBackToTheDefaultMember()
        {
            Assert.Equal(MeshingBackend.Isotropic, MeshingSettings.BackendFromIndex(-1));
            Assert.Equal(EdgeSampling.Adaptive, MeshingSettings.EdgeSamplingFromIndex(-1));
        }

        [Fact]
        public void DisabledColumn_DisablesEverything()
        {
            MeshingColumnEnableState state =
                MeshingSettings.EnableStateFor(false, true, MeshingBackend.Isotropic, true);

            Assert.False(state.Scale);
            Assert.False(state.Backend);
            Assert.False(state.EdgeSampling);
            Assert.False(state.TargetEdgeLength);
            Assert.False(state.EdgeLengthFraction);
        }

        /// <summary>
        /// Nothing reads these values unless the column exports a SuperDex format, so an enabled
        /// column with no such format still greys the whole group.
        /// </summary>
        [Fact]
        public void NoSuperDexFormat_DisablesEverything()
        {
            MeshingColumnEnableState state =
                MeshingSettings.EnableStateFor(true, false, MeshingBackend.Isotropic, true);

            Assert.False(state.Scale);
            Assert.False(state.Backend);
            Assert.False(state.EdgeSampling);
            Assert.False(state.TargetEdgeLength);
            Assert.False(state.EdgeLengthFraction);
        }

        /// <summary>
        /// Delabella ignores the sampling settings but is still scaled, so Scale does not follow the
        /// backend the way the rest of the group does.
        /// </summary>
        [Fact]
        public void NonIsotropicBackend_LeavesOnlyTheBackendAndScaleEnabled()
        {
            MeshingColumnEnableState state =
                MeshingSettings.EnableStateFor(true, true, MeshingBackend.Delabella, true);

            Assert.True(state.Scale);
            Assert.True(state.Backend);
            Assert.False(state.EdgeSampling);
            Assert.False(state.TargetEdgeLength);
            Assert.False(state.EdgeLengthFraction);
        }

        [Fact]
        public void IsotropicBackend_EnablesTheSamplingSettings()
        {
            MeshingColumnEnableState state =
                MeshingSettings.EnableStateFor(true, true, MeshingBackend.Isotropic, true);

            Assert.True(state.Scale);
            Assert.True(state.Backend);
            Assert.True(state.EdgeSampling);
            Assert.True(state.TargetEdgeLength);
            Assert.True(state.EdgeLengthFraction);
        }

        /// <summary>
        /// The fraction is only meaningful while the absolute target length is zero.
        /// </summary>
        [Fact]
        public void NonZeroTargetEdgeLength_DisablesTheFraction()
        {
            MeshingColumnEnableState state =
                MeshingSettings.EnableStateFor(true, true, MeshingBackend.Isotropic, false);

            Assert.True(state.TargetEdgeLength);
            Assert.False(state.EdgeLengthFraction);
        }

        [Theory]
        [InlineData(5, 0, 10, 5)]
        [InlineData(-1, 0, 10, 0)]
        [InlineData(11, 0, 10, 10)]
        [InlineData(0, 0, 10, 0)]
        [InlineData(10, 0, 10, 10)]
        public void Clamp_KeepsValuesWithinTheControlRange(
            decimal value, decimal min, decimal max, decimal expected)
        {
            Assert.Equal(expected, MeshingSettings.Clamp(value, min, max));
        }

        [Theory]
        [InlineData(0.1, 0.1, false)]
        [InlineData(0.2, 0.1, true)]
        [InlineData(0.1, 0.1000004, false)]
        [InlineData(0.1, 0.1006, true)]
        public void DiffersAsShown_ComparesAtTheDisplayedPrecision(
            double shown, double baseline, bool expected)
        {
            Assert.Equal(
                expected,
                MeshingSettings.DiffersAsShown((decimal)shown, baseline, 0M, 100M, 3));
        }

        [Fact]
        public void DiffersAsShown_ClampsTheBaselineLikeTheSpinnerWould()
        {
            Assert.False(MeshingSettings.DiffersAsShown(100M, 250.0, 0M, 100M, 3));
        }
    }
}
