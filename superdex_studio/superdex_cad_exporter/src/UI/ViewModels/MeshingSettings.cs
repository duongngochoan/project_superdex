/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;

using Meshing;

namespace CADRobotExporter.UI.ViewModels
{
    /// <summary>
    /// Which controls in one meshing column are usable, given that column's own state.
    /// </summary>
    public struct MeshingColumnEnableState
    {
        public bool Scale;
        public bool Backend;
        public bool EdgeSampling;
        public bool TargetEdgeLength;
        public bool EdgeLengthFraction;
    }

    /// <summary>
    /// Presentation rules for the meshing settings columns: how combo box indices map to the
    /// meshing enums, which controls are usable, and how persisted values are kept in range.
    /// </summary>
    public static class MeshingSettings
    {
        // Mapped explicitly rather than cast, so the combo item order stays a presentation choice
        // and cannot silently change which backend a saved configuration means.
        public const int BackendIndexIsotropic = 0;
        public const int BackendIndexDelabella = 1;
        public const int EdgeSamplingIndexAdaptive = 0;
        public const int EdgeSamplingIndexUniform = 1;

        public static MeshingBackend BackendFromIndex(int index)
        {
            return index == BackendIndexDelabella ? MeshingBackend.Delabella : MeshingBackend.Isotropic;
        }

        public static int IndexFromBackend(MeshingBackend backend)
        {
            return backend == MeshingBackend.Delabella ? BackendIndexDelabella : BackendIndexIsotropic;
        }

        public static EdgeSampling EdgeSamplingFromIndex(int index)
        {
            return index == EdgeSamplingIndexUniform ? EdgeSampling.Uniform : EdgeSampling.Adaptive;
        }

        public static int IndexFromEdgeSampling(EdgeSampling sampling)
        {
            return sampling == EdgeSampling.Uniform ? EdgeSamplingIndexUniform : EdgeSamplingIndexAdaptive;
        }

        public static MeshingColumnEnableState EnableStateFor(
            bool columnEnabled,
            bool superDexSelected,
            MeshingBackend backend,
            bool targetEdgeLengthIsZero)
        {
            bool available = columnEnabled && superDexSelected;
            bool isotropic = backend == MeshingBackend.Isotropic;
            return new MeshingColumnEnableState
            {
                Scale = available,
                Backend = available,
                EdgeSampling = available && isotropic,
                TargetEdgeLength = available && isotropic,
                EdgeLengthFraction = available && isotropic && targetEdgeLengthIsZero,
            };
        }

        /// <summary>
        /// Persisted values are never validated against the control's range, and NumericUpDown
        /// throws rather than clamping when assigned out of range -- inside a CAD host that means
        /// taking the session down over a stale configuration.
        /// </summary>
        public static decimal Clamp(decimal value, decimal minimum, decimal maximum)
        {
            if (value < minimum)
            {
                return minimum;
            }
            if (value > maximum)
            {
                return maximum;
            }
            return value;
        }

        /// <summary>
        /// Whether a spinner showing <paramref name="shown"/> would look different from one given
        /// <paramref name="baseline"/>. Compared as displayed, after the same clamping and rounding,
        /// so a value that only differs past the shown digits is not flagged as an override.
        /// </summary>
        public static bool DiffersAsShown(
            decimal shown, double baseline, decimal minimum, decimal maximum, int decimalPlaces)
        {
            decimal clampedBaseline = Clamp((decimal)baseline, minimum, maximum);
            return Math.Round(shown, decimalPlaces, MidpointRounding.AwayFromZero)
                != Math.Round(clampedBaseline, decimalPlaces, MidpointRounding.AwayFromZero);
        }
    }
}
