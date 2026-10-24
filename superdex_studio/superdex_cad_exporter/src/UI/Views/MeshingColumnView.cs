/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.Windows;
using System.Windows.Controls;

using CADRobotExporter.UI.ViewModels;

namespace CADRobotExporter.UI.Views
{
    /// <summary>
    /// One meshing column (visual or collision). The WinForms original had no column type, so
    /// every read, write and enable rule was written twice over as
    /// <c>exportingCollision ? xCollision : x</c>; holding a column's controls together lets those
    /// call sites drop the ternaries entirely.
    /// </summary>
    public class MeshingColumnView
    {
        public NumericSpinner LinearDeflection { get; private set; }
        public NumericSpinner AngularDeflection { get; private set; }
        public NumericSpinner Scale { get; private set; }
        public ComboBox Backend { get; private set; }
        public ComboBox EdgeSampling { get; private set; }
        public NumericSpinner TargetEdgeLength { get; private set; }
        public NumericSpinner EdgeLengthFraction { get; private set; }

        public event EventHandler SettingChanged;

        public MeshingColumnView(bool collision)
        {
            LinearDeflection = MakeSpinner(0M, 100M, 0.1M, 3, collision ? 0.3M : 0.1M);
            AngularDeflection = MakeSpinner(0M, 180M, 0.1M, 3, collision ? 0.75M : 0.5M);
            Scale = MakeSpinner(0.01M, 10000M, 10M, 3, 1M);
            TargetEdgeLength = MakeSpinner(0M, 1000M, 0.1M, 3, 0M);
            EdgeLengthFraction = MakeSpinner(0.001M, 0.5M, 0.005M, 4, 0.02M);

            Backend = MakeCombo("Isotropic", "Delabella");
            EdgeSampling = MakeCombo("Adaptive", "Uniform");

            // Every control raises, not only the two the enable rules read, so the override bold
            // follows each edit as it is made.
            LinearDeflection.ValueChanged += (s, e) => Raise();
            AngularDeflection.ValueChanged += (s, e) => Raise();
            Scale.ValueChanged += (s, e) => Raise();
            Backend.SelectionChanged += (s, e) => Raise();
            EdgeSampling.SelectionChanged += (s, e) => Raise();
            TargetEdgeLength.ValueChanged += (s, e) => Raise();
            EdgeLengthFraction.ValueChanged += (s, e) => Raise();
        }

        private void Raise()
        {
            EventHandler handler = SettingChanged;
            if (handler != null)
            {
                handler(this, EventArgs.Empty);
            }
        }

        private static NumericSpinner MakeSpinner(
            decimal minimum, decimal maximum, decimal increment, int decimalPlaces, decimal value)
        {
            NumericSpinner spinner = new NumericSpinner
            {
                Minimum = minimum,
                Maximum = maximum,
                Increment = increment,
                DecimalPlaces = decimalPlaces,
                MinWidth = 70,
                Margin = new Thickness(0, 1, 0, 1),
            };
            spinner.Value = value;
            return spinner;
        }

        private static ComboBox MakeCombo(params string[] items)
        {
            ComboBox combo = new ComboBox { Margin = new Thickness(0, 1, 0, 1), MinWidth = 70 };
            foreach (string item in items)
            {
                combo.Items.Add(item);
            }
            combo.SelectedIndex = 0;
            return combo;
        }

        public void ReadInto(Export.MeshingOptions target)
        {
            target.linearDeflection = (double)LinearDeflection.Value;
            target.angularDeflection = (double)AngularDeflection.Value;
            target.scale = (double)Scale.Value;
            target.backend = MeshingSettings.BackendFromIndex(Backend.SelectedIndex);
            target.edgeSampling = MeshingSettings.EdgeSamplingFromIndex(EdgeSampling.SelectedIndex);
            target.targetEdgeLength = (double)TargetEdgeLength.Value;
            target.targetEdgeLengthFraction = (double)EdgeLengthFraction.Value;
        }

        /// <summary>
        /// Values arrive from persisted JSON that nothing validates against these ranges, so every
        /// assignment clamps.
        /// </summary>
        public void Write(Export.MeshingOptions source)
        {
            LinearDeflection.Value = (decimal)source.linearDeflection;
            AngularDeflection.Value = (decimal)source.angularDeflection;
            Scale.Value = (decimal)source.scale;
            Backend.SelectedIndex = MeshingSettings.IndexFromBackend(source.backend);
            EdgeSampling.SelectedIndex = MeshingSettings.IndexFromEdgeSampling(source.edgeSampling);
            TargetEdgeLength.Value = (decimal)source.targetEdgeLength;
            EdgeLengthFraction.Value = (decimal)source.targetEdgeLengthFraction;
        }

        /// <param name="superDexSelected">
        /// Whether this column has a SuperDex mesh format selected. The SuperDex-only settings are
        /// greyed out when it does not.
        /// </param>
        public void ApplyEnableState(bool columnEnabled, bool superDexSelected)
        {
            MeshingColumnEnableState state = MeshingSettings.EnableStateFor(
                columnEnabled,
                superDexSelected,
                MeshingSettings.BackendFromIndex(Backend.SelectedIndex),
                TargetEdgeLength.Value == 0M);

            Scale.IsEnabled = state.Scale;
            Backend.IsEnabled = state.Backend;
            EdgeSampling.IsEnabled = state.EdgeSampling;
            TargetEdgeLength.IsEnabled = state.TargetEdgeLength;
            EdgeLengthFraction.IsEnabled = state.EdgeLengthFraction;

            LinearDeflection.IsEnabled = columnEnabled;
            AngularDeflection.IsEnabled = columnEnabled;
        }

        // Bolds each value that differs from baseline
        public void BoldValuesDifferentFrom(Export.MeshingOptions baseline)
        {
            Bold(LinearDeflection, baseline != null
                && LinearDeflection.DiffersAsShownFrom(baseline.linearDeflection));
            Bold(AngularDeflection, baseline != null
                && AngularDeflection.DiffersAsShownFrom(baseline.angularDeflection));
            Bold(Scale, baseline != null && Scale.DiffersAsShownFrom(baseline.scale));
            Bold(TargetEdgeLength, baseline != null
                && TargetEdgeLength.DiffersAsShownFrom(baseline.targetEdgeLength));
            Bold(EdgeLengthFraction, baseline != null
                && EdgeLengthFraction.DiffersAsShownFrom(baseline.targetEdgeLengthFraction));

            // Set on the ComboBox itself, which reaches the selected item shown in the closed box.
            Backend.FontWeight = Weight(baseline != null
                && Backend.SelectedIndex != MeshingSettings.IndexFromBackend(baseline.backend));
            EdgeSampling.FontWeight = Weight(baseline != null
                && EdgeSampling.SelectedIndex
                    != MeshingSettings.IndexFromEdgeSampling(baseline.edgeSampling));
        }

        private static void Bold(NumericSpinner spinner, bool bold)
        {
            spinner.SetTextWeight(Weight(bold));
        }

        private static FontWeight Weight(bool bold)
        {
            return bold ? FontWeights.Bold : FontWeights.Normal;
        }
    }
}
