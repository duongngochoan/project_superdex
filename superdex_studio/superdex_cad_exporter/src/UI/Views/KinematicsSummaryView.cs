/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;

using CADRobotExporter.UI.ViewModels;

namespace CADRobotExporter.UI.Views
{
    /// <summary>Raised when the user activates a summary row, to jump to the editing tab.</summary>
    public class KinematicsRowActivatedEventArgs : EventArgs
    {
        public string LinkName { get; set; }
        public string JointName { get; set; }

        /// <summary>
        /// True when the activated column describes the joint rather than the link, which decides
        /// which tab to switch to.
        /// </summary>
        public bool IsJointColumn { get; set; }
    }

    /// <summary>
    /// The Kinematics Summary tab. Read-only; the grid is a preview and edits happen on the Link
    /// and Joint tabs.
    /// </summary>
    public class KinematicsSummaryView : UserControl
    {
        // WPF's DataGridColumn has no Tag, so joint columns are identified by the property they
        // bind to. These are the columns that describe the joint rather than the link.
        private static readonly HashSet<string> JointColumnProperties = new HashSet<string>
        {
            "JointName", "JointType", "Axis", "LowerLimit", "UpperLimit",
            "EffortLimit", "VelocityLimit", "Friction", "Damping",
        };

        private readonly DataGrid grid;
        private readonly CheckBox useDegreesBox;

        public event EventHandler UseDegreesChanged;
        public event EventHandler<KinematicsRowActivatedEventArgs> RowActivated;

        public bool UseDegrees
        {
            get { return useDegreesBox.IsChecked == true; }
            set { useDegreesBox.IsChecked = value; }
        }

        public KinematicsSummaryView()
        {
            TextBlock title = new TextBlock
            {
                Text = "Kinematics Summary View",
                FontWeight = FontWeights.Bold,
                FontSize = 14,
            };

            TextBlock subtitle = new TextBlock
            {
                Text = "This page is a summary of Link and Joint properties, to edit values, please use "
                    + "their respective tabs." + Environment.NewLine
                    + "Double-clicking a cell will switch to the Joint or Link tab for editing.",
                Margin = new Thickness(0, 4, 0, 0),
            };

            StackPanel heading = new StackPanel();
            heading.Children.Add(title);
            heading.Children.Add(subtitle);

            useDegreesBox = new CheckBox
            {
                Content = "Use Degrees",
                IsChecked = true,
                VerticalAlignment = VerticalAlignment.Top,
                HorizontalAlignment = HorizontalAlignment.Right,
            };
            useDegreesBox.Checked += (s, e) => RaiseUseDegreesChanged();
            useDegreesBox.Unchecked += (s, e) => RaiseUseDegreesChanged();

            Grid header = new Grid { Margin = new Thickness(0, 0, 0, 8) };
            header.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            header.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            Grid.SetColumn(heading, 0);
            Grid.SetColumn(useDegreesBox, 1);
            header.Children.Add(heading);
            header.Children.Add(useDegreesBox);

            grid = new DataGrid
            {
                AutoGenerateColumns = false,
                IsReadOnly = true,
                CanUserAddRows = false,
                CanUserDeleteRows = false,
                CanUserReorderColumns = true,
                CanUserResizeRows = false,
                SelectionUnit = DataGridSelectionUnit.Cell,
                HeadersVisibility = DataGridHeadersVisibility.Column,
                GridLinesVisibility = DataGridGridLinesVisibility.All,
            };
            grid.MouseDoubleClick += OnGridDoubleClick;

            Grid root = new Grid { Margin = new Thickness(8) };
            root.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            root.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });
            Grid.SetRow(header, 0);
            Grid.SetRow(grid, 1);
            root.Children.Add(header);
            root.Children.Add(grid);

            Content = root;
        }

        private void RaiseUseDegreesChanged()
        {
            EventHandler handler = UseDegreesChanged;
            if (handler != null)
            {
                handler(this, EventArgs.Empty);
            }
        }

        /// <summary>
        /// Rebuilds the columns and rows. Columns are rebuilt because three headers carry the
        /// current angle unit.
        /// </summary>
        public void ShowRows(IReadOnlyList<KinematicsSummaryRow> rows, bool useDegrees)
        {
            string unit = KinematicsSummary.LimitUnit(useDegrees);

            grid.Columns.Clear();
            AddColumn("Idx", "Index");
            AddColumn("Link Name", "LinkName");
            AddColumn("Joint Name", "JointName");
            AddColumn("Joint Type", "JointType");
            AddColumn("Axis", "Axis");
            AddColumn($"Lower ({unit})", "LowerLimit");
            AddColumn($"Upper ({unit})", "UpperLimit");
            AddColumn("Effort (N-m, N)", "EffortLimit");
            AddColumn($"Velocity ({unit}/s, m/s)", "VelocityLimit");
            AddColumn("Friction", "Friction");
            AddColumn("Damping", "Damping");
            AddColumn("Mass (kg)", "Mass");
            AddColumn("Inertia (diag)", "Inertia");
            AddColumn("# Ine", "InertialBodies");
            AddColumn("# Col", "CollisionBodies");
            AddColumn("# Vis", "VisualBodies");

            grid.ItemsSource = rows;
        }

        private void AddColumn(string header, string property)
        {
            grid.Columns.Add(new DataGridTextColumn
            {
                Header = header,
                Binding = new Binding(property),
                SortMemberPath = property,
                Width = DataGridLength.Auto,
            });
        }

        private void OnGridDoubleClick(object sender, MouseButtonEventArgs e)
        {
            KinematicsSummaryRow row = grid.CurrentItem as KinematicsSummaryRow;
            if (row == null || grid.CurrentColumn == null)
            {
                return;
            }

            string linkName = row.LinkName == null ? null : row.LinkName.Trim();
            if (string.IsNullOrEmpty(linkName))
            {
                return;
            }

            bool isJointColumn = JointColumnProperties.Contains(grid.CurrentColumn.SortMemberPath ?? "");

            // The base link has no joint, so there is nothing to jump to.
            if (isJointColumn && string.IsNullOrWhiteSpace(row.JointName))
            {
                return;
            }

            EventHandler<KinematicsRowActivatedEventArgs> handler = RowActivated;
            if (handler != null)
            {
                handler(this, new KinematicsRowActivatedEventArgs
                {
                    LinkName = linkName,
                    JointName = row.JointName,
                    IsJointColumn = isJointColumn,
                });
            }
        }
    }
}
