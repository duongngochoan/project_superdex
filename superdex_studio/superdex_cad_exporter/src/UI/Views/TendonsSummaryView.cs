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

using CADRobotExporter.UI.ViewModels;

namespace CADRobotExporter.UI.Views
{
    /// <summary>Raised when the user selects a tendon routing element.</summary>
    public class TendonRowSelectedEventArgs : EventArgs
    {
        public string TendonName { get; set; }
        public int ElementIndex { get; set; }
    }

    /// <summary>
    /// The Tendons Summary tab. Read-only preview of the routing elements, plus the two
    /// visualization toggles.
    /// </summary>
    public class TendonsSummaryView : UserControl
    {
        private readonly DataGrid grid;
        private readonly CheckBox showSelectedBox;
        private readonly CheckBox showAllBox;

        public event EventHandler ShowSelectedTendonChanged;
        public event EventHandler ShowAllTendonsChanged;
        public event EventHandler<TendonRowSelectedEventArgs> TendonRowSelected;

        public bool ShowSelectedTendon
        {
            get { return showSelectedBox.IsChecked == true; }
            set { showSelectedBox.IsChecked = value; }
        }

        public bool ShowAllTendons
        {
            get { return showAllBox.IsChecked == true; }
            set { showAllBox.IsChecked = value; }
        }

        public TendonsSummaryView()
        {
            TextBlock title = new TextBlock
            {
                Text = "Tendons Summary View",
                FontWeight = FontWeights.Bold,
                FontSize = 14,
            };

            TextBlock subtitle = new TextBlock
            {
                Text = "This page is a sumamry of the Tendons. Values cannot be edited directly, "
                    + "this is only for previewing.",
                Margin = new Thickness(0, 4, 0, 0),
            };

            StackPanel heading = new StackPanel();
            heading.Children.Add(title);
            heading.Children.Add(subtitle);

            showSelectedBox = new CheckBox { Content = "Show Selected Tendon", IsChecked = true };
            showAllBox = new CheckBox { Content = "Show All Tendons", Margin = new Thickness(16, 0, 0, 0) };
            showSelectedBox.Checked += (s, e) => Raise(ShowSelectedTendonChanged);
            showSelectedBox.Unchecked += (s, e) => Raise(ShowSelectedTendonChanged);
            showAllBox.Checked += (s, e) => Raise(ShowAllTendonsChanged);
            showAllBox.Unchecked += (s, e) => Raise(ShowAllTendonsChanged);

            StackPanel toggles = new StackPanel { Orientation = Orientation.Horizontal };
            toggles.Children.Add(showSelectedBox);
            toggles.Children.Add(showAllBox);

            GroupBox visualization = new GroupBox
            {
                Header = "Visualization",
                Content = toggles,
                Padding = new Thickness(6),
                VerticalAlignment = VerticalAlignment.Top,
            };

            Grid header = new Grid { Margin = new Thickness(0, 0, 0, 8) };
            header.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            header.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            Grid.SetColumn(heading, 0);
            Grid.SetColumn(visualization, 1);
            header.Children.Add(heading);
            header.Children.Add(visualization);

            grid = new DataGrid
            {
                AutoGenerateColumns = false,
                IsReadOnly = true,
                CanUserAddRows = false,
                CanUserDeleteRows = false,
                CanUserResizeRows = false,
                SelectionMode = DataGridSelectionMode.Single,
                SelectionUnit = DataGridSelectionUnit.FullRow,
                HeadersVisibility = DataGridHeadersVisibility.Column,
                GridLinesVisibility = DataGridGridLinesVisibility.All,
            };
            grid.SelectionChanged += OnGridSelectionChanged;

            AddColumn("Idx", "Index");
            AddColumn("Tendon", "TendonName");
            AddColumn("Elem", "Elem");
            AddColumn("Link", "Link");
            AddColumn("Type", "Type");
            AddColumn("X", "X");
            AddColumn("Y", "Y");
            AddColumn("Z", "Z");
            AddColumn("Coefficient", "Coefficient");

            Grid root = new Grid { Margin = new Thickness(8) };
            root.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            root.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });
            Grid.SetRow(header, 0);
            Grid.SetRow(grid, 1);
            root.Children.Add(header);
            root.Children.Add(grid);

            Content = root;
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

        private void Raise(EventHandler handler)
        {
            if (handler != null)
            {
                handler(this, EventArgs.Empty);
            }
        }

        public void ShowRows(IReadOnlyList<TendonSummaryRow> rows)
        {
            grid.ItemsSource = rows;
        }

        private void OnGridSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            TendonSummaryRow row = grid.SelectedItem as TendonSummaryRow;
            if (row == null)
            {
                return;
            }

            EventHandler<TendonRowSelectedEventArgs> handler = TendonRowSelected;
            if (handler != null)
            {
                handler(this, new TendonRowSelectedEventArgs
                {
                    TendonName = row.OwningTendonName,
                    ElementIndex = row.Elem,
                });
            }
        }
    }
}
