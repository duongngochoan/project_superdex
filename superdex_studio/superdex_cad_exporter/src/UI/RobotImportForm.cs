/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Windows;
using System.Windows.Controls;

using SolidWorks.Interop.sldworks;

using OpenFileDialog = System.Windows.Forms.OpenFileDialog;

namespace CADRobotExporter.UI
{
    public class RobotImportForm : Window
    {
        private static readonly string DefaultCoordSysName = "-- default --";

        private readonly ModelDoc2 ActiveSWModel;
        private readonly TextBox textBoxFilePath;
        private readonly ComboBox comboBoxCoordinateSystem;
        private readonly CheckBox checkBoxCreateCSYS;
        private readonly CheckBox checkBoxCreateRobotConfiguration;
        private readonly Button buttonOK;

        public string UrdfFilePath => textBoxFilePath.Text;

        public bool CreateCoordinateSystems => checkBoxCreateCSYS.IsChecked == true;

        public bool CreateRobotConfiguration => checkBoxCreateRobotConfiguration.IsChecked == true;

        public string SelectedCoordinateSystem
        {
            get
            {
                string selected = comboBoxCoordinateSystem.SelectedItem as string;
                if (selected == DefaultCoordSysName)
                {
                    return null;
                }

                return selected;
            }
        }

        public RobotImportForm(SldWorks swApp)
        {
            Theme.FluentTheme.Apply(this);

            Title = "Robot Importer";
            Width = 620;
            SizeToContent = SizeToContent.Height;
            ResizeMode = ResizeMode.NoResize;
            WindowStartupLocation = WindowStartupLocation.CenterOwner;
            ShowInTaskbar = false;
            Background = (System.Windows.Media.Brush)Resources["LayerFill"];

            ActiveSWModel = (ModelDoc2)swApp.ActiveDoc;

            textBoxFilePath = new TextBox();
            textBoxFilePath.TextChanged += (sender, e) =>
                buttonOK.IsEnabled = !string.IsNullOrWhiteSpace(textBoxFilePath.Text);

            comboBoxCoordinateSystem = new ComboBox { Margin = new Thickness(0, 6, 0, 0) };
            checkBoxCreateCSYS = new CheckBox
            {
                Content = "Create Coordinate Systems",
                IsChecked = true,
            };
            checkBoxCreateRobotConfiguration = new CheckBox
            {
                Content = "Create Robot Configuration",
                IsChecked = true,
                Margin = new Thickness(0, 8, 0, 0),
            };

            buttonOK = new Button
            {
                Content = "Import",
                IsDefault = true,
                MinWidth = 110,
                IsEnabled = false,
                Style = (Style)Resources[Theme.FluentTheme.AccentButtonKey],
            };
            buttonOK.Click += (sender, e) => DialogResult = true;

            Button buttonCancel = new Button
            {
                Content = "Cancel",
                IsCancel = true,
                MinWidth = 110,
                Margin = new Thickness(8, 0, 0, 0),
            };

            StackPanel root = new StackPanel { Margin = new Thickness(12) };
            root.Children.Add(BuildFileGroup());
            root.Children.Add(BuildOptionsGroup());

            StackPanel buttons = new StackPanel
            {
                Orientation = Orientation.Horizontal,
                HorizontalAlignment = HorizontalAlignment.Right,
                Margin = new Thickness(0, 14, 0, 0),
            };
            buttons.Children.Add(buttonOK);
            buttons.Children.Add(buttonCancel);
            root.Children.Add(buttons);

            Content = root;

            AddCoordinateSystems();
        }

        private UIElement BuildFileGroup()
        {
            Button buttonBrowse = new Button
            {
                Content = "Browse...",
                MinWidth = 110,
                Margin = new Thickness(8, 0, 0, 0),
            };
            buttonBrowse.Click += ButtonBrowseClick;

            Grid row = new Grid();
            row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            row.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            Grid.SetColumn(textBoxFilePath, 0);
            Grid.SetColumn(buttonBrowse, 1);
            row.Children.Add(textBoxFilePath);
            row.Children.Add(buttonBrowse);

            return new GroupBox
            {
                Header = "File",
                Padding = new Thickness(10),
                Content = row,
            };
        }

        private UIElement BuildOptionsGroup()
        {
            StackPanel body = new StackPanel();
            body.Children.Add(checkBoxCreateCSYS);
            body.Children.Add(checkBoxCreateRobotConfiguration);
            body.Children.Add(new TextBlock
            {
                Text = "Base/World Coordinate System",
                Margin = new Thickness(0, 14, 0, 0),
            });
            body.Children.Add(comboBoxCoordinateSystem);

            return new GroupBox
            {
                Header = "Import Options",
                Padding = new Thickness(10),
                Margin = new Thickness(0, 10, 0, 0),
                Content = body,
            };
        }

        private void AddCoordinateSystems()
        {
            comboBoxCoordinateSystem.Items.Clear();
            comboBoxCoordinateSystem.Items.Add(DefaultCoordSysName);
            comboBoxCoordinateSystem.SelectedIndex = 0;

            object[] features = ActiveSWModel.FeatureManager.GetFeatures(true) as object[];
            if (features != null)
            {
                foreach (Feature feat in features)
                {
                    if (feat.GetTypeName2() == "CoordSys")
                    {
                        comboBoxCoordinateSystem.Items.Add(feat.Name);
                    }
                }
            }
        }

        private void ButtonBrowseClick(object sender, RoutedEventArgs e)
        {
            using (OpenFileDialog dialog = new OpenFileDialog
            {
                Title = "Select URDF File to Import",
                Filter = "URDF files (*.urdf)|*.urdf|XML files (*.xml)|*.xml|All files (*.*)|*.*",
                FilterIndex = 1,
            })
            {
                if (dialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                {
                    textBoxFilePath.Text = dialog.FileName;
                }
            }
        }
    }
}
