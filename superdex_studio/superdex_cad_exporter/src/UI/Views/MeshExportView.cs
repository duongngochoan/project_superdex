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
using System.Windows.Documents;
using System.Windows.Media;

using CADRobotExporter.Export;

namespace CADRobotExporter.UI.Views
{
    /// <summary>
    /// The lower two thirds of the mesh export group: the mesh format table on the left, and the
    /// meshing options for the visual and collision columns on the right.
    /// </summary>
    public class MeshExportView : UserControl
    {
        public MeshingColumnView Visual { get; private set; }
        public MeshingColumnView Collision { get; private set; }

        private readonly CheckBox meshingOverride =
            new CheckBox { Content = "Link Meshing Override" };
        private readonly CheckBox collisionMeshing = new CheckBox { Content = "Collision" };
        private readonly Button resetToDefaults = new Button
        {
            Content = "Reset to Default",
            Padding = new Thickness(10, 2, 10, 2),
        };
        private readonly Button exportLinkMesh = new Button
        {
            Content = "Export Link Mesh...",
            Padding = new Thickness(10, 2, 10, 2),
        };

        private readonly TextBox visualExtension = new TextBox { Text = "glb" };
        private readonly TextBox collisionExtension = new TextBox { Text = "glb" };
        private readonly ComboBox folderStructure = new ComboBox();

        private readonly Dictionary<MeshFormat, CheckBox> visualFormatBoxes =
            new Dictionary<MeshFormat, CheckBox>();
        private readonly Dictionary<MeshFormat, CheckBox> collisionFormatBoxes =
            new Dictionary<MeshFormat, CheckBox>();
        private readonly Dictionary<MeshFormat, TextBlock> methodLabels =
            new Dictionary<MeshFormat, TextBlock>();

        /// <summary>
        /// Held so the group can be greyed as a whole, which is the only way to dim its caption
        /// column and header. It spans both columns, so it stays enabled while either one has a
        /// SuperDex format
        /// </summary>
        private GroupBox superdexGroup;
        private readonly GroupBox meshingGroup;

        private static readonly Brush OverrideTint = CreateOverrideTint();

        private static Brush CreateOverrideTint()
        {
            SolidColorBrush brush = new SolidColorBrush(Color.FromRgb(0xDC, 0xEB, 0xFA));
            brush.Freeze();
            return brush;
        }

        public event EventHandler MeshingOverrideChanged;
        public event EventHandler CollisionMeshingChanged;
        public event EventHandler ResetToDefaultsRequested;
        public event EventHandler ExportLinkMeshRequested;
        public event EventHandler HelpMeChooseRequested;
        public event EventHandler<MeshFormatToggledEventArgs> VisualFormatToggled;
        public event EventHandler<MeshFormatToggledEventArgs> CollisionFormatToggled;
        public event EventHandler MesherSettingChanged;

        /// <summary>Row order matches the original table, top to bottom.</summary>
        private static readonly MeshFormat[] FormatRows =
        {
            MeshFormat.stlOpenCascade,
            MeshFormat.glbOpenCascade,
            MeshFormat.objOpenCascade,
            MeshFormat.stlSolidworks,
            MeshFormat.glbCAD,
            MeshFormat.objCAD,
            MeshFormat.stepSolidworks,
        };

        /// <summary>
        /// The order formats are handed to the exporter, and therefore the order meshes are
        /// written in. Deliberately different from the row order above, and from the order
        /// <c>SaveFormToExporterConfiguration</c> persists them in; all three are load bearing.
        /// </summary>
        private static readonly MeshFormat[] ExportOrder =
        {
            MeshFormat.stepSolidworks,
            MeshFormat.stlSolidworks,
            MeshFormat.glbCAD,
            MeshFormat.objCAD,
            MeshFormat.objOpenCascade,
            MeshFormat.stlOpenCascade,
            MeshFormat.glbOpenCascade,
        };

        private static string RowLabel(MeshFormat format)
        {
            switch (format)
            {
                case MeshFormat.stlOpenCascade:
                case MeshFormat.stlSolidworks:
                    return "STL";
                case MeshFormat.glbOpenCascade:
                case MeshFormat.glbCAD:
                    return "GLB";
                case MeshFormat.objOpenCascade:
                case MeshFormat.objCAD:
                    return "OBJ";
                default:
                    return "STEP";
            }
        }

        private static bool IsSuperDexRow(MeshFormat format)
        {
            return format == MeshFormat.stlOpenCascade
                || format == MeshFormat.glbOpenCascade
                || format == MeshFormat.objOpenCascade;
        }

        public MeshExportView()
        {
            Visual = new MeshingColumnView(collision: false);
            Collision = new MeshingColumnView(collision: true);
            Visual.SettingChanged += (s, e) => Raise(MesherSettingChanged);
            Collision.SettingChanged += (s, e) => Raise(MesherSettingChanged);

            meshingOverride.Checked += (s, e) => { ApplyOverrideTint(); Raise(MeshingOverrideChanged); };
            meshingOverride.Unchecked += (s, e) => { ApplyOverrideTint(); Raise(MeshingOverrideChanged); };
            collisionMeshing.Checked += (s, e) => Raise(CollisionMeshingChanged);
            collisionMeshing.Unchecked += (s, e) => Raise(CollisionMeshingChanged);
            resetToDefaults.Click += (s, e) => Raise(ResetToDefaultsRequested);
            exportLinkMesh.Click += (s, e) => Raise(ExportLinkMeshRequested);

            Grid root = new Grid();
            root.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            root.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1.55, GridUnitType.Star) });

            GroupBox formatGroup = MakeGroup("Mesh Format", BuildMeshFormat());
            meshingGroup = MakeGroup("Meshing Options", BuildMeshingOptions());
            meshingGroup.Margin = new Thickness(8, 0, 0, 0);

            Grid.SetColumn(formatGroup, 0);
            Grid.SetColumn(meshingGroup, 1);
            root.Children.Add(formatGroup);
            root.Children.Add(meshingGroup);

            Content = root;
        }

        private void ApplyOverrideTint()
        {
            if (MeshingOverride)
            {
                meshingGroup.Background = OverrideTint;
            }
            else
            {
                meshingGroup.ClearValue(Control.BackgroundProperty);
            }
        }

        private void Raise(EventHandler handler)
        {
            if (handler != null)
            {
                handler(this, EventArgs.Empty);
            }
        }

        private static GroupBox MakeGroup(string header, UIElement content)
        {
            return new GroupBox
            {
                Header = header,
                Content = content,
                Padding = new Thickness(6),
                VerticalAlignment = VerticalAlignment.Top,
            };
        }

        private static TextBlock MakeHeader(string text)
        {
            return new TextBlock
            {
                Text = text,
                FontWeight = FontWeights.Bold,
                HorizontalAlignment = HorizontalAlignment.Left,
                Margin = new Thickness(4, 0, 4, 2),
            };
        }

        private static TextBlock MakeCaption(string text)
        {
            return new TextBlock
            {
                Text = text,
                VerticalAlignment = VerticalAlignment.Center,
                HorizontalAlignment = HorizontalAlignment.Right,
                Margin = new Thickness(0, 0, 8, 0),
            };
        }

        private static void Place(Grid grid, UIElement element, int row, int column)
        {
            Grid.SetRow(element, row);
            Grid.SetColumn(element, column);
            grid.Children.Add(element);
        }

        private UIElement BuildMeshFormat()
        {
            Grid table = new Grid();
            table.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            table.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            table.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });

            table.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            Place(table, MakeHeader("Visual"), 0, 0);
            Place(table, MakeHeader("Collision"), 0, 1);
            Place(table, MakeHeader("Export Method"), 0, 2);

            int row = 0;
            for (int i = 0; i < FormatRows.Length; i++)
            {
                MeshFormat format = FormatRows[i];

                // The SuperDex-meshed and CAD-meshed rows offer the same formats, so a gap keeps
                // the two blocks from reading as one seven-row list.
                if (i > 0 && IsSuperDexRow(FormatRows[i - 1]) && !IsSuperDexRow(format))
                {
                    table.RowDefinitions.Add(new RowDefinition { Height = new GridLength(8) });
                    row++;
                }

                row++;
                table.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });

                CheckBox visualBox = MakeFormatBox(format);
                CheckBox collisionBox = MakeFormatBox(format);
                visualFormatBoxes[format] = visualBox;
                collisionFormatBoxes[format] = collisionBox;

                MeshFormat captured = format;
                visualBox.Checked += (s, e) => RaiseToggle(VisualFormatToggled, captured, true);
                visualBox.Unchecked += (s, e) => RaiseToggle(VisualFormatToggled, captured, false);
                collisionBox.Checked += (s, e) => RaiseToggle(CollisionFormatToggled, captured, true);
                collisionBox.Unchecked += (s, e) => RaiseToggle(CollisionFormatToggled, captured, false);

                TextBlock method = new TextBlock
                {
                    Text = IsSuperDexRow(format) ? "SuperDex" : "",
                    VerticalAlignment = VerticalAlignment.Center,
                    Margin = new Thickness(8, 0, 0, 0),
                };
                methodLabels[format] = method;

                Place(table, visualBox, row, 0);
                Place(table, collisionBox, row, 1);
                Place(table, method, row, 2);
            }

            table.RowDefinitions.Add(new RowDefinition { Height = new GridLength(6) });
            table.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            row += 2;
            visualExtension.Margin = new Thickness(4, 0, 4, 0);
            collisionExtension.Margin = new Thickness(4, 0, 4, 0);
            Place(table, visualExtension, row, 0);
            Place(table, collisionExtension, row, 1);
            TextBlock extensionCaption = new TextBlock
            {
                Text = "<mesh> extension",
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(8, 0, 0, 0),
            };
            Place(table, extensionCaption, row, 2);

            Hyperlink link = new Hyperlink(new Run("Help me choose"));
            link.Click += (s, e) => Raise(HelpMeChooseRequested);
            TextBlock helpText = new TextBlock(link) { Margin = new Thickness(0, 6, 0, 0) };

            Grid folderRow = new Grid { Margin = new Thickness(0, 6, 0, 0) };
            folderRow.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            folderRow.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            folderRow.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            Place(folderRow, MakeCaption("Folder Structure"), 0, 0);
            Place(folderRow, folderStructure, 0, 1);

            StackPanel panel = new StackPanel();
            panel.Children.Add(table);
            panel.Children.Add(helpText);
            panel.Children.Add(folderRow);
            return panel;
        }

        private static CheckBox MakeFormatBox(MeshFormat format)
        {
            return new CheckBox
            {
                Content = RowLabel(format),
                Margin = new Thickness(4, 1, 10, 1),
                VerticalAlignment = VerticalAlignment.Center,
            };
        }

        private void RaiseToggle(
            EventHandler<MeshFormatToggledEventArgs> handler, MeshFormat format, bool isChecked)
        {
            if (handler != null)
            {
                handler(this, new MeshFormatToggledEventArgs { Format = format, IsChecked = isChecked });
            }
        }

        private UIElement BuildMeshingOptions()
        {
            Grid grid = new Grid();
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });

            int row = 0;
            grid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            Place(grid, meshingOverride, row, 0);
            TextBlock visualHeader = MakeHeader("Visual");
            visualHeader.HorizontalAlignment = HorizontalAlignment.Center;
            Place(grid, visualHeader, row, 1);
            collisionMeshing.HorizontalAlignment = HorizontalAlignment.Center;
            Place(grid, collisionMeshing, row, 2);
            row++;

            row = AddColumnRow(grid, row, "Linear Deflection (mm)", Visual.LinearDeflection, Collision.LinearDeflection);
            row = AddColumnRow(grid, row, "Angular Deflection (rad)", Visual.AngularDeflection, Collision.AngularDeflection);

            Grid superdex = new Grid();
            superdex.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            superdex.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            superdex.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            int sub = 0;
            sub = AddColumnRow(superdex, sub, "Scale (multiplier)", Visual.Scale, Collision.Scale);
            sub = AddColumnRow(superdex, sub, "Mesher", Visual.Backend, Collision.Backend);
            sub = AddColumnRow(superdex, sub, "Edge Sampling", Visual.EdgeSampling, Collision.EdgeSampling);
            sub = AddColumnRow(superdex, sub, "Edge Length mm (0=auto)", Visual.TargetEdgeLength, Collision.TargetEdgeLength);
            AddColumnRow(superdex, sub, "Edge Length Fraction", Visual.EdgeLengthFraction, Collision.EdgeLengthFraction);

            superdexGroup = MakeGroup("SuperDex Meshing Options", superdex);
            superdexGroup.Margin = new Thickness(0, 8, 0, 0);
            grid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            Grid.SetRow(superdexGroup, row);
            Grid.SetColumnSpan(superdexGroup, 3);
            grid.Children.Add(superdexGroup);
            row++;

            StackPanel buttons = new StackPanel
            {
                Orientation = Orientation.Horizontal,
                HorizontalAlignment = HorizontalAlignment.Right,
                Margin = new Thickness(0, 8, 0, 0),
            };
            exportLinkMesh.Margin = new Thickness(8, 0, 0, 0);
            buttons.Children.Add(resetToDefaults);
            buttons.Children.Add(exportLinkMesh);
            grid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            Grid.SetRow(buttons, row);
            Grid.SetColumnSpan(buttons, 3);
            grid.Children.Add(buttons);

            return grid;
        }

        private static int AddColumnRow(
            Grid grid, int row, string caption, UIElement visual, UIElement collision)
        {
            grid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            Place(grid, MakeCaption(caption), row, 0);

            FrameworkElement v = visual as FrameworkElement;
            FrameworkElement c = collision as FrameworkElement;
            if (v != null) { v.Margin = new Thickness(0, 1, 6, 1); }
            if (c != null) { c.Margin = new Thickness(6, 1, 0, 1); }

            Place(grid, visual, row, 1);
            Place(grid, collision, row, 2);
            return row + 1;
        }

        /// <summary>
        /// Whether the selected link carries its own meshing options rather than the robot-wide ones.
        /// </summary>
        public bool MeshingOverride
        {
            get { return meshingOverride.IsChecked == true; }
            set { meshingOverride.IsChecked = value; }
        }

        /// <summary>
        /// Links with no geometry to mesh cannot carry an override, so the box is disabled for them
        /// rather than left tickable with no effect.
        /// </summary>
        public bool MeshingOverrideEnabled
        {
            get { return meshingOverride.IsEnabled; }
            set { meshingOverride.IsEnabled = value; }
        }

        public bool CollisionMeshing
        {
            get { return collisionMeshing.IsChecked == true; }
            set { collisionMeshing.IsChecked = value; }
        }

        public string VisualExtension
        {
            get { return visualExtension.Text; }
            set { visualExtension.Text = value; }
        }

        public string CollisionExtension
        {
            get { return collisionExtension.Text; }
            set { collisionExtension.Text = value; }
        }

        public void SetFolderStructures(IEnumerable<string> names)
        {
            folderStructure.Items.Clear();
            foreach (string name in names)
            {
                folderStructure.Items.Add(name);
            }
        }

        public string FolderStructure
        {
            get { return folderStructure.SelectedItem as string; }
            set { folderStructure.SelectedItem = value; }
        }

        public List<MeshFormat> CheckedVisualFormats
        {
            get { return Checked(visualFormatBoxes); }
        }

        public List<MeshFormat> CheckedCollisionFormats
        {
            get { return Checked(collisionFormatBoxes); }
        }

        private static List<MeshFormat> Checked(Dictionary<MeshFormat, CheckBox> boxes)
        {
            List<MeshFormat> formats = new List<MeshFormat>();
            foreach (MeshFormat format in ExportOrder)
            {
                CheckBox box;
                if (boxes.TryGetValue(format, out box) && box.IsChecked == true)
                {
                    formats.Add(format);
                }
            }
            return formats;
        }

        public void ShowVisualFormats(IEnumerable<MeshFormat> formats)
        {
            Show(visualFormatBoxes, formats);
        }

        public void ShowCollisionFormats(IEnumerable<MeshFormat> formats)
        {
            Show(collisionFormatBoxes, formats);
        }

        private static void Show(Dictionary<MeshFormat, CheckBox> boxes, IEnumerable<MeshFormat> formats)
        {
            List<MeshFormat> selected = new List<MeshFormat>(formats);
            foreach (KeyValuePair<MeshFormat, CheckBox> pair in boxes)
            {
                pair.Value.IsChecked = selected.Contains(pair.Key);
            }
        }

        public bool IsVisualFormatChecked(MeshFormat format)
        {
            CheckBox box;
            return visualFormatBoxes.TryGetValue(format, out box) && box.IsChecked == true;
        }

        public bool IsCollisionFormatChecked(MeshFormat format)
        {
            CheckBox box;
            return collisionFormatBoxes.TryGetValue(format, out box) && box.IsChecked == true;
        }

        /// <summary>Labels the host-provided rows and disables the ones this host cannot produce.</summary>
        public void ApplyHostProfile(string cadName, bool cadObjAndGlbSupported, string objRowSuffix)
        {
            foreach (MeshFormat format in FormatRows)
            {
                if (!IsSuperDexRow(format))
                {
                    methodLabels[format].Text = cadName;
                }
            }

            if (objRowSuffix != null)
            {
                methodLabels[MeshFormat.objCAD].Text += objRowSuffix;
            }

            if (!cadObjAndGlbSupported)
            {
                SetRowEnabled(MeshFormat.objCAD, false);
                SetRowEnabled(MeshFormat.glbCAD, false);
            }
        }

        private void SetRowEnabled(MeshFormat format, bool enabled)
        {
            visualFormatBoxes[format].IsEnabled = enabled;
            collisionFormatBoxes[format].IsEnabled = enabled;
            methodLabels[format].IsEnabled = enabled;
        }

        public void UncheckRow(MeshFormat format)
        {
            visualFormatBoxes[format].IsChecked = false;
            collisionFormatBoxes[format].IsChecked = false;
        }

        public void SetRowToolTip(MeshFormat format, string visualTip, string collisionTip)
        {
            visualFormatBoxes[format].ToolTip = visualTip;
            collisionFormatBoxes[format].ToolTip = collisionTip;
        }

        /// <summary>
        /// Whether the visual column has a format that the SuperDex mesher produces, and therefore
        /// whether the SuperDex-only settings in that column would be read at all.
        /// </summary>
        public bool HasSuperDexVisualFormat
        {
            get { return HasSuperDexFormat(visualFormatBoxes); }
        }

        public bool HasSuperDexCollisionFormat
        {
            get { return HasSuperDexFormat(collisionFormatBoxes); }
        }

        private static bool HasSuperDexFormat(Dictionary<MeshFormat, CheckBox> boxes)
        {
            foreach (KeyValuePair<MeshFormat, CheckBox> pair in boxes)
            {
                if (IsSuperDexRow(pair.Key) && pair.Value.IsChecked == true)
                {
                    return true;
                }
            }
            return false;
        }

        public void ApplyEnableState()
        {
            bool visualSuperDex = HasSuperDexVisualFormat;
            bool collisionSuperDex = CollisionMeshing && HasSuperDexCollisionFormat;

            Visual.ApplyEnableState(columnEnabled: true, superDexSelected: visualSuperDex);
            Collision.ApplyEnableState(
                columnEnabled: CollisionMeshing, superDexSelected: collisionSuperDex);

            // A disabled ancestor disables everything beneath it, so this can only go false once
            // neither column has any use for the group.
            superdexGroup.IsEnabled = visualSuperDex || collisionSuperDex;
        }

        public void BoldValuesDifferentFrom(
            Export.MeshingOptions visualBaseline, Export.MeshingOptions collisionBaseline)
        {
            Visual.BoldValuesDifferentFrom(visualBaseline);
            Collision.BoldValuesDifferentFrom(collisionBaseline);
        }
    }

    public class MeshFormatToggledEventArgs : EventArgs
    {
        public MeshFormat Format { get; set; }
        public bool IsChecked { get; set; }
    }
}
