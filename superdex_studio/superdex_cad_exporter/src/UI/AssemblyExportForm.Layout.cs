/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Media;

using CADRobotExporter.UI.Views;


namespace CADRobotExporter.UI
{
    /// <summary>
    /// Replaces the generated WinForms designer. The original placed every control at an absolute
    /// <c>Point</c> and set a 1622x1207 minimum, which is taller than a 1080p display; this layout
    /// is authored to reflow instead.
    /// </summary>
    public partial class AssemblyExportForm
    {
        private TabControl tabControl;
        private TabItem tabPageJointProperties;
        private TabItem tabPageLinkProperties;
        private TabItem tabPageKinematicsSummary;
        private TabItem tabPageTendonsSummary;

        private TreeView treeViewJointTree;
        private TreeView treeViewLinkProperties;

        private CheckBox checkBoxShowJointVisualization;
        private CheckBox checkBoxShowJointHighlights;
        private CheckBox checkBoxShowLinkVisualization;
        private CheckBox checkBoxLinkHighlights;
        private Slider trackBarJointGizmoSize;
        private Slider trackBarLinkGizmoSize;

        private TextBox textBoxRobotName;
        private Button buttonPrevious;
        private Button buttonNext;
        private Button buttonClose;
        private Button buttonLinksFinish;
        private Button buttonLinksExportUrdfOnly;

        private void InitializeComponent()
        {
            Theme.FluentTheme.Apply(this);

            Title = "SuperDex CAD Exporter - Export";
            Width = 1100;
            Height = 850;
            MinWidth = 1100;
            MinHeight = 700;
            Background = (System.Windows.Media.Brush)Resources["LayerFill"];
            WindowStartupLocation = WindowStartupLocation.CenterOwner;
            SnapsToDevicePixels = true;
            UseLayoutRounding = true;

            tabControl = new TabControl { Margin = new Thickness(8, 0, 8, 0) };
            tabPageJointProperties = new TabItem { Header = "Joint Properties" };
            tabPageLinkProperties = new TabItem { Header = "Link Properties" };
            tabPageKinematicsSummary = new TabItem { Header = "Kinematics Summary" };
            tabPageTendonsSummary = new TabItem { Header = "Tendons Summary" };

            tabControl.Items.Add(tabPageJointProperties);
            tabControl.Items.Add(tabPageLinkProperties);
            tabControl.Items.Add(tabPageKinematicsSummary);
            tabControl.Items.Add(tabPageTendonsSummary);

            treeViewJointTree = MakeTree(markMeshingOverrides: false);
            treeViewLinkProperties = MakeTree(markMeshingOverrides: true);

            checkBoxShowJointVisualization = new CheckBox();
            checkBoxShowJointHighlights = new CheckBox();
            checkBoxShowLinkVisualization = new CheckBox();
            checkBoxLinkHighlights = new CheckBox();
            trackBarJointGizmoSize = new Slider();
            trackBarLinkGizmoSize = new Slider();

            Grid root = new Grid();
            root.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            root.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });
            root.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });

            UIElement header = BuildHeader();
            Grid.SetRow(header, 0);
            root.Children.Add(header);

            Grid.SetRow(tabControl, 1);
            root.Children.Add(tabControl);

            UIElement footer = BuildFooter();
            Grid.SetRow(footer, 2);
            root.Children.Add(footer);

            Content = root;
        }

        /// <summary>
        /// Left column: heading, tree, visualization box. Right column: the ported WPF panel.
        /// A splitter lets the tree give way on narrow displays.
        /// </summary>
        private static Grid BuildTreeAndPanel(
            string heading, string hint, UIElement tree, UIElement visualization, UIElement panel)
        {
            Grid grid = new Grid { Margin = new Thickness(8) };
            grid.ColumnDefinitions.Add(new ColumnDefinition
            {
                Width = new GridLength(1, GridUnitType.Star),
                MinWidth = 220,
            });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            grid.ColumnDefinitions.Add(new ColumnDefinition
            {
                Width = new GridLength(2.4, GridUnitType.Star),
                MinWidth = 520,
            });

            Grid left = new Grid();
            left.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            left.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });
            left.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });

            StackPanel headings = new StackPanel { Margin = new Thickness(0, 0, 0, 4) };
            headings.Children.Add(new TextBlock
            {
                Text = heading,
                FontWeight = FontWeights.Bold,
                FontSize = 14,
            });
            if (!string.IsNullOrEmpty(hint))
            {
                headings.Children.Add(new TextBlock
                {
                    Text = hint,
                    TextWrapping = TextWrapping.Wrap,
                    Margin = new Thickness(0, 2, 0, 0),
                    Opacity = 0.8,
                });
            }
            Grid.SetRow(headings, 0);
            left.Children.Add(headings);

            Grid.SetRow(tree, 1);
            left.Children.Add(tree);

            if (visualization != null)
            {
                Grid.SetRow(visualization, 2);
                left.Children.Add(visualization);
            }

            GridSplitter splitter = new GridSplitter
            {
                Width = 6,
                HorizontalAlignment = HorizontalAlignment.Stretch,
                VerticalAlignment = VerticalAlignment.Stretch,
                Background = System.Windows.Media.Brushes.Transparent,
            };

            Grid.SetColumn(left, 0);
            Grid.SetColumn(splitter, 1);
            Grid.SetColumn((UIElement)panel, 2);
            grid.Children.Add(left);
            grid.Children.Add(splitter);
            grid.Children.Add(panel);
            return grid;
        }

        /// <summary>
        /// The template is built here rather than in the theme because it binds to
        /// <see cref="LinkTreeItem"/>, which is application data, not chrome.
        /// </summary>
        /// <param name="markMeshingOverrides">
        /// Bolds links that carry a meshing override. Only the link tree asks for this: both trees
        /// wrap the same <c>Link</c> objects, so binding it unconditionally would bold the joint tree
        /// too, where meshing is not on show.
        /// </param>
        private static TreeView MakeTree(bool markMeshingOverrides)
        {
            FrameworkElementFactory label = new FrameworkElementFactory(typeof(TextBlock));
            label.SetBinding(TextBlock.TextProperty, new Binding("DisplayName"));

            Style labelStyle = new Style(typeof(TextBlock));

            if (markMeshingOverrides)
            {
                AddWeightTrigger(labelStyle, selected: true, overridden: false, FontWeights.SemiBold);
                AddWeightTrigger(labelStyle, selected: false, overridden: true, FontWeights.Bold);
                AddWeightTrigger(labelStyle, selected: true, overridden: true, FontWeights.Bold);
            }
            else
            {
                DataTrigger selected = new DataTrigger
                {
                    Binding = new Binding("IsSelected"),
                    Value = true,
                };
                selected.Setters.Add(
                    new Setter(TextBlock.FontWeightProperty, FontWeights.SemiBold));
                labelStyle.Triggers.Add(selected);
            }

            label.SetValue(FrameworkElement.StyleProperty, labelStyle);

            HierarchicalDataTemplate template = new HierarchicalDataTemplate(typeof(LinkTreeItem))
            {
                ItemsSource = new Binding("Children"),
                VisualTree = label,
            };
            template.Seal();

            return new TreeView { ItemTemplate = template };
        }

        /// <summary>
        /// Bold marks a link whose meshing properties are its own; semi-bold marks the node whose
        /// properties the panel on the right is showing.
        /// </summary>
        private static void AddWeightTrigger(
            Style style, bool selected, bool overridden, FontWeight weight)
        {
            MultiDataTrigger trigger = new MultiDataTrigger();
            trigger.Conditions.Add(new Condition(new Binding("IsSelected"), selected));
            trigger.Conditions.Add(new Condition(new Binding("HasMeshingOverride"), overridden));
            trigger.Setters.Add(new Setter(TextBlock.FontWeightProperty, weight));
            style.Triggers.Add(trigger);
        }

        private GroupBox BuildVisualization(
            string visibilityCaption,
            CheckBox visibility,
            CheckBox highlights,
            Slider gizmoSize)
        {
            visibility.Content = visibilityCaption;
            visibility.IsChecked = true;
            highlights.Content = "Highlight Selected Bodies";
            highlights.IsChecked = true;

            gizmoSize.Minimum = 1;
            gizmoSize.Maximum = 20;
            gizmoSize.Value = 10;
            gizmoSize.TickFrequency = 1;
            gizmoSize.IsSnapToTickEnabled = true;
            gizmoSize.MinWidth = 140;
            gizmoSize.VerticalAlignment = VerticalAlignment.Center;

            Grid sizeRow = new Grid { Margin = new Thickness(0, 4, 0, 0) };
            sizeRow.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            sizeRow.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            TextBlock sizeCaption = new TextBlock
            {
                Text = "Gizmo Size",
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(0, 0, 8, 0),
            };
            Grid.SetColumn(sizeCaption, 0);
            Grid.SetColumn(gizmoSize, 1);
            sizeRow.Children.Add(sizeCaption);
            sizeRow.Children.Add(gizmoSize);

            StackPanel body = new StackPanel();
            visibility.Margin = new Thickness(0, 1, 0, 1);
            highlights.Margin = new Thickness(0, 1, 0, 1);
            body.Children.Add(visibility);
            body.Children.Add(highlights);
            body.Children.Add(sizeRow);

            return new GroupBox
            {
                Header = "Visualization",
                Padding = new Thickness(6),
                Margin = new Thickness(0, 6, 0, 0),
                Content = body,
            };
        }

        /// <summary>
        /// The robot name applies to the whole export rather than to any one tab, so it sits above
        /// the tab strip instead of in the footer alongside the navigation buttons.
        /// </summary>
        private UIElement BuildHeader()
        {
            textBoxRobotName = new TextBox
            {
                MinWidth = 240,
                VerticalAlignment = VerticalAlignment.Center,
            };

            StackPanel namePanel = new StackPanel
            {
                Orientation = Orientation.Horizontal,
                HorizontalAlignment = HorizontalAlignment.Right,
                Margin = new Thickness(8, 6, 8, 6),
            };
            namePanel.Children.Add(new TextBlock
            {
                Text = "Robot Name",
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(0, 0, 8, 0),
            });
            namePanel.Children.Add(textBoxRobotName);
            return namePanel;
        }

        private UIElement BuildFooter()
        {
            buttonPrevious = MakeFooterButton("Previous");
            buttonNext = MakeFooterButton("Next");
            buttonLinksExportUrdfOnly = MakeFooterButton("Export Robot Only...");
            buttonLinksFinish = MakeFooterButton("Export Robot and Meshes...");
            buttonLinksFinish.Style = (Style)Resources[Theme.FluentTheme.AccentButtonKey];
            buttonClose = MakeFooterButton("Close");

            StackPanel buttons = new StackPanel
            {
                Orientation = Orientation.Horizontal,
                HorizontalAlignment = HorizontalAlignment.Right,
                Margin = new Thickness(8, 6, 8, 8),
            };
            buttons.Children.Add(buttonPrevious);
            buttons.Children.Add(buttonNext);
            buttons.Children.Add(buttonLinksExportUrdfOnly);
            buttons.Children.Add(buttonLinksFinish);
            buttons.Children.Add(buttonClose);

            return buttons;
        }

        private void PopulateTabs(
            UIElement jointPanel,
            UIElement linkPanel,
            UIElement kinematicsPanel,
            UIElement tendonsPanel)
        {
            tabPageJointProperties.Content = BuildTreeAndPanel(
                "Configure Joint Properties",
                "Customize the joint properties. If you want to adjust the coordinate systems and "
                    + "axes in the model, click cancel and restart the export. The tool will recognize "
                    + "your changes on the next run.",
                treeViewJointTree,
                BuildVisualization(
                    "Show Joint Axis and Limits",
                    checkBoxShowJointVisualization,
                    checkBoxShowJointHighlights,
                    trackBarJointGizmoSize),
                jointPanel);

            tabPageLinkProperties.Content = BuildTreeAndPanel(
                "Configure Link Properties",
                "Use this page to make any changes to the links' properties",
                treeViewLinkProperties,
                BuildVisualization(
                    "Show Inertia and Center of Mass",
                    checkBoxShowLinkVisualization,
                    checkBoxLinkHighlights,
                    trackBarLinkGizmoSize),
                linkPanel);

            tabPageKinematicsSummary.Content = kinematicsPanel;
            tabPageTendonsSummary.Content = tendonsPanel;
        }

        private static Button MakeFooterButton(string caption)
        {
            return new Button
            {
                Content = caption,
                Padding = new Thickness(12, 3, 12, 3),
                Margin = new Thickness(6, 0, 0, 0),
                MinWidth = 90,
            };
        }
    }
}
