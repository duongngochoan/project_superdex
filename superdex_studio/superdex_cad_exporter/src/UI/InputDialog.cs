/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Windows;
using System.Windows.Controls;

namespace CADRobotExporter.UI
{
    public class InputDialog : Window
    {
        private readonly TextBox textBox;

        public string InputText
        {
            get { return textBox.Text; }
        }

        public InputDialog(string title, string prompt)
        {
            Theme.FluentTheme.Apply(this);

            Title = title;
            Topmost = true;
            Width = 300;
            Height = 150;
            ResizeMode = ResizeMode.NoResize;
            WindowStartupLocation = WindowStartupLocation.CenterOwner;
            ShowInTaskbar = false;

            textBox = new TextBox { Margin = new Thickness(0, 8, 0, 0) };

            Button okButton = new Button
            {
                Content = "OK",
                IsDefault = true,
                Width = 75,
                Height = 23,
            };
            okButton.Click += (sender, e) => { DialogResult = true; };

            Button cancelButton = new Button
            {
                Content = "Cancel",
                IsCancel = true,
                Width = 75,
                Height = 23,
                Margin = new Thickness(8, 0, 0, 0),
            };

            StackPanel buttons = new StackPanel
            {
                Orientation = Orientation.Horizontal,
                HorizontalAlignment = HorizontalAlignment.Right,
                Margin = new Thickness(0, 16, 0, 0),
            };
            buttons.Children.Add(okButton);
            buttons.Children.Add(cancelButton);

            StackPanel root = new StackPanel { Margin = new Thickness(10) };
            root.Children.Add(new TextBlock { Text = prompt });
            root.Children.Add(textBox);
            root.Children.Add(buttons);

            Content = root;

            Loaded += (sender, e) => textBox.Focus();
        }
    }
}
