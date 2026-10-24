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
using System.Windows.Media;
using System.Windows.Threading;

namespace CADRobotExporter.UI
{
    /// <summary>
    /// Progress readout for a running export. Lives on its own STA thread with its own dispatcher
    /// (see <see cref="ProgressIndicatorManager"/>) so it stays responsive while the CAD host's
    /// thread is blocked doing the export.
    /// </summary>
    public class ProgressIndicatorWindow : Window
    {
        private readonly ProgressBar progressBar;
        private readonly TextBlock labelStep;
        private readonly TextBlock labelSubSteps;
        private readonly TextBlock labelTimeElapsed;
        private readonly TextBlock labelCancelNotification;
        private readonly DispatcherTimer timer;

        private class Step
        {
            public string Name;
            public string Text;
        }

        // Read from the export thread while the UI thread writes it.
        private volatile bool isCancelled;
        public bool IsCancelled
        {
            get { return isCancelled; }
        }

        private List<Step> steps;
        private List<Step> subSteps;

        private int currentStepIndex;
        private int currentSubStepIndex;

        private DateTime startTime;

        public ProgressIndicatorWindow()
        {
            Theme.FluentTheme.Apply(this);

            Title = "Superdex CAD Exporter is Processing...";
            Topmost = true;
            Width = 520;
            Height = 380;
            ResizeMode = ResizeMode.CanMinimize;
            WindowStartupLocation = WindowStartupLocation.CenterScreen;
            Background = (Brush)Resources["LayerFill"];

            progressBar = new ProgressBar();

            labelTimeElapsed = new TextBlock();
            labelStep = new TextBlock { TextWrapping = TextWrapping.Wrap };
            labelSubSteps = new TextBlock();

            Grid detail = new Grid { Margin = new Thickness(0, 14, 0, 0) };
            detail.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            detail.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            AddDetailRow(detail, 0, "Time Elapsed:", labelTimeElapsed);
            AddDetailRow(detail, 1, "Processing:", labelStep);

            StackPanel progressBody = new StackPanel();
            progressBody.Children.Add(progressBar);
            progressBody.Children.Add(detail);

            GroupBox progressGroup = new GroupBox
            {
                Header = "Progress",
                Padding = new Thickness(10),
                Content = progressBody,
            };

            // The substep list is as long as the export has stages, so it scrolls rather than
            // clipping against this window's fixed height.
            GroupBox stepsGroup = new GroupBox
            {
                Header = "Steps",
                Padding = new Thickness(10),
                Margin = new Thickness(0, 10, 0, 0),
                Content = new ScrollViewer
                {
                    VerticalScrollBarVisibility = ScrollBarVisibility.Auto,
                    HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled,
                    Content = labelSubSteps,
                },
            };

            labelCancelNotification = new TextBlock
            {
                FontWeight = FontWeights.SemiBold,
                VerticalAlignment = VerticalAlignment.Center,
                TextWrapping = TextWrapping.Wrap,
                Margin = new Thickness(0, 0, 12, 0),
            };

            Button buttonCancel = new Button
            {
                Content = "Cancel",
                MinWidth = 110,
            };
            buttonCancel.Click += ButtonCancel_Click;

            Grid footer = new Grid { Margin = new Thickness(0, 14, 0, 0) };
            footer.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            footer.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            Grid.SetColumn(labelCancelNotification, 0);
            Grid.SetColumn(buttonCancel, 1);
            footer.Children.Add(labelCancelNotification);
            footer.Children.Add(buttonCancel);

            Grid root = new Grid { Margin = new Thickness(12) };
            root.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            root.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });
            root.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            Grid.SetRow(progressGroup, 0);
            Grid.SetRow(stepsGroup, 1);
            Grid.SetRow(footer, 2);
            root.Children.Add(progressGroup);
            root.Children.Add(stepsGroup);
            root.Children.Add(footer);

            Content = root;

            timer = new DispatcherTimer(DispatcherPriority.Normal, Dispatcher)
            {
                Interval = TimeSpan.FromMilliseconds(500),
            };
            timer.Tick += OnTick;

            Reset();
        }

        private static void AddDetailRow(Grid grid, int row, string caption, TextBlock value)
        {
            grid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });

            TextBlock captionBlock = new TextBlock
            {
                Text = caption,
                Margin = new Thickness(0, 0, 16, 4),
            };
            Grid.SetRow(captionBlock, row);
            Grid.SetColumn(captionBlock, 0);
            grid.Children.Add(captionBlock);

            value.Margin = new Thickness(0, 0, 0, 4);
            Grid.SetRow(value, row);
            Grid.SetColumn(value, 1);
            grid.Children.Add(value);
        }

        private void ButtonCancel_Click(object sender, RoutedEventArgs e)
        {
            labelCancelNotification.Text = "Attemping to cancel, please wait...";
            labelStep.Foreground = Brushes.Gray;
            labelSubSteps.Foreground = Brushes.Gray;
            progressBar.Foreground = Brushes.Gray;
            isCancelled = true;
        }

        public void Reset()
        {
            isCancelled = false;
            steps = new List<Step>();
            subSteps = new List<Step>();
            SetProgress(0, 10);
            labelSubSteps.Text = "";
            labelStep.Text = "";
            labelTimeElapsed.Text = "";
        }

        public void StartTimer()
        {
            startTime = DateTime.Now;
            timer.Start();
        }

        private void OnTick(object sender, EventArgs e)
        {
            TimeSpan elapsed = DateTime.Now - startTime;
            int minutes = (int)elapsed.TotalMinutes;
            int seconds = elapsed.Seconds;
            if (minutes > 0)
            {
                labelTimeElapsed.Text = $"{minutes} minutes {seconds} seconds";
            }
            else
            {
                labelTimeElapsed.Text = $"{seconds} seconds";
            }
        }

        public void AddStep(string name, string text)
        {
            steps.Add(new Step { Name = name, Text = text });
        }

        public void AddSubStep(string name, string text)
        {
            subSteps.Add(new Step { Name = name, Text = text });
        }

        public void SetCurrentStep(string name)
        {
            Step step = steps.Find(st => st.Name == name);
            if (step != null)
            {
                currentStepIndex = steps.IndexOf(step);
                currentSubStepIndex = 0;
                UpdateProgress();
            }
        }

        public void SetCurrentSubstep(string name)
        {
            Step step = subSteps.Find(st => st.Name == name);
            if (step != null)
            {
                currentSubStepIndex = subSteps.IndexOf(step);
                UpdateProgress();
            }
        }

        private void SetProgress(int value, int max)
        {
            progressBar.Maximum = max;
            progressBar.Value = value;
        }

        private void UpdateProgress()
        {
            int maxValue = steps.Count * Math.Max(1, subSteps.Count);
            int currentValue = currentStepIndex * subSteps.Count + currentSubStepIndex;

            labelStep.Text = $"{steps[currentStepIndex].Text} ({currentStepIndex + 1} of {steps.Count})";

            string subStepText = "";
            for (int i = 0; i < subSteps.Count; i++)
            {
                bool isDone = currentSubStepIndex > i;

                string line = "";
                line += isDone ? "✅ " : "⌛ ";
                line += subSteps[i].Text;
                line += currentSubStepIndex == i ? " ←" : "";
                subStepText += line + "\n";
            }

            labelSubSteps.Text = subStepText;
            SetProgress(currentValue, maxValue);
        }
    }
}
