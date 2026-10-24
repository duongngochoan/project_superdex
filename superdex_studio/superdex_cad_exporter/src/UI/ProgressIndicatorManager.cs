/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows.Threading;

namespace CADRobotExporter.UI
{
    /// <summary>
    /// Owns the progress window and the dedicated STA thread it runs on. The export runs on the
    /// CAD host's thread and blocks it, so the window needs its own thread and its own dispatcher
    /// to keep painting and to keep the Cancel button responsive.
    ///
    /// Every mutating call marshals onto that thread; only <see cref="IsCancelled"/> is read
    /// directly, and that is backed by a volatile field.
    /// </summary>
    public class ProgressIndicatorManager : IDisposable
    {
        private ProgressIndicatorWindow window;
        private Thread uiThread;
        private readonly ManualResetEvent windowReady = new ManualResetEvent(false);
        private volatile bool isClosed;
        private bool isDisposed;

        public void Show()
        {
            if (uiThread != null && uiThread.IsAlive)
            {
                return;
            }

            windowReady.Reset();
            isClosed = false;

            uiThread = new Thread(() =>
            {
                window = new ProgressIndicatorWindow();

                // SourceInitialized is the WPF counterpart to HandleCreated: the HWND exists and
                // the dispatcher is pumping, so it is safe to marshal work in.
                window.SourceInitialized += (s, e) => windowReady.Set();
                window.Closed += (s, e) =>
                {
                    isClosed = true;
                    Dispatcher.CurrentDispatcher.BeginInvokeShutdown(DispatcherPriority.Background);
                };

                window.Show();
                Dispatcher.Run();
            });

            uiThread.SetApartmentState(ApartmentState.STA);
            uiThread.IsBackground = true;
            uiThread.Start();

            // Wait for the window to be ready
            windowReady.WaitOne();
            StartTimer();
        }

        public void Close()
        {
            InvokeOnWindow(() => window.Close());
        }

        public void Reset()
        {
            InvokeOnWindow(() => window.Reset());
        }

        public void AddStep(string name, string text)
        {
            InvokeOnWindow(() => window.AddStep(name, text));
        }

        public void AddSteps(IEnumerable<(string name, string text)> steps)
        {
            InvokeOnWindow(() =>
            {
                foreach (var (name, text) in steps)
                {
                    window.AddStep(name, text);
                }
            });
        }

        public void AddSubStep(string name, string text)
        {
            InvokeOnWindow(() => window.AddSubStep(name, text));
        }

        public void AddSubSteps(IEnumerable<(string name, string text)> subSteps)
        {
            InvokeOnWindow(() =>
            {
                foreach (var (name, text) in subSteps)
                {
                    window.AddSubStep(name, text);
                }
            });
        }

        public void SetCurrentStep(string name)
        {
            InvokeOnWindow(() => window.SetCurrentStep(name));
        }

        public void SetCurrentSubStep(string name)
        {
            InvokeOnWindow(() => window.SetCurrentSubstep(name));
        }

        public void StartTimer()
        {
            InvokeOnWindow(() => window.StartTimer());
        }

        private void InvokeOnWindow(Action action)
        {
            ProgressIndicatorWindow target = window;
            if (target == null || isClosed)
            {
                return;
            }

            Dispatcher dispatcher = target.Dispatcher;
            if (dispatcher.HasShutdownStarted || dispatcher.HasShutdownFinished)
            {
                return;
            }

            if (dispatcher.CheckAccess())
            {
                action();
            }
            else
            {
                dispatcher.Invoke(action);
            }
        }

        public bool IsCancelled
        {
            get
            {
                ProgressIndicatorWindow target = window;
                if (target == null || isClosed)
                {
                    return true;  // Treat as cancelled if the window is gone
                }

                return target.IsCancelled;
            }
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (isDisposed)
            {
                return;
            }

            Close();
            windowReady.Dispose();

            isDisposed = true;
        }
    }
}
