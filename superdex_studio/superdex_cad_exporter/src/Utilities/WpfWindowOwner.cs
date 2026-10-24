/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Interop;

namespace CADRobotExporter.Utilities
{
    /// <summary>
    /// The WPF counterpart to <see cref="NativeWindowWrapper"/>. A WPF Window has no
    /// ShowDialog(IWin32Window) overload, so ownership has to be established through
    /// WindowInteropHelper instead.
    ///
    /// Owning a dialog to the host keeps it above the host and lets it hide and restore with the
    /// host, without staying on top of unrelated applications.
    /// </summary>
    [ComVisible(false)]
    public static class WpfWindowOwner
    {
        /// <summary>
        /// Must be called before Show or ShowDialog. WindowInteropHelper.Owner is only honoured
        /// while the window has not yet been created; setting it afterwards silently does nothing.
        /// </summary>
        public static void OwnTo(Window window, IntPtr ownerHandle)
        {
            if (window == null || ownerHandle == IntPtr.Zero)
            {
                return;
            }

            new WindowInteropHelper(window).Owner = ownerHandle;
        }

        /// <summary>
        /// Owns the dialog to an existing WinForms window, which is the usual case while the port
        /// is part way through: a WPF dialog raised from a WinForms form that is itself owned to
        /// the CAD host.
        /// </summary>
        public static void OwnTo(Window window, System.Windows.Forms.IWin32Window owner)
        {
            OwnTo(window, owner == null ? IntPtr.Zero : owner.Handle);
        }
    }
}
