/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

using Microsoft.Win32;

namespace CADRobotExporter.Utilities
{
    /// <summary>
    /// Recovers the directory last browsed to by the host CAD application through a common
    /// file dialog, by decoding the shell's LastVisitedPidlMRU registry blob. This reproduces
    /// what a dialog's RestoreDirectory would give us, but for a dialog owned by the host
    /// process rather than by us.
    /// </summary>
    [ComVisible(false)]
    public static class ShellMruDirectory
    {
        // This essentially pulls the "RestoreDirectory = true" directory
        private const string LastVisitedMruPath =
            @"Software\Microsoft\Windows\CurrentVersion\Explorer\ComDlg32\LastVisitedPidlMRU";

        [DllImport("shell32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool SHGetPathFromIDListW(
            IntPtr pidl, [MarshalAs(UnmanagedType.LPTStr)] StringBuilder pszPath);

        /// <summary>
        /// Returns the last directory the host application browsed to, or null if it
        /// cannot be determined.
        /// </summary>
        public static string Get()
        {
            try
            {
                using (RegistryKey rk = Registry.CurrentUser.OpenSubKey(LastVisitedMruPath))
                {
                    if (rk == null)
                        return null;

                    byte[] mruList = rk.GetValue("MRUListEx") as byte[];
                    if (mruList == null || mruList.Length < 4)
                        return null;

                    // Walk the MRU list looking for ugraf.exe entries
                    for (int i = 0; i < mruList.Length - 4; i += 4)
                    {
                        int index = BitConverter.ToInt32(mruList, i);
                        if (index == -1)
                            break;

                        byte[] entry = rk.GetValue(index.ToString()) as byte[];
                        if (entry == null || entry.Length < 4)
                            continue;

                        // Entry format: null-terminated Unicode exe name, followed by PIDL
                        string exeName = Encoding.Unicode.GetString(entry, 0, entry.Length);
                        int nullIdx = exeName.IndexOf('\0');
                        if (nullIdx < 0)
                            continue;

                        string app = exeName.Substring(0, nullIdx);
#if NX
                        if (!app.Equals("ugraf.exe", StringComparison.OrdinalIgnoreCase))
#elif SOLIDWORKS
                        if (!app.Equals("SLDWORKS.exe", StringComparison.OrdinalIgnoreCase))
#endif
                        continue;

                        // PIDL starts after the null-terminated exe name (double-null in Unicode)
                        int pidlOffset = (nullIdx + 1) * 2;
                        if (pidlOffset >= entry.Length)
                            continue;

                        byte[] pidlBytes = new byte[entry.Length - pidlOffset];
                        Array.Copy(entry, pidlOffset, pidlBytes, 0, pidlBytes.Length);

                        GCHandle handle = GCHandle.Alloc(pidlBytes, GCHandleType.Pinned);
                        try
                        {
                            StringBuilder path = new StringBuilder(260);
                            if (SHGetPathFromIDListW(handle.AddrOfPinnedObject(), path))
                            {
                                string dir = path.ToString();
                                if (Directory.Exists(dir))
                                    return dir;
                            }
                        }
                        finally
                        {
                            handle.Free();
                        }
                    }
                }
            }
            catch
            {
            }

            return null;
        }
    }
}
