/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Collections.Generic;

using CADRobotExporter.Export;

namespace CADRobotExporter.UI.ViewModels
{
    /// <summary>
    /// The mesh format rules for a single export category (visual or collision).
    ///
    /// Two rules apply: a format may only be produced by one engine at a time, so selecting an
    /// engine deselects its counterpart; and the file extension tracks the highest priority
    /// format still selected, in the order glb, obj, stl, stp. Deselecting the last format
    /// leaves the extension on its previous value rather than blanking it.
    /// </summary>
    public class MeshFormatSelection
    {
        private readonly HashSet<MeshFormat> selected = new HashSet<MeshFormat>();

        public string FileExtension { get; private set; }

        public bool IsSelected(MeshFormat format)
        {
            return selected.Contains(format);
        }

        public IEnumerable<MeshFormat> SelectedFormats()
        {
            return new List<MeshFormat>(selected);
        }

        public void Reset(IEnumerable<MeshFormat> selectedFormats, string fileExtension)
        {
            selected.Clear();
            foreach (MeshFormat format in selectedFormats)
            {
                selected.Add(format);
            }
            FileExtension = fileExtension;
        }

        public void Toggle(MeshFormat format, bool isSelected)
        {
            if (!isSelected)
            {
                selected.Remove(format);
                string remaining = HighestPriorityExtension();
                if (remaining != null)
                {
                    FileExtension = remaining;
                }
                return;
            }

            selected.Add(format);
            FileExtension = ExtensionOf(format);

            MeshFormat otherEngine;
            if (TryGetOtherEngine(format, out otherEngine))
            {
                selected.Remove(otherEngine);
            }
        }

        private string HighestPriorityExtension()
        {
            if (selected.Contains(MeshFormat.glbOpenCascade) || selected.Contains(MeshFormat.glbCAD))
            {
                return "glb";
            }
            if (selected.Contains(MeshFormat.objOpenCascade) || selected.Contains(MeshFormat.objCAD))
            {
                return "obj";
            }
            if (selected.Contains(MeshFormat.stlSolidworks) || selected.Contains(MeshFormat.stlOpenCascade))
            {
                return "stl";
            }
            if (selected.Contains(MeshFormat.stepSolidworks))
            {
                return "stp";
            }
            return null;
        }

        private static string ExtensionOf(MeshFormat format)
        {
            switch (format)
            {
                case MeshFormat.stepSolidworks:
                    return "stp";
                case MeshFormat.objOpenCascade:
                case MeshFormat.objCAD:
                    return "obj";
                case MeshFormat.stlSolidworks:
                case MeshFormat.stlOpenCascade:
                    return "stl";
                default:
                    return "glb";
            }
        }

        private static bool TryGetOtherEngine(MeshFormat format, out MeshFormat otherEngine)
        {
            switch (format)
            {
                case MeshFormat.objOpenCascade:
                    otherEngine = MeshFormat.objCAD;
                    return true;
                case MeshFormat.objCAD:
                    otherEngine = MeshFormat.objOpenCascade;
                    return true;
                case MeshFormat.stlSolidworks:
                    otherEngine = MeshFormat.stlOpenCascade;
                    return true;
                case MeshFormat.stlOpenCascade:
                    otherEngine = MeshFormat.stlSolidworks;
                    return true;
                case MeshFormat.glbOpenCascade:
                    otherEngine = MeshFormat.glbCAD;
                    return true;
                case MeshFormat.glbCAD:
                    otherEngine = MeshFormat.glbOpenCascade;
                    return true;
                default:
                    otherEngine = MeshFormat.stepSolidworks;
                    return false;
            }
        }
    }
}
