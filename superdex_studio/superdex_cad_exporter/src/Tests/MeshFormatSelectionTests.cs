/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using CADRobotExporter.Export;
using CADRobotExporter.UI.ViewModels;
using Xunit;

namespace CADRobotExporter.Tests
{
    /// <summary>
    /// Covers the mesh format mutual-exclusion and file-extension rules that were previously
    /// duplicated across the visual and collision CheckedChanged handlers, using CheckBox.Checked
    /// as their only state store.
    /// </summary>
    public class MeshFormatSelectionTests
    {
        private static MeshFormatSelection Empty(string extension = "stl")
        {
            MeshFormatSelection selection = new MeshFormatSelection();
            selection.Reset(new MeshFormat[0], extension);
            return selection;
        }

        [Theory]
        [InlineData(MeshFormat.stepSolidworks, "stp")]
        [InlineData(MeshFormat.objOpenCascade, "obj")]
        [InlineData(MeshFormat.objCAD, "obj")]
        [InlineData(MeshFormat.stlSolidworks, "stl")]
        [InlineData(MeshFormat.stlOpenCascade, "stl")]
        [InlineData(MeshFormat.glbOpenCascade, "glb")]
        [InlineData(MeshFormat.glbCAD, "glb")]
        public void SelectingAFormat_SetsMatchingExtension(MeshFormat format, string expected)
        {
            MeshFormatSelection selection = Empty();

            selection.Toggle(format, true);

            Assert.True(selection.IsSelected(format));
            Assert.Equal(expected, selection.FileExtension);
        }

        [Theory]
        [InlineData(MeshFormat.objOpenCascade, MeshFormat.objCAD)]
        [InlineData(MeshFormat.objCAD, MeshFormat.objOpenCascade)]
        [InlineData(MeshFormat.stlSolidworks, MeshFormat.stlOpenCascade)]
        [InlineData(MeshFormat.stlOpenCascade, MeshFormat.stlSolidworks)]
        [InlineData(MeshFormat.glbOpenCascade, MeshFormat.glbCAD)]
        [InlineData(MeshFormat.glbCAD, MeshFormat.glbOpenCascade)]
        public void SelectingOneEngine_DeselectsTheOtherEngineForThatFormat(
            MeshFormat chosen, MeshFormat other)
        {
            MeshFormatSelection selection = Empty();
            selection.Toggle(other, true);

            selection.Toggle(chosen, true);

            Assert.True(selection.IsSelected(chosen));
            Assert.False(selection.IsSelected(other));
        }

        [Fact]
        public void DifferentFormatsCanBeSelectedTogether()
        {
            MeshFormatSelection selection = Empty();

            selection.Toggle(MeshFormat.stlSolidworks, true);
            selection.Toggle(MeshFormat.objCAD, true);

            Assert.True(selection.IsSelected(MeshFormat.stlSolidworks));
            Assert.True(selection.IsSelected(MeshFormat.objCAD));
        }

        [Fact]
        public void StepHasNoCounterpartAndDeselectsNothing()
        {
            MeshFormatSelection selection = Empty();
            selection.Toggle(MeshFormat.glbCAD, true);

            selection.Toggle(MeshFormat.stepSolidworks, true);

            Assert.True(selection.IsSelected(MeshFormat.stepSolidworks));
            Assert.True(selection.IsSelected(MeshFormat.glbCAD));
            Assert.Equal("stp", selection.FileExtension);
        }

        [Fact]
        public void DeselectingFallsBackToTheHighestPriorityRemainingFormat()
        {
            MeshFormatSelection selection = Empty();
            selection.Toggle(MeshFormat.stlSolidworks, true);
            selection.Toggle(MeshFormat.glbCAD, true);
            Assert.Equal("glb", selection.FileExtension);

            selection.Toggle(MeshFormat.glbCAD, false);

            Assert.Equal("stl", selection.FileExtension);
        }

        [Fact]
        public void FallbackPriorityIsGlbThenObjThenStlThenStp()
        {
            MeshFormatSelection selection = Empty();
            selection.Toggle(MeshFormat.stepSolidworks, true);
            selection.Toggle(MeshFormat.stlSolidworks, true);
            selection.Toggle(MeshFormat.objCAD, true);
            selection.Toggle(MeshFormat.glbCAD, true);

            Assert.Equal("glb", selection.FileExtension);

            selection.Toggle(MeshFormat.glbCAD, false);
            Assert.Equal("obj", selection.FileExtension);

            selection.Toggle(MeshFormat.objCAD, false);
            Assert.Equal("stl", selection.FileExtension);

            selection.Toggle(MeshFormat.stlSolidworks, false);
            Assert.Equal("stp", selection.FileExtension);
        }

        /// <summary>
        /// Deselecting the last remaining format leaves the extension alone rather than clearing
        /// it, so the exporter never ends up with an empty extension.
        /// </summary>
        [Fact]
        public void DeselectingTheLastFormat_LeavesTheExtensionUnchanged()
        {
            MeshFormatSelection selection = Empty();
            selection.Toggle(MeshFormat.glbCAD, true);
            Assert.Equal("glb", selection.FileExtension);

            selection.Toggle(MeshFormat.glbCAD, false);

            Assert.Equal("glb", selection.FileExtension);
        }

        [Fact]
        public void DeselectingTheOtherEngine_KeepsTheFormatExtension()
        {
            MeshFormatSelection selection = Empty();
            selection.Toggle(MeshFormat.objCAD, true);
            selection.Toggle(MeshFormat.objOpenCascade, true);

            // ObjCad was already dropped when ObjOpenCascade won; an echo must not change anything.
            selection.Toggle(MeshFormat.objCAD, false);

            Assert.True(selection.IsSelected(MeshFormat.objOpenCascade));
            Assert.Equal("obj", selection.FileExtension);
        }

        [Fact]
        public void Reset_ReplacesTheWholeSelection()
        {
            MeshFormatSelection selection = Empty();
            selection.Toggle(MeshFormat.glbCAD, true);

            selection.Reset(new[] { MeshFormat.stlSolidworks }, "stl");

            Assert.False(selection.IsSelected(MeshFormat.glbCAD));
            Assert.True(selection.IsSelected(MeshFormat.stlSolidworks));
            Assert.Equal("stl", selection.FileExtension);
        }
    }
}
