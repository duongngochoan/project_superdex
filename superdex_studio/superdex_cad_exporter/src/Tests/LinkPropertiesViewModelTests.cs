/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI.ViewModels;
using Xunit;

namespace CADRobotExporter.Tests
{
    /// <summary>
    /// Covers the link properties page mapping previously inlined in AssemblyExportFormExtension.
    /// </summary>
    public class LinkPropertiesViewModelTests
    {
        private static Link PopulatedLink()
        {
            Link link = new Link { Name = "arm" };
            link.Inertial.Origin.X = 1.5;
            link.Inertial.Origin.Y = 2.5;
            link.Inertial.Origin.Z = 3.5;
            link.Inertial.Origin.Roll = 0.1;
            link.Inertial.Origin.Pitch = 0.2;
            link.Inertial.Origin.Yaw = 0.3;
            link.Visual.Origin.X = 4.5;
            link.Visual.Origin.Y = 5.5;
            link.Visual.Origin.Z = 6.5;
            link.Visual.Origin.Roll = 0.4;
            link.Visual.Origin.Pitch = 0.5;
            link.Visual.Origin.Yaw = 0.6;
            link.Inertial.Mass.Value = 7.25;
            link.Inertial.Inertia.Ixx = 1.0;
            link.Inertial.Inertia.Ixy = 2.0;
            link.Inertial.Inertia.Ixz = 3.0;
            link.Inertial.Inertia.Iyy = 4.0;
            link.Inertial.Inertia.Iyz = 5.0;
            link.Inertial.Inertia.Izz = 6.0;
            link.Visual.Material.Color.Red = 0.25;
            link.Visual.Material.Color.Green = 0.5;
            link.Visual.Material.Color.Blue = 0.75;
            link.Visual.Material.Color.Alpha = 1.0;
            return link;
        }

        [Fact]
        public void LoadFrom_MarksAPopulatedLinkEditable()
        {
            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();

            properties.LoadFrom(PopulatedLink());

            Assert.True(properties.IsEditable);
        }

        [Fact]
        public void LoadFrom_TreatsANullLinkAsNotEditable()
        {
            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();

            properties.LoadFrom(null);

            Assert.False(properties.IsEditable);
        }

        /// <summary>A fixed frame carries no editable link properties.</summary>
        [Fact]
        public void LoadFrom_TreatsAFixedFrameAsNotEditable()
        {
            Link link = PopulatedLink();
            link.isFixedFrame = true;
            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();

            properties.LoadFrom(link);

            Assert.False(properties.IsEditable);
            Assert.Equal(string.Empty, properties.Mass);
        }

        [Fact]
        public void SaveTo_LeavesAFixedFrameUntouched()
        {
            Link link = PopulatedLink();
            link.isFixedFrame = true;
            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();
            properties.Clear();
            properties.Mass = "999";

            properties.SaveTo(link);

            Assert.Equal(7.25, link.Inertial.Mass.Value);
        }

        /// <summary>
        /// The whole page must survive a load/save cycle, since navigating between links runs one
        /// of each on every selection change.
        /// </summary>
        [Fact]
        public void LoadThenSave_RoundTripsEveryValue()
        {
            Link source = PopulatedLink();
            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();
            properties.LoadFrom(source);

            Link target = new Link { Name = "target" };
            properties.SaveTo(target);

            Assert.Equal(1.5, target.Inertial.Origin.X);
            Assert.Equal(0.3, target.Inertial.Origin.Yaw);
            Assert.Equal(4.5, target.Visual.Origin.X);
            Assert.Equal(0.6, target.Visual.Origin.Yaw);
            Assert.Equal(7.25, target.Inertial.Mass.Value);
            Assert.Equal(1.0, target.Inertial.Inertia.Ixx);
            Assert.Equal(6.0, target.Inertial.Inertia.Izz);
            Assert.Equal(0.25, target.Visual.Material.Color.Red);
            Assert.Equal(1.0, target.Visual.Material.Color.Alpha);
        }

        /// <summary>
        /// "R" round-trips a double exactly, so a value shown and re-saved must not drift.
        /// </summary>
        [Fact]
        public void LoadThenSave_PreservesFullDoublePrecision()
        {
            Link source = PopulatedLink();
            source.Inertial.Mass.Value = 0.1 + 0.2;
            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();
            properties.LoadFrom(source);

            Link target = new Link { Name = "target" };
            properties.SaveTo(target);

            Assert.Equal(source.Inertial.Mass.Value, target.Inertial.Mass.Value);
        }

        /// <summary>
        /// Text that is not a number must leave the model value alone rather than zeroing it.
        /// </summary>
        [Fact]
        public void SaveTo_IgnoresUnparseableText()
        {
            Link link = PopulatedLink();
            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();
            properties.LoadFrom(link);
            properties.Mass = "not a number";
            properties.Inertia.Ixx = string.Empty;

            properties.SaveTo(link);

            Assert.Equal(7.25, link.Inertial.Mass.Value);
            Assert.Equal(1.0, link.Inertial.Inertia.Ixx);
        }

        [Fact]
        public void OriginFields_LoadFromNullLeavesEveryFieldBlank()
        {
            OriginFields fields = new OriginFields();

            fields.LoadFrom(null);

            Assert.Equal(string.Empty, fields.X);
            Assert.Equal(string.Empty, fields.Yaw);
        }

        [Fact]
        public void OriginFields_SaveToIgnoresBlankFields()
        {
            Origin origin = new Origin(false);
            origin.X = 42.0;
            OriginFields fields = new OriginFields();
            fields.Clear();

            fields.SaveTo(origin);

            Assert.Equal(42.0, origin.X);
        }

        [Fact]
        public void ColorFields_TryGetRgbScalesAndClampsToBytes()
        {
            ColorFields fields = new ColorFields { Red = "1", Green = "0.5", Blue = "-2" };

            Assert.True(fields.TryGetRgb(out byte r, out byte g, out byte b));

            Assert.Equal(255, r);
            Assert.Equal(128, g);
            Assert.Equal(0, b);
        }

        [Fact]
        public void ColorFields_TryGetRgbFailsOnAnUnparseableChannel()
        {
            ColorFields fields = new ColorFields { Red = "1", Green = "", Blue = "0" };

            Assert.False(fields.TryGetRgb(out _, out _, out _));
        }

        /// <summary>
        /// Every 0-255 value must survive being written as text and read back, or picking the same
        /// color twice would drift.
        /// </summary>
        [Fact]
        public void ColorFields_SetRgbRoundTripsEveryByte()
        {
            ColorFields fields = new ColorFields();
            for (int value = 0; value <= 255; value++)
            {
                fields.SetRgb((byte)value, (byte)value, (byte)value);

                Assert.True(fields.TryGetRgb(out byte r, out _, out _));
                Assert.Equal(value, r);
            }
        }

        [Fact]
        public void ColorFields_SetRgbLeavesAlphaAlone()
        {
            ColorFields fields = new ColorFields { Alpha = "0.25" };

            fields.SetRgb(10, 20, 30);

            Assert.Equal("0.25", fields.Alpha);
        }
    }
}
