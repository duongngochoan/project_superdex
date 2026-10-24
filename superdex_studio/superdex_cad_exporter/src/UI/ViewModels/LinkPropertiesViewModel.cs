/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;

using CADRobotExporter.RobotDescription;

namespace CADRobotExporter.UI.ViewModels
{
    // "R" round-trips a double, so a displayed value re-parses to the same number.
    internal static class FieldFormat
    {
        internal const string RoundTrip = "R";
    }

    /// <summary>
    /// The six origin values as display text. Attribute getters throw when the underlying value
    /// is unset, so a partially populated origin leaves the remaining fields blank.
    /// </summary>
    public class OriginFields
    {
        public string X { get; set; }
        public string Y { get; set; }
        public string Z { get; set; }
        public string Roll { get; set; }
        public string Pitch { get; set; }
        public string Yaw { get; set; }

        public void Clear()
        {
            X = Y = Z = Roll = Pitch = Yaw = string.Empty;
        }

        public void LoadFrom(Origin origin)
        {
            LoadFrom(origin, null);
        }

        /// <param name="convertAngle">
        /// Applied to roll, pitch and yaw only. Null leaves them in model units.
        /// </param>
        public void LoadFrom(Origin origin, Func<double, double> convertAngle)
        {
            Clear();
            if (origin == null)
            {
                return;
            }
            try
            {
                X = origin.X.ToString(FieldFormat.RoundTrip);
                Y = origin.Y.ToString(FieldFormat.RoundTrip);
                Z = origin.Z.ToString(FieldFormat.RoundTrip);
                Roll = Convert(origin.Roll, convertAngle);
                Pitch = Convert(origin.Pitch, convertAngle);
                Yaw = Convert(origin.Yaw, convertAngle);
            }
            catch (NullReferenceException)
            {
                // Attribute values are unset; leave the remaining fields blank.
            }
        }

        private static string Convert(double value, Func<double, double> convertAngle)
        {
            double converted = convertAngle == null ? value : convertAngle(value);
            return converted.ToString(FieldFormat.RoundTrip);
        }

        /// <summary>Unparseable text leaves the existing model value untouched.</summary>
        public void SaveTo(Origin origin)
        {
            SaveTo(origin, null);
        }

        /// <param name="convertAngle">
        /// Applied to roll, pitch and yaw only. Null leaves them in display units.
        /// </param>
        public void SaveTo(Origin origin, Func<double, double> convertAngle)
        {
            double value;
            if (double.TryParse(X, out value)) { origin.X = value; }
            if (double.TryParse(Y, out value)) { origin.Y = value; }
            if (double.TryParse(Z, out value)) { origin.Z = value; }
            if (double.TryParse(Roll, out value))
            {
                origin.Roll = convertAngle == null ? value : convertAngle(value);
            }
            if (double.TryParse(Pitch, out value))
            {
                origin.Pitch = convertAngle == null ? value : convertAngle(value);
            }
            if (double.TryParse(Yaw, out value))
            {
                origin.Yaw = convertAngle == null ? value : convertAngle(value);
            }
        }
    }

    /// <summary>The six independent inertia tensor terms as display text.</summary>
    public class InertiaFields
    {
        public string Ixx { get; set; }
        public string Ixy { get; set; }
        public string Ixz { get; set; }
        public string Iyy { get; set; }
        public string Iyz { get; set; }
        public string Izz { get; set; }

        public void Clear()
        {
            Ixx = Ixy = Ixz = Iyy = Iyz = Izz = string.Empty;
        }

        public void LoadFrom(Inertia inertia)
        {
            Clear();
            if (inertia == null)
            {
                return;
            }
            try
            {
                Ixx = inertia.Ixx.ToString(FieldFormat.RoundTrip);
                Ixy = inertia.Ixy.ToString(FieldFormat.RoundTrip);
                Ixz = inertia.Ixz.ToString(FieldFormat.RoundTrip);
                Iyy = inertia.Iyy.ToString(FieldFormat.RoundTrip);
                Iyz = inertia.Iyz.ToString(FieldFormat.RoundTrip);
                Izz = inertia.Izz.ToString(FieldFormat.RoundTrip);
            }
            catch (NullReferenceException)
            {
                // Attribute values are unset; leave the remaining fields blank.
            }
        }

        public void SaveTo(Inertia inertia)
        {
            double value;
            if (double.TryParse(Ixx, out value)) { inertia.Ixx = value; }
            if (double.TryParse(Ixy, out value)) { inertia.Ixy = value; }
            if (double.TryParse(Ixz, out value)) { inertia.Ixz = value; }
            if (double.TryParse(Iyy, out value)) { inertia.Iyy = value; }
            if (double.TryParse(Iyz, out value)) { inertia.Iyz = value; }
            if (double.TryParse(Izz, out value)) { inertia.Izz = value; }
        }
    }

    /// <summary>The four color channels as display text.</summary>
    public class ColorFields
    {
        public string Red { get; set; }
        public string Green { get; set; }
        public string Blue { get; set; }
        public string Alpha { get; set; }

        public void Clear()
        {
            Red = Green = Blue = Alpha = string.Empty;
        }

        public void LoadFrom(Color color)
        {
            Clear();
            if (color == null)
            {
                return;
            }
            try
            {
                Red = color.Red.ToString(FieldFormat.RoundTrip);
                Green = color.Green.ToString(FieldFormat.RoundTrip);
                Blue = color.Blue.ToString(FieldFormat.RoundTrip);
                Alpha = color.Alpha.ToString(FieldFormat.RoundTrip);
            }
            catch (NullReferenceException)
            {
                // Attribute values are unset; leave the remaining fields blank.
            }
        }

        public void SaveTo(Color color)
        {
            double value;
            if (double.TryParse(Red, out value)) { color.Red = value; }
            if (double.TryParse(Green, out value)) { color.Green = value; }
            if (double.TryParse(Blue, out value)) { color.Blue = value; }
            if (double.TryParse(Alpha, out value)) { color.Alpha = value; }
        }

        /// <summary>
        /// The red, green and blue channels as 0-255 values, for a color picker. False when any of
        /// the three is blank or not a number.
        /// </summary>
        public bool TryGetRgb(out byte red, out byte green, out byte blue)
        {
            red = green = blue = 0;
            double r, g, b;
            if (!double.TryParse(Red, out r) || !double.TryParse(Green, out g)
                || !double.TryParse(Blue, out b))
            {
                return false;
            }
            red = ToByte(r);
            green = ToByte(g);
            blue = ToByte(b);
            return true;
        }

        /// <summary>
        /// Sets red, green and blue from 0-255 values, leaving alpha alone. Written to three
        /// decimals, which is enough to recover the same 0-255 value.
        /// </summary>
        public void SetRgb(byte red, byte green, byte blue)
        {
            Red = ToChannelText(red);
            Green = ToChannelText(green);
            Blue = ToChannelText(blue);
        }

        private static byte ToByte(double channel)
        {
            return (byte)Math.Round(Math.Max(0.0, Math.Min(1.0, channel)) * 255.0);
        }

        private static string ToChannelText(byte channel)
        {
            return Math.Round(channel / 255.0, 3).ToString(FieldFormat.RoundTrip);
        }
    }

    /// <summary>
    /// The link properties page as display text, decoupled from the controls that show it.
    /// </summary>
    public class LinkPropertiesViewModel
    {
        public OriginFields VisualOrigin { get; } = new OriginFields();
        public OriginFields InertialOrigin { get; } = new OriginFields();
        public InertiaFields Inertia { get; } = new InertiaFields();
        public ColorFields Color { get; } = new ColorFields();
        public string Mass { get; set; }

        /// <summary>A fixed frame carries no editable link properties.</summary>
        public bool IsEditable { get; private set; }

        public void Clear()
        {
            VisualOrigin.Clear();
            InertialOrigin.Clear();
            Inertia.Clear();
            Color.Clear();
            Mass = string.Empty;
        }

        public void LoadFrom(Link link)
        {
            Clear();
            IsEditable = link != null && !link.isFixedFrame;
            if (!IsEditable)
            {
                return;
            }

            VisualOrigin.LoadFrom(link.Visual?.Origin);
            InertialOrigin.LoadFrom(link.Inertial?.Origin);
            Inertia.LoadFrom(link.Inertial?.Inertia);
            Color.LoadFrom(link.Visual?.Material?.Color);

            if (link.Inertial?.Mass != null)
            {
                try
                {
                    Mass = link.Inertial.Mass.Value.ToString(FieldFormat.RoundTrip);
                }
                catch (NullReferenceException)
                {
                    // Attribute value is unset; leave the field blank.
                }
            }
        }

        public void SaveTo(Link link)
        {
            if (link == null || link.isFixedFrame)
            {
                return;
            }

            InertialOrigin.SaveTo(link.Inertial.Origin);
            VisualOrigin.SaveTo(link.Visual.Origin);
            Inertia.SaveTo(link.Inertial.Inertia);
            Color.SaveTo(link.Visual.Material.Color);

            double mass;
            if (double.TryParse(Mass, out mass))
            {
                link.Inertial.Mass.Value = mass;
            }
        }
    }
}
