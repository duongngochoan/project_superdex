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
    /// <summary>The three axis values as display text.</summary>
    public class AxisFields
    {
        public string X { get; set; }
        public string Y { get; set; }
        public string Z { get; set; }

        public void Clear()
        {
            X = Y = Z = string.Empty;
        }

        public void LoadFrom(Axis axis)
        {
            Clear();
            if (axis == null)
            {
                return;
            }
            try
            {
                X = axis.X.ToString(FieldFormat.RoundTrip);
                Y = axis.Y.ToString(FieldFormat.RoundTrip);
                Z = axis.Z.ToString(FieldFormat.RoundTrip);
            }
            catch (NullReferenceException)
            {
                // Attribute values are unset; leave the remaining fields blank.
            }
        }

        public void SaveTo(Axis axis)
        {
            double value;
            if (double.TryParse(X, out value)) { axis.X = value; }
            if (double.TryParse(Y, out value)) { axis.Y = value; }
            if (double.TryParse(Z, out value)) { axis.Z = value; }
        }
    }

    /// <summary>The six joint limit and dynamics captions, which carry the units.</summary>
    public struct JointLabels
    {
        public string Lower;
        public string Upper;
        public string Effort;
        public string Velocity;
        public string Friction;
        public string Damping;

        public static JointLabels Default()
        {
            return new JointLabels
            {
                Lower = "Lower",
                Upper = "Upper",
                Effort = "Effort",
                Velocity = "Velocity",
                Friction = "Friction",
                Damping = "Damping",
            };
        }

        public static JointLabels For(string jointType, bool useDegrees)
        {
            if (jointType == "revolute" || jointType == "continuous")
            {
                string angleUnit = useDegrees ? "deg" : "rad";
                return new JointLabels
                {
                    Lower = "Lower (" + angleUnit + ")",
                    Upper = "Upper (" + angleUnit + ")",
                    Effort = "Effort (N-m)",
                    Velocity = "Velocity (" + angleUnit + "/s)",
                    Friction = "Friction (N-m)",
                    Damping = "Damping (N-m-s/" + angleUnit + ")",
                };
            }
            if (jointType == "prismatic")
            {
                return new JointLabels
                {
                    Lower = "Lower (m)",
                    Upper = "Upper (m)",
                    Effort = "Effort (N)",
                    Velocity = "Velocity (m/s)",
                    Friction = "Friction (N)",
                    Damping = "Damping (N-s/m)",
                };
            }
            return Default();
        }
    }

    /// <summary>
    /// Angle unit policy for the joint page.
    ///
    /// The origin rotations are always angular, so they follow the degrees checkbox on its own.
    /// The limits and velocity are only angular on a revolute joint -- on a prismatic joint they
    /// are metres and must never be converted.
    /// </summary>
    public static class JointAngles
    {
        private const decimal RevoluteIncrementDegrees = 5M;
        private const decimal RevoluteIncrementRadians = 0.0872665M;
        private const decimal LinearIncrement = 0.1M;

        public static double RadiansToDegrees(double radians)
        {
            return radians * (180.0 / Math.PI);
        }

        public static double DegreesToRadians(double degrees)
        {
            return degrees * (Math.PI / 180.0);
        }

        public static double ToDisplay(double radians, bool useDegrees)
        {
            return useDegrees ? RadiansToDegrees(radians) : radians;
        }

        public static double ToModel(double display, bool useDegrees)
        {
            return useDegrees ? DegreesToRadians(display) : display;
        }

        public static bool LimitsAreAngular(string jointType)
        {
            return jointType == "revolute";
        }

        public static double LimitToDisplay(double radians, bool useDegrees, string jointType)
        {
            return ToDisplay(radians, useDegrees && LimitsAreAngular(jointType));
        }

        public static double LimitToModel(double display, bool useDegrees, string jointType)
        {
            return ToModel(display, useDegrees && LimitsAreAngular(jointType));
        }

        public static decimal LimitIncrementFor(string jointType, bool useDegrees)
        {
            if (!LimitsAreAngular(jointType))
            {
                return LinearIncrement;
            }
            return useDegrees ? RevoluteIncrementDegrees : RevoluteIncrementRadians;
        }
    }

    /// <summary>
    /// The joint properties page as display text, decoupled from the controls that show it.
    /// </summary>
    public class JointPropertiesViewModel
    {
        public string Name { get; set; }
        public string Type { get; set; }
        public string ParentLink { get; set; }
        public string ChildLink { get; set; }
        public string CoordinateSystemName { get; set; }
        public string AxisName { get; set; }

        public OriginFields Origin { get; } = new OriginFields();
        public AxisFields Axis { get; } = new AxisFields();

        public decimal LimitLower { get; set; }
        public decimal LimitUpper { get; set; }
        public string LimitEffort { get; set; }
        public string LimitVelocity { get; set; }
        public string Damping { get; set; }
        public string Friction { get; set; }

        /// <summary>False for the base link, or when nothing is selected.</summary>
        public bool HasJoint { get; private set; }

        /// <summary>A fixed joint has no axis, limits or dynamics to edit.</summary>
        public bool AllowsMotion { get; private set; }

        public JointLabels Labels { get; private set; }
        public decimal LimitIncrement { get; private set; }

        public void Clear()
        {
            Name = Type = ParentLink = ChildLink = string.Empty;
            CoordinateSystemName = AxisName = string.Empty;
            Origin.Clear();
            Axis.Clear();
            LimitEffort = LimitVelocity = Damping = Friction = string.Empty;
            Labels = JointLabels.Default();
            LimitIncrement = JointAngles.LimitIncrementFor(null, false);
        }

        public void LoadFrom(Joint joint, bool useDegrees)
        {
            Clear();
            HasJoint = joint != null;
            if (!HasJoint)
            {
                AllowsMotion = false;
                return;
            }

            Name = joint.Name ?? string.Empty;
            Type = joint.Type ?? string.Empty;
            ParentLink = joint.Parent?.Name ?? string.Empty;
            ChildLink = joint.Child?.Name ?? string.Empty;
            CoordinateSystemName = joint.CoordinateSystemName ?? string.Empty;
            AxisName = joint.AxisName ?? string.Empty;

            AllowsMotion = joint.Type != "fixed";
            Labels = JointLabels.For(joint.Type, useDegrees);
            LimitIncrement = JointAngles.LimitIncrementFor(joint.Type, useDegrees);

            Origin.LoadFrom(joint.Origin, radians => JointAngles.ToDisplay(radians, useDegrees));

            if (AllowsMotion)
            {
                Axis.LoadFrom(joint.Axis);
            }

            if (joint.Limit != null && AllowsMotion && joint.Limit.ElementContainsData())
            {
                try
                {
                    LimitLower = Convert.ToDecimal(
                        JointAngles.LimitToDisplay(joint.Limit.Lower, useDegrees, joint.Type));
                    LimitUpper = Convert.ToDecimal(
                        JointAngles.LimitToDisplay(joint.Limit.Upper, useDegrees, joint.Type));
                    LimitEffort = joint.Limit.Effort.ToString(FieldFormat.RoundTrip);
                    LimitVelocity = JointAngles
                        .LimitToDisplay(joint.Limit.Velocity, useDegrees, joint.Type)
                        .ToString(FieldFormat.RoundTrip);
                }
                catch (NullReferenceException)
                {
                    // Attribute values are unset; leave the remaining fields blank.
                }
            }

            if (joint.Dynamics != null && joint.Dynamics.ElementContainsData())
            {
                try
                {
                    Damping = joint.Dynamics.Damping.ToString(FieldFormat.RoundTrip);
                    Friction = joint.Dynamics.Friction.ToString(FieldFormat.RoundTrip);
                }
                catch (NullReferenceException)
                {
                    // Attribute values are unset; leave the remaining fields blank.
                }
            }
        }

        public void SaveTo(Joint joint, bool useDegrees)
        {
            if (joint == null)
            {
                return;
            }

            joint.Name = Name;
            joint.Type = Type;
            joint.Parent.Name = ParentLink;
            joint.Child.Name = ChildLink;

            Origin.SaveTo(joint.Origin, display => JointAngles.ToModel(display, useDegrees));
            Axis.SaveTo(joint.Axis);

            joint.Limit.SetRequired(joint.Type == "revolute" || joint.Type == "prismatic");
            bool limitLeftBlank =
                string.IsNullOrWhiteSpace(LimitEffort) &&
                string.IsNullOrWhiteSpace(LimitVelocity) &&
                !joint.Limit.IsRequired();
            if (!limitLeftBlank)
            {
                joint.Limit.Lower = JointAngles.LimitToModel((double)LimitLower, useDegrees, joint.Type);
                joint.Limit.Upper = JointAngles.LimitToModel((double)LimitUpper, useDegrees, joint.Type);

                double effort;
                if (double.TryParse(LimitEffort, out effort))
                {
                    joint.Limit.Effort = effort;
                }
                double velocity;
                if (double.TryParse(LimitVelocity, out velocity))
                {
                    joint.Limit.Velocity =
                        JointAngles.LimitToModel(velocity, useDegrees, joint.Type);
                }
            }

            if (string.IsNullOrWhiteSpace(Friction) && string.IsNullOrWhiteSpace(Damping))
            {
                joint.Dynamics.Unset();
            }
            else
            {
                double damping;
                if (double.TryParse(Damping, out damping)) { joint.Dynamics.Damping = damping; }
                double friction;
                if (double.TryParse(Friction, out friction)) { joint.Dynamics.Friction = friction; }
            }
        }
    }
}
