/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.Collections.Generic;

using CADRobotExporter.RobotDescription;

namespace CADRobotExporter.UI.ViewModels
{
    /// <summary>
    /// How many CAD bodies back a link. Resolved by the host, since SolidWorks counts
    /// <c>Component2</c> instances and NX counts body handles.
    /// </summary>
    public struct LinkBodyCounts
    {
        public int Inertial;
        public int Collision;
        public int Visual;
    }

    /// <summary>One row of the kinematics summary, already formatted for display.</summary>
    public class KinematicsSummaryRow
    {
        public int Index { get; set; }
        public string LinkName { get; set; }
        public string JointName { get; set; }
        public string JointType { get; set; }
        public string Axis { get; set; }
        public string LowerLimit { get; set; }
        public string UpperLimit { get; set; }
        public string EffortLimit { get; set; }
        public string VelocityLimit { get; set; }
        public string Friction { get; set; }
        public string Damping { get; set; }
        public string Mass { get; set; }
        public string Inertia { get; set; }
        public string InertialBodies { get; set; }
        public string CollisionBodies { get; set; }
        public string VisualBodies { get; set; }
    }

    /// <summary>
    /// Flattens the link tree into the kinematics summary table.
    /// </summary>
    public static class KinematicsSummary
    {
        private const string Format = "G4";
        private const string AxisFormat = "G3";

        /// <summary>
        /// Only revolute joints carry angular limits, so only they respond to the degrees toggle.
        /// </summary>
        public static string LimitUnit(bool useDegrees)
        {
            return useDegrees ? "deg" : "rad";
        }

        public static List<KinematicsSummaryRow> Build(
            Link baseLink, bool useDegrees, Func<Link, LinkBodyCounts> bodyCounts)
        {
            List<KinematicsSummaryRow> rows = new List<KinematicsSummaryRow>();
            if (baseLink == null)
            {
                return rows;
            }

            int index = 0;
            Add(rows, baseLink, true, 0, ref index, useDegrees, bodyCounts);
            return rows;
        }

        private static void Add(
            List<KinematicsSummaryRow> rows,
            Link link,
            bool isBase,
            int depth,
            ref int index,
            bool useDegrees,
            Func<Link, LinkBodyCounts> bodyCounts)
        {
            KinematicsSummaryRow row = new KinematicsSummaryRow
            {
                Index = index,
                LinkName = new string(' ', depth * 2) + link.Name,
                JointName = "",
                JointType = "",
                Axis = "",
                LowerLimit = "",
                UpperLimit = "",
                EffortLimit = "",
                VelocityLimit = "",
                Friction = "",
                Damping = "",
            };

            if (!isBase && link.Joint != null && !string.IsNullOrWhiteSpace(link.Joint.Name))
            {
                FillJoint(row, link, useDegrees);
            }

            row.Mass = link.Inertial.Mass.Value.ToString(Format);
            row.Inertia = string.Format(
                "{0}, {1}, {2}",
                link.Inertial.Inertia.Ixx.ToString(Format),
                link.Inertial.Inertia.Iyy.ToString(Format),
                link.Inertial.Inertia.Izz.ToString(Format));

            LinkBodyCounts counts = bodyCounts == null ? default(LinkBodyCounts) : bodyCounts(link);
            row.InertialBodies = counts.Inertial.ToString();
            row.CollisionBodies = counts.Collision.ToString();
            row.VisualBodies = counts.Visual.ToString();

            rows.Add(row);

            foreach (Link child in link.Children)
            {
                index += 1;
                Add(rows, child, false, depth + 1, ref index, useDegrees, bodyCounts);
            }
        }

        private static void FillJoint(KinematicsSummaryRow row, Link link, bool useDegrees)
        {
            row.JointName = link.Joint.Name;
            row.JointType = link.Joint.Type ?? "";

            if (link.isSite)
            {
                row.JointType = "(site)";
            }

            bool showDegrees = useDegrees && row.JointType == "revolute";

            if (link.Joint.Limit.IsLowerSet())
            {
                row.LowerLimit = Display(link.Joint.Limit.Lower, showDegrees);
            }
            if (link.Joint.Limit.IsUpperSet())
            {
                row.UpperLimit = Display(link.Joint.Limit.Upper, showDegrees);
            }
            if (link.Joint.Limit.IsEffortSet())
            {
                row.EffortLimit = link.Joint.Limit.Effort.ToString(Format);
            }

            // Velocity is only reported for revolute joints, matching the original grid.
            if (link.Joint.Limit.IsVelocitySet() && row.JointType == "revolute")
            {
                row.VelocityLimit = Display(link.Joint.Limit.Velocity, showDegrees);
            }

            if (link.Joint.Dynamics.IsFrictionSet())
            {
                row.Friction = link.Joint.Dynamics.Friction.ToString(Format);
            }
            if (link.Joint.Dynamics.IsDampingSet())
            {
                row.Damping = link.Joint.Dynamics.Damping.ToString(Format);
            }

            if (row.JointType != "fixed" && !string.IsNullOrWhiteSpace(row.JointType))
            {
                double[] xyz = link.Joint.Axis.GetXYZ();
                row.Axis = string.Format(
                    "{0}, {1}, {2}",
                    xyz[0].ToString(AxisFormat),
                    xyz[1].ToString(AxisFormat),
                    xyz[2].ToString(AxisFormat));
            }
        }

        private static string Display(double radians, bool showDegrees)
        {
            return JointAngles.ToDisplay(radians, showDegrees).ToString(Format);
        }
    }
}
