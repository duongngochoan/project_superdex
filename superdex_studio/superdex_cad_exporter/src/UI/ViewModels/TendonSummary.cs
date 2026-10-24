/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Collections.Generic;

using CADRobotExporter.RobotDescription;

namespace CADRobotExporter.UI.ViewModels
{
    /// <summary>One routing element of one tendon, formatted for display.</summary>
    public class TendonSummaryRow
    {
        public int Index { get; set; }

        /// <summary>
        /// Shown only on a tendon's first routing element, so the table reads as grouped blocks.
        /// Use <see cref="OwningTendonName"/> to identify the tendon a row belongs to.
        /// </summary>
        public string TendonName { get; set; }

        /// <summary>The tendon this row belongs to, set on every row.</summary>
        public string OwningTendonName { get; set; }

        public int Elem { get; set; }
        public string Link { get; set; }
        public string Type { get; set; }
        public string X { get; set; }
        public string Y { get; set; }
        public string Z { get; set; }
        public string Coefficient { get; set; }
    }

    /// <summary>Flattens the tendon list into the tendons summary table.</summary>
    public static class TendonSummary
    {
        private const string CoefficientFormat = "G4";
        private const string PositionFormat = "G5";

        /// <summary>Linear joint elements have no meaningful position.</summary>
        private const string NoPosition = "-";

        public static List<TendonSummaryRow> Build(IEnumerable<Tendon> tendons)
        {
            List<TendonSummaryRow> rows = new List<TendonSummaryRow>();
            if (tendons == null)
            {
                return rows;
            }

            int index = 0;
            foreach (Tendon tendon in tendons)
            {
                for (int i = 0; i < tendon.RoutingElements.Count; i++)
                {
                    RoutingElement elem = tendon.RoutingElements[i];
                    bool isLinearJoint = elem.Type == RoutingElement.TypeLinearJoint;

                    rows.Add(new TendonSummaryRow
                    {
                        Index = index,
                        TendonName = i == 0 ? tendon.Name : "",
                        OwningTendonName = tendon.Name,
                        Elem = i,
                        Link = elem.Link,
                        Type = elem.Type,
                        X = isLinearJoint ? NoPosition : elem.X.ToString(PositionFormat),
                        Y = isLinearJoint ? NoPosition : elem.Y.ToString(PositionFormat),
                        Z = isLinearJoint ? NoPosition : elem.Z.ToString(PositionFormat),
                        Coefficient = elem.Coefficient != 0 ? elem.Coefficient.ToString(CoefficientFormat) : "",
                    });

                    index++;
                }
            }

            return rows;
        }
    }
}
