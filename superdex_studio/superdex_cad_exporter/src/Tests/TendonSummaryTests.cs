/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Collections.Generic;

using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI.ViewModels;

using Xunit;

namespace CADRobotExporter.Tests
{
    public class TendonSummaryTests
    {
        private static RoutingElement Elem(string link, string type, double x, double y, double z, double coefficient)
        {
            RoutingElement elem = new RoutingElement();
            elem.Link = link;
            elem.Type = type;
            elem.X = x;
            elem.Y = y;
            elem.Z = z;
            elem.Coefficient = coefficient;
            return elem;
        }

        private static Tendon MakeTendon(string name, params RoutingElement[] elements)
        {
            Tendon tendon = new Tendon();
            tendon.Name = name;
            foreach (RoutingElement e in elements)
            {
                tendon.RoutingElements.Add(e);
            }
            return tendon;
        }

        [Fact]
        public void BuildReturnsEmptyForNullTendons()
        {
            Assert.Empty(TendonSummary.Build(null));
        }

        [Fact]
        public void BuildReturnsEmptyForNoTendons()
        {
            Assert.Empty(TendonSummary.Build(new List<Tendon>()));
        }

        [Fact]
        public void IndexIsContinuousAcrossTendons()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("a", "site", 0, 0, 0, 0), Elem("b", "site", 0, 0, 0, 0)),
                MakeTendon("t2", Elem("c", "site", 0, 0, 0, 0)),
            };

            List<TendonSummaryRow> rows = TendonSummary.Build(tendons);

            Assert.Equal(3, rows.Count);
            Assert.Equal(0, rows[0].Index);
            Assert.Equal(1, rows[1].Index);
            Assert.Equal(2, rows[2].Index);
        }

        [Fact]
        public void ElemIndexRestartsPerTendon()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("a", "site", 0, 0, 0, 0), Elem("b", "site", 0, 0, 0, 0)),
                MakeTendon("t2", Elem("c", "site", 0, 0, 0, 0)),
            };

            List<TendonSummaryRow> rows = TendonSummary.Build(tendons);

            Assert.Equal(0, rows[0].Elem);
            Assert.Equal(1, rows[1].Elem);
            Assert.Equal(0, rows[2].Elem);
        }

        [Fact]
        public void DisplayedTendonNameOnlyAppearsOnTheFirstElement()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("a", "site", 0, 0, 0, 0), Elem("b", "site", 0, 0, 0, 0)),
            };

            List<TendonSummaryRow> rows = TendonSummary.Build(tendons);

            Assert.Equal("t1", rows[0].TendonName);
            Assert.Equal("", rows[1].TendonName);
        }

        [Fact]
        public void OwningTendonNameIsSetOnEveryRow()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("a", "site", 0, 0, 0, 0), Elem("b", "site", 0, 0, 0, 0)),
            };

            List<TendonSummaryRow> rows = TendonSummary.Build(tendons);

            Assert.Equal("t1", rows[0].OwningTendonName);
            Assert.Equal("t1", rows[1].OwningTendonName);
        }

        [Fact]
        public void LinearJointElementsHaveNoPosition()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("a", RoutingElement.TypeLinearJoint, 1.0, 2.0, 3.0, 0)),
            };

            List<TendonSummaryRow> rows = TendonSummary.Build(tendons);

            Assert.Equal("-", rows[0].X);
            Assert.Equal("-", rows[0].Y);
            Assert.Equal("-", rows[0].Z);
        }

        [Fact]
        public void NonLinearJointElementsFormatTheirPosition()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("a", "site", 1.5, -2.25, 0.0, 0)),
            };

            List<TendonSummaryRow> rows = TendonSummary.Build(tendons);

            Assert.Equal("1.5", rows[0].X);
            Assert.Equal("-2.25", rows[0].Y);
            Assert.Equal("0", rows[0].Z);
        }

        [Fact]
        public void ZeroCoefficientIsBlank()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("a", "site", 0, 0, 0, 0.0)),
            };

            Assert.Equal("", TendonSummary.Build(tendons)[0].Coefficient);
        }

        [Fact]
        public void NonZeroCoefficientIsFormatted()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("a", "site", 0, 0, 0, -1.25)),
            };

            Assert.Equal("-1.25", TendonSummary.Build(tendons)[0].Coefficient);
        }

        [Fact]
        public void LinkAndTypeArePassedThrough()
        {
            List<Tendon> tendons = new List<Tendon>
            {
                MakeTendon("t1", Elem("link_1", "site", 0, 0, 0, 0)),
            };

            List<TendonSummaryRow> rows = TendonSummary.Build(tendons);

            Assert.Equal("link_1", rows[0].Link);
            Assert.Equal("site", rows[0].Type);
        }

        [Fact]
        public void TendonWithNoRoutingElementsProducesNoRows()
        {
            Assert.Empty(TendonSummary.Build(new List<Tendon> { MakeTendon("empty") }));
        }
    }
}
