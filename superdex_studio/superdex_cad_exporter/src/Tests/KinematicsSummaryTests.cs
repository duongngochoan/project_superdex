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
    public class KinematicsSummaryTests
    {
        private static Link MakeLink(string name)
        {
            Link link = new Link();
            link.Name = name;
            return link;
        }

        private static Link MakeJointedChild(string linkName, string jointName, string jointType)
        {
            Link link = MakeLink(linkName);
            link.Joint = new Joint();
            link.Joint.Name = jointName;
            link.Joint.Type = jointType;
            return link;
        }

        [Fact]
        public void BuildReturnsEmptyForNullBaseLink()
        {
            Assert.Empty(KinematicsSummary.Build(null, true, null));
        }

        [Fact]
        public void BuildEmitsOneRowPerLinkInDepthFirstOrder()
        {
            Link root = MakeLink("base_link");
            Link a = MakeJointedChild("link_1", "joint_link_1", "revolute");
            Link b = MakeJointedChild("link_2", "joint_link_2", "revolute");
            root.Children.Add(a);
            a.Children.Add(b);

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(root, true, null);

            Assert.Equal(3, rows.Count);
            Assert.Equal("base_link", rows[0].LinkName);
            Assert.Equal("  link_1", rows[1].LinkName);
            Assert.Equal("    link_2", rows[2].LinkName);
        }

        [Fact]
        public void BuildIndentsByDepth()
        {
            Link root = MakeLink("r");
            Link child = MakeJointedChild("c", "j", "fixed");
            root.Children.Add(child);

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(root, false, null);

            Assert.Equal("r", rows[0].LinkName);
            Assert.Equal("  c", rows[1].LinkName);
        }

        [Fact]
        public void BaseLinkHasNoJointColumnsEvenWhenItCarriesAJoint()
        {
            Link root = MakeJointedChild("base_link", "should_be_ignored", "revolute");

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(root, true, null);

            Assert.Equal("", rows[0].JointName);
            Assert.Equal("", rows[0].JointType);
        }

        [Fact]
        public void SiteLinksReportSiteAsJointType()
        {
            Link root = MakeLink("base_link");
            Link site = MakeJointedChild("s", "joint_s", "revolute");
            site.isSite = true;
            root.Children.Add(site);

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(root, true, null);

            Assert.Equal("(site)", rows[1].JointType);
        }

        [Fact]
        public void RevoluteLimitsConvertToDegreesWhenRequested()
        {
            Link root = MakeLink("base_link");
            Link child = MakeJointedChild("c", "j", "revolute");
            child.Joint.Limit.Lower = -System.Math.PI / 2.0;
            child.Joint.Limit.Upper = System.Math.PI / 2.0;
            root.Children.Add(child);

            List<KinematicsSummaryRow> degrees = KinematicsSummary.Build(root, true, null);
            Assert.Equal("-90", degrees[1].LowerLimit);
            Assert.Equal("90", degrees[1].UpperLimit);

            List<KinematicsSummaryRow> radians = KinematicsSummary.Build(root, false, null);
            Assert.Equal("-1.571", radians[1].LowerLimit);
        }

        [Fact]
        public void NonRevoluteLimitsIgnoreTheDegreesToggle()
        {
            Link root = MakeLink("base_link");
            Link child = MakeJointedChild("c", "j", "prismatic");
            child.Joint.Limit.Lower = 0.25;
            root.Children.Add(child);

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(root, true, null);

            Assert.Equal("0.25", rows[1].LowerLimit);
        }

        [Fact]
        public void VelocityIsOnlyReportedForRevoluteJoints()
        {
            Link root = MakeLink("base_link");
            Link prismatic = MakeJointedChild("p", "jp", "prismatic");
            prismatic.Joint.Limit.Velocity = 1.5;
            root.Children.Add(prismatic);

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(root, false, null);

            Assert.Equal("", rows[1].VelocityLimit);
        }

        [Fact]
        public void FixedJointsHaveNoAxis()
        {
            Link root = MakeLink("base_link");
            Link child = MakeJointedChild("c", "j", "fixed");
            root.Children.Add(child);

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(root, true, null);

            Assert.Equal("", rows[1].Axis);
        }

        [Fact]
        public void MovingJointsReportTheirAxis()
        {
            Link root = MakeLink("base_link");
            Link child = MakeJointedChild("c", "j", "revolute");
            child.Joint.Axis.SetXYZ(new double[] { 0.0, 0.0, 1.0 });
            root.Children.Add(child);

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(root, true, null);

            Assert.Equal("0, 0, 1", rows[1].Axis);
        }

        [Fact]
        public void BodyCountsComeFromTheSuppliedResolver()
        {
            Link root = MakeLink("base_link");

            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(
                root,
                true,
                link => new LinkBodyCounts { Inertial = 3, Collision = 2, Visual = 7 });

            Assert.Equal("3", rows[0].InertialBodies);
            Assert.Equal("2", rows[0].CollisionBodies);
            Assert.Equal("7", rows[0].VisualBodies);
        }

        [Fact]
        public void BodyCountsDefaultToZeroWithoutAResolver()
        {
            List<KinematicsSummaryRow> rows = KinematicsSummary.Build(MakeLink("base_link"), true, null);

            Assert.Equal("0", rows[0].InertialBodies);
            Assert.Equal("0", rows[0].CollisionBodies);
            Assert.Equal("0", rows[0].VisualBodies);
        }

        [Theory]
        [InlineData(true, "deg")]
        [InlineData(false, "rad")]
        public void LimitUnitFollowsTheToggle(bool useDegrees, string expected)
        {
            Assert.Equal(expected, KinematicsSummary.LimitUnit(useDegrees));
        }
    }
}
