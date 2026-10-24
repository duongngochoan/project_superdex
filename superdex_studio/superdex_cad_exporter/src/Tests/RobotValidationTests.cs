/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;

using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI.ViewModels;
using Xunit;

namespace CADRobotExporter.Tests
{
    /// <summary>
    /// Covers the pre-export validation walks previously inlined in AssemblyExportForm.
    /// </summary>
    public class RobotValidationTests
    {
        private static Link NamedLink(string name)
        {
            return new Link { Name = name };
        }

        /// <summary>
        /// A revolute joint requires limits, which a bare joint does not have. This keeps the link
        /// itself named so the report can be asserted on.
        /// </summary>
        private static Link InvalidLink(string name)
        {
            Link link = NamedLink(name);
            link.Joint.Name = name + "_joint";
            link.Joint.Type = "revolute";
            return link;
        }

        private static string[] Lines(string report)
        {
            return report.Split(new[] { "\r\n" }, StringSplitOptions.RemoveEmptyEntries);
        }

        [Fact]
        public void LinksMissingRequiredFields_ReportsNothingForAValidTree()
        {
            Link root = NamedLink("root");
            root.AddChild(NamedLink("child"));

            Assert.Equal(string.Empty, RobotValidation.LinksMissingRequiredFields(root));
        }

        [Fact]
        public void LinksMissingRequiredFields_ReportsAnInvalidTree()
        {
            Link root = NamedLink("root");
            root.AddChild(InvalidLink("bad"));

            Assert.Contains("bad", Lines(RobotValidation.LinksMissingRequiredFields(root)));
        }

        /// <summary>
        /// Link.AreRequiredFieldsSatisfied recurses into Children, so validating a subtree at every
        /// node would name every ancestor of an offender. Only the offender should be reported.
        /// </summary>
        [Fact]
        public void LinksMissingRequiredFields_DoesNotReportAncestorsOfAnInvalidLink()
        {
            Link root = NamedLink("root");
            Link middle = NamedLink("middle");
            middle.AddChild(InvalidLink("bad"));
            root.AddChild(middle);

            string[] lines = Lines(RobotValidation.LinksMissingRequiredFields(root));

            Assert.Equal(new[] { "bad" }, lines);
        }

        [Fact]
        public void LinksMissingRequiredFields_IgnoresAValidSibling()
        {
            Link root = NamedLink("root");
            root.AddChild(NamedLink("healthy"));
            root.AddChild(InvalidLink("bad"));

            string[] lines = Lines(RobotValidation.LinksMissingRequiredFields(root));

            Assert.Equal(new[] { "bad" }, lines);
        }

        [Fact]
        public void LinksWithInvalidInertials_ReportsNothingForLinksWithoutGeometry()
        {
            Link root = NamedLink("root");
            root.AddChild(NamedLink("child"));

            Assert.Equal(string.Empty, RobotValidation.LinksWithInvalidInertials(root));
        }

        /// <summary>
        /// The base link has no joint of its own, so the joint walk must start at its children.
        /// </summary>
        [Fact]
        public void JointsMissingRequiredFields_SkipsTheBaseLink()
        {
            Link root = NamedLink("root");

            Assert.Equal(string.Empty, RobotValidation.JointsMissingRequiredFields(root));
        }

        [Fact]
        public void JointsMissingRequiredFields_ReportsJointNamesNotLinkNames()
        {
            Link root = NamedLink("root");
            Link child = NamedLink("child_link");
            child.Joint.Name = "child_joint";
            child.Joint.Type = "revolute";
            root.AddChild(child);

            string[] lines = Lines(RobotValidation.JointsMissingRequiredFields(root));

            // A revolute joint requires limits, which a bare joint does not have.
            Assert.Contains("child_joint", lines);
            Assert.DoesNotContain("child_link", lines);
        }
    }
}
