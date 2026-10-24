/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.Text;

using CADRobotExporter.RobotDescription;

namespace CADRobotExporter.UI.ViewModels
{
    /// <summary>
    /// Pre-export validation over the Link graph. Each check returns the offending names, one
    /// per line, and an empty string when everything passes.
    /// </summary>
    public static class RobotValidation
    {
        /// <summary>
        /// Links whose required link or joint fields are unset.
        /// </summary>
        public static string LinksMissingRequiredFields(Link baseLink)
        {
            return Collect(baseLink, link => !link.AreOwnRequiredFieldsSatisfied(), link => link.Name);
        }

        /// <summary>
        /// Links whose inertial properties look missing or degenerate.
        /// </summary>
        public static string LinksWithInvalidInertials(Link baseLink)
        {
            return Collect(baseLink, link => !link.HasValidInertialProperties(), link => link.Name);
        }

        /// <summary>
        /// Joints whose required fields are unset. The base link has no joint, so the walk starts
        /// at its children.
        /// </summary>
        public static string JointsMissingRequiredFields(Link baseLink)
        {
            StringBuilder builder = new StringBuilder();
            foreach (Link child in baseLink.Children)
            {
                Collect(child, link => !link.Joint.AreRequiredFieldsSatisfied(), link => link.Joint.Name, builder);
            }
            return builder.ToString();
        }

        private static string Collect(Link root, Func<Link, bool> isOffending, Func<Link, string> describe)
        {
            StringBuilder builder = new StringBuilder();
            Collect(root, isOffending, describe, builder);
            return builder.ToString();
        }

        private static void Collect(
            Link link, Func<Link, bool> isOffending, Func<Link, string> describe, StringBuilder builder)
        {
            if (isOffending(link))
            {
                builder.Append(describe(link)).Append("\r\n");
            }
            foreach (Link child in link.Children)
            {
                Collect(child, isOffending, describe, builder);
            }
        }
    }
}
