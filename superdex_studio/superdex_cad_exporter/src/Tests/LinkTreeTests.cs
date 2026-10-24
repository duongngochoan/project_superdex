/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Collections.Generic;

using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI;
using Xunit;

namespace CADRobotExporter.Tests
{
    public class LinkTreeTests
    {
        private static Link NamedLink(string name)
        {
            return new Link { Name = name };
        }

        [Fact]
        public void AddChild_LinksBothDirections()
        {
            Link parent = NamedLink("parent");
            Link child = NamedLink("child");

            parent.AddChild(child);

            Assert.Same(parent, child.Parent);
            Assert.Collection(parent.Children, c => Assert.Same(child, c));
        }

        [Fact]
        public void RemoveChild_UnlinksBothDirections()
        {
            Link parent = NamedLink("parent");
            Link child = NamedLink("child");
            parent.AddChild(child);

            parent.RemoveChild(child);

            Assert.Null(child.Parent);
            Assert.Empty(parent.Children);
        }

        [Fact]
        public void RemoveChild_LeavesSiblingsIntact()
        {
            Link parent = NamedLink("parent");
            Link first = NamedLink("first");
            Link second = NamedLink("second");
            parent.AddChild(first);
            parent.AddChild(second);

            parent.RemoveChild(first);

            Assert.Collection(parent.Children, c => Assert.Same(second, c));
            Assert.Same(parent, second.Parent);
        }

        /// <summary>
        /// Regression: Clone previously set each cloned child's Parent to the *source* link,
        /// so a cloned subtree still pointed back into the tree it was copied from.
        /// </summary>
        [Fact]
        public void Clone_ChildParentPointsAtTheClone_NotTheSource()
        {
            Link root = NamedLink("root");
            root.AddChild(NamedLink("child"));

            Link clone = root.Clone();

            Link clonedChild = Assert.Single(clone.Children);
            Assert.Same(clone, clonedChild.Parent);
            Assert.NotSame(root, clonedChild.Parent);
        }

        [Fact]
        public void Clone_RewiresParentAtEveryDepth()
        {
            Link root = NamedLink("root");
            Link middle = NamedLink("middle");
            Link leaf = NamedLink("leaf");
            root.AddChild(middle);
            middle.AddChild(leaf);

            Link clone = root.Clone();

            Link clonedMiddle = Assert.Single(clone.Children);
            Link clonedLeaf = Assert.Single(clonedMiddle.Children);
            Assert.Same(clone, clonedMiddle.Parent);
            Assert.Same(clonedMiddle, clonedLeaf.Parent);
        }

        [Fact]
        public void Clone_ProducesDistinctNodeInstances()
        {
            Link root = NamedLink("root");
            Link child = NamedLink("child");
            root.AddChild(child);

            Link clone = root.Clone();

            Assert.NotSame(root, clone);
            Assert.NotSame(child, clone.Children[0]);
        }

        [Fact]
        public void Clone_MutatingTheCloneDoesNotAffectTheSource()
        {
            Link root = NamedLink("root");
            root.AddChild(NamedLink("child"));
            Link clone = root.Clone();

            clone.AddChild(NamedLink("extra"));

            Assert.Single(root.Children);
            Assert.Equal(2, clone.Children.Count);
        }

        /// <summary>
        /// Part-document configurations store solid body PIDs where assembly configurations store
        /// component PIDs. Both constructors must seed the lists, because callers append to them
        /// without a null check.
        /// </summary>
        [Fact]
        public void Ctor_InitializesBodyPidLists()
        {
            Link parentless = new Link();
            Link withParent = new Link(parentless);

            foreach (Link link in new[] { parentless, withParent })
            {
                Assert.NotNull(link.SWVisualBodyPIDs);
                Assert.NotNull(link.SWCollisionBodyPIDs);
                Assert.NotNull(link.SWInertialBodyPIDs);
                Assert.Empty(link.SWVisualBodyPIDs);
                Assert.Empty(link.SWCollisionBodyPIDs);
                Assert.Empty(link.SWInertialBodyPIDs);
            }
        }

        /// <summary>
        /// Clone runs on every deserialize, so anything SetElement forgets to copy is silently lost
        /// when a saved configuration is reopened. Several existing fields already have that bug;
        /// this pins the body PIDs so they do not join them.
        /// </summary>
        [Fact]
        public void Clone_PreservesBodyPids()
        {
            Link root = NamedLink("root");
            root.SWVisualBodyPIDs.Add(new byte[] { 1, 2, 3 });
            root.SWCollisionBodyPIDs.Add(new byte[] { 4, 5 });
            root.SWInertialBodyPIDs.Add(new byte[] { 6 });

            Link clone = root.Clone();

            Assert.Equal(new byte[] { 1, 2, 3 }, Assert.Single(clone.SWVisualBodyPIDs));
            Assert.Equal(new byte[] { 4, 5 }, Assert.Single(clone.SWCollisionBodyPIDs));
            Assert.Equal(new byte[] { 6 }, Assert.Single(clone.SWInertialBodyPIDs));
        }

        [Fact]
        public void Clone_PreservesBodyPidsOnChildren()
        {
            Link root = NamedLink("root");
            Link child = NamedLink("child");
            child.SWVisualBodyPIDs.Add(new byte[] { 9 });
            root.AddChild(child);

            Link clone = root.Clone();

            Link clonedChild = Assert.Single(clone.Children);
            Assert.Equal(new byte[] { 9 }, Assert.Single(clonedChild.SWVisualBodyPIDs));
        }

        /// <summary>
        /// The clone must own its lists, otherwise editing one configuration mutates the other.
        /// </summary>
        [Fact]
        public void Clone_BodyPidListsAreIndependent()
        {
            Link root = NamedLink("root");
            root.SWVisualBodyPIDs.Add(new byte[] { 1 });
            Link clone = root.Clone();

            clone.SWVisualBodyPIDs.Add(new byte[] { 2 });

            Assert.Single(root.SWVisualBodyPIDs);
            Assert.Equal(2, clone.SWVisualBodyPIDs.Count);
        }

        [Fact]
        public void Clone_PreservesMeshingOptions()
        {
            Link root = NamedLink("root");
            root.visualMeshingOptions.linearDeflection = 0.25;
            root.collisionMeshingOptions.angularDeflection = 1.5;

            Link clone = root.Clone();

            Assert.Equal(0.25, clone.visualMeshingOptions.linearDeflection);
            Assert.Equal(1.5, clone.collisionMeshingOptions.angularDeflection);
        }

        /// <summary>
        /// SetElement used to assign the references straight across, so a cloned link shared one
        /// MeshingOptions with its source and editing either link's meshing properties edited both.
        /// </summary>
        [Fact]
        public void Clone_MeshingOptionsAreIndependent()
        {
            Link root = NamedLink("root");
            Link clone = root.Clone();

            clone.visualMeshingOptions.linearDeflection = 9.0;
            clone.collisionMeshingOptions.linearDeflection = 9.0;

            Assert.NotSame(root.visualMeshingOptions, clone.visualMeshingOptions);
            Assert.NotSame(root.collisionMeshingOptions, clone.collisionMeshingOptions);
            Assert.Equal(0.1, root.visualMeshingOptions.linearDeflection);
            Assert.Equal(0.5, root.collisionMeshingOptions.linearDeflection);
        }
        /// <summary>
        /// The tree binds this to bold overridden links, and Link.meshingOverride is a plain field
        /// with no notification of its own, so the raise has to be explicit.
        /// </summary>
        [Fact]
        public void HasMeshingOverride_ReadsThroughToTheLink()
        {
            Link link = new Link { Name = "link_1" };
            LinkTreeItem item = new LinkTreeItem(link);

            Assert.False(item.HasMeshingOverride);

            link.meshingOverride = true;

            Assert.True(item.HasMeshingOverride);
        }

        [Fact]
        public void RefreshMeshingOverride_RaisesPropertyChanged()
        {
            LinkTreeItem item = new LinkTreeItem(new Link { Name = "link_1" });
            List<string> raised = new List<string>();
            item.PropertyChanged += (s, e) => raised.Add(e.PropertyName);

            item.RefreshMeshingOverride();

            Assert.Contains(nameof(LinkTreeItem.HasMeshingOverride), raised);
        }
    }
}
