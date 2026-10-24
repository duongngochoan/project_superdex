/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Linq;
using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI;
using Xunit;

namespace CADRobotExporter.Tests
{
    public class LinkNodeTests
    {
        private static Link NamedLink(string name)
        {
            return new Link { Name = name };
        }

        private static Link LinkWithChildren(string name, params string[] childNames)
        {
            Link link = NamedLink(name);
            foreach (string childName in childNames)
            {
                link.AddChild(NamedLink(childName));
            }
            return link;
        }

        [Fact]
        public void Ctor_MirrorsExistingChildLinks()
        {
            Link root = LinkWithChildren("root", "a", "b");

            LinkNode node = new LinkNode(root);

            Assert.Equal(2, node.Nodes.Count);
            Assert.Same(root.Children[0], ((LinkNode)node.Nodes[0]).Link);
            Assert.Same(root.Children[1], ((LinkNode)node.Nodes[1]).Link);
        }

        [Fact]
        public void Ctor_MirrorsNestedChildLinks()
        {
            Link root = NamedLink("root");
            Link middle = NamedLink("middle");
            middle.AddChild(NamedLink("leaf"));
            root.AddChild(middle);

            LinkNode node = new LinkNode(root);

            LinkNode middleNode = (LinkNode)node.Nodes[0];
            Assert.Single(middleNode.Nodes);
            Assert.Same(middle.Children[0], ((LinkNode)middleNode.Nodes[0]).Link);
        }

        /// <summary>
        /// Regression: building the view must not dirty the model. IsIncomplete is persisted,
        /// so a constructor that set it would silently write to saved configuration.
        /// </summary>
        [Fact]
        public void Ctor_DoesNotMutateModelState()
        {
            Link root = LinkWithChildren("root", "a");
            root.IsIncomplete = false;

            LinkNode unused = new LinkNode(root);

            Assert.False(root.IsIncomplete);
            Assert.False(root.Children[0].IsIncomplete);
        }

        [Fact]
        public void Ctor_TreatsParentlessLinkAsBase()
        {
            Link root = LinkWithChildren("root", "a");

            LinkNode node = new LinkNode(root);

            Assert.True(node.IsBaseNode);
            Assert.False(((LinkNode)node.Nodes[0]).IsBaseNode);
        }

        [Fact]
        public void IsIncomplete_ReadsAndWritesThroughToLink()
        {
            Link link = NamedLink("link");
            LinkNode node = new LinkNode(link);

            node.IsIncomplete = true;
            Assert.True(link.IsIncomplete);

            link.IsIncomplete = false;
            Assert.False(node.IsIncomplete);
        }

        [Fact]
        public void WhyIncomplete_ReadsAndWritesThroughToLink()
        {
            Link link = NamedLink("link");
            LinkNode node = new LinkNode(link);

            node.WhyIncomplete = "no coordinate system";
            Assert.Equal("no coordinate system", link.WhyIncomplete);

            link.WhyIncomplete = "no axis";
            Assert.Equal("no axis", node.WhyIncomplete);
        }

        [Fact]
        public void IsBaseNode_ReadsAndWritesThroughToLink()
        {
            Link link = NamedLink("link");
            LinkNode node = new LinkNode(link);

            node.IsBaseNode = true;
            Assert.True(link.IsBaseLink);

            link.IsBaseLink = false;
            Assert.False(node.IsBaseNode);
        }

        [Fact]
        public void AddChildNode_UpdatesViewAndModelTogether()
        {
            LinkNode parent = new LinkNode(NamedLink("parent"));
            LinkNode child = new LinkNode(NamedLink("child"));

            parent.AddChildNode(child);

            Assert.Same(child, parent.Nodes[0]);
            Assert.Collection(parent.Link.Children, c => Assert.Same(child.Link, c));
            Assert.Same(parent.Link, child.Link.Parent);
        }

        [Fact]
        public void InsertChildNode_UsesTheSameIndexInBothTrees()
        {
            LinkNode parent = new LinkNode(NamedLink("parent"));
            parent.AddChildNode(new LinkNode(NamedLink("first")));
            parent.AddChildNode(new LinkNode(NamedLink("third")));
            LinkNode inserted = new LinkNode(NamedLink("second"));

            parent.InsertChildNode(1, inserted);

            Assert.Same(inserted, parent.Nodes[1]);
            Assert.Same(inserted.Link, parent.Link.Children[1]);
            Assert.Same(parent.Link, inserted.Link.Parent);
        }

        [Fact]
        public void RemoveChildNode_UpdatesViewAndModelTogether()
        {
            LinkNode parent = new LinkNode(NamedLink("parent"));
            LinkNode child = new LinkNode(NamedLink("child"));
            parent.AddChildNode(child);

            parent.RemoveChildNode(child);

            Assert.Empty(parent.Nodes);
            Assert.Empty(parent.Link.Children);
            Assert.Null(child.Link.Parent);
        }

        [Fact]
        public void RemoveLastChildNode_RemovesOnlyTheLastEntry()
        {
            LinkNode parent = new LinkNode(NamedLink("parent"));
            LinkNode kept = new LinkNode(NamedLink("kept"));
            LinkNode dropped = new LinkNode(NamedLink("dropped"));
            parent.AddChildNode(kept);
            parent.AddChildNode(dropped);

            parent.RemoveLastChildNode();

            Assert.Collection(parent.Nodes.Cast<LinkNode>(), n => Assert.Same(kept, n));
            Assert.Collection(parent.Link.Children, c => Assert.Same(kept.Link, c));
        }

        [Fact]
        public void RemoveLastChildNode_OnEmptyNodeIsANoOp()
        {
            LinkNode parent = new LinkNode(NamedLink("parent"));

            parent.RemoveLastChildNode();

            Assert.Empty(parent.Nodes);
            Assert.Empty(parent.Link.Children);
        }

        [Fact]
        public void DetachFromParent_UpdatesViewAndModelTogether()
        {
            LinkNode parent = new LinkNode(NamedLink("parent"));
            LinkNode child = new LinkNode(NamedLink("child"));
            parent.AddChildNode(child);

            child.DetachFromParent();

            Assert.Empty(parent.Nodes);
            Assert.Empty(parent.Link.Children);
            Assert.Null(child.Link.Parent);
        }

        [Fact]
        public void DetachThenReattach_MovesTheLinkToTheNewParent()
        {
            LinkNode oldParent = new LinkNode(NamedLink("old"));
            LinkNode newParent = new LinkNode(NamedLink("new"));
            LinkNode moved = new LinkNode(NamedLink("moved"));
            oldParent.AddChildNode(moved);

            moved.DetachFromParent();
            newParent.AddChildNode(moved);

            Assert.Empty(oldParent.Link.Children);
            Assert.Collection(newParent.Link.Children, c => Assert.Same(moved.Link, c));
            Assert.Same(newParent.Link, moved.Link.Parent);
        }

        [Fact]
        public void UpdateLinkTree_LeavesAnAlreadyConsistentTreeUnchanged()
        {
            Link root = LinkWithChildren("root", "a", "b");
            root.Children[0].AddChild(NamedLink("a1"));
            LinkNode node = new LinkNode(root);

            Link result = node.UpdateLinkTree(null);

            Assert.Same(root, result);
            Assert.Null(result.Parent);
            Assert.Equal(2, result.Children.Count);
            Assert.Equal("a", result.Children[0].Name);
            Assert.Equal("b", result.Children[1].Name);
            Assert.Equal("a1", result.Children[0].Children[0].Name);
            Assert.Same(result, result.Children[0].Parent);
            Assert.Same(result.Children[0], result.Children[0].Children[0].Parent);
        }

        [Fact]
        public void UpdateLinkTree_RebuildsModelFromViewAfterNodeOnlyEdit()
        {
            LinkNode root = new LinkNode(NamedLink("root"));
            LinkNode child = new LinkNode(NamedLink("child"));

            // Bypass the helpers to simulate a stale model, which is exactly what the
            // divergence check in UpdateLinkTree is there to catch.
            root.Nodes.Add(child);
            Assert.Empty(root.Link.Children);

            Link result = root.UpdateLinkTree(null);

            Assert.Collection(result.Children, c => Assert.Same(child.Link, c));
            Assert.Same(result, child.Link.Parent);
        }
    }
}
