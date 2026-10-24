/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Windows.Forms;

using Serilog;

using CADRobotExporter.Utilities;
using CADRobotExporter.RobotDescription;

namespace CADRobotExporter.UI
{
    // A LinkNode is derived from a TreeView TreeNode. It is a *view* over a Link: all shared
    // state lives on the Link so that the node tree can be rebuilt from the model at any time.
    public class LinkNode : TreeNode
    {
        private static readonly ILogger logger = Logger.GetLogger();

        public Link Link
        { get; set; }

        public bool IsBaseNode
        {
            get => Link.IsBaseLink;
            set => Link.IsBaseLink = value;
        }

        public bool IsIncomplete
        {
            get => Link.IsIncomplete;
            set => Link.IsIncomplete = value;
        }

        public string WhyIncomplete
        {
            get => Link.WhyIncomplete;
            set => Link.WhyIncomplete = value;
        }

        public LinkNode()
        {
            Link = new Link();
        }

        public LinkNode(Link link)
        {
            logger.Information("Building node " + link.Name);

            Link = link;

            IsBaseNode = link.Parent == null;

            Text = Link.Name;

            foreach (Link child in link.Children)
            {
                Nodes.Add(new LinkNode(child));
            }
        }

        public Link UpdateLinkTree(Link parent)
        {
            WarnIfModelDivergesFromView();

            Link.Children.Clear();
            Link.Parent = parent;
            foreach (LinkNode child in Nodes)
            {
                Link.Children.Add(child.UpdateLinkTree(Link));
            }
            return Link;
        }

        /// <summary>
        /// The view tree and Link.Children are supposed to be kept in step by the mutation
        /// helpers below. This rebuild is retained as a safety net, but any divergence means a
        /// structural edit bypassed those helpers, so log it loudly rather than silently
        /// papering over it.
        /// </summary>
        private void WarnIfModelDivergesFromView()
        {
            if (Link.Children.Count != Nodes.Count)
            {
                logger.Warning(
                    "Link tree diverged from view tree at '{LinkName}': model has {ModelCount} "
                        + "children, view has {ViewCount}. A structural edit bypassed the "
                        + "LinkNode mutation helpers.",
                    Link.Name,
                    Link.Children.Count,
                    Nodes.Count);
                return;
            }

            for (int i = 0; i < Nodes.Count; i++)
            {
                if (!ReferenceEquals(Link.Children[i], ((LinkNode)Nodes[i]).Link))
                {
                    logger.Warning(
                        "Link tree diverged from view tree at '{LinkName}': child {Index} is "
                            + "'{ModelChild}' in the model but '{ViewChild}' in the view.",
                        Link.Name,
                        i,
                        Link.Children[i]?.Name,
                        ((LinkNode)Nodes[i]).Link?.Name);
                    return;
                }
            }
        }

        public Link RebuildLink()
        {
            Link.Children.Clear();
            foreach (LinkNode child in Nodes)
            {
                Link.Children.Add(child.RebuildLink());
            }
            return Link;
        }

        // Structural mutations below keep the view tree (Nodes) and the model tree
        // (Link.Children/Link.Parent) in step. Every edit to the shape of the tree must go
        // through one of these so that Link stays authoritative and the view can be rebuilt
        // from it at any time. Construction paths that build nodes from an already-correct
        // Link graph must NOT use them, or children would be added twice.

        public void AddChildNode(LinkNode child)
        {
            Nodes.Add(child);
            Link.AddChild(child.Link);
        }

        public void InsertChildNode(int index, LinkNode child)
        {
            Nodes.Insert(index, child);
            child.Link.Parent = Link;
            Link.Children.Insert(index, child.Link);
        }

        public void RemoveChildNode(LinkNode child)
        {
            Nodes.Remove(child);
            Link.RemoveChild(child.Link);
        }

        public void RemoveLastChildNode()
        {
            if (Nodes.Count == 0)
            {
                return;
            }
            RemoveChildNode((LinkNode)Nodes[Nodes.Count - 1]);
        }

        /// <summary>
        /// Detaches this node from its parent in both the view and the model.
        /// </summary>
        public void DetachFromParent()
        {
            LinkNode parent = Parent as LinkNode;
            if (parent != null)
            {
                parent.RemoveChildNode(this);
            }
            else
            {
                Remove();
                if (Link.Parent != null)
                {
                    Link.Parent.RemoveChild(Link);
                }
            }
        }
    }
}
