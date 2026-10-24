/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using CADRobotExporter.RobotDescription;

namespace CADRobotExporter.UI
{
    /// <summary>
    /// View model backing the WPF trees in <see cref="AssemblyExportForm"/>.
    /// </summary>
    /// <remarks>
    /// This exists instead of binding straight to <see cref="LinkNode"/> because that type derives
    /// from the WinForms TreeNode and has no change notification, so a rename or a joint-name
    /// toggle could not reach the UI. It is a pure view over <see cref="Link"/> and owns no state
    /// of its own beyond presentation: the form cannot edit tree structure, so the shape here is
    /// fixed once built and can never diverge from Link.Children.
    /// </remarks>
    public sealed class LinkTreeItem : INotifyPropertyChanged
    {
        private bool useJointName;
        private bool isSelected;
        private bool isExpanded = true;

        public LinkTreeItem(Link link, bool useJointName = false)
            : this(link, useJointName, null)
        {
        }

        private LinkTreeItem(Link link, bool useJointName, LinkTreeItem parent)
        {
            Link = link;
            Parent = parent;
            this.useJointName = useJointName;
            Children = new ObservableCollection<LinkTreeItem>();
            foreach (Link child in link.Children)
            {
                Children.Add(new LinkTreeItem(child, useJointName, this));
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        public Link Link { get; }

        public LinkTreeItem Parent { get; }

        public ObservableCollection<LinkTreeItem> Children { get; }

        public bool IsBaseNode => Link.IsBaseLink;

        /// <summary>
        /// The joint tree labels nodes by the joint that attaches them to their parent; the link
        /// tree labels them by the link itself. The base link has no joint, so it falls back.
        /// </summary>
        public string DisplayName
        {
            get
            {
                if (useJointName && Link.Joint != null && !string.IsNullOrEmpty(Link.Joint.Name))
                {
                    return Link.Joint.Name;
                }

                return Link.Name;
            }
        }

        /// <summary>
        /// Two-way bound to TreeViewItem.IsSelected, which is how the form selects a node
        /// programmatically -- TreeView.SelectedItem is read only.
        /// </summary>
        public bool IsSelected
        {
            get => isSelected;
            set
            {
                if (isSelected == value)
                {
                    return;
                }

                isSelected = value;
                Raise(nameof(IsSelected));
            }
        }

        public bool IsExpanded
        {
            get => isExpanded;
            set
            {
                if (isExpanded == value)
                {
                    return;
                }

                isExpanded = value;
                Raise(nameof(IsExpanded));
            }
        }

        /// <summary>
        /// Called after the property panel writes a new name back to the Link, so the tree label
        /// follows the edit instead of going stale until the tab is rebuilt.
        /// </summary>
        public void RefreshDisplayName()
        {
            Raise(nameof(DisplayName));
        }

        /// <summary>
        /// Whether this link carries its own meshing options. Bound by the link tree so overridden
        /// links can be spotted without selecting each one in turn.
        /// </summary>
        public bool HasMeshingOverride => Link.meshingOverride;

        /// <summary>
        /// Called after the meshing override is toggled on the Link. Link.meshingOverride is a plain
        /// field with no notification of its own, so every site that flips it has to say so.
        /// </summary>
        public void RefreshMeshingOverride()
        {
            Raise(nameof(HasMeshingOverride));
        }

        public IEnumerable<LinkTreeItem> SelfAndDescendants()
        {
            yield return this;
            foreach (LinkTreeItem child in Children)
            {
                foreach (LinkTreeItem descendant in child.SelfAndDescendants())
                {
                    yield return descendant;
                }
            }
        }

        public LinkTreeItem FindByLinkName(string linkName)
        {
            foreach (LinkTreeItem item in SelfAndDescendants())
            {
                if (item.Link.Name == linkName)
                {
                    return item;
                }
            }

            return null;
        }

        /// <summary>
        /// Expands every ancestor so a programmatic selection is actually realised and on screen.
        /// </summary>
        public void ExpandAncestors()
        {
            for (LinkTreeItem ancestor = Parent; ancestor != null; ancestor = ancestor.Parent)
            {
                ancestor.IsExpanded = true;
            }
        }

        private void Raise(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
