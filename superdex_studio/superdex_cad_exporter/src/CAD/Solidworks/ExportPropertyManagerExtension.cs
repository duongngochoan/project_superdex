/*
Copyright (c) 2015 Stephen Brawner
Copyright (c) Meta Platforms, Inc. and affiliates.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#if SOLIDWORKS

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;

using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI;

namespace CADRobotExporter.CAD
{
    public partial class ExportPropertyManager : PropertyManagerPage2Handler9
    {
        //As nodes are created and destroyed, this menu gets called a lot. It basically just
        // adds the context menu (right-click menu) to the node
        public void AddDocMenu(LinkNode node)
        {
            node.ContextMenuStrip = docMenu;
            foreach (LinkNode child in node.Nodes)
            {
                AddDocMenu(child);
            }
        }

        // Populates the combo box with feature names
        private void FillComboBox(PropertyManagerPageCombobox box, List<string> featureNames)
        {
            box.Clear();
            box.AddItems("Automatically Generate");
            foreach (string name in featureNames)
            {
                box.AddItems(name);
            }
        }

        // Finds the specified item in a combobox and sets the box to it. I'm not sure why I
        // couldn't do this with a foreach loop or even a for loop, but there is no way to get
        // the current number of items in the menu
        private void SelectComboBox(PropertyManagerPageCombobox box, string item)
        {
            short i = 0;
            string itemtext = "nothing";
            box.CurrentSelection = 0;

            // Cycles through the menu items until it finds what its looking for, it finds
            // blank strings, or itemtext is null
            while (!string.IsNullOrWhiteSpace(itemtext) && itemtext != item)
            {
                // Gets the item text at index in a pull-down menu. No way to now how many
                // items are in the combobox
                itemtext = box.get_ItemText(i);
                if (itemtext == item)
                {
                    box.CurrentSelection = i;
                }
                i++;
            }
        }

        // Adds the number of empty nodes to the currently active node
        private void CreateNewNodes(LinkNode currentNode, int number)
        {
            for (int i = 0; i < number; i++)
            {
                LinkNode node = CreateEmptyNode(currentNode);
                currentNode.AddChildNode(node);
            }
            for (int i = 0; i < -number; i++)
            {
                currentNode.RemoveLastChildNode();
            }
            currentNode.ExpandAll();
        }

        private void CreateSerialChain(int number)
        {
            LinkNode currentNode = (LinkNode)Tree.SelectedNode;
            LinkNode node = currentNode;
            for (int i = 0; i < number; i++)
            {
                LinkNode newNode = CreateEmptyNode(node);
                node.AddChildNode(newNode);
                node = newNode;
            }
            currentNode.ExpandAll();
        }

        private void InsertParentNode()
        {
            LinkNode currentNode = (LinkNode)Tree.SelectedNode;
            if (currentNode == _rootNode)
            {
                return;
            }

            LinkNode parent = (LinkNode)currentNode.Parent;

            int index = parent.Nodes.IndexOf(currentNode);

            parent.RemoveChildNode(currentNode);

            LinkNode newParent = CreateEmptyNode(parent);
            newParent.AddChildNode(currentNode);

            parent.InsertChildNode(index, newParent);

            newParent.ExpandAll();

            Tree.SelectedNode = newParent;
        }

        private void ImportTreeFromFile()
        {
            try
            {
                using (OpenFileDialog openFileDialog = new OpenFileDialog())
                {
                    openFileDialog.Filter = "Text files (*.txt)|*.txt|All files (*.*)|*.*";
                    openFileDialog.Title = "Import Tree Structure";
                    openFileDialog.DefaultExt = "txt";

                    if (openFileDialog.ShowDialog() == DialogResult.OK)
                    {
                        string filePath = openFileDialog.FileName;
                        string text = System.IO.File.ReadAllText(filePath);

                        if (Import.TreeTextImporter.TryParse(text, out var importedRoot, out string error))
                        {
                            // Confirm replacement
                            if (_rootNode != null && _rootNode.Nodes.Count > 0)
                            {
                                var result = MessageBox.Show(
                                    "This will replace the existing tree. Continue?",
                                    "Import Tree",
                                    MessageBoxButtons.YesNo,
                                    MessageBoxIcon.Question);

                                if (result != DialogResult.Yes)
                                    return;
                            }

                            // Clear existing tree
                            Tree.Nodes.Clear();

                            // Build tree from imported structure
                            _rootNode = BuildTreeFromImported(importedRoot);
                            Tree.Nodes.Add(_rootNode);
                            AddDocMenu(_rootNode);

                            _rootNode.ExpandAll();
                            Tree.SelectedNode = _rootNode;

                            MessageBox.Show(
                                $"Successfully imported tree from:\n{filePath}",
                                "Import Tree",
                                MessageBoxButtons.OK,
                                MessageBoxIcon.Information);
                        }
                        else
                        {
                            MessageBox.Show(
                                $"Failed to parse file:\n{error}",
                                "Import Tree",
                                MessageBoxButtons.OK,
                                MessageBoxIcon.Error);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                logger.Error($"ImportTreeFromFile error: {ex.Message}");
                MessageBox.Show(ex.Message, "Import Tree", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void ExportTreeToFile()
        {
            try
            {
                if (_rootNode == null)
                {
                    MessageBox.Show(
                        "No tree to export.",
                        "Export Tree",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Warning);
                    return;
                }

                // Build ImportedTreeNode structure from current tree
                var exportedRoot = BuildExportedTree(_rootNode.Link);
                string text = Import.TreeTextImporter.Export(exportedRoot);

                using (SaveFileDialog saveFileDialog = new SaveFileDialog())
                {
                    saveFileDialog.Filter = "Text files (*.txt)|*.txt|All files (*.*)|*.*";
                    saveFileDialog.Title = "Export Tree Structure";
                    saveFileDialog.DefaultExt = "txt";
                    saveFileDialog.FileName = $"{_rootNode.Link.Name}_tree.txt";

                    if (saveFileDialog.ShowDialog() == DialogResult.OK)
                    {
                        System.IO.File.WriteAllText(saveFileDialog.FileName, text);
                        MessageBox.Show(
                            $"Successfully exported tree to:\n{saveFileDialog.FileName}",
                            "Export Tree",
                            MessageBoxButtons.OK,
                            MessageBoxIcon.Information);
                    }
                }
            }
            catch (Exception ex)
            {
                logger.Error($"ExportTreeToFile error: {ex.Message}");
                MessageBox.Show(ex.Message, "Export Tree", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        /// <summary>
        /// Builds a LinkNode tree from an imported tree structure.
        /// </summary>
        private LinkNode BuildTreeFromImported(Import.ImportedTreeNode importedRoot)
        {
            Link rootLink = new Link
            {
                Name = importedRoot.LinkName,
                IsBaseLink = true,
            };
            rootLink.Joint.Type = "fixed";

            LinkNode rootNode = new LinkNode(rootLink)
            {
                IsBaseNode = true,
                Name = importedRoot.LinkName,
                Text = importedRoot.LinkName,
            };

            foreach (var child in importedRoot.Children)
            {
                rootNode.AddChildNode(CreateNodeFromImported(child, rootLink));
            }

            return rootNode;
        }

        private LinkNode CreateNodeFromImported(Import.ImportedTreeNode imported, Link parentLink)
        {
            Link link = new Link
            {
                Name = imported.LinkName,
                Parent = parentLink,
                IsBaseLink = false,
            };
            link.Joint.Name = imported.JointName ?? $"joint_{imported.LinkName}";
            link.Joint.Type = imported.JointType;

            LinkNode node = new LinkNode(link)
            {
                IsBaseNode = false,
                Name = imported.LinkName,
                Text = imported.LinkName,
            };

            foreach (var child in imported.Children)
            {
                node.AddChildNode(CreateNodeFromImported(child, link));
            }

            return node;
        }

        /// <summary>
        /// Builds an ImportedTreeNode structure from the current tree for export.
        /// </summary>
        private Import.ImportedTreeNode BuildExportedTree(Link link)
        {
            var exported = new Import.ImportedTreeNode(
                link.Name,
                link.Joint.Name,
                link.IsBaseLink ? "fixed" : link.Joint.Type);

            foreach (Link child in link.Children)
            {
                var childExported = BuildExportedTree(child);
                childExported.Parent = exported;
                exported.Children.Add(childExported);
            }

            return exported;
        }

        private void CreateChildNode()
        {
            LinkNode currentNode = (LinkNode)Tree.SelectedNode;

            LinkNode newNode = CreateEmptyNode(currentNode);
            currentNode.AddChildNode(newNode);

            currentNode.ExpandAll();

            Tree.SelectedNode = newNode;
        }

        // When a new node is selected or another node is found that needs to be visited, this
        // method saves the previously active node and fills in the property mananger with the new one
        public void SwitchActiveNodes(LinkNode node)
        {
            SaveActiveNode();

            Font fontRegular = new Font(Tree.Font, FontStyle.Regular);
            Font fontBold = new Font(Tree.Font, FontStyle.Bold);
            if (previouslySelectedNode != null)
            {
                previouslySelectedNode.NodeFont = fontRegular;
            }
            FillPropertyManager(node);

            //If this flag is set to true, it prevents this method from getting called again when
            // changing the selected node
            automaticallySwitched = true;

            //Change the selected node to the argument node. This highlights the newly activated node
            Tree.SelectedNode = node;

            node.NodeFont = fontBold;
            // Reassigning Text forces the native tree to re-measure the node; without it
            // the bold text is clipped to the width computed for the regular font.
            node.Text = node.Text;
            previouslySelectedNode = node;
            CheckNodeComplete(node.Link);
        }

        // This method runs through first the child nodes of the selected node to see if there are
        // more to visit then it runs through the nodes top to bottom to find the next to visit.
        // Returns the node if one is found otherwise it returns null.
        public LinkNode FindNextLinkToVisit(TreeView tree)
        {
            // First check if SelectedNode has any nodes to visit
            if (tree.SelectedNode != null)
            {
                LinkNode nodeToReturn = FindNextLinkToVisit((LinkNode)tree.SelectedNode);
                if (nodeToReturn != null)
                {
                    return nodeToReturn;
                }
            }

            // Now run through tree to see if any other nodes need to be visited
            return FindNextLinkToVisit((LinkNode)tree.Nodes[0]);
        }

        // Finds the next incomplete node and returns that
        public LinkNode FindNextLinkToVisit(LinkNode nodeToCheck)
        {
            if (nodeToCheck.Link.IsIncomplete)
            {
                return nodeToCheck;
            }
            foreach (LinkNode node in nodeToCheck.Nodes)
            {
                return FindNextLinkToVisit(node);
            }
            return null;
        }

        //Sets the link's isIncomplete flag if the link has key items that need to be completed
        public void CheckNodeComplete(Link link)
        {
            link.WhyIncomplete = "";
            link.IsIncomplete = false;
            if (String.IsNullOrWhiteSpace(link.Name))
            {
                link.IsIncomplete = true;
                link.WhyIncomplete += "        Link name is empty. Fill in a unique link name\r\n";
            }
            if (String.IsNullOrWhiteSpace(link.Joint.Name) && !link.IsBaseLink)
            {
                link.IsIncomplete = true;
                link.WhyIncomplete += "        Joint name is empty. Fill in a unique joint name\r\n";
            }
        }

        private void CheckModelDocsExist(Link link, List<string> problemComponents)
        {
            foreach (Component2 component in link.SWVisualComponents)
            {
                ModelDoc2 doc = component.GetModelDoc2();
                if (doc == null)
                {
                    problemComponents.Add(component.Name2);
                }
            }

            foreach (Link child in link.Children)
            {
                CheckModelDocsExist(child, problemComponents);
            }
        }

        //Recursive function to iterate though links and build a message containing those that are incomplete
        public string CheckNodesComplete(Link link, string incompleteNodes)
        {
            // Determine if the link is incomplete
            CheckNodeComplete(link);
            if (link.IsIncomplete)
            {
                //Building the message
                incompleteNodes += "    '" + link.Name + "':\r\n" + link.WhyIncomplete + "\r\n\r\n";
            }
            // Cycle through the rest of the links
            foreach (Link child in link.Children)
            {
                incompleteNodes = CheckNodesComplete(child, incompleteNodes);
            }
            return incompleteNodes;
        }

        // Finds all the nodes in a TreeView that need to be completed before exporting
        public bool CheckNodesComplete(TreeView tree)
        {
            //Calls the recursive function starting with the base_link node and retrieves a string
            // identifying the incomplete nodes
            string incompleteNodes = CheckNodesComplete(((LinkNode)tree.Nodes[0]).Link, "");
            if (!String.IsNullOrWhiteSpace(incompleteNodes))
            {
                MessageBox.Show(
                    "The following nodes are incomplete. You need to fix them before continuing.\r\n\r\n" + incompleteNodes);
                return false;
            }
            return true;
        }

        // NB: This is not robust. We'll assume no sane human being uses backslashes in their
        // coordinate system names. This removes stuff like '\Y Axis'
        private string GetCoordinateSystemName(string SelectedCoordinateSystemName)
        {
            if (!SelectedCoordinateSystemName.Contains("\\"))
            {
                return SelectedCoordinateSystemName;
            }
            int trimStart = SelectedCoordinateSystemName.IndexOf("\\");
            int trimEnd = SelectedCoordinateSystemName.IndexOf("@");

            if (trimEnd < 0)
            {
                trimEnd = SelectedCoordinateSystemName.Length;
            }

            return SelectedCoordinateSystemName.Remove(trimStart, trimEnd - trimStart);
        }

        // When the selected node is changed, the previously active node needs to be saved
        public void SaveActiveNode()
        {
            if (previouslySelectedNode != null)
            {
                previouslySelectedNode.Link.Name = PMTextBoxLinkName.Text;

                if (previouslySelectedNode.Link.isSite)
                {
                    // Sites only need CSYS and have a fixed joint
                    previouslySelectedNode.Link.Joint.Name = previouslySelectedNode.Link.Name + "_joint";
                    previouslySelectedNode.Link.Joint.Type = "fixed";
                    previouslySelectedNode.Link.Joint.AxisName = "";
                    previouslySelectedNode.Link.Joint.SWRefAxisFeature = null;

                    Feature coordSys = model.SelectionManager.GetSelectedObject6(1, 2);
                    if (coordSys != null)
                    {
                        previouslySelectedNode.Link.Joint.SWCoordinateSystemFeature = coordSys;
                        previouslySelectedNode.Link.Joint.CoordinateSystemName = coordSys.GetNameForSelection(out _);
                    }
                }
                else if (!previouslySelectedNode.IsBaseNode)
                {
                    previouslySelectedNode.Link.Joint.Name = PMTextBoxJointName.Text;

                    // Check if using Axis from CSYS
                    if (PMOptionAxisX.Checked)
                    {
                        previouslySelectedNode.Link.Joint.AxisName = Joint.AxisFromCsysX;
                        previouslySelectedNode.Link.Joint.SWRefAxisFeature = null;
                    }
                    else if (PMOptionAxisY.Checked)
                    {
                        previouslySelectedNode.Link.Joint.AxisName = Joint.AxisFromCsysY;
                        previouslySelectedNode.Link.Joint.SWRefAxisFeature = null;
                    }
                    else if (PMOptionAxisZ.Checked)
                    {
                        previouslySelectedNode.Link.Joint.AxisName = Joint.AxisFromCsysZ;
                        previouslySelectedNode.Link.Joint.SWRefAxisFeature = null;
                    }
                    else
                    {
                        // Using reference axis selection
                        Feature refAxis = model.SelectionManager.GetSelectedObject6(1, 4);
                        if (refAxis != null)
                        {
                            previouslySelectedNode.Link.Joint.SWRefAxisFeature = refAxis;
                            previouslySelectedNode.Link.Joint.AxisName = refAxis.GetNameForSelection(out _);
                        }
                    }

                    Feature coordSys = model.SelectionManager.GetSelectedObject6(1, 2);
                    if (coordSys != null)
                    {
                        previouslySelectedNode.Link.Joint.SWCoordinateSystemFeature = coordSys;
                        previouslySelectedNode.Link.Joint.CoordinateSystemName = coordSys.GetNameForSelection(out _);
                    }

                    previouslySelectedNode.Link.Joint.Type = GetJointTypeFromOptions();

                    ReadGeometrySelections(previouslySelectedNode.Link);
                    previouslySelectedNode.Link.shouldFlipAxis = PMRefAxisFlipCheckbox.Checked;
                    previouslySelectedNode.Link.visualsOnly = PMCheckboxLinkPurelyVisual.Checked;
                    previouslySelectedNode.Link.inertialsOnly = PMCheckboxLinkPurelyInertial.Checked;
                }
                else
                {
                    previouslySelectedNode.Link.Joint.Name = ""; // base links should not have a joint name
                    Feature coordSys = model.SelectionManager.GetSelectedObject6(1, 2);
                    if (coordSys != null)
                    {
                        previouslySelectedNode.Link.Joint.SWCoordinateSystemFeature = coordSys;
                        previouslySelectedNode.Link.Joint.CoordinateSystemName = coordSys.GetNameForSelection(out _);
                    }
                    ReadGeometrySelections(previouslySelectedNode.Link);
                    previouslySelectedNode.Link.shouldFlipAxis = PMRefAxisFlipCheckbox.Checked;
                    previouslySelectedNode.Link.visualsOnly = PMCheckboxLinkPurelyVisual.Checked;
                    previouslySelectedNode.Link.inertialsOnly = PMCheckboxLinkPurelyInertial.Checked;
                }
            }
        }

        //Creates an Empty node when children are added to a link
        public LinkNode CreateEmptyNode(LinkNode Parent, bool autoNumber = true)
        {
            LinkNode node = new LinkNode();
            if (Parent == null)  // For the base_link node
            {
                node.Link.Name = "base_link";
                node.Link.Joint.AxisName = "";
                node.Link.Joint.CoordinateSystemName = "";
                node.Link.SWVisualComponents = new List<Component2>();
                node.Link.SWCollisionComponents = new List<Component2>();
                node.Link.SWInertialComponents = new List<Component2>();
                node.IsBaseNode = true;
                node.IsIncomplete = true;
            }
            else
            {
                node.IsBaseNode = false;

                // Apply auto-numbering if enabled
                if (autoNumber)
                {
                    node.Link.Name = GenerateLinkName(Parent.Link);
                    node.Link.Joint.Name = $"joint_{node.Link.Name}";
                }
                else
                {
                    node.Link.Name = "empty_link";
                }

                node.Link.Joint.AxisName = "";
                node.Link.Joint.CoordinateSystemName = "";
                node.Link.Joint.Type = "revolute";
                node.Link.SWVisualComponents = new List<Component2>();
                node.Link.SWCollisionComponents = new List<Component2>();
                node.IsIncomplete = true;
            }

            node.Text = node.Link.Name;
            node.ContextMenuStrip = docMenu;

            return node;
        }

        // Generate link name based on parent and its existing children
        private string GenerateLinkName(Link parent)
        {
            string prefix;

            // Check if parent already has children
            if (parent.Children != null && parent.Children.Count > 0)
            {
                // Subsequent child: use first sibling's full name as prefix
                prefix = parent.Children[0].Name;
            }
            else
            {
                // First child: extract prefix from parent's name
                if (parent.IsBaseLink)
                {
                    prefix = "link";  // Default for children of base_link
                }
                else
                {
                    prefix = ExtractPrefix(parent.Name);
                }
            }

            int nextNumber = GetNextNumber(prefix);
            return $"{prefix}_{nextNumber}";
        }

        // Extract prefix from a link name
        // "link_1" -> "link", "link_2" -> "link", "foo_1" -> "foo", "foo_2_1" -> "foo_2"
        private static string ExtractPrefix(string linkName)
        {
            if (string.IsNullOrEmpty(linkName))
            {
                return "link";
            }

            var parts = linkName.Split('_');

            // Check if last part is a number
            if (parts.Length > 1 && int.TryParse(parts[parts.Length - 1], out _))
            {
                // Return everything except the last number
                return string.Join("_", parts, 0, parts.Length - 1);
            }

            // No number found, return the whole name as prefix
            return linkName;
        }

        // Get and increment the counter for a specific prefix
        private int GetNextNumber(string prefix)
        {
            if (!_linkCounters.ContainsKey(prefix))
            {
                _linkCounters[prefix] = 0;
            }

            _linkCounters[prefix]++;
            return _linkCounters[prefix];
        }

        // Optional: Call this when user manually renames a node
        public void OnNodeRenamed(LinkNode node, string newName)
        {
            // Update counters if the new name has a number
            var parts = newName.Split('_');
            if (parts.Length > 1 && int.TryParse(parts[parts.Length - 1], out int number))
            {
                string prefix = string.Join("_", parts, 0, parts.Length - 1);

                if (!_linkCounters.ContainsKey(prefix))
                {
                    _linkCounters[prefix] = number;
                }
                else
                {
                    _linkCounters[prefix] = Math.Max(_linkCounters[prefix], number);
                }
            }

            node.Link.Name = newName;
            node.Text = newName;
        }

        // Optional: Sync counters from existing tree (call after loading)
        public void SyncCountersWithTree(LinkNode rootNode)
        {
            _linkCounters.Clear();
            TraverseAndRegister(rootNode?.Link);
        }

        private void TraverseAndRegister(Link link)
        {
            if (link == null) return;

            if (!link.IsBaseLink)
            {
                var parts = link.Name.Split('_');
                if (parts.Length > 1 && int.TryParse(parts[parts.Length - 1], out int number))
                {
                    string prefix = string.Join("_", parts, 0, parts.Length - 1);

                    if (!_linkCounters.ContainsKey(prefix))
                    {
                        _linkCounters[prefix] = number;
                    }
                    else
                    {
                        _linkCounters[prefix] = Math.Max(_linkCounters[prefix], number);
                    }
                }
            }

            if (link.Children != null)
            {
                foreach (Link child in link.Children)
                {
                    TraverseAndRegister(child);
                }
            }
        }

        // Optional: Reset all counters (for new documents)
        public void ResetLinkCounters()
        {
            _linkCounters.Clear();
        }

        private void ReadGeometrySelections(Link link)
        {
            if (isPartDoc)
            {
                CommonSwOperations.GetSelectedBodies(
                    model, link.SWVisualBodies, PMLinkVisualComponentsSelection.Mark);
                CommonSwOperations.GetSelectedBodies(
                    model, link.SWCollisionBodies, PMLinkCollisionComponentsSelection.Mark);
                CommonSwOperations.GetSelectedBodies(
                    model, link.SWInertialBodies, PMLinkInertialComponentsSelection.Mark);
                return;
            }

            CommonSwOperations.GetSelectedComponents(
                model, link.SWVisualComponents, PMLinkVisualComponentsSelection.Mark);
            CommonSwOperations.GetSelectedComponents(
                model, link.SWCollisionComponents, PMLinkCollisionComponentsSelection.Mark);
            CommonSwOperations.GetSelectedComponents(
                model, link.SWInertialComponents, PMLinkInertialComponentsSelection.Mark);
        }

        private void WriteGeometrySelections(Link link)
        {
            if (isPartDoc)
            {
                CommonSwOperations.SelectBodies(
                    model, link.SWVisualBodies, true, PMLinkVisualComponentsSelection.Mark);
                CommonSwOperations.SelectBodies(
                    model, link.SWCollisionBodies, false, PMLinkCollisionComponentsSelection.Mark);
                CommonSwOperations.SelectBodies(
                    model, link.SWInertialBodies, false, PMLinkInertialComponentsSelection.Mark);
                return;
            }

            CommonSwOperations.SelectComponents(
                model, link.SWVisualComponents, true, PMLinkVisualComponentsSelection.Mark);
            CommonSwOperations.SelectComponents(
                model, link.SWCollisionComponents, false, PMLinkCollisionComponentsSelection.Mark);
            CommonSwOperations.SelectComponents(
                model, link.SWInertialComponents, false, PMLinkInertialComponentsSelection.Mark);
        }

        // Sets all the controls in the Property Manager from the Selected Node
        public void FillPropertyManager(LinkNode node)
        {
            PMTextBoxLinkName.Text = node.Link.Name;
            PMOptionLinkTypeLink.Checked = !node.Link.isSite;
            PMOptionLinkTypeSite.Checked = node.Link.isSite;

            model.ClearSelection2(true);

            if (node.Link.isSite)
            {
                if (node.Link.Joint.SWCoordinateSystemFeature != null)
                {
                    string coordSysName = node.Link.Joint.SWCoordinateSystemFeature.GetNameForSelection(out _);
                    CommonSwOperations.SelectCoordinateSystem(model, coordSysName, PMCoordinateSystemSelection.Mark);
                }

                PMTextBoxJointName.Text = "";
                PMLabelParentLinkName.Caption = node.Link.Parent?.Name ?? "(none)";
                PMLabelCoordSys.Caption = "Site Reference Coordinate System:";
                SelectOptionsFromJointType("NONE");
                SelectAxisOption("");
            }
            else if (!node.IsBaseNode)
            {
                PMRefAxisFlipCheckbox.Checked = node.Link.shouldFlipAxis;

                WriteGeometrySelections(node.Link);

                PMCheckboxLinkPurelyInertial.Checked = node.Link.inertialsOnly;
                PMCheckboxLinkPurelyVisual.Checked = node.Link.visualsOnly;

                if (node.Link.Joint.SWCoordinateSystemFeature != null)
                {
                    string coordSysName = node.Link.Joint.SWCoordinateSystemFeature.GetNameForSelection(out _);
                    CommonSwOperations.SelectCoordinateSystem(model, coordSysName, PMCoordinateSystemSelection.Mark);
                }

                PMLabelCoordSys.Caption = "Link Reference Coodinate System:";

                string axisName = node.Link.Joint.AxisName;
                SelectAxisOption(axisName);
                if (!Joint.IsAxisFromCsys(axisName) && node.Link.Joint.SWRefAxisFeature != null)
                {
                    string refAxisName = node.Link.Joint.SWRefAxisFeature.GetNameForSelection(out _);
                    CommonSwOperations.SelectAxis(model, refAxisName, PMRefAxisSelection.Mark);
                }

                PMTextBoxJointName.Text = node.Link.Joint.Name;
                PMLabelParentLinkName.Caption = node.Link.Parent?.Name ?? "(none)";
                SelectOptionsFromJointType(node.Link.Joint.Type);
            }
            else
            {
                WriteGeometrySelections(node.Link);
                PMCheckboxLinkPurelyInertial.Checked = node.Link.inertialsOnly;
                PMCheckboxLinkPurelyVisual.Checked = node.Link.visualsOnly;
                PMRefAxisFlipCheckbox.Checked = node.Link.shouldFlipAxis;

                if (node.Link.Joint.SWCoordinateSystemFeature != null)
                {
                    string coordSysName = node.Link.Joint.SWCoordinateSystemFeature.GetNameForSelection(out _);
                    CommonSwOperations.SelectCoordinateSystem(model, coordSysName, PMCoordinateSystemSelection.Mark);
                }

                PMLabelParentLinkName.Caption = "(none)";
                PMLabelCoordSys.Caption = "Global Reference Coodinate System:";
                PMTextBoxJointName.Text = "";
                SelectOptionsFromJointType("NONE");
                SelectAxisOption("");
            }

            UpdateKinematicControlStates(node);
        }

        private void UpdateKinematicControlStates()
        {
            UpdateKinematicControlStates(previouslySelectedNode);
        }

        private void UpdateKinematicControlStates(LinkNode node)
        {
            if (node == null) return;

            bool isBaseNode = node.IsBaseNode;
            bool isSite = node.Link.isSite;
            bool isFixedJoint = node.Link.Joint.Type == "fixed";
            bool axisFromCsys = Joint.IsAxisFromCsys(node.Link.Joint.AxisName);

            ((PropertyManagerPageControl)PMOptionLinkTypeLink).Enabled = !isBaseNode;
            ((PropertyManagerPageControl)PMOptionLinkTypeSite).Enabled = !isBaseNode;

            ((PropertyManagerPageControl)PMTextBoxJointName).Enabled = !isBaseNode && !isSite;
            ((PropertyManagerPageControl)PMLabelJointName).Enabled = !isBaseNode && !isSite;
            ((PropertyManagerPageControl)PMLabelJointType).Enabled = !isBaseNode && !isSite;
            ((PropertyManagerPageControl)PMOptionRevoluteJoint).Enabled = !isBaseNode && !isSite;
            ((PropertyManagerPageControl)PMOptionPrismaticJoint).Enabled = !isBaseNode && !isSite;
            ((PropertyManagerPageControl)PMOptionFixedJoint).Enabled = !isBaseNode && !isSite;

            ((PropertyManagerPageControl)PMOptionAxisRefAxis).Enabled = !isBaseNode && !isSite && !isFixedJoint;
            ((PropertyManagerPageControl)PMOptionAxisX).Enabled = !isBaseNode && !isSite && !isFixedJoint;
            ((PropertyManagerPageControl)PMOptionAxisY).Enabled = !isBaseNode && !isSite && !isFixedJoint;
            ((PropertyManagerPageControl)PMOptionAxisZ).Enabled = !isBaseNode && !isSite && !isFixedJoint;

            ((IPropertyManagerPageControl)PMRefAxisSelection).Enabled = !isBaseNode && !isSite && !isFixedJoint && !axisFromCsys;
            ((PropertyManagerPageControl)PMLabelAxes).Enabled = !isBaseNode && !isSite && !isFixedJoint && !axisFromCsys;
            ((PropertyManagerPageControl)PMRefAxisFlipCheckbox).Enabled = !isBaseNode && !isSite && !isFixedJoint;

            ((PropertyManagerPageControl)PMLinkVisualComponentsSelection).Enabled = !isSite;
            ((PropertyManagerPageControl)PMLinkCollisionComponentsSelection).Enabled = !isSite;
            ((PropertyManagerPageControl)PMLinkInertialComponentsSelection).Enabled = !isSite;
            ((PropertyManagerPageControl)PMLabelLinkVisualComponents).Enabled = !isSite;
            ((PropertyManagerPageControl)PMLabelLinkCollisionComponents).Enabled = !isSite;
            ((PropertyManagerPageControl)PMLabelLinkInertialComponents).Enabled = !isSite;
            ((PropertyManagerPageControl)PMCheckboxLinkPurelyVisual).Enabled = !isSite;
            ((PropertyManagerPageControl)PMCheckboxLinkPurelyInertial).Enabled = !isSite;
        }

        private void OnLinkTypeChanged(int item)
        {
            if (previouslySelectedNode == null || previouslySelectedNode.IsBaseNode) return;

            bool isSite = (item == 1);
            previouslySelectedNode.Link.isSite = isSite;

            if (isSite)
            {
                previouslySelectedNode.Link.Joint.Type = "fixed";
                previouslySelectedNode.Link.Joint.AxisName = "";
                previouslySelectedNode.Link.Joint.SWRefAxisFeature = null;
                previouslySelectedNode.Link.SWVisualComponents.Clear();
                previouslySelectedNode.Link.SWCollisionComponents.Clear();
                previouslySelectedNode.Link.SWInertialComponents.Clear();
                previouslySelectedNode.Link.visualsOnly = false;
                previouslySelectedNode.Link.inertialsOnly = false;

                if (!previouslySelectedNode.Link.Name.StartsWith("site_"))
                {
                    previouslySelectedNode.Link.Name = "site_" + previouslySelectedNode.Link.Name;
                    PMTextBoxLinkName.Text = previouslySelectedNode.Link.Name;
                    previouslySelectedNode.Text = previouslySelectedNode.Link.Name;
                }

                SelectOptionsFromJointType("NONE");
            }
            else
            {
                previouslySelectedNode.Link.Joint.Type = "revolute";
                SelectOptionsFromJointType("revolute");
            }

            UpdateKinematicControlStates();
        }

        public string GetJointTypeFromOptions()
        {
            if (PMOptionRevoluteJoint.Checked)
                return "revolute";

            if (PMOptionPrismaticJoint.Checked)
                return "prismatic";

            if (PMOptionFixedJoint.Checked)
                return "fixed";

            return "";
        }

        public void SelectOptionsFromJointType(string jointType)
        {
            switch (jointType)
            {
                case "revolute":
                    PMOptionRevoluteJoint.Checked = true;
                    PMOptionPrismaticJoint.Checked = false;
                    PMOptionFixedJoint.Checked = false;
                    break;
                case "prismatic":
                    PMOptionRevoluteJoint.Checked = false;
                    PMOptionPrismaticJoint.Checked = true;
                    PMOptionFixedJoint.Checked = false;
                    break;
                case "fixed":
                    PMOptionRevoluteJoint.Checked = false;
                    PMOptionPrismaticJoint.Checked = false;
                    PMOptionFixedJoint.Checked = true;
                    break;
                default:
                case "NONE":
                    PMOptionRevoluteJoint.Checked = false;
                    PMOptionPrismaticJoint.Checked = false;
                    PMOptionFixedJoint.Checked = false;
                    break;

            }
        }

        private void SelectAxisOption(string axisName)
        {
            bool isX = axisName == Joint.AxisFromCsysX;
            bool isY = axisName == Joint.AxisFromCsysY;
            bool isZ = axisName == Joint.AxisFromCsysZ;
            PMOptionAxisRefAxis.Checked = !isX && !isY && !isZ;
            PMOptionAxisX.Checked = isX;
            PMOptionAxisY.Checked = isY;
            PMOptionAxisZ.Checked = isZ;
        }

        // Populates the TreeView with the organized links from the robot
        public void FillTreeViewFromRobot(RobotDescription.Robot robot)
        {
            Tree.Nodes.Clear();
            LinkNode baseNode = new LinkNode();
            Link baseLink = robot.BaseLink;
            baseNode.Text = baseLink.Name;
            baseNode.Link = baseLink;
            baseNode.ContextMenuStrip = docMenu;

            foreach (Link child in baseLink.Children)
            {
                baseNode.Nodes.Add(CreateLinkNodeFromLink(child));
            }
            Tree.Nodes.Add(baseNode);
            Tree.ExpandAll();
    }

        // Similar to the AssemblyExportForm method. It creates a LinkNode from a Link object
        public LinkNode CreateLinkNodeFromLink(Link Link)
        {
            LinkNode node = new LinkNode();
            node.Text = Link.Name;
            node.Link = Link;
            node.ContextMenuStrip = docMenu;

            foreach (Link child in Link.Children)
            {
                node.Nodes.Add(CreateLinkNodeFromLink(child));
            }

            return node;
        }

        /// <summary>
        /// Loads configuration tree into PM Page. If an error occurs, this will do nothing
        /// </summary>
        /// <returns>bool representing success of load. If false, PMPage should not open</returns>
        public bool LoadConfigTree(LinkNode rootNode)
        {
            _rootNode = rootNode;

            SyncCountersWithTree(_rootNode);
            SetConfigTree(_rootNode);

            IPropertyManagerPageControl loadConfigurationControl = (IPropertyManagerPageControl)PMButtonLoad;

            if (_rootNode == null || !_rootNode.RebuildLink().AreRequiredFieldsSatisfied())
            {
                loadConfigurationControl.Tip =
                    "Your configuration has not been fully exported. This feature may not work correctly";
            }

            return true;
        }

        private void SetConfigTree(LinkNode baseNode)
        {
            if (baseNode == null)
            {
                logger.Information("Starting new configuration");
                baseNode = CreateEmptyNode(null);
                _rootNode = baseNode;
            }
            else
            {
                List<string> problemLinks = new List<string>();
                CommonSwOperations.LoadSWComponents(model, baseNode.Link, problemLinks);

                if (problemLinks.Count > 0)
                {
                    string msg = "The following links had issues loading their associated SolidWorks components. " +
                        "Please inspect before exporting\r\n\r\n" +
                        string.Join(", ", problemLinks);
                    MessageBox.Show(msg);
                }
            }

            AddDocMenu(baseNode);

            Tree.Nodes.Clear();
            Tree.Nodes.Add(baseNode);
            Tree.ExpandAll();
            Tree.SelectedNode = Tree.Nodes[0];
        }

        public void MoveComponentsToFolder(LinkNode node)
        {
            bool needToCreateFolder = true;
            Object[] objects = model.FeatureManager.GetFeatures(true);
            const string robotExporterItemsFolder = "Robot Exporter Items";
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                if (feat.Name == robotExporterItemsFolder)
                {
                    needToCreateFolder = false;
                }
            }
            model.ClearSelection2(true);
            model.Extension.SelectByID2(
                "Origin_global", "COORDSYS", 0, 0, 0, true, 0, null, 0);
            if (needToCreateFolder)
            {
                Feature folderFeature =
                    model.FeatureManager.InsertFeatureTreeFolder2(
                        (int)swFeatureTreeFolderType_e.swFeatureTreeFolder_Containing);
                folderFeature.Name = "URDF Export Items";
            }
            model.Extension.SelectByID2
                ("Robot Exporter Reference", "SKETCH", 0, 0, 0, true, 0, null, 0);
            model.FeatureManager.MoveToFolder(robotExporterItemsFolder, "", false);
            model.Extension.SelectByID2
                (ConfigurationSerialization.UrdfCongfigurationFeatureName, "ATTRIBUTE", 0, 0, 0, true, 0, null, 0);
            model.FeatureManager.MoveToFolder(robotExporterItemsFolder, "", false);
            SelectFeatures(node.Link);
            model.FeatureManager.MoveToFolder(robotExporterItemsFolder, "", false);
        }

        public void SelectFeatures(Link link)
        {
            model.Extension.SelectByID2(
                link.Joint.CoordinateSystemName, "COORDSYS", 0, 0, 0, true, -1, null, 0);
            if (link.Joint.AxisName != "None")
            {
                model.Extension.SelectByID2(
                    link.Joint.AxisName, "AXIS", 0, 0, 0, true, -1, null, 0);
            }
            foreach (Link child in link.Children)
            {
                SelectFeatures(child);
            }
        }

        public bool CheckAllLinksHaveCoordinateSystems(LinkNode node)
        {
            List<string> problematicLinks = new List<string>();

            CheckAllLinksHaveCoordinateSystemsRecursive(node.Link, problematicLinks);

            if (problematicLinks.Count > 0)
            {
                string linkList = "";
                foreach (string link in problematicLinks)
                {
                    linkList += link + "\n";
                }
                MessageBox.Show("The following links do not have coordinate axes defined:\n" + linkList);

                return false;
            }

            return true;
        }

        public bool CheckAllLinksHaveRefAxes(LinkNode node)
        {
            List<string> problematicLinks = new List<string>();

            CheckAllLinksHaveJointAxesRecursive(node.Link, problematicLinks);

            if (problematicLinks.Count > 0)
            {
                string linkList = "";
                foreach (string link in problematicLinks)
                {
                    linkList += link + "\n";
                }
                MessageBox.Show("The following links do not have joint axes defined:\n" + linkList);

                return false;
            }

            return true;
        }

        public void CheckAllLinksHaveCoordinateSystemsRecursive(Link link, List<string> problematicLinks)
        {
            if (string.IsNullOrEmpty(link.Joint.CoordinateSystemName))
            {
                problematicLinks.Add(link.Name);
            }

            foreach (Link child in link.Children)
            {
                CheckAllLinksHaveCoordinateSystemsRecursive(child, problematicLinks);
            }
        }

        public void CheckAllLinksHaveJointAxesRecursive(Link link, List<string> problematicLinks)
        {
            if (!link.IsBaseLink && link.Joint.Type != "fixed" && string.IsNullOrEmpty(link.Joint.AxisName))
            {
                problematicLinks.Add(link.Name);
            }

            foreach (Link child in link.Children)
            {
                CheckAllLinksHaveJointAxesRecursive(child, problematicLinks);
            }
        }

        public void CheckIfLinkNamesAreUnique(Link link, string linkName, List<string> conflict)
        {
            if (link.Name == linkName)
            {
                conflict.Add(link.Name);
            }

            foreach (Link child in link.Children)
            {
                CheckIfLinkNamesAreUnique(child, linkName, conflict);
            }
        }

        public void CheckIfJointNamesAreUnique(Link link, string jointName, List<string> conflict)
        {
            if (link.Joint.Name == jointName)
            {
                conflict.Add(link.Joint.Name);
            }
            foreach (Link child in link.Children)
            {
                CheckIfJointNamesAreUnique(child, jointName, conflict);
            }
        }

        public bool CheckIfNamesAreUnique(LinkNode node)
        {
            List<List<string>> linkConflicts = new List<List<string>>();
            List<List<string>> jointConflicts = new List<List<string>>();
            CheckIfLinkNamesAreUnique(node.Link, node.Link, linkConflicts);
            CheckIfJointNamesAreUnique(node.Link, node.Link, jointConflicts);

            string message = "\r\nPlease fix these errors before proceeding.";
            string specificErrors = "";
            bool displayInitialMessage = true;
            bool linkNamesInConflict = false;
            foreach (List<string> conflict in linkConflicts)
            {
                if (conflict.Count > 1)
                {
                    linkNamesInConflict = true;
                    if (displayInitialMessage)
                    {
                        specificErrors +=
                            "The following links have LINK names that conflict:\r\n\r\n";
                        displayInitialMessage = false;
                    }
                    bool isFirst = true;
                    foreach (string linkName in conflict)
                    {
                        specificErrors += (isFirst) ? "     " + linkName : ", " + linkName;
                        isFirst = false;
                    }
                    specificErrors += "\r\n";
                }
            }
            displayInitialMessage = true;
            foreach (List<string> conflict in jointConflicts)
            {
                if (conflict.Count > 1)
                {
                    linkNamesInConflict = true;
                    if (displayInitialMessage)
                    {
                        specificErrors +=
                            "The following links have JOINT names that conflict:\r\n\r\n";
                        displayInitialMessage = false;
                    }
                    bool isFirst = true;
                    foreach (string linkName in conflict)
                    {
                        specificErrors += (isFirst) ? "     " + linkName : ", " + linkName;
                        isFirst = false;
                    }
                    specificErrors += "\r\n";
                }
            }
            if (linkNamesInConflict)
            {
                MessageBox.Show(specificErrors + message);
                return false;
            }
            return true;
        }

        public void CheckIfLinkNamesAreUnique(
            Link baselink, Link currentLink, List<List<string>> conflicts)
        {
            List<string> conflict = new List<string>();

            //Finds the conflicts of the currentLink with all the other links
            CheckIfLinkNamesAreUnique(baselink, currentLink.Name, conflict);
            bool alreadyExists = false;
            foreach (List<string> existingConflict in conflicts)
            {
                if (existingConflict.Contains(conflict[0]))
                {
                    alreadyExists = true;
                }
            }
            if (!alreadyExists)
            {
                conflicts.Add(conflict);
            }
            foreach (Link child in currentLink.Children)
            {
                //Proceeds recursively through the children nodes and adds to the conflicts
                // list of lists.
                CheckIfLinkNamesAreUnique(baselink, child, conflicts);
            }
        }

        public void CheckIfJointNamesAreUnique(
            Link baselink, Link currentLink, List<List<string>> conflicts)
        {
            List<string> conflict = new List<string>();

            //Finds the conflicts of the currentLink with all the other links
            CheckIfJointNamesAreUnique(baselink, currentLink.Joint.Name, conflict);
            bool alreadyExists = false;
            foreach (List<string> existingConflict in conflicts)
            {
                if (conflict.Count > 0 && existingConflict.Contains(conflict[0]))
                {
                    alreadyExists = true;
                }
            }

            if (!alreadyExists)
            {
                conflicts.Add(conflict);
            }
            foreach (Link child in currentLink.Children)
            {
                //Proceeds recursively through the children nodes and adds to the conflicts
                // list of lists.
                CheckIfJointNamesAreUnique(baselink, child, conflicts);
            }
        }
    }
}

#endif
