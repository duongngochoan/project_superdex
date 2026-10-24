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

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Collections.ObjectModel;

using CADRobotExporter.Export;
using CADRobotExporter.UI.ViewModels;
using CADRobotExporter.RobotDescription;

namespace CADRobotExporter.UI
{
    //This source file contains all the non-handler methods for the assembly export form,
    // the ones that are helpers.
    public partial class AssemblyExportForm
    {
        //From the link, this method fills the property boxes on the Link Properties page
        private static void ShowOrigin(
            OriginFields fields, Control x, Control y, Control z, Control roll, Control pitch, Control yaw)
        {
            x.Text = fields.X;
            y.Text = fields.Y;
            z.Text = fields.Z;
            roll.Text = fields.Roll;
            pitch.Text = fields.Pitch;
            yaw.Text = fields.Yaw;
        }

        private static void ReadOrigin(
            OriginFields fields, Control x, Control y, Control z, Control roll, Control pitch, Control yaw)
        {
            fields.X = x.Text;
            fields.Y = y.Text;
            fields.Z = z.Text;
            fields.Roll = roll.Text;
            fields.Pitch = pitch.Text;
            fields.Yaw = yaw.Text;
        }

        private LinkPropertiesViewModel ReadLinkPropertyBoxes()
        {
            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();
            visualPropertiesView.ReadInto(properties);
            inertialPropertiesView.ReadInto(properties);

            return properties;
        }

        private void ShowLinkPropertyBoxes(LinkPropertiesViewModel properties)
        {
            visualPropertiesView.Show(properties);
            inertialPropertiesView.Show(properties);
        }

        public void FillLinkPropertyBoxes(Link link)
        {
            AutoUpdatingForm = true;
            visualPropertiesView.Clear();
            inertialPropertiesView.Clear();

            LinkPropertiesViewModel properties = new LinkPropertiesViewModel();
            properties.LoadFrom(link);
            if (properties.IsEditable)
            {
                ShowLinkPropertyBoxes(properties);
            }

            ShowMeshingColumnsFor(link);

            AutoUpdatingForm = false;
        }

        //Fills the property boxes on the joint properties page
        public void FillJointPropertyBoxes(Joint joint)
        {
            AutoUpdatingForm = true;

            jointPropertiesView.Clear();

            JointPropertiesViewModel properties = new JointPropertiesViewModel();
            properties.LoadFrom(joint, jointPropertiesView.UseDegrees);
            jointPropertiesView.Show(properties);

            AutoUpdatingForm = false;
        }

        //Converts the text boxes back into values for the link
        public void SaveLinkDataFromPropertyBoxes(Link link)
        {
            if (link.isFixedFrame)
            {
                return;
            }

            ReadLinkPropertyBoxes().SaveTo(link);
        }

        //Saves data from text boxes back into a joint
        public void SaveJointDataFromPropertyBoxes(Joint joint)
        {
            jointPropertiesView.Read().SaveTo(joint, jointPropertiesView.UseDegrees);
        }

        //Fills specifically the joint TreeView
        public void FillJointTree()
        {
            // The joints tab shows the base link's children as roots, labelled by joint name.
            // BaseNode itself is left alone so the model is never reshaped by switching tabs.
            ObservableCollection<LinkTreeItem> roots = new ObservableCollection<LinkTreeItem>();
            foreach (Link child in BaseNode.Link.Children)
            {
                roots.Add(new LinkTreeItem(child, true));
            }

            treeViewJointTree.ItemsSource = roots;

            if (roots.Count > 0)
            {
                roots[0].IsSelected = true;
            }
        }

        public void FillLinkTree()
        {
            SanitizeMeshingOverrides(BaseNode.Link);

            ObservableCollection<LinkTreeItem> roots = new ObservableCollection<LinkTreeItem>
            {
                new LinkTreeItem(BaseNode.Link),
            };

            treeViewLinkProperties.ItemsSource = roots;
            roots[0].IsSelected = true;
        }

        // Clears the override flag on links that cannot carry one
        private static void SanitizeMeshingOverrides(Link link)
        {
            if (link == null)
            {
                return;
            }

            if (link.meshingOverride && !CanOverrideMeshing(link))
            {
                link.meshingOverride = false;
            }

            foreach (Link child in link.Children)
            {
                SanitizeMeshingOverrides(child);
            }
        }

        //Converts a Link to a LinkNode
        public static LinkNode CreateLinkNodeFromLink(Link Link)
        {
            return new LinkNode(Link);
        }

        //Converts the link tree back into a robot
        public RobotDescription.Robot CreateRobotFromTreeView()
        {
            RobotDescription.Robot Robot = Exporter.Robot;
            Robot.SetBaseLink(BaseNode.Link);
            Robot.Name = textBoxRobotName.Text;
            return Robot;
        }

        // this SaveConfigTree is called when the robot may have a name already
        public CAD.ConfigSaveResult SaveConfigurationFromTree(
            LinkNode BaseNode, bool warnUser, bool allowCancel = false)
        {
            ExporterConfiguration config = SaveFormToExporterConfiguration();

            return CadBridge.SaveConfigurationFromTree(
                config, BaseNode, warnUser, featureNameOverride: null, allowCancel: allowCancel);
        }
    }
}
