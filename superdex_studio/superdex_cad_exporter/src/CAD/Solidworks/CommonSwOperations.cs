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
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using Serilog;

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI;
using CADRobotExporter.Utilities;

namespace CADRobotExporter.CAD
{
    public static class CommonSwOperations
    {
        private static readonly ILogger logger = Logger.GetLogger();

        // https://www.codestack.net/solidworks-api/document/assembly/components/show-with-dependents/
        [DllImport("User32.dll", EntryPoint = "SendMessageA", CharSet = CharSet.Ansi)]
        private static extern int SendMessage(IntPtr hWnd, int wMsg, int wParam, int lParam);
        private const int WM_COMMAND = 0x111;
        private const int SHOW_WITH_DEPENDENTS_CMD = 33227;

        private static void ShowWithDependents(ISldWorks swApp, Component2 comp)
        {
            comp.Select4(false, null, false);
            Frame swFrame = (Frame)swApp.Frame();
            IntPtr hWnd = new IntPtr(swFrame.GetHWnd());
            int _ = SendMessage(hWnd, WM_COMMAND, SHOW_WITH_DEPENDENTS_CMD, 0);
        }

        //Selects the components of a link. Helps highlight when the associated node is
        // selected from the tree
        public static void SelectComponents(ModelDoc2 model, Link Link, bool clearSelection, int mark = -1)
        {
            if (clearSelection)
            {
                model.ClearSelection2(true);
            }
            SelectionMgr manager = model.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = mark;
            SelectComponents(model, Link.SWVisualComponents, false);
            foreach (Link child in Link.Children)
            {
                SelectComponents(model, child, false, mark);
            }
        }

        //Selects components from a list.
        public static void SelectComponents(
            ModelDoc2 model, List<Component2> components, bool clearSelection = true, int mark = -1)
        {
            if (clearSelection)
            {
                model.ClearSelection2(true);
            }
            SelectionMgr manager = model.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = mark;
            foreach (Component2 component in components)
            {
                component.Select4(true, data, false);
            }
        }

        public static void DeselectByMark(ModelDoc2 model, int mark)
        {
            SelectionMgr manager = model.SelectionManager;
            int numObjects = manager.GetSelectedObjectCount2(mark);
            int[] objectIdx = new int[numObjects];
            for (int i = 0; i < numObjects; i++)
            {
                objectIdx[i] = i + 1;
            }
            manager.DeSelect2(objectIdx, mark);
        }

        //Finds the selected components and returns them, used when pulling the items from
        // the selection box because it would be too hard for SolidWorks to allow you to
        // access the selectionbox components directly.
        public static void GetSelectedComponents(
            ModelDoc2 model, List<Component2> Components, int Mark = -1)
        {
            SelectionMgr selectionManager = model.SelectionManager;
            Components.Clear();
            for (int i = 0; i < selectionManager.GetSelectedObjectCount2(Mark); i++)
            {
                object obj = selectionManager.GetSelectedObject6(i + 1, Mark);
                Component2 comp = obj as Component2;
                if (comp != null)
                {
                    Components.Add(comp);
                }
            }
        }

        public static bool SelectionHasFaceOrBody(ModelDoc2 model, int Mark = -1)
        {
            SelectionMgr selectionManager = model.SelectionManager;
            for (int i = 0; i < selectionManager.GetSelectedObjectCount2(Mark); i++)
            {
                int type = selectionManager.GetSelectedObjectType3(i + 1, Mark);
                if (type == (int)swSelectType_e.swSelFACES
                    || type == (int)swSelectType_e.swSelSURFACEBODIES
                    || type == (int)swSelectType_e.swSelSOLIDBODIES)
                {
                    return true;
                }
            }

            return false;
        }

        public static void SelectCoordinateSystem(ModelDoc2 model, string name, int mark = -1)
        {
            model.Extension.SelectByID2(
                name, "COORDSYS", 0, 0, 0, true, mark, null, 0);
        }

        public static void SelectAxis(ModelDoc2 model, string name, int mark = -1)
        {
            model.Extension.SelectByID2(
                name, "AXIS", 0, 0, 0, true, mark, null, 0);
        }

        public static string GetSelectedObjectName(ModelDoc2 model, int mark = -1)
        {
            SelectionMgr selectionManager = model.SelectionManager;
            Feature selectedObject = selectionManager.GetSelectedObject6(1, mark);
            return selectedObject.GetNameForSelection(out _);
        }

        //finds all the hidden components, which will be added to a new display state. Also
        // used when exporting STLs, so that hidden components remain hidden
        public static List<Component2> FindHiddenComponents(object[] varComp)
        {
            List<Component2> hiddenComp = new List<Component2>();
            foreach (object obj in varComp)
            {
                Component2 comp = (Component2)obj;
                if (comp.IsHidden(false))
                {
                    hiddenComp.Add(comp);
                }
            }
            return hiddenComp;
        }

        //Except for an exclusionary list, this shows all the components
        public static void ShowAllComponents(ModelDoc2 model, List<Component2> hiddenComponents)
        {
            AssemblyDoc assyDoc = (AssemblyDoc)model;
            List<Component2> componentsToShow = new List<Component2>();
            object[] varComps = assyDoc.GetComponents(false);
            foreach (Component2 comp in varComps)
            {
                componentsToShow.Add(comp);
            }
            ShowComponents(componentsToShow);
            HideComponents(hiddenComponents);
        }

        public static void HideAllComponents(ModelDoc2 model)
        {
            AssemblyDoc assyDoc = (AssemblyDoc)model;
            List<Component2> componentsToHide = new List<Component2>();
            object[] varComps = assyDoc.GetComponents(false);
            foreach (Component2 comp in varComps)
            {
                componentsToHide.Add(comp);
            }
            HideComponents(componentsToHide);
        }

        public static void ShowComponentsWithDependents(ISldWorks swApp, List<Component2> components)
        {
            foreach (var component in components)
            {
                if (component.GetChildren() != null)
                {
                    ShowWithDependents(swApp, component);
                }
                component.Visible = (int)swComponentVisibilityState_e.swComponentVisible;
            }
        }

        //Shows the components in the list. Useful for exporting STLs
        public static void ShowComponents(List<Component2> components)
        {
            foreach (var component in components)
            {
                component.Visible = (int)swComponentVisibilityState_e.swComponentVisible;
            }
        }

        //Hides the components from a list
        public static void HideComponents(List<Component2> components)
        {
            foreach (var component in components)
            {
                component.Visible = (int)swComponentVisibilityState_e.swComponentHidden;
            }
        }

        #region Solid body operations (part documents)

        // Finds the selected bodies and returns them. Similar to GetSelectedComponents
        public static void GetSelectedBodies(ModelDoc2 model, List<Body2> bodies, int Mark = -1)
        {
            SelectionMgr selectionManager = model.SelectionManager;
            bodies.Clear();
            for (int i = 0; i < selectionManager.GetSelectedObjectCount2(Mark); i++)
            {
                object obj = selectionManager.GetSelectedObject6(i + 1, Mark);
                Body2 body = obj as Body2;
                if (body != null)
                {
                    bodies.Add(body);
                }
            }
        }

        // Selects bodies from a list.
        public static void SelectBodies(
            ModelDoc2 model, List<Body2> bodies, bool clearSelection = true, int mark = -1)
        {
            if (clearSelection)
            {
                model.ClearSelection2(true);
            }
            SelectionMgr manager = model.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = mark;
            foreach (Body2 body in bodies)
            {
                body.Select2(true, data);
            }
        }

        // All solid bodies in a part document. Sheet, wire and other body types are excluded
        // because only solids contribute mass or a watertight mesh.
        public static List<Body2> GetAllSolidBodies(ModelDoc2 model, bool visibleOnly = false)
        {
            List<Body2> bodies = new List<Body2>();
            PartDoc partDoc = model as PartDoc;
            if (partDoc == null)
            {
                return bodies;
            }

            object[] varBodies = (object[])partDoc.GetBodies2((int)swBodyType_e.swSolidBody, visibleOnly);
            if (varBodies == null)
            {
                return bodies;
            }

            foreach (object obj in varBodies)
            {
                Body2 body = obj as Body2;
                if (body != null)
                {
                    bodies.Add(body);
                }
            }
            return bodies;
        }

        public static List<string> GetSolidBodyNames(ModelDoc2 model, bool visibleOnly = false)
        {
            List<string> names = new List<string>();
            foreach (Body2 body in GetAllSolidBodies(model, visibleOnly))
            {
                names.Add(body.Name);
            }
            return names;
        }

        // Hides or shows bodies by name, re-resolving them from the document first.
        // We use names here because somehow we lose references to Body2 after a SaveAs operation.
        public static void SetBodyVisibilityByName(ModelDoc2 model, ICollection<string> names, bool hide)
        {
            if (names == null || names.Count == 0)
            {
                return;
            }

            foreach (Body2 body in GetAllSolidBodies(model))
            {
                if (names.Contains(body.Name))
                {
                    body.HideBody(hide);
                }
            }
        }

        public static byte[] GetBodyPersistId(ModelDoc2 model, Body2 body)
        {
            if (body != null)
            {
                return model.Extension.GetPersistReference3(body);
            }
            return null;
        }

        #endregion Solid body operations (part documents)

        public static void RetrieveSWComponentPIDs(ModelDoc2 model, Link link)
        {
            if (link.SWVisualComponents != null)
            {
                link.SWComponentPIDs = new List<byte[]>();
                foreach (IComponent2 comp in link.SWVisualComponents)
                {
                    byte[] PID = model.Extension.GetPersistReference3(comp);
                    link.SWComponentPIDs.Add(PID);
                }
            }
            if (link.SWCollisionComponents != null)
            {
                link.SWCollisionComponentPIDs = new List<byte[]>();
                foreach (IComponent2 comp in link.SWCollisionComponents)
                {
                    byte[] PID = model.Extension.GetPersistReference3(comp);
                    link.SWCollisionComponentPIDs.Add(PID);
                }
            }
            if (link.SWInertialComponents != null)
            {
                link.SWInertialComponentPIDs = new List<byte[]>();
                foreach (IComponent2 comp in link.SWInertialComponents)
                {
                    byte[] PID = model.Extension.GetPersistReference3(comp);
                    link.SWInertialComponentPIDs.Add(PID);
                }
            }

            if (link.SWVisualBodies != null)
            {
                link.SWVisualBodyPIDs = new List<byte[]>();
                foreach (Body2 body in link.SWVisualBodies)
                {
                    link.SWVisualBodyPIDs.Add(GetBodyPersistId(model, body));
                }
            }
            if (link.SWCollisionBodies != null)
            {
                link.SWCollisionBodyPIDs = new List<byte[]>();
                foreach (Body2 body in link.SWCollisionBodies)
                {
                    link.SWCollisionBodyPIDs.Add(GetBodyPersistId(model, body));
                }
            }
            if (link.SWInertialBodies != null)
            {
                link.SWInertialBodyPIDs = new List<byte[]>();
                foreach (Body2 body in link.SWInertialBodies)
                {
                    link.SWInertialBodyPIDs.Add(GetBodyPersistId(model, body));
                }
            }

            if (link.Joint.SWRefAxisFeature != null)
            {
                byte[] axisPID = model.Extension.GetPersistReference3(link.Joint.SWRefAxisFeature);
                link.Joint.AxisPID = axisPID;
            }

            if (link.Joint.SWCoordinateSystemFeature != null)
            {
                byte[] coordinateSystemPID = model.Extension.GetPersistReference3(link.Joint.SWCoordinateSystemFeature);
                link.Joint.CoordinateSystemPID = coordinateSystemPID;
            }

            foreach (Link child in link.Children)
            {
                RetrieveSWComponentPIDs(model, child);
            }
        }

        //Converts the SW component references to PIDs
        public static void SaveSWComponents(ModelDoc2 model, Link Link)
        {
            model.ClearSelection2(true);
            Link.SWComponentPIDs = SaveSWComponents(model, Link.SWVisualComponents);
            Link.SWCollisionComponentPIDs = SaveSWComponents(model, Link.SWCollisionComponents);
            Link.SWInertialComponentPIDs = SaveSWComponents(model, Link.SWInertialComponents);

            foreach (Link Child in Link.Children)
            {
                SaveSWComponents(model, Child);
            }
        }

        //Converts SW component references to PIDs
        public static List<byte[]> SaveSWComponents(ModelDoc2 model, List<Component2> components)
        {
            List<byte[]> PIDs = new List<byte[]>();
            foreach (Component2 component in components)
            {
                byte[] PID = SaveSWComponent(model, component);
                if (PID != null)
                {
                    PIDs.Add(PID);
                }
            }
            return PIDs;
        }

        public static byte[] SaveSWComponent(ModelDoc2 model, Component2 component)
        {
            if (component != null)
            {
                return model.Extension.GetPersistReference3(component);
            }
            return null;
        }

        // Converts the PIDs to actual references to the components and proceeds recursively
        // through the child nodes
        public static void LoadSWComponents(ModelDoc2 model, Link link, List<string> problemLinks)
        {
            logger.Information("Loading SolidWorks components for " +
                link.Name + " from " + model.GetPathName());

            link.SWVisualComponents = LoadSWComponents(model, link.SWComponentPIDs);
            link.SWCollisionComponents = LoadSWComponents(model, link.SWCollisionComponentPIDs);
            link.SWInertialComponents = LoadSWComponents(model, link.SWInertialComponentPIDs);
            link.SWVisualBodies = LoadSWBodies(model, link.SWVisualBodyPIDs);
            link.SWCollisionBodies = LoadSWBodies(model, link.SWCollisionBodyPIDs);
            link.SWInertialBodies = LoadSWBodies(model, link.SWInertialBodyPIDs);
            LoadRefAxisAndCoordsys(model, link);
            if (link.SWVisualComponents.Count != link.SWComponentPIDs.Count
                || link.SWCollisionComponents.Count != link.SWCollisionComponentPIDs.Count
                || link.SWInertialComponents.Count != link.SWInertialComponentPIDs.Count)
            {
                problemLinks.Add(link.Name);
                logger.Error("Link " + link.Name + " did not fully load all components");
            }
            else if (link.SWVisualBodies.Count != CountOf(link.SWVisualBodyPIDs)
                || link.SWCollisionBodies.Count != CountOf(link.SWCollisionBodyPIDs)
                || link.SWInertialBodies.Count != CountOf(link.SWInertialBodyPIDs))
            {
                problemLinks.Add(link.Name);
                logger.Error("Link " + link.Name + " did not fully load all bodies");
            }
            logger.Information("Loaded " + link.SWVisualComponents.Count + " visual components for link " + link.Name);
            logger.Information("Loaded " + link.SWCollisionComponents.Count + " collision components for link " + link.Name);
            logger.Information("Loaded " + link.SWInertialComponents.Count + " inertial components for link " + link.Name);
            if (link.SWVisualBodies.Count > 0 || link.SWCollisionBodies.Count > 0 || link.SWInertialBodies.Count > 0)
            {
                logger.Information("Loaded " + link.SWVisualBodies.Count + " visual bodies, "
                    + link.SWCollisionBodies.Count + " collision bodies, "
                    + link.SWInertialBodies.Count + " inertial bodies for link " + link.Name);
            }

            foreach (Link child in link.Children)
            {
                LoadSWComponents(model, child, problemLinks);
            }
        }

        public static bool LoadRefAxisAndCoordsys(ModelDoc2 model, Link link)
        {
            bool success = true;

            string axisName = link.Joint.AxisName;
            if (link.Joint.AxisPID != null && !Joint.IsAxisFromCsys(axisName))
            {
                Feature refAxis = LoadSWFeature(model, link.Joint.AxisPID);
                if (refAxis == null)
                {
                    string byteAsString = PIDToString(link.Joint.AxisPID);
                    logger.Warning("RefAxis feature with PID " + byteAsString + " failed to load");
                    success = false;
                }
                else
                {
                    link.Joint.SWRefAxisFeature = refAxis;
                    link.Joint.AxisName = refAxis.GetNameForSelection(out _);
                }
            }
            else
            {
                if (!string.IsNullOrEmpty(axisName) && !Joint.IsAxisFromCsys(axisName))
                {
                    // This was probably saved with a previous vresion so we'll populate the field now
                    model.ClearSelection2(true);
                    bool selected =
                        model.Extension.SelectByID2(axisName, "AXIS", 0, 0, 0, false, 0, null, 0);
                    if (selected)
                    {
                        Feature refAxis = model.SelectionManager.GetSelectedObject6(1, -1);
                        link.Joint.SWRefAxisFeature = refAxis;
                    }
                    else
                    {
                        MessageBox.Show("Couldn't find " + axisName + ", this reference will be missing, you will have to re-select this Axis.");
                        success = false;
                    }
                    model.ClearSelection2(true);
                }
            }

            if (link.Joint.CoordinateSystemPID != null)
            {
                Feature coordSys = LoadSWFeature(model, link.Joint.CoordinateSystemPID);
                if (coordSys == null)
                {
                    string byteAsString = PIDToString(link.Joint.CoordinateSystemPID);
                    logger.Warning("CoordSys feature with PID " + byteAsString + " failed to load");
                    success = false;
                }
                else
                {
                    link.Joint.SWCoordinateSystemFeature = coordSys;
                    link.Joint.CoordinateSystemName = coordSys.GetNameForSelection(out _);
                }
            }
            else
            {
                string coordSysName = link.Joint.CoordinateSystemName;
                if (!string.IsNullOrEmpty(coordSysName))
                {
                    // This was probably saved with a previous vresion so we'll populate the field now
                    model.ClearSelection2(true);
                    bool selected =
                        model.Extension.SelectByID2(coordSysName, "COORDSYS", 0, 0, 0, true, 0, null, 0);
                    if (selected)
                    {
                        Feature coordSys = model.SelectionManager.GetSelectedObject6(1, -1);
                        link.Joint.SWCoordinateSystemFeature = coordSys;
                    }
                    else
                    {
                        MessageBox.Show("Couldn't find " + coordSysName + ", this reference will be missing, you will have to re-select this Coordinate System.");
                        success = false;
                    }
                    model.ClearSelection2(true);
                }
            }

            return success;
        }

        // Converts the PIDs to actual references to the components
        public static List<Component2> LoadSWComponents(ModelDoc2 model, List<byte[]> PIDs)
        {
            List<Component2> components = new List<Component2>();
            foreach (byte[] PID in PIDs)
            {
                string byteAsString = PIDToString(PID);
                logger.Information("Loading component with PID " + byteAsString);
                Component2 comp = LoadSWComponent(model, PID);
                if (comp == null)
                {
                    logger.Warning("Component with PID " + byteAsString + " failed to load");
                }
                else
                {
                    components.Add(comp);
                    logger.Information("Successfully loaded component " + comp.GetPathName());
                }
            }
            return components;
        }

        public static Feature LoadSWFeature(ModelDoc2 model, byte[] PID)
        {
            string byteAsString = PIDToString(PID);
            if (PID == null)
            {
                throw new System.Exception("PID " + byteAsString + " was null. Is the configuration corrupted?");
            }

            object obj = model.Extension.GetObjectByPersistReference3(PID, out int Errors);
            if (Errors == 0)
            {
                return (Feature)obj;
            }
            switch ((swPersistReferencedObjectStates_e)Errors)
            {
                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Deleted:
                    logger.Error("The feature associated with PID " + byteAsString + " was deleted");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Invalid:
                    logger.Error("The feature associated with PID " + byteAsString + " was found to be invalid");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Suppressed:
                    logger.Error("The feature associated with PID " + byteAsString + " is suppressed");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Ok:
                    break;

                default:
                    logger.Error("The feature associated with PID " + byteAsString +
                        " was not loaded due to an unspecified error (" + Errors + ")");
                    break;
            }
            return null;
        }

        // Converts a single PID to a Component2 object
        public static Component2 LoadSWComponent(ModelDoc2 model, byte[] PID)
        {
            string byteAsString = PIDToString(PID);
            if (PID == null)
            {
                throw new System.Exception("PID " + byteAsString + " was null. Is the configuration corrupted?");
            }

            object obj = model.Extension.GetObjectByPersistReference3(PID, out int Errors);
            if (Errors == 0)
            {
                return (Component2)obj;
            }
            switch ((swPersistReferencedObjectStates_e)Errors)
            {
                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Deleted:
                    logger.Error("The component associated with PID " + byteAsString + " was deleted");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Invalid:
                    logger.Error("The component associated with PID " + byteAsString + " was found to be invalid");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Suppressed:
                    logger.Error("The component associated with PID " + byteAsString + " is suppressed");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Ok:
                    break;

                default:
                    logger.Error("The component associated with PID " + byteAsString +
                        " was not loaded due to an unspecified error (" + Errors + ")");
                    break;
            }
            return null;
        }

        public static string PIDToString(byte[] pid)
        {
            return Encoding.ASCII.GetString(pid);
        }

        private static int CountOf(List<byte[]> pids)
        {
            return pids == null ? 0 : pids.Count;
        }

        // Converts the PIDs to actual references to the bodies. Counterpart of LoadSWComponents.
        public static List<Body2> LoadSWBodies(ModelDoc2 model, List<byte[]> PIDs)
        {
            List<Body2> bodies = new List<Body2>();
            if (PIDs == null)
            {
                return bodies;
            }

            foreach (byte[] PID in PIDs)
            {
                Body2 body = LoadSWBody(model, PID);
                if (body == null)
                {
                    logger.Warning("Body failed to load from its persistent reference");
                }
                else
                {
                    bodies.Add(body);
                    logger.Information("Successfully loaded body " + body.Name);
                }
            }
            return bodies;
        }

        // Converts a single PID to a Body2 object
        public static Body2 LoadSWBody(ModelDoc2 model, byte[] PID)
        {
            if (PID == null)
            {
                throw new System.Exception("A body PID was null. Is the configuration corrupted?");
            }

            string byteAsString = PIDToString(PID);
            object obj = model.Extension.GetObjectByPersistReference3(PID, out int Errors);
            if (Errors == 0)
            {
                return obj as Body2;
            }
            switch ((swPersistReferencedObjectStates_e)Errors)
            {
                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Deleted:
                    logger.Error("The body associated with PID " + byteAsString + " was deleted");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Invalid:
                    logger.Error("The body associated with PID " + byteAsString + " was found to be invalid");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Suppressed:
                    logger.Error("The body associated with PID " + byteAsString + " is suppressed");
                    break;

                case swPersistReferencedObjectStates_e.swPersistReferencedObject_Ok:
                    break;

                default:
                    logger.Error("The body associated with PID " + byteAsString +
                        " was not loaded due to an unspecified error (" + Errors + ")");
                    break;
            }
            return null;
        }

        public static Matrix GetRotationMatrix(MathTransform transform)
        {
            Matrix rot = new DenseMatrix(3);
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    rot.At(i, j, transform.ArrayData[i + 3 * j]);
                }
            }

            return rot;
        }

        public static Matrix<double> GetTransformation(MathTransform transform)
        {
            Matrix<double> m = new DenseMatrix(4);

            m[0, 0] = transform.ArrayData[0];
            m[1, 0] = transform.ArrayData[1];
            m[2, 0] = transform.ArrayData[2];
            m[0, 1] = transform.ArrayData[3];
            m[1, 1] = transform.ArrayData[4];
            m[2, 1] = transform.ArrayData[5];
            m[0, 2] = transform.ArrayData[6];
            m[1, 2] = transform.ArrayData[7];
            m[2, 2] = transform.ArrayData[8];

            m[0, 3] = transform.ArrayData[9];
            m[1, 3] = transform.ArrayData[10];
            m[2, 3] = transform.ArrayData[11];
            m[3, 3] = transform.ArrayData[12];
            return m;
        }
    }
}

#endif
