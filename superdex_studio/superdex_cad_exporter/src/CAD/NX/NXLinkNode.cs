/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using NXOpen;
using NXOpen.BlockStyler;
using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI.ViewModels;
using System;
using System.Collections.Generic;
using System.Linq;
using Export = CADRobotExporter.Export;
using Joint = CADRobotExporter.RobotDescription.Joint;

namespace NXRobotExporter.CAD.NX
{
    /// <summary>
    /// Thin wrapper over an NX Tree Node that provides typed access to link/joint data
    /// stored in the node's DataContainer. The NX Tree is the single source of truth.
    /// </summary>
    public class NXLinkNode
    {
        // Column IDs for the tree
        public const int ColumnLink = 0;
        public const int ColumnJointType = 1;
        public const int ColumnCSYS = 2;
        public const int ColumnAxis = 3;
        public const int ColumnVisual = 4;
        public const int ColumnCollision = 5;
        public const int ColumnInertial = 6;

        // Display symbols
        private const string SymbolCheck = "✓";
        private const string SymbolX = "✗";
        private const string SymbolNA = "-";

        // Data container keys
        private const string DataKeyLinkName = "LinkName";
        private const string DataKeyJointType = "JointType";
        private const string DataKeyJointName = "JointName";
        private const string DataKeyIsBaseLink = "IsBaseLink";
        private const string DataKeyVisualBodies = "VisualBodies";
        private const string DataKeyCollisionBodies = "CollisionBodies";
        private const string DataKeyInertialBodies = "InertialBodies";
        private const string DataKeyCoordinateSystem = "CoordinateSystem";
        private const string DataKeyJointAxis = "JointAxis";
        private const string DataKeyFlipAxis = "FlipAxis";
        private const string DataKeyPureVisual = "PureVisual";
        private const string DataKeyPureInertial = "PureInertial";
        private const string DataKeyIsSite = "IsSite";
        private const string DataKeyMeshingOverride = "MeshingOverride";

        private const string DataKeyLimitUpper = "LimitUpper";
        private const string DataKeyLimitLower = "LimitLower";
        private const string DataKeyLimitEffort = "LimitEffort";
        private const string DataKeyLimitVelocity = "LimitVelocity";
        private const string DataKeyDynamicsDamping = "DynamicsDamping";
        private const string DataKeyDynamicsFriction = "DynamicsFriction";

        private static readonly MeshingOptionsData VisualMeshData =
            new MeshingOptionsData("VisualMesh", linearDeflection: 0.1, angularDeflection: 0.5);
        private static readonly MeshingOptionsData CollisionMeshData =
            new MeshingOptionsData("CollisionMesh", linearDeflection: 0.5, angularDeflection: 0.75);

        private static readonly string[] JointTypes = { "revolute", "continuous", "prismatic", "fixed", "floating", "planar" };

        public Node TreeNode { get; private set; }

        /// <summary>
        /// Creates an NXLinkNode wrapper for an existing tree node.
        /// </summary>
        public NXLinkNode(Node treeNode)
        {
            TreeNode = treeNode ?? throw new ArgumentNullException(nameof(treeNode));
        }

        /// <summary>
        /// Initializes all DataContainer keys with default values.
        /// Call this after creating a new node to avoid exception-based fallbacks on first access.
        /// </summary>
        public void InitializeDataContainer()
        {
            var data = GetData();

            data.AddString(DataKeyLinkName, TreeNode.DisplayText);
            data.AddString(DataKeyJointName, string.Empty);
            data.AddInteger(DataKeyJointType, 3); // "fixed" by default
            data.AddInteger(DataKeyIsBaseLink, 0);
            data.AddStrings(DataKeyVisualBodies, Array.Empty<string>());
            data.AddStrings(DataKeyCollisionBodies, Array.Empty<string>());
            data.AddStrings(DataKeyInertialBodies, Array.Empty<string>());
            data.AddString(DataKeyCoordinateSystem, string.Empty);
            data.AddString(DataKeyJointAxis, string.Empty);
            data.AddInteger(DataKeyFlipAxis, 0);
            data.AddInteger(DataKeyPureVisual, 0);
            data.AddInteger(DataKeyPureInertial, 0);
            data.AddInteger(DataKeyIsSite, 0);
            data.AddInteger(DataKeyMeshingOverride, 0);

            data.AddDouble(DataKeyLimitUpper, double.NaN);
            data.AddDouble(DataKeyLimitLower, double.NaN);
            data.AddDouble(DataKeyLimitEffort, double.NaN);
            data.AddDouble(DataKeyLimitVelocity, double.NaN);
            data.AddDouble(DataKeyDynamicsDamping, double.NaN);
            data.AddDouble(DataKeyDynamicsFriction, double.NaN);

            VisualMeshData.Register(data);
            CollisionMeshData.Register(data);
        }

        /// <summary>
        /// Updates the tree node reference. Used when nodes are moved via drag-and-drop.
        /// </summary>
        public void UpdateTreeNode(Node newTreeNode)
        {
            TreeNode = newTreeNode ?? throw new ArgumentNullException(nameof(newTreeNode));
        }

        #region Properties - Read/Write to DataContainer

        public string LinkName
        {
            get => GetString(DataKeyLinkName, TreeNode.DisplayText);
            set
            {
                SetString(DataKeyLinkName, value);
                TreeNode.DisplayText = value;
            }
        }

        public string JointName
        {
            get => GetString(DataKeyJointName, string.Empty);
            set => SetString(DataKeyJointName, value);
        }

        public string JointType
        {
            get => JointTypes[Math.Max(0, Math.Min(GetInteger(DataKeyJointType, 3), JointTypes.Length - 1))];
            set
            {
                int index = Array.IndexOf(JointTypes, value?.ToLower() ?? "fixed");
                SetInteger(DataKeyJointType, index >= 0 ? index : 3);
                TreeNode.SetColumnDisplayText(ColumnJointType, GetJointTypeDisplayText(value));
            }
        }

        public int JointTypeIndex
        {
            get => GetInteger(DataKeyJointType, 3);
            set
            {
                SetInteger(DataKeyJointType, value);
                TreeNode.SetColumnDisplayText(ColumnJointType, GetJointTypeDisplayText(JointTypes[Math.Max(0, Math.Min(value, JointTypes.Length - 1))]));
            }
        }

        public bool IsBaseLink
        {
            get => GetInteger(DataKeyIsBaseLink, 0) == 1;
            set => SetInteger(DataKeyIsBaseLink, value ? 1 : 0);
        }

        public bool IsRootNode => TreeNode.ParentNode == null;

        public string[] VisualBodiesHandles
        {
            get => GetStrings(DataKeyVisualBodies);
            set => SetStrings(DataKeyVisualBodies, value ?? Array.Empty<string>());
        }

        public string[] CollisionBodiesHandles
        {
            get => GetStrings(DataKeyCollisionBodies);
            set => SetStrings(DataKeyCollisionBodies, value ?? Array.Empty<string>());
        }

        public string[] InertialBodiesHandles
        {
            get => GetStrings(DataKeyInertialBodies);
            set => SetStrings(DataKeyInertialBodies, value ?? Array.Empty<string>());
        }

        public string CoordinateSystemHandle
        {
            get => GetString(DataKeyCoordinateSystem, string.Empty);
            set => SetString(DataKeyCoordinateSystem, value ?? string.Empty);
        }

        public string JointAxisHandle
        {
            get => GetString(DataKeyJointAxis, string.Empty);
            set => SetString(DataKeyJointAxis, value ?? string.Empty);
        }

        public bool FlipAxis
        {
            get => GetInteger(DataKeyFlipAxis, 0) == 1;
            set => SetInteger(DataKeyFlipAxis, value ? 1 : 0);
        }

        public bool PureVisual
        {
            get => GetInteger(DataKeyPureVisual, 0) == 1;
            set => SetInteger(DataKeyPureVisual, value ? 1 : 0);
        }

        public bool PureInertial
        {
            get => GetInteger(DataKeyPureInertial, 0) == 1;
            set => SetInteger(DataKeyPureInertial, value ? 1 : 0);
        }

        public bool IsSite
        {
            get => GetInteger(DataKeyIsSite, 0) == 1;
            set => SetInteger(DataKeyIsSite, value ? 1 : 0);
        }

        public double? LimitUpper
        {
            get => GetNullableDouble(DataKeyLimitUpper);
            set => SetNullableDouble(DataKeyLimitUpper, value);
        }

        public double? LimitLower
        {
            get => GetNullableDouble(DataKeyLimitLower);
            set => SetNullableDouble(DataKeyLimitLower, value);
        }

        public double? LimitEffort
        {
            get => GetNullableDouble(DataKeyLimitEffort);
            set => SetNullableDouble(DataKeyLimitEffort, value);
        }

        public double? LimitVelocity
        {
            get => GetNullableDouble(DataKeyLimitVelocity);
            set => SetNullableDouble(DataKeyLimitVelocity, value);
        }

        public double? DynamicsDamping
        {
            get => GetNullableDouble(DataKeyDynamicsDamping);
            set => SetNullableDouble(DataKeyDynamicsDamping, value);
        }

        public double? DynamicsFriction
        {
            get => GetNullableDouble(DataKeyDynamicsFriction);
            set => SetNullableDouble(DataKeyDynamicsFriction, value);
        }

        public Export.MeshingOptions VisualMeshingOptions
        {
            get => VisualMeshData.Read(this);
            set => VisualMeshData.Write(this, value);
        }

        public Export.MeshingOptions CollisionMeshingOptions
        {
            get => CollisionMeshData.Read(this);
            set => CollisionMeshData.Write(this, value);
        }

        /// <summary>
        /// Whether the link's own meshing options are used instead of the robot-wide ones.
        /// </summary>
        public bool MeshingOverride
        {
            get => GetInteger(DataKeyMeshingOverride, 0) == 1;
            set => SetInteger(DataKeyMeshingOverride, value ? 1 : 0);
        }

        #endregion

        #region Validation

        public bool IsIncomplete => CheckIncomplete(out _);

        /// <summary>
        /// Checks if this node is missing required data.
        /// </summary>
        /// <param name="reason">Description of what's missing</param>
        /// <returns>True if incomplete, false if valid</returns>
        public bool CheckIncomplete(out string reason)
        {
            reason = string.Empty;

            // All links need a name
            if (string.IsNullOrWhiteSpace(LinkName))
            {
                reason = "Link name is empty.";
                return true;
            }

            // All links need a coordinate system
            if (string.IsNullOrEmpty(CoordinateSystemHandle))
            {
                reason = "Coordinate system (CSYS) is not defined.";
                return true;
            }

            // Sites only need a name and CSYS
            if (IsSite)
            {
                return false;
            }

            // Use IsRootNode (structural check) instead of IsBaseLink (data check)
            // to correctly identify the base link even if IsBaseLink flag isn't set
            bool isRoot = IsRootNode;

            if (!isRoot)
            {
                // Joint name is required
                if (string.IsNullOrWhiteSpace(JointName))
                {
                    reason = "Joint name is empty.";
                    return true;
                }

                // Non-fixed joints need an axis
                string jointType = JointType?.ToLower();
                bool needsAxis = jointType == "revolute" ||
                                 jointType == "continuous" ||
                                 jointType == "prismatic";

                if (needsAxis && string.IsNullOrEmpty(JointAxisHandle))
                {
                    reason = $"Joint axis is required for {JointType} joints.";
                    return true;
                }
            }

            return false;
        }

        #endregion

        #region Column Display Updates

        /// <summary>
        /// Updates all display columns for this node based on current data.
        /// Call this after modifying properties that affect column display.
        /// </summary>
        public void UpdateAllColumns()
        {
            // Use IsRootNode (structural check) instead of IsBaseLink (data check)
            // to ensure correct display even if IsBaseLink flag isn't set yet
            bool isBase = IsRootNode;

            // Joint Type column
            if (IsSite)
            {
                TreeNode.SetColumnDisplayText(ColumnJointType, "Site");
            }
            else
            {
                TreeNode.SetColumnDisplayText(ColumnJointType, isBase ? SymbolNA : GetJointTypeDisplayText(JointType));
            }

            // CSYS column (checkmark if defined, X if not)
            bool hasCSYS = !string.IsNullOrEmpty(CoordinateSystemHandle);
            TreeNode.SetColumnDisplayText(ColumnCSYS, hasCSYS ? SymbolCheck : SymbolX);

            // Sites don't have axis or bodies
            if (IsSite)
            {
                TreeNode.SetColumnDisplayText(ColumnAxis, SymbolNA);
                TreeNode.SetColumnDisplayText(ColumnVisual, SymbolNA);
                TreeNode.SetColumnDisplayText(ColumnCollision, SymbolNA);
                TreeNode.SetColumnDisplayText(ColumnInertial, SymbolNA);
                return;
            }

            // Axis column (checkmark if defined, X if not, - for base link)
            if (isBase)
            {
                TreeNode.SetColumnDisplayText(ColumnAxis, SymbolNA);
            }
            else
            {
                bool hasAxis = !string.IsNullOrEmpty(JointAxisHandle);
                string axisSymbol = SymbolCheck;
                if (Joint.IsAxisFromCsys(JointAxisHandle))
                {
                    axisSymbol = FlipAxis ? "−" : "+";
                    if (JointAxisHandle == Joint.AxisFromCsysX) axisSymbol += "X";
                    else if (JointAxisHandle == Joint.AxisFromCsysY) axisSymbol += "Y";
                    else if (JointAxisHandle == Joint.AxisFromCsysZ) axisSymbol += "Z";
                }

                string noAxisSymbol = SymbolX;
                if (JointType == "fixed")
                {
                    noAxisSymbol = SymbolNA;
                }

                TreeNode.SetColumnDisplayText(ColumnAxis, hasAxis ? axisSymbol : noAxisSymbol);
            }

            // Body count columns
            var visualBodies = VisualBodiesHandles;
            var collisionBodies = CollisionBodiesHandles;
            var inertialBodies = InertialBodiesHandles;

            TreeNode.SetColumnDisplayText(ColumnVisual, GetBodyCountDisplay(visualBodies));
            TreeNode.SetColumnDisplayText(ColumnCollision, GetBodyCountDisplay(collisionBodies));
            TreeNode.SetColumnDisplayText(ColumnInertial, GetBodyCountDisplay(inertialBodies));
        }

        private static string GetBodyCountDisplay(string[] handles)
        {
            if (handles == null || handles.Length == 0)
                return "0";
            return handles.Length.ToString();
        }

        #endregion

        #region Link Building (for export)

        /// <summary>
        /// Creates a Link object from this node's data. Used for export.
        /// Does not include children - use NXTreeManager.BuildLinkTree() for full hierarchy.
        /// </summary>
        public Link ToLink()
        {
            var link = new Link
            {
                Name = LinkName,
                IsBaseLink = IsBaseLink,
                IsIncomplete = IsIncomplete,
                isSite = IsSite,
                NXVisualBodiesHandles = VisualBodiesHandles?.ToList() ?? new List<string>(),
                NXCollisionBodiesHandles = CollisionBodiesHandles?.ToList() ?? new List<string>(),
                NXInertialBodiesHandles = InertialBodiesHandles?.ToList() ?? new List<string>(),
                shouldFlipAxis = FlipAxis,
                visualsOnly = PureVisual,
                inertialsOnly = PureInertial
            };

            link.Joint.Name = JointName;
            if (link.isSite)
            {
                link.Joint.Name = link.Name + "_joint";
            }
            link.Joint.Type = JointType;
            link.Joint.CoordinateSystemName = CoordinateSystemHandle;
            link.Joint.AxisName = JointAxisHandle;

            if (LimitUpper.HasValue) link.Joint.Limit.Upper = LimitUpper.Value;
            if (LimitLower.HasValue) link.Joint.Limit.Lower = LimitLower.Value;
            if (LimitEffort.HasValue) link.Joint.Limit.Effort = LimitEffort.Value;
            if (LimitVelocity.HasValue) link.Joint.Limit.Velocity = LimitVelocity.Value;

            if (DynamicsDamping.HasValue) link.Joint.Dynamics.Damping = DynamicsDamping.Value;
            if (DynamicsFriction.HasValue) link.Joint.Dynamics.Friction = DynamicsFriction.Value;

            link.visualMeshingOptions = VisualMeshingOptions;
            link.collisionMeshingOptions = CollisionMeshingOptions;
            link.meshingOverride = MeshingOverride;

            return link;
        }

        /// <summary>
        /// Populates this node's data from a Link object. Used for import.
        /// </summary>
        public void FromLink(Link link)
        {
            if (link == null) return;

            LinkName = link.Name;
            IsBaseLink = link.IsBaseLink;
            IsSite = link.isSite;
            PureVisual = link.visualsOnly;
            PureInertial = link.inertialsOnly;

            if (link.Joint != null)
            {
                JointName = link.Joint.Name;
                JointType = link.Joint.Type;
                CoordinateSystemHandle = link.Joint.CoordinateSystemName;
                JointAxisHandle = link.Joint.AxisName;

                if (link.Joint.Limit != null)
                {
                    if (link.Joint.Limit.IsUpperSet()) LimitUpper = link.Joint.Limit.Upper;
                    if (link.Joint.Limit.IsLowerSet()) LimitLower = link.Joint.Limit.Lower;
                    if (link.Joint.Limit.IsEffortSet()) LimitEffort = link.Joint.Limit.Effort;
                    if (link.Joint.Limit.IsVelocitySet()) LimitVelocity = link.Joint.Limit.Velocity;
                }

                if (link.Joint.Dynamics != null)
                {
                    if (link.Joint.Dynamics.IsDampingSet()) DynamicsDamping = link.Joint.Dynamics.Damping;
                    if (link.Joint.Dynamics.IsFrictionSet()) DynamicsFriction = link.Joint.Dynamics.Friction;
                }
            }

            if (link.visualMeshingOptions != null)
            {
                VisualMeshingOptions = link.visualMeshingOptions;
            }

            if (link.collisionMeshingOptions != null)
            {
                CollisionMeshingOptions = link.collisionMeshingOptions;
            }

            MeshingOverride = link.meshingOverride;

            VisualBodiesHandles = link.NXVisualBodiesHandles?.ToArray();
            CollisionBodiesHandles = link.NXCollisionBodiesHandles?.ToArray();
            InertialBodiesHandles = link.NXInertialBodiesHandles?.ToArray();
            FlipAxis = link.shouldFlipAxis;
        }

        #endregion

        #region DataContainer Helpers

        private DataContainer GetData() => TreeNode.GetNodeData();

        private string GetString(string key, string defaultValue)
        {
            try { return GetData().GetString(key); }
            catch { return defaultValue; }
        }

        private void SetString(string key, string value)
        {
            var data = GetData();
            try { data.SetString(key, value); }
            catch { data.AddString(key, value); }
        }

        private int GetInteger(string key, int defaultValue)
        {
            try { return GetData().GetInteger(key); }
            catch { return defaultValue; }
        }

        private void SetInteger(string key, int value)
        {
            var data = GetData();
            try { data.SetInteger(key, value); }
            catch { data.AddInteger(key, value); }
        }

        private string[] GetStrings(string key)
        {
            try { return GetData().GetStrings(key); }
            catch { return Array.Empty<string>(); }
        }

        private void SetStrings(string key, string[] value)
        {
            var data = GetData();
            try { data.SetStrings(key, value); }
            catch { data.AddStrings(key, value); }
        }

        private double GetDouble(string key, double defaultValue)
        {
            try { return GetData().GetDouble(key); }
            catch { return defaultValue; }
        }

        private void SetDouble(string key, double value)
        {
            var data = GetData();
            try { data.SetDouble(key, value); }
            catch { data.AddDouble(key, value); }
        }

        private double? GetNullableDouble(string key)
        {
            try
            {
                double val = GetData().GetDouble(key);
                if (double.IsNaN(val))
                {
                    return null;
                }
                else
                {
                    return val;
                }
            }
            catch { return null; }
        }

        private void SetNullableDouble(string key, double? value)
        {
            double storeValue = value ?? double.NaN;
            var data = GetData();
            try { data.SetDouble(key, storeValue); }
            catch { data.AddDouble(key, storeValue); }
        }

        private static string GetJointTypeDisplayText(string jointType)
        {
            switch (jointType?.ToLower())
            {
                case "revolute": return "Revolute";
                case "continuous": return "Continuous";
                case "prismatic": return "Prismatic";
                case "floating": return "Floating";
                case "planar": return "Planar";
                case "fixed":
                default: return "Fixed";
            }
        }

        #endregion

        /// <summary>
        /// One meshing column's DataContainer keys, their defaults, and the conversion to and from
        /// <see cref="Export.MeshingOptions"/>.
        /// </summary>
        private sealed class MeshingOptionsData
        {
            private readonly string linearKey;
            private readonly string angularKey;
            private readonly string scaleKey;
            private readonly string backendKey;
            private readonly string edgeSamplingKey;
            private readonly string targetEdgeLengthKey;
            private readonly string edgeLengthFractionKey;
            private readonly Export.MeshingOptions defaults;

            public MeshingOptionsData(string prefix, double linearDeflection, double angularDeflection)
            {
                linearKey = prefix + "Linear";
                angularKey = prefix + "Angular";
                scaleKey = prefix + "Scale";
                backendKey = prefix + "Backend";
                edgeSamplingKey = prefix + "EdgeSampling";
                targetEdgeLengthKey = prefix + "TargetEdgeLength";
                edgeLengthFractionKey = prefix + "EdgeLengthFraction";

                defaults = new Export.MeshingOptions
                {
                    linearDeflection = linearDeflection,
                    angularDeflection = angularDeflection,
                    scale = 1.0,
                };
            }

            public void Register(DataContainer data)
            {
                data.AddDouble(linearKey, defaults.linearDeflection);
                data.AddDouble(angularKey, defaults.angularDeflection);
                data.AddDouble(scaleKey, defaults.scale);
                data.AddInteger(backendKey, MeshingSettings.IndexFromBackend(defaults.backend));
                data.AddInteger(
                    edgeSamplingKey, MeshingSettings.IndexFromEdgeSampling(defaults.edgeSampling));
                data.AddDouble(targetEdgeLengthKey, defaults.targetEdgeLength);
                data.AddDouble(edgeLengthFractionKey, defaults.targetEdgeLengthFraction);
            }

            public Export.MeshingOptions Read(NXLinkNode node)
            {
                return new Export.MeshingOptions
                {
                    linearDeflection = node.GetDouble(linearKey, defaults.linearDeflection),
                    angularDeflection = node.GetDouble(angularKey, defaults.angularDeflection),
                    scale = node.GetDouble(scaleKey, defaults.scale),
                    backend = MeshingSettings.BackendFromIndex(node.GetInteger(
                        backendKey, MeshingSettings.IndexFromBackend(defaults.backend))),
                    edgeSampling = MeshingSettings.EdgeSamplingFromIndex(node.GetInteger(
                        edgeSamplingKey,
                        MeshingSettings.IndexFromEdgeSampling(defaults.edgeSampling))),
                    targetEdgeLength =
                        node.GetDouble(targetEdgeLengthKey, defaults.targetEdgeLength),
                    targetEdgeLengthFraction = node.GetDouble(
                        edgeLengthFractionKey, defaults.targetEdgeLengthFraction),
                };
            }

            public void Write(NXLinkNode node, Export.MeshingOptions options)
            {
                if (options == null)
                {
                    return;
                }

                node.SetDouble(linearKey, options.linearDeflection);
                node.SetDouble(angularKey, options.angularDeflection);
                node.SetDouble(scaleKey, options.scale);
                node.SetInteger(backendKey, MeshingSettings.IndexFromBackend(options.backend));
                node.SetInteger(
                    edgeSamplingKey, MeshingSettings.IndexFromEdgeSampling(options.edgeSampling));
                node.SetDouble(targetEdgeLengthKey, options.targetEdgeLength);
                node.SetDouble(edgeLengthFractionKey, options.targetEdgeLengthFraction);
            }
        }
    }
}
