/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.Collections.Generic;

namespace CADRobotExporter.Export
{
    [Serializable]
    public class ExporterConfiguration
    {
        public MeshingOptions visualMeshingOptions;
        public MeshingOptions collisionMeshingOptions;
        public List<MeshFormat> meshFormats;
        public List<MeshFormat> collisionMeshFormats;
        public bool hasCollisionMeshFormats;
        public string meshTagExtension;
        public bool enableCollisionMeshing;
        public string collisionMeshTagExtension;
        public string robotName;
        public FolderStructure folderStructure;

        public ExporterConfiguration()
        {
            visualMeshingOptions = new MeshingOptions
            {
                linearDeflection = 0.1,
                angularDeflection = 0.5,
                scale = 1.0
            };

            collisionMeshingOptions = new MeshingOptions
            {
                linearDeflection = 0.5,
                angularDeflection = 0.75,
                scale = 1.0
            };

            meshFormats = new List<MeshFormat>
            {
#if NX
                MeshFormat.glbCAD,
#elif SOLIDWORKS
                MeshFormat.glbOpenCascade,
#endif
            };

            collisionMeshFormats = new List<MeshFormat>
            {
#if NX
                MeshFormat.objCAD,
#elif SOLIDWORKS
                MeshFormat.stlSolidworks,
#endif
            };

            hasCollisionMeshFormats = true;

#if NX
            meshTagExtension = "glb";
            collisionMeshTagExtension = "obj";
#elif SOLIDWORKS
            meshTagExtension = "glb";
            collisionMeshTagExtension = "stl";
#endif

            enableCollisionMeshing = true;

            robotName = "";

            folderStructure = FolderStructure.SuperDex;
        }
    }
}
