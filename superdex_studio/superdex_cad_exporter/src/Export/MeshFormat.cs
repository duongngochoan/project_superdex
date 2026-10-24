/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

namespace CADRobotExporter.Export
{
    public enum MeshFormat
    {
        glbOpenCascade,
        objOpenCascade,
        stlOpenCascade,
        stlSolidworks,
        stepSolidworks,
        objCAD,
        glbCAD
    }

    public enum FolderStructure
    {
        Legacy,
        ROS,
        MuJoCo,
        SuperDex,
    }
}
