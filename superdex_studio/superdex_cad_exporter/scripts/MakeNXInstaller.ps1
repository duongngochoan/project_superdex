# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Builds the MSI installer for the NX add-in. Build NXRobotExporter (Release) first.

param (
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"

& (Join-Path $PSScriptRoot "MakeSolidWorksInstaller.ps1") `
    -Configuration $Configuration `
    -WixProject (Join-Path (Split-Path -Parent $PSScriptRoot) "installer\nx\SuperDexCadExporterNX.wixproj")
