# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Configuration dataclass for the MochiRenderer viewer.
"""

from __future__ import annotations

from superdex.physics.utils.configclasses import configclass
from superdex.physics.utils.coordinate_systems import CoordinateSystem

########################################################################################


@configclass
class CameraCfg:
    """Configuration for a single named camera.

    Camera pose can be specified in two ways (checked in this order):
      1. ``position`` + ``rotation`` ΓÇö full 6-DOF transform (preferred).
      2. ``look_from`` + ``look_at`` ΓÇö classic look-at (fallback).
    If neither pair is set the server default is used.
    """

    width: int = 640
    """Width of the camera image in pixels."""

    height: int = 480
    """Height of the camera image in pixels."""

    position: tuple[float, float, float] | None = None
    """Camera position (x, y, z). Used with ``rotation``."""

    rotation: tuple[float, float, float, float] | None = None
    """Camera orientation as an XYZW quaternion. Used with ``position``."""

    look_from: tuple[float, float, float] | None = None
    """Camera eye position (x, y, z). Fallback when ``position``/``rotation``
    are not set. None means use server default."""

    look_at: tuple[float, float, float] | None = None
    """Camera target position (x, y, z). Fallback when ``position``/``rotation``
    are not set. None means use server default."""

    horizontal_fov_deg: float | None = None
    """Horizontal field of view in degrees. If set, converted to vertical
    FOV and sent to the renderer server. None means use server default."""


########################################################################################


@configclass
class MochiRendererViewerCfg:
    """Configuration options for the MochiRenderer viewer."""

    host: str = "localhost"
    """Hostname of the mochi_renderer server. Must resolve to loopback: the
    server only binds 127.0.0.1."""

    port: int = 9000
    """Port for the mochi_renderer TCP connection. Ignored when ``auto_launch``
    is enabled, since the launched server picks its own free port."""

    auto_launch: bool = False
    """Whether to spawn a mochi_viewer_app server instead of connecting to one
    that is already running. The spawned server is shut down by ``close()``."""

    viewer_app_path: str | None = None
    """Explicit path to the mochi_viewer_app executable. When None, the
    executable is discovered automatically (``MOCHI_VIEWER_APP_PATH``, next to
    the native extension, the packaged ``superdex/physics/bin`` directory, then
    the working directory). Only used when ``auto_launch`` is enabled."""

    windowed: bool | None = None
    """Whether the launched server opens an interactive window. When None, a
    window is opened unless ``offscreen`` is set. Only used when ``auto_launch``
    is enabled."""

    launch_timeout: float = 30.0
    """Seconds to wait for a launched server to report its listening port."""

    size: tuple[int, int] = (640, 480)
    """Width and height of captured images."""

    offscreen: bool = False
    """Whether rendering should occur offscreen. If True, render() returns
    captured images as numpy arrays."""

    camera_name: str = "default"
    """Name of the camera to use for image capture."""

    cameras: dict[str, CameraCfg] | None = None
    """Optional multi-camera configuration. Keys are camera names, values are
    per-camera settings. When set, named cameras are created at startup."""

    connection_timeout: float = 5.0
    """Timeout in seconds for the initial connection to the server."""

    retry_attempts: int = 3
    """Number of connection retry attempts before giving up."""

    environment_gltf: str | None = None
    """Path to a static environment glTF file (table, room, etc.)."""

    actor_gltfs: dict[str, str] | None = None
    """Mapping from mochi actor name to glTF file path.
    Actors with a glTF mapping will be rendered using the glTF mesh
    instead of the physics mesh. For articulated actors, map each
    link name to its glTF file."""

    environment_ibl: str | None = None
    """Path to an HDR/EXR environment map for image-based lighting.
    When set, the IBL is loaded on connection and provides realistic
    indirect lighting and reflections."""

    skybox_visible: bool = False
    """Whether the IBL skybox is visible in the background. When False
    (default), the IBL still provides indirect lighting but the
    background remains the solid-color skybox."""

    background_color: tuple[float, float, float] | None = (0.92, 0.92, 0.94)
    """sRGB color rendered behind the scene, applied on connection.

    The server's own default is pure green, which is meant for chroma keying, so the
    viewer overrides it with a neutral tone. Set to None to leave the server's default
    alone. Ignored when ``environment_ibl`` is set, since the IBL supplies the
    background."""

    coordinate_system: CoordinateSystem | str | None = None
    """Convention of the scene's coordinates, including camera poses, grids and camera
    configs. If None, the default coordinate system (right-handed, Y-up, -Z-forward) is
    used, which the server renders as is. You can specify a custom coordinate system
    using the CoordinateSystem class or use named presets like "unity", "unreal", etc.
    glTF assets cannot be mirrored, so they require a right-handed one. The IBL and sun
    are not rotated."""
