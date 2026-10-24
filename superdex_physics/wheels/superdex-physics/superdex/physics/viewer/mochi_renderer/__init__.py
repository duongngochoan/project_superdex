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

"""Client-side viewer for the standalone ``mochi_viewer`` TCP server.

The TCP server is a separate application and must be installed and running before
the client connects.
"""

from __future__ import annotations


def _detect_mochi_viewer_available() -> bool:
    """True when the ``mochi_viewer_app`` binary can be located.

    Reports whether the standalone TCP server the client drives is installed, so
    callers can degrade gracefully instead of hard-failing when it is absent.
    """
    try:
        from .viewer_app_process import find_viewer_app_executable

        return find_viewer_app_executable() is not None
    except Exception:
        return False


MOCHI_RENDERER_VIEWER_AVAILABLE = _detect_mochi_viewer_available()

from .mochi_renderer_client import CommandEntry, MochiRendererClient, ResponseEntry
from .mochi_renderer_viewer import MochiRendererViewer
from .mochi_renderer_viewer_cfg import CameraCfg, MochiRendererViewerCfg

__all__ = [
    "MOCHI_RENDERER_VIEWER_AVAILABLE",
    "CameraCfg",
    "CommandEntry",
    "MochiRendererClient",
    "MochiRendererViewer",
    "MochiRendererViewerCfg",
    "ResponseEntry",
]
