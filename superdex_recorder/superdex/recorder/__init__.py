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

"""Transactional, RRD-backed SuperDex episode recording."""

from __future__ import annotations

from typing import TYPE_CHECKING

import superdex.robotics  # noqa: F401
from superdex.physics.loader import forward_module, import_module, NativePayload

_RECORDER_NATIVE_PAYLOAD = NativePayload(
    subpackage="recorder",
    distribution="superdex-recorder",
    fp64_package="superdex_recorder_fp64",
)

_extension = import_module(
    "superdex_recorder",
    payload=_RECORDER_NATIVE_PAYLOAD,
    allow_source_build=True,
)

__all__ = forward_module(_extension, globals())

from .reader import (  # noqa: F401
    ComponentSample as ComponentSample,
    EpisodeReader as EpisodeReader,
)

__all__.extend(["ComponentSample", "EpisodeReader"])

if TYPE_CHECKING:
    from superdex_recorder import *  # noqa: F401,F403
