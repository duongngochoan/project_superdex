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

from __future__ import annotations

import os
import subprocess
from pathlib import Path


def assert_rerun_accepts(path: Path) -> None:
    """Fail with the verifier's report unless ``rerun rrd verify`` accepts ``path``."""
    result = subprocess.run(
        [os.environ["RERUN_CLI"], "rrd", "verify", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
    )
    if result.returncode != 0:
        raise AssertionError(f"rerun rrd verify rejected {path}:\n{result.stdout}")
