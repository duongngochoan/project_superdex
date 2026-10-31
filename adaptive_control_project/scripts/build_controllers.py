#!/usr/bin/env python3
"""Compile C++ controllers locally using a C++17 compiler and Eigen 3 headers."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX", "c++"))
    parser.add_argument("--eigen-include", type=Path,
                        default=Path(os.environ.get("EIGEN3_INCLUDE_DIR", "/usr/include/eigen3")))
    args = parser.parse_args()
    compiler = shutil.which(args.cxx)
    if compiler is None:
        parser.error(f"C++ compiler {args.cxx!r} is unavailable; install a C++17 compiler or set CXX")
    if not (args.eigen_include / "Eigen" / "Core").is_file():
        parser.error("Eigen 3 headers are unavailable; install libeigen3-dev or set EIGEN3_INCLUDE_DIR")
    pipeline = Path(__file__).resolve().parents[1] / "retarget_pipeline"
    sources = sorted((pipeline / "cpp").glob("*.cpp"))
    if not sources:
        parser.error("no C++ controller sources found")
    output = pipeline / "_native"
    output.mkdir(exist_ok=True)
    library = output / "libretarget_controllers.so"
    with tempfile.TemporaryDirectory(prefix="build-", dir=output) as temporary:
        built = Path(temporary) / library.name
        subprocess.run([compiler, "-std=c++17", "-O3", "-DNDEBUG", "-fPIC", "-shared",
                        "-I", str(args.eigen_include), *map(str, sources), "-ldl", "-o", str(built)], check=True)
        built.replace(library)
    print(f"Built {library}")


if __name__ == "__main__":
    main()
