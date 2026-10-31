"""Load the local C++ controller library; Python only owns buffer conversion here."""
from __future__ import annotations

import ctypes
from functools import lru_cache
from pathlib import Path

import numpy as np

DOUBLE_POINTER = ctypes.POINTER(ctypes.c_double)
INT_POINTER = ctypes.POINTER(ctypes.c_int)


def double_array(values, shape=None):
    """A finite, contiguous float64 buffer, with optional exact shape validation."""
    out = np.ascontiguousarray(values, dtype=np.float64)
    if shape is not None and out.shape != tuple(shape):
        raise ValueError(f"expected shape {tuple(shape)}, got {out.shape}")
    if not np.isfinite(out).all():
        raise ValueError("controller buffers must contain only finite values")
    return out


def double_pointer(values):
    return None if values is None else values.ctypes.data_as(DOUBLE_POINTER)


@lru_cache(maxsize=1)
def load_library():
    path = Path(__file__).resolve().parent / "_native" / "libretarget_controllers.so"
    if not path.is_file():
        raise RuntimeError(
            "Build the native controllers from the repository root: "
            ".venv/bin/python adaptive_control_project/scripts/build_controllers.py"
        )
    try:
        return ctypes.CDLL(str(path))
    except OSError as exc:
        raise RuntimeError(f"Cannot load native controllers at {path}; rebuild them with scripts/build_controllers.py") from exc
