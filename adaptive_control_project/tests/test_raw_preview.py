"""Preview resampling must preserve source timing for video comparison."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/render_raw_preview.py"
SPEC = importlib.util.spec_from_file_location("render_raw_preview", SCRIPT)
PREVIEW = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREVIEW)


def _sequence():
    times = 5 + np.arange(4) / 10
    return {"fps": 10., "timestamps": times, "keypoints": np.broadcast_to(times[:, None, None], (4, 21, 3)).copy()}


def test_preview_interpolation_preserves_source_clock():
    points, times, rate = PREVIEW.resample_preview({"left": _sequence(), "right": _sequence()}, 20., None)
    assert rate == 20.
    np.testing.assert_allclose(times, 5 + np.arange(7) / 20)
    np.testing.assert_allclose(points["left"][:, 0, 0], times)
    np.testing.assert_array_equal(points["left"], points["right"])


def test_preview_frame_limit_and_invalid_bimanual_timing():
    _, times, _ = PREVIEW.resample_preview({"right": _sequence()}, None, 2)
    np.testing.assert_allclose(times, [5., 5.1])
    other = _sequence()
    other["timestamps"] += .01
    with pytest.raises(ValueError, match="same timestamps"):
        PREVIEW.resample_preview({"left": _sequence(), "right": other}, None, None)


@pytest.mark.parametrize("fps,maximum", [(0., None), (np.nan, None), (10., 0)])
def test_preview_rejects_invalid_output_timing(fps, maximum):
    with pytest.raises(ValueError):
        PREVIEW.resample_preview({"right": _sequence()}, fps, maximum)
