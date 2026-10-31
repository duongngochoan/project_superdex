"""The tactile dashboard displays saved force signals without inventing contact."""
import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.tactile_dashboard import TactileDashboard


NAMES = ["right_link_01_wrist", "right_link_25_ring_finger_middle", "right_link_32_pinky_middle",
         "left_link_25_ring_finger_middle"]


def test_contact_intervals_and_groups_use_only_selected_hand_without_thresholding():
    forces = [[0, 0, 0, 10], [0, .000001, 0, 0], [0, 2, 3, 0], [0, 0, 0, 5]]
    board = TactileDashboard([0., .1, .2, .3], NAMES, forces)
    np.testing.assert_allclose(board.totals, [0, .000001, 5, 0])
    np.testing.assert_allclose(board.groups[2], [0, 0, 0, 0, 2, 3])
    np.testing.assert_array_equal(board.contact, [False, True, True, False])
    assert board.contact_intervals == [(.1, .2)]


def test_multiple_contact_intervals_include_first_and_last_sample():
    board = TactileDashboard([0., 1., 2., 3., 4.], NAMES, [[1, 0, 0, 0], [1, 0, 0, 0],
                            [0, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]])
    assert board.contact_intervals == [(0., 1.), (4., 4.)]


def test_markers_use_only_visible_selected_hand_and_keep_values_unclipped():
    board = TactileDashboard([0.], NAMES, [[12., 0., 4., 100.]])
    wrist = np.full((480, 640, 3), 100, dtype=np.uint8)
    external = np.full_like(wrist, 120)
    points = np.array([[200, 160], [250, 180], [350, 160], [400, 200]])
    visible = [True, True, False, True]
    plain = board.frame(0, wrist, external)
    marked = board.frame(0, wrist, external, points, visible)
    # Native image is fitted to a 718x470 area; y is not cropped.
    ox, oy = 66, 93
    sx, sy = 627 / 640, 470 / 480
    for j in (2, 3):
        x, y = round(ox + sx * points[j, 0]), round(oy + sy * points[j, 1])
        np.testing.assert_array_equal(marked[y-2:y+3, x-2:x+3], plain[y-2:y+3, x-2:x+3])
    x, y = round(ox + sx * points[0, 0]), round(oy + sy * points[0, 1])
    assert np.any(marked[y-10:y+11, x-10:x+11] != plain[y-10:y+11, x-10:x+11])
    assert board.force_n[0, 0] == 12.
    assert board.totals[0] == 16.
    assert marked.shape == (720, 1280, 3) and marked.dtype == np.uint8


@pytest.mark.parametrize("timestamps,forces", [([0., 0.], np.zeros((2, 4))),
                            ([0.], [[-1., 0, 0, 0]]), ([0.], [[np.nan, 0, 0, 0]])])
def test_invalid_saved_signals_are_rejected(timestamps, forces):
    with pytest.raises(ValueError):
        TactileDashboard(timestamps, NAMES, forces)


def test_marker_alignment_and_frame_shape_are_validated():
    board = TactileDashboard([0.], NAMES, np.zeros((1, 4)), width=640, height=360)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    assert board.frame(0, rgb, rgb).shape == (360, 640, 3)
    with pytest.raises(ValueError, match="align"):
        board.frame(0, rgb, rgb, np.zeros((3, 2)))
    with pytest.raises(ValueError, match="RGB uint8"):
        board.frame(0, rgb.astype(float), rgb)
