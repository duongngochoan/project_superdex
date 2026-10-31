"""Simulated tactile values must represent measured object-contact magnitudes."""
from unittest.mock import Mock

import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.tactile import (
    FORCE_SCALE_N, force_color, grouped_forces, hand_link_names, sample_tactile,
)


def test_contact_mapping_preserves_both_hands_and_unloaded_links():
    scene = Mock()
    scene.link_names = ["openarm_right_link1", "right_link_01_wrist", "right_link_10_index_finger_proximal",
                        "right_link_04_thumb_distal", "left_link_04_thumb_distal"]
    scene.hand_cup_contact.side_effect = [({"left_link_04_thumb_distal": 2.}, 0., 2.),
                                        ({"right_link_10_index_finger_proximal": .7,
                                          "right_link_04_thumb_distal": .3}, 0., 1.)]
    names = hand_link_names(scene)
    assert "openarm_right_link1" not in names
    values = sample_tactile(scene, names)
    np.testing.assert_allclose(values, [0., .7, .3, 2.])
    assert grouped_forces(names, values, "right") == dict(palm=0., thumb=.3, index=.7, middle=0., ring=0., pinky=0.)
    assert sum(grouped_forces(names, values, "left").values()) == 2.


@pytest.mark.parametrize("bad", [np.nan, -1.])
def test_invalid_contact_values_cannot_be_published(bad):
    scene = Mock()
    scene.hand_cup_contact.return_value = ({"right_link_01_wrist": bad}, 0., 0.)
    with pytest.raises(ValueError, match="finite and nonnegative"):
        sample_tactile(scene, ["right_link_01_wrist"])


def test_force_display_has_fixed_scale_and_clips_only_colour():
    names = ["right_link_01_wrist"]
    assert grouped_forces(names, [2*FORCE_SCALE_N], "right")["palm"] == 2*FORCE_SCALE_N
    np.testing.assert_array_equal(force_color(FORCE_SCALE_N), force_color(2*FORCE_SCALE_N))
    assert not np.allclose(force_color(0), force_color(FORCE_SCALE_N))
