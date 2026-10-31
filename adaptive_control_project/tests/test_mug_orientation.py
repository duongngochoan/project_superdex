"""The retained DexYCB mug starts upright, with mesh-based table clearance."""
import json
from pathlib import Path

import numpy as np
import pytest
import trimesh


ASSETS = Path(__file__).resolve().parents[1] / "tasks/pickup_the_cup/assets"


def mug_setup():
    scene = json.loads((ASSETS / "scene.json").read_text())
    asset = json.loads((ASSETS / "asset.json").read_text())
    qx, qy, qz, qw = scene["object_rotation_xyzw"]
    rotation = trimesh.transformations.quaternion_matrix([qw, qx, qy, qz])[:3, :3]
    return scene, asset, rotation


def test_mug_open_rim_faces_up_in_world():
    _, asset, rotation = mug_setup()
    # The STL's wide open cavity lies at +Z; the solid base lies at -Z.
    np.testing.assert_allclose(rotation @ asset["local_up_axis"], [0, 0, 1], atol=1e-12)


def test_mug_actual_mesh_starts_one_millimetre_above_table():
    scene, asset, rotation = mug_setup()
    mesh = trimesh.load(ASSETS / asset["collision_mesh"], force="mesh", process=False)
    rotated_vertices = scene["cup_scale"] * mesh.vertices @ rotation.T
    np.testing.assert_allclose(scene["object_translation"][:2], scene["cup_xy"])
    assert -rotated_vertices[:, 2].min() == pytest.approx(scene["object_bottom_offset"], abs=1e-12)
    lowest_world_z = rotated_vertices[:, 2].min() + scene["object_translation"][2]
    assert lowest_world_z - scene["table_top"] == pytest.approx(0.001, abs=1e-12)
