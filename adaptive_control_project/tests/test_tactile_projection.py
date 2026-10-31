"""Saved contact-force markers follow the rendered, measured robot geometry."""
import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.tactile_projection import (
    project_points, recorded_link_centers,
)


def test_projection_matches_opengl_axes_and_excludes_invalid_or_offscreen_points():
    points = [[0, 0, -2], [.5, .5, -2], [0, 0, 2], [0, 0, 0],
              [5, 0, -2], [np.nan, 0, -2]]
    pixels, visible = project_points(points, np.eye(4), 640, 480, yfov_deg=90)
    np.testing.assert_allclose(pixels[:2], [[320, 240], [380, 180]])
    np.testing.assert_array_equal(visible, [True, True, False, False, False, False])
    assert np.isnan(pixels[[2, 3, 5]]).all()


def test_projection_broadcasts_and_uses_world_camera_pose():
    poses = np.repeat(np.eye(4)[None], 2, axis=0)
    # Camera 1 looks down -Z from x=1; camera 2 looks toward world -X.
    poses[0, 0, 3] = 1
    poses[1, :3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    points = np.array([[[1., 0, -2]], [[-2, 0, 0]]])
    pixels, visible = project_points(points, poses, 640, 480)
    np.testing.assert_allclose(pixels[:, 0], [[320, 240], [320, 240]])
    assert visible.all()
    static_pixels, _ = project_points(points[0], poses, 640, 480)
    assert static_pixels.shape == (2, 1, 2)
    fixed_pixels, _ = project_points(points, poses[0], 640, 480)
    assert fixed_pixels.shape == (2, 1, 2)


@pytest.mark.parametrize("points,camera,width,height,fov", [
    ([0, 0, -1], np.eye(4), 640, 480, 42),
    ([[0, 0, -1]], np.eye(3), 640, 480, 42),
    ([[0, 0, -1]], np.eye(4), 0, 480, 42),
    ([[0, 0, -1]], np.eye(4), 640, 480, 180),
    (np.zeros((2, 1, 3)), np.zeros((3, 4, 4)), 640, 480, 42),
])
def test_projection_rejects_ambiguous_shapes_or_intrinsics(points, camera, width, height, fov):
    with pytest.raises(ValueError):
        project_points(points, camera, width, height, fov)


def test_recorded_centers_match_actual_prefab_link_mesh_and_measured_pose():
    import superdex.physics as sdp
    import superdex.robotics as sdr
    from scipy.spatial.transform import Rotation
    from superdex.physics.paths import resolve_asset

    from adaptive_control_project.retarget_pipeline.paths import BOT_ASSET
    from adaptive_control_project.retarget_pipeline.render import load_link_mesh

    sdp.initialize(num_worker_threads=0)
    scene = bot = None
    try:
        prefab = sdr.load_bot_prefab_from_file(str(resolve_asset(BOT_ASSET)))
        q = np.asarray(prefab.default_pose, dtype=float)
        # A nonzero measured finger configuration catches incorrect DOF indexing.
        for index, joint in enumerate(j for j in prefab.joints if j.type == sdp.ArticulatedJointType.REVOLUTE):
            axis = np.asarray(joint.axis)
            lo, hi = np.asarray(joint.min_limit) @ axis, np.asarray(joint.max_limit) @ axis
            q[index] = .6 * lo + .4 * hi
        prefab.default_pose = q.tolist()
        names = ["right_link_14_index_finger_pad", "right_link_15_index_finger_tip"]
        centers = recorded_link_centers(prefab, sdp, q[None], names)
        scene = sdp.create_scene("recorded tactile geometry")
        bot = sdr.create_bot(scene, prefab, sdr.create_context())
        transforms = sdp.DynamicArrayTransformRT(len(prefab.links))
        bot.get_articulated_actor().get_articulated_link_transforms(transforms)
        link_names = [link.name for link in prefab.links]
        for column, name in enumerate(names):
            index = link_names.index(name)
            link = prefab.links[index]
            local_center = np.zeros(3)
            if link.render_model_file:
                mesh = load_link_mesh(link.render_model_file, [255, 255, 255, 255])
                local_center = np.asarray(mesh.bounds).mean(axis=0)
            pose = transforms[index]
            expected = (Rotation.from_quat(pose.rotation).apply(local_center)
                        + np.asarray(pose.translation))
            np.testing.assert_allclose(centers[0, column], expected, atol=1e-6)
        with pytest.raises(ValueError, match="shape"):
            recorded_link_centers(prefab, sdp, q[None, :-1], names)
        with pytest.raises(ValueError, match="finite"):
            recorded_link_centers(prefab, sdp, np.full_like(q[None], np.nan), names)
        with pytest.raises(ValueError, match="exist"):
            recorded_link_centers(prefab, sdp, q[None], ["missing"])
    finally:
        if bot is not None:
            sdr.destroy_bot(scene, bot)
        sdp.shutdown()
