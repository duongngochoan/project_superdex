"""Place saved per-link force markers on the measured robot geometry.

Marker anchors are centers of rendered link bounds, transformed with exact
prefab forward kinematics. They are schematic link-force markers, not measured
contact points, taxel positions, or force-vector directions. This module reads
saved joint states; it does not step or replay the physics simulation.
"""
from __future__ import annotations

import numpy as np

from .arm_ik import ArmChain


def project_points(points_world, world_from_camera, width, height, yfov_deg=42):
    """Return pixel coordinates and a finite, front-facing viewport mask.

    Points have shape ``(L, 3)`` or ``(T, L, 3)``. Camera poses have shape
    ``(4, 4)`` or ``(T, 4, 4)``; a single pose or point set broadcasts over
    frames. Poses use the renderer's OpenGL convention: local -Z points forward,
    +X points right, and +Y points up. Pixels use a top-left origin. Behind-camera
    and nonfinite points receive NaN coordinates and are never visible. The
    mask checks the viewport only, not occlusion by scene geometry.
    """
    points = np.asarray(points_world, dtype=float)
    cameras = np.asarray(world_from_camera, dtype=float)
    if points.ndim not in (2, 3) or points.shape[-1] != 3:
        raise ValueError("points_world must have shape (L, 3) or (T, L, 3)")
    if cameras.ndim not in (2, 3) or cameras.shape[-2:] != (4, 4):
        raise ValueError("world_from_camera must have shape (4, 4) or (T, 4, 4)")
    if (not isinstance(width, (int, np.integer)) or isinstance(width, bool) or width <= 0
            or not isinstance(height, (int, np.integer)) or isinstance(height, bool) or height <= 0):
        raise ValueError("width and height must be positive integers")
    if not np.isfinite(yfov_deg) or not 0 < yfov_deg < 180:
        raise ValueError("yfov_deg must be finite and between 0 and 180 degrees")
    try:
        batch = np.broadcast_shapes(points.shape[:-2], cameras.shape[:-2])
    except ValueError as error:
        raise ValueError("Point frames and camera frames must match or broadcast") from error
    points = np.broadcast_to(points, batch + points.shape[-2:])
    cameras = np.broadcast_to(cameras, batch + (4, 4))
    # Row-vector equivalent of inverse(camera_pose) @ homogeneous_point.
    camera_points = np.einsum(
        "...li,...ij->...lj", points - cameras[..., None, :3, 3], cameras[..., :3, :3])
    depth = -camera_points[..., 2]
    valid = (np.isfinite(camera_points).all(axis=-1)
             & np.isfinite(cameras).all(axis=(-2, -1))[..., None] & (depth > 0))
    focal = height / (2 * np.tan(np.deg2rad(yfov_deg) / 2))
    pixels = np.full(points.shape[:-1] + (2,), np.nan)
    np.divide(camera_points[..., :2] * focal, depth[..., None],
              out=pixels, where=valid[..., None])
    pixels[..., 0] += width / 2
    pixels[..., 1] = height / 2 - pixels[..., 1]
    visible = (valid & np.isfinite(pixels).all(axis=-1)
               & (pixels[..., 0] >= 0) & (pixels[..., 0] < width)
               & (pixels[..., 1] >= 0) & (pixels[..., 1] < height))
    return pixels, visible


def recorded_link_centers(prefab, sdp, joint_samples, link_names):
    """Return ``(T, L, 3)`` rendered-link centers from measured joint states.

    ``joint_samples`` must contain the full prefab DOF vector for every sampled
    frame in prefab revolute-joint order, as stored in replay ``log.npz['q']``.
    Callers choose saved states on their video clock before calling this helper.
    Each mesh uses the same GLB-to-Z-up conversion as the video renderer. Links
    without rendered geometry use their exact link-frame origin instead.

    These centers identify the link carrying an aggregated contact-force value;
    they do not locate individual contacts or physical tactile sensors.
    """
    q = np.asarray(joint_samples, dtype=float)
    dof_count = sum(joint.type == sdp.ArticulatedJointType.REVOLUTE for joint in prefab.joints)
    if q.ndim != 2 or q.shape[1] != dof_count or not np.isfinite(q).all():
        raise ValueError(f"joint_samples must be finite with shape (T, {dof_count})")
    links = {link.name: link for link in prefab.links}
    names = list(link_names)
    if any(name not in links for name in names):
        raise ValueError("Every requested link name must exist in the prefab")
    centers = np.empty((len(q), len(names), 3), dtype=float)
    meshes = {}
    for column, name in enumerate(names):
        chain = ArmChain(prefab, "world", name, sdp)
        center = np.zeros(3)
        path = links[name].render_model_file
        if path:
            if path not in meshes:
                # Import lazily: projection itself needs no rendering stack.
                from .render import load_link_mesh

                mesh = load_link_mesh(path, [255, 255, 255, 255])
                meshes[path] = (np.asarray(mesh.bounds, dtype=float).mean(axis=0)
                                if len(mesh.vertices) else np.zeros(3))
            center = meshes[path]
        if not np.isfinite(center).all():
            raise ValueError(f"Rendered mesh center is nonfinite for {name}")
        for row, sample in enumerate(q):
            pose, _, _ = chain.fk(sample[chain.dof_indices])
            centers[row, column] = pose[:3, :3] @ center + pose[:3, 3]
    return centers
