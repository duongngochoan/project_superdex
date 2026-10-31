"""Object-contact forces used as a simulated, per-link tactile proxy (newtons).

These are sums of contact-force magnitudes, not pressure, a taxel array, or
measurements from a hardware sensor. Only hand contact with the task object is
included. Multiple contacts on a link add; vector cancellation is not applied.
"""
from __future__ import annotations

import numpy as np

FORCE_SCALE_N = 5.0
FINGERS = ("palm", "thumb", "index", "middle", "ring", "pinky")
SIGNAL_DESCRIPTION = "SuperDex object-hand contact-force magnitude sum per link [N]; simulated tactile proxy"


def hand_link_names(scene):
    return [name for name in scene.link_names if name.startswith(("left_link_", "right_link_"))]


def sample_tactile(scene, names):
    forces = {}
    for side in ("left", "right"):
        per_link, _, _ = scene.hand_cup_contact(side)
        forces.update(per_link)
    values = np.asarray([forces.get(name, 0.) for name in names], dtype=float)
    if not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError("Contact-force magnitudes must be finite and nonnegative")
    return values


def grouped_forces(names, values, side):
    groups = dict.fromkeys(FINGERS, 0.)
    for name, value in zip(names, values):
        if name.startswith(f"{side}_link_"):
            group = next((finger for finger in FINGERS[1:] if finger in name), "palm")
            groups[group] += float(value)
    return groups


def force_color(force):
    """Fixed linear 0..5 N colour scale; the displayed numeric value is unclipped."""
    weight = np.clip(float(force) / FORCE_SCALE_N, 0., 1.)
    return np.r_[(1-weight)*np.array([.20, .60, .85]) + weight*np.array([1., .12, .02]), 1.]


def annotate_tactile(frame, names, values, hands):
    from PIL import Image, ImageDraw

    im = Image.fromarray(np.ascontiguousarray(frame))
    draw = ImageDraw.Draw(im)
    w, h = im.size
    # A compact legend stays visible on 320px camera captures as well.
    panel_h = min(108, h // 3)
    top = h - panel_h
    draw.rectangle((0, top, w, h), fill=(247, 250, 253))
    draw.text((8, top+4), "SIMULATED TACTILE: object contact [N]", fill=(15, 30, 45))
    draw.text((8, top+18), "Link colour: blue 0 -> red 5 N (clipped)", fill=(45, 60, 75))
    for column, side in enumerate(hands):
        x = 8 + column * w // len(hands)
        groups = grouped_forces(names, values, side)
        draw.text((x, top+33), f"{side}: {sum(groups.values()):.3f} N", fill=(15, 30, 45))
        step = max(24, (w // len(hands) - 12) // 6)
        for index, (finger, value) in enumerate(groups.items()):
            bx = x + index * step
            draw.rectangle((bx, top+49, bx+max(1, int((step-4)*min(value/FORCE_SCALE_N, 1))), top+55),
                           fill=tuple((force_color(value)[:3]*255).astype(int)))
            draw.text((bx, top+58), finger[:3], fill=(45, 60, 75))
            if panel_h >= 90:
                draw.text((bx, top+72), f"{value:.2f}", fill=(15, 30, 45))
    return np.asarray(im)
