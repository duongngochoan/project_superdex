"""Readable views of saved simulated hand-object contact forces.

Markers represent force on a whole link, placed at its rendered mesh centre.
They do not represent contact-point locations, pressure or hardware taxels.
The compositor never changes, smooths or amplifies the saved force values.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .tactile import FINGERS, FORCE_SCALE_N, force_color, grouped_forces


class TactileDashboard:
    """Combine synchronized wrist/context frames with their recorded forces.

    ``link_pixels`` and ``link_visible`` passed to :meth:`frame` have the same
    order as ``link_names``. Pixels use the supplied wrist image's coordinates.
    Visibility means inside the camera frustum, without an occlusion test.
    """

    def __init__(self, timestamps, link_names, force_n, side="right", width=1280, height=720):
        self.timestamps = np.asarray(timestamps, dtype=float)
        self.link_names = np.asarray(link_names, dtype=str)
        self.force_n = np.asarray(force_n, dtype=float)
        if (self.timestamps.ndim != 1 or not len(self.timestamps)
                or not np.isfinite(self.timestamps).all()
                or np.any(np.diff(self.timestamps) <= 0)):
            raise ValueError("timestamps must be finite and strictly increasing")
        if (self.link_names.ndim != 1 or len(set(self.link_names)) != len(self.link_names)
                or self.force_n.shape != (len(self.timestamps), len(self.link_names))
                or not np.isfinite(self.force_n).all() or np.any(self.force_n < 0)):
            raise ValueError("force_n must contain finite nonnegative values for unique link_names")
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if not isinstance(width, int) or not isinstance(height, int) or min(width, height) < 1:
            raise ValueError("width and height must be positive integers")
        self.side, self.width, self.height = side, width, height
        self.active_links = np.asarray([n.startswith(f"{side}_link_") for n in self.link_names])
        self.groups = np.asarray([list(grouped_forces(self.link_names, f, side).values())
                                  for f in self.force_n])
        self.totals = self.groups.sum(axis=1)
        # Zero remains zero: even weak recorded contact is retained, not gated.
        self.contact = self.totals > 0
        edges = np.diff(np.r_[False, self.contact, False].astype(int))
        self.contact_intervals = [(float(self.timestamps[start]), float(self.timestamps[end - 1]))
                                  for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))]
        self.group_max_n = max(5., 5. * np.ceil(float(self.groups.max(initial=0)) / 5.))
        self.plot_max_n = max(5., 5. * np.ceil(float(self.totals.max(initial=0)) / 5.))
        self.fonts = {(size, bold): self._font(size, bold) for size in (14, 15, 16, 17, 18, 20, 22, 24, 28, 30, 46)
                      for bold in (False, True)}
        self._background = self._make_background()

    @staticmethod
    def _font(size, bold=False):
        name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
        for path in (Path("/usr/share/fonts/truetype/dejavu") / name, name):
            try:
                return ImageFont.truetype(str(path), size)
            except OSError:
                pass
        return ImageFont.load_default(size=size)

    def _text(self, draw, xy, text, size=18, fill=(227, 235, 244), bold=False, **kwargs):
        draw.text(xy, text, font=self.fonts[size, bold], fill=fill, **kwargs)

    @staticmethod
    def _force_text(value):
        return "<0.01 N" if 0 < value < .005 else f"{value:.2f} N"

    @staticmethod
    def _rgb(array):
        array = np.asarray(array)
        if array.ndim != 3 or array.shape[2] != 3 or min(array.shape[:2]) < 1 or array.dtype != np.uint8:
            raise ValueError("camera frames must be nonempty RGB uint8 images")
        return Image.fromarray(array)

    @staticmethod
    def _fit(canvas, array, box):
        image = TactileDashboard._rgb(array)
        x, y, w, h = box
        scale = min(w / image.width, h / image.height)
        size = (max(1, round(image.width * scale)), max(1, round(image.height * scale)))
        offset = (x + (w - size[0]) // 2, y + (h - size[1]) // 2)
        canvas.paste(image.resize(size, Image.Resampling.LANCZOS), offset)
        return offset, (size[0] / image.width, size[1] / image.height)

    def _plot_x(self, timestamp):
        span = max(float(self.timestamps[-1] - self.timestamps[0]), 1e-9)
        return 66 + 1178 * (timestamp - self.timestamps[0]) / span

    def _make_background(self):
        im = Image.new("RGB", (1280, 720), (12, 20, 32))
        d = ImageDraw.Draw(im)
        self._text(d, (20, 12), "TACTILE SIGNAL & FORCE ON HAND", 30, bold=True)
        self._text(d, (21, 50), "Simulated hand-cup contact  |  Raw force magnitudes in newtons", 17,
                   fill=(156, 179, 202))
        d.rounded_rectangle((19, 91, 741, 564), radius=10, fill=(23, 35, 50), outline=(59, 83, 104), width=2)
        self._text(d, (20, 73), f"{self.side.upper()} WRIST CAMERA + LINK-FORCE MARKERS", 15, bold=True)
        self._text(d, (1025, 73), "ROBOT CONTEXT", 15, bold=True)
        d.rounded_rectangle((1020, 91, 1261, 273), radius=8, fill=(23, 35, 50), outline=(59, 83, 104))
        self._text(d, (764, 94), f"{self.side.upper()} HAND TOTAL", 20, bold=True)
        self._text(d, (764, 247), "FINGER / PALM FORCE", 20, bold=True)
        self._text(d, (764, 277), f"Bar range: 0-{self.group_max_n:.0f} N; numbers are unclipped", 15,
                   fill=(156, 179, 202))
        for j, finger in enumerate(FINGERS):
            y = 305 + j * 42
            self._text(d, (764, y), finger.capitalize(), 20)
            d.rounded_rectangle((870, y + 7, 1132, y + 23), radius=5, fill=(35, 54, 73))
        self._text(d, (22, 573), "Link marker colour:", 16, fill=(180, 198, 214))
        for x in range(160):
            color = tuple((255 * force_color(FORCE_SCALE_N * x / 159)[:3]).astype(int))
            d.line((185 + x, 578, 185 + x, 589), fill=color)
        self._text(d, (353, 572), "0 to >=5 N  |  Numbers show recorded force", 16,
                   fill=(180, 198, 214))
        self._text(d, (22, 598), "TOTAL CONTACT FORCE [N]", 16, bold=True)
        if self.contact_intervals:
            if len(self.contact_intervals) <= 2:
                label = "Contact: " + ", ".join(f"{start:.2f}-{end:.2f} s" for start, end in self.contact_intervals)
            else:
                label = f"Contact: {len(self.contact_intervals)} intervals (shaded)"
        else:
            label = "No hand-cup contact recorded"
        self._text(d, (1244, 598), label, 16, fill=(137, 216, 187), anchor="ra")
        for start, end in self.contact_intervals:
            x0, x1 = self._plot_x(start), self._plot_x(end)
            d.rectangle((x0, 624, max(x0 + 2, x1), 675), fill=(24, 67, 66))
        for fraction in (0., .5, 1.):
            y = round(675 - 51 * fraction)
            d.line((66, y, 1244, y), fill=(42, 60, 78))
            self._text(d, (56, y - 8), f"{self.plot_max_n * fraction:g}", 14,
                       fill=(163, 184, 204), anchor="ra")
        points = [(self._plot_x(t), 675 - 51 * f / self.plot_max_n)
                  for t, f in zip(self.timestamps, self.totals)]
        if len(points) > 1:
            d.line(points, fill=(74, 209, 218), width=2)
        for fraction in (0., .25, .5, .75, 1.):
            t = self.timestamps[0] + fraction * (self.timestamps[-1] - self.timestamps[0])
            self._text(d, (self._plot_x(t), 678), f"{t:.1f}s", 14, fill=(163, 184, 204), anchor="ma")
        self._text(d, (20, 702), "Markers show force on each link at its mesh centre; not contact points, pressure or hardware taxels.", 14,
                   fill=(150, 171, 190))
        return im

    def _markers(self, canvas, index, offset, scale, link_pixels, link_visible, wrist_shape):
        if link_pixels is None:
            if link_visible is not None:
                raise ValueError("link_visible requires link_pixels")
            return
        pixels = np.asarray(link_pixels, dtype=float)
        if pixels.shape != (len(self.link_names), 2):
            raise ValueError("link_pixels must align to link_names with shape (L,2)")
        visible = np.ones(len(pixels), dtype=bool) if link_visible is None else np.asarray(link_visible, dtype=bool)
        if visible.shape != (len(pixels),):
            raise ValueError("link_visible must align to link_names with shape (L,)")
        valid = (visible & self.active_links & np.isfinite(pixels).all(axis=1)
                 & (pixels[:, 0] >= 0) & (pixels[:, 0] < wrist_shape[1])
                 & (pixels[:, 1] >= 0) & (pixels[:, 1] < wrist_shape[0]))
        positions = pixels * np.asarray(scale) + np.asarray(offset)
        overlay = Image.new("RGBA", canvas.size)
        draw = ImageDraw.Draw(overlay)
        for j in np.flatnonzero(valid):
            x, y = positions[j]
            f = self.force_n[index, j]
            rgb = tuple(int(v) for v in force_color(f)[:3] * 255)
            if f > 0:
                radius = 8 + 9 * min(f / FORCE_SCALE_N, 1.)
                draw.ellipse((x-radius-7, y-radius-7, x+radius+7, y+radius+7), fill=(*rgb, 75))
                draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=(*rgb, 160), outline=(255, 247, 211, 255), width=2)
                draw.ellipse((x-3, y-3, x+3, y+3), fill=(255, 255, 255, 255))
            else:
                draw.ellipse((x-2, y-2, x+2, y+2), fill=(*rgb, 150))
        canvas.paste(Image.alpha_composite(canvas.convert("RGBA"), overlay).convert("RGB"))
        draw = ImageDraw.Draw(canvas)
        strongest = np.flatnonzero(valid & (self.force_n[index] > 0))
        strongest = strongest[np.argsort(self.force_n[index, strongest])[::-1]][:2]
        for slot, j in enumerate(strongest):
            x, y = positions[j]
            name = self.link_names[j].partition("_link_")[2].partition("_")[2]
            label = name.replace("_finger", "").replace("_", " ")
            if len(label) > 21:
                label = label[:21]
            text = f"{label}: {self._force_text(self.force_n[index, j])}"
            bx, by = 35, 479 + slot * 36
            draw.line((x, y, bx + 270, by + 13), fill=(255, 230, 144), width=2)
            draw.rounded_rectangle((bx, by, bx + 286, by + 29), radius=5,
                                   fill=(15, 27, 40), outline=(255, 216, 111))
            self._text(draw, (bx + 8, by + 3), text, 16, fill=(255, 239, 186), bold=True)

    def frame(self, index, wrist_rgb, external_rgb, link_pixels=None, link_visible=None):
        """Render one frame with measured signals and projected link markers."""
        if not isinstance(index, (int, np.integer)) or not 0 <= index < len(self.timestamps):
            raise IndexError("dashboard frame index is outside the recorded signal")
        im = self._background.copy()
        offset, scale = self._fit(im, wrist_rgb, (21, 93, 718, 470))
        self._fit(im, external_rgb, (1022, 93, 237, 178))
        self._markers(im, index, offset, scale, link_pixels, link_visible, np.asarray(wrist_rgb).shape)
        d = ImageDraw.Draw(im)
        self._text(d, (1257, 20), f"t = {self.timestamps[index]:06.3f} s", 28, bold=True, anchor="ra")
        self._text(d, (761, 125), self._force_text(self.totals[index]), 46, bold=True,
                   fill=(255, 209, 123) if self.contact[index] else (172, 203, 228))
        status = "CONTACT DETECTED" if self.contact[index] else "NO HAND-CUP CONTACT"
        color = (116, 239, 175) if self.contact[index] else (170, 192, 211)
        self._text(d, (764, 188), status, 18, bold=True, fill=color)
        self._text(d, (764, 218), f"Run peak: {self.totals.max():.2f} N", 16, fill=(156, 179, 202))
        for j, value in enumerate(self.groups[index]):
            y = 305 + j * 42
            if value > 0:
                color = tuple((force_color(value)[:3] * 255).astype(int))
                d.rounded_rectangle((870, y + 7, max(872, 870 + 262 * value / self.group_max_n), y + 23),
                                    radius=4, fill=color)
            self._text(d, (1255, y - 1), self._force_text(value), 22, bold=True, anchor="ra")
        x = self._plot_x(self.timestamps[index])
        d.line((x, 621, x, 675), fill=(255, 232, 169), width=2)
        y = 675 - 51 * self.totals[index] / self.plot_max_n
        d.ellipse((x-4, y-4, x+4, y+4), fill=(255, 232, 169))
        if (self.width, self.height) != im.size:
            im = im.resize((self.width, self.height), Image.Resampling.LANCZOS)
        return np.asarray(im)
