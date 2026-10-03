"""2D ground-truth geometry of a Gazebo world, read from the world SDF.

Only collision boxes and cylinders that reach down to the floor (bottom below `max_base_z`) are
kept: that is what a flat-floor camera scan can see (wall bases, legs, boxes); table tops and
seats hanging above the floor are ignored. Planes and thin floor patches are skipped.
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


@dataclass
class Box:
    name: str
    x: float
    y: float
    yaw: float
    sx: float
    sy: float

    def corners(self) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.sx / 2, self.sy / 2
        local = np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]])
        return local @ np.array([[c, s], [-s, c]]) + [self.x, self.y]


@dataclass
class Circle:
    name: str
    x: float
    y: float
    r: float


def _floats(text, n=None):
    v = [float(t) for t in text.split()]
    return v if n is None else (v + [0.0] * n)[:n]


def load_world(sdf_path: str, max_base_z: float = 0.02, min_height: float = 0.005):
    """Return (boxes, circles) for floor-standing collision geometry in the world SDF."""
    root = ET.parse(sdf_path).getroot()
    boxes, circles = [], []
    for model in root.iter('model'):
        mpose = _floats(model.findtext('pose', '0 0 0 0 0 0'), 6)
        for coll in model.iter('collision'):
            cpose = _floats(coll.findtext('pose', '0 0 0 0 0 0'), 6)
            x, y, z = mpose[0] + cpose[0], mpose[1] + cpose[1], mpose[2] + cpose[2]
            yaw = mpose[5] + cpose[5]
            geom = coll.find('geometry')
            if geom.find('box') is not None:
                sx, sy, sz = _floats(geom.find('box').findtext('size'), 3)
                if z - sz / 2 > max_base_z or sz < min_height:
                    continue
                boxes.append(Box(model.get('name'), x, y, yaw, sx, sy))
            elif geom.find('cylinder') is not None:
                r = float(geom.find('cylinder').findtext('radius'))
                length = float(geom.find('cylinder').findtext('length'))
                if z - length / 2 > max_base_z or length < min_height:
                    continue
                circles.append(Circle(model.get('name'), x, y, r))
    return boxes, circles


def raycast(boxes, circles, ox, oy, angles, max_range):
    """Distance from (ox, oy) along each world-frame angle to the first obstacle (inf if none)."""
    out = np.full(len(angles), np.inf)
    dx, dy = np.cos(angles), np.sin(angles)
    for b in boxes:
        c = b.corners()
        for i in range(4):
            p, q = c[i], c[(i + 1) % 4]
            ex, ey = q[0] - p[0], q[1] - p[1]
            den = dx * ey - dy * ex
            with np.errstate(divide='ignore', invalid='ignore'):
                t = ((p[0] - ox) * ey - (p[1] - oy) * ex) / den
                u = ((p[0] - ox) * dy - (p[1] - oy) * dx) / den
            hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
            out = np.where(hit & (t < out), t, out)
    for cc in circles:
        fx, fy = ox - cc.x, oy - cc.y
        bq = fx * dx + fy * dy
        cq = fx * fx + fy * fy - cc.r ** 2
        disc = bq * bq - cq
        with np.errstate(invalid='ignore'):
            t = -bq - np.sqrt(disc)
        hit = (disc >= 0) & (t > 0)
        out = np.where(hit & (t < out), t, out)
    return np.where(out <= max_range, out, np.inf)


def rasterize(boxes, circles, resolution, origin, shape) -> np.ndarray:
    """Occupancy (bool, shape = (rows, cols)) of the obstacles on a grid with the given origin
    (x, y of cell [0, 0] corner) and resolution, row 0 = lowest y (ROS map convention)."""
    import cv2
    grid = np.zeros(shape, np.uint8)
    to_px = lambda pts: np.round((np.asarray(pts) - origin) / resolution - 0.5).astype(np.int32)  # noqa: E731
    for b in boxes:
        cv2.fillPoly(grid, [to_px(b.corners())], 1)
    for c in circles:
        cx, cy = to_px([c.x, c.y])
        cv2.circle(grid, (int(cx), int(cy)), max(1, int(round(c.r / resolution))), 1, -1)
    return grid.astype(bool)
