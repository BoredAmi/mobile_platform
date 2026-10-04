"""Temperature heatmap core (no ROS imports, unit-testable).

1. LagCompensator: the sensor reads the air temperature through a first-order lag
   (time constant tau): T_s' = (T_air - T_s) / tau, so T_air = T_s + tau * T_s'. A line is fitted
   to the readings of the last window_s seconds; its value and slope at the window centre give
   the air temperature at that time (the fit also averages away noise and quantisation).
2. build_heatmap: (x, y, T) samples in the map frame are averaged per cell, then smoothed with a
   normalised Gaussian kernel (Nadaraya-Watson), so cells between the driven paths are
   interpolated. Cells farther than max_distance from any sample stay unknown (nan).
3. mask_with_map / render: hide cells that the SLAM map does not show as free, and draw the
   heatmap over the map with a colour bar.
"""
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np


# --------------------------------------------------------------------------- sensor lag
class LagCompensator:
    def __init__(self, time_constant_s: float, window_s: float, min_samples: int = 3):
        self.tau = time_constant_s
        self.window = window_s
        self.min_samples = min_samples
        self.buf = deque()

    def add(self, t: float, value: float):
        """Add a reading; returns (t_estimate, T_air) or None until the window is full."""
        self.buf.append((t, value))
        while self.buf and self.buf[0][0] < t - self.window:
            self.buf.popleft()
        if len(self.buf) < self.min_samples or t - self.buf[0][0] < 0.75 * self.window:
            return None
        ts, vs = np.asarray(self.buf).T
        tc = ts.mean()
        slope, value_c = np.polyfit(ts - tc, vs, 1)
        return float(tc), float(value_c + self.tau * slope)


# --------------------------------------------------------------------------- gridding
@dataclass
class Heatmap:
    values: np.ndarray        # (H, W) float32, C; nan = unknown. Row 0 = lowest y.
    origin: np.ndarray        # (x, y) of the lower-left corner of cell (0, 0), map frame
    resolution: float         # m per cell

    def cell_centres(self):
        h, w = self.values.shape
        xs = self.origin[0] + (np.arange(w) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(h) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)

    def lookup(self, x, y):
        """Nearest-cell value at (x, y) arrays; nan outside the grid."""
        c = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        r = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        h, w = self.values.shape
        ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
        out = np.full(np.shape(c), np.nan, np.float32)
        out[ok] = self.values[r[ok], c[ok]]
        return out


def build_heatmap(xy: np.ndarray, temps: np.ndarray, resolution: float, sigma: float,
                  max_distance: float) -> Heatmap:
    xy = np.asarray(xy, float).reshape(-1, 2)
    temps = np.asarray(temps, float).ravel()
    margin = max_distance + resolution
    origin = np.floor((xy.min(axis=0) - margin) / resolution) * resolution
    w, h = (np.ceil((xy.max(axis=0) + margin - origin) / resolution).astype(int) + 1)
    c = ((xy[:, 0] - origin[0]) / resolution).astype(int)
    r = ((xy[:, 1] - origin[1]) / resolution).astype(int)
    total = np.zeros((h, w), np.float64)
    count = np.zeros((h, w), np.float64)
    np.add.at(total, (r, c), temps)
    np.add.at(count, (r, c), 1.0)
    # each visited cell counts once (its mean), so long stops do not dominate their surroundings
    visited = count > 0
    mean = np.where(visited, total / np.maximum(count, 1.0), 0.0)
    s = max(sigma / resolution, 1e-3)
    k = int(2 * np.ceil(3 * s) + 1)
    num = cv2.GaussianBlur(mean, (k, k), s, borderType=cv2.BORDER_CONSTANT)
    den = cv2.GaussianBlur(visited.astype(np.float64), (k, k), s, borderType=cv2.BORDER_CONSTANT)
    dist = cv2.distanceTransform((~visited).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    values = np.where((dist <= max_distance) & (den > 1e-6), num / np.maximum(den, 1e-12), np.nan)
    return Heatmap(values.astype(np.float32), origin, resolution)


# --------------------------------------------------------------------------- SLAM map
@dataclass
class OccMap:
    """Occupancy map: data (H, W) int8 as in nav_msgs/OccupancyGrid (-1 unknown, 0..100), row 0 =
    lowest y, origin = (x, y) of cell (0, 0) corner (map yaw must be 0)."""
    data: np.ndarray
    origin: np.ndarray
    resolution: float

    def classes(self, x, y, free_thresh=25, occupied_thresh=65):
        """0 unknown / outside, 1 free, 2 occupied at (x, y) arrays."""
        c = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        r = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        h, w = self.data.shape
        ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
        v = np.full(np.shape(c), -1, np.int16)
        v[ok] = self.data[r[ok], c[ok]]
        return np.where((v >= 0) & (v <= free_thresh), 1, np.where(v >= occupied_thresh, 2, 0))


def mask_with_map(hm: Heatmap, occ: OccMap) -> Heatmap:
    """Unknown wherever the map cell under the heatmap cell centre is not free."""
    xs, ys = hm.cell_centres()
    free = occ.classes(xs, ys) == 1
    return Heatmap(np.where(free, hm.values, np.nan).astype(np.float32), hm.origin, hm.resolution)


# --------------------------------------------------------------------------- output
def colour_range(values: np.ndarray, t_min: float = None, t_max: float = None):
    finite = values[np.isfinite(values)]
    lo = t_min if t_min is not None else (float(finite.min()) if finite.size else 0.0)
    hi = t_max if t_max is not None else (float(finite.max()) if finite.size else 1.0)
    return lo, max(hi, lo + 0.1)


def to_grid_values(values: np.ndarray, t_min: float, t_max: float) -> np.ndarray:
    """int8 OccupancyGrid data: 1..98 along the RViz 'costmap' colour scheme, -1 unknown."""
    scaled = np.clip((values - t_min) / (t_max - t_min), 0.0, 1.0) * 97.0 + 1.0
    return np.where(np.isfinite(values), np.round(scaled), -1).astype(np.int8)


def colourize(values: np.ndarray, t_min: float, t_max: float) -> np.ndarray:
    """BGR colours (TURBO / JET colormap) for finite values."""
    u8 = (np.clip((np.nan_to_num(values, nan=t_min) - t_min) / (t_max - t_min), 0, 1) * 255).astype(np.uint8)
    cmap = getattr(cv2, 'COLORMAP_TURBO', cv2.COLORMAP_JET)
    return cv2.applyColorMap(u8, cmap)


def render(hm: Heatmap, occ: OccMap = None, samples_xy: np.ndarray = None, t_min=None, t_max=None,
           px_per_m: float = 100.0, alpha: float = 0.8, title: str = '') -> np.ndarray:
    """Heatmap over the SLAM map (if given), sample positions as dots, colour bar on the right."""
    lo, hi = colour_range(hm.values, t_min, t_max)
    h_, w_ = hm.values.shape
    x0, y0 = hm.origin
    x1, y1 = x0 + w_ * hm.resolution, y0 + h_ * hm.resolution
    if occ is not None:
        oh, ow = occ.data.shape
        x0, y0 = min(x0, occ.origin[0]), min(y0, occ.origin[1])
        x1, y1 = max(x1, occ.origin[0] + ow * occ.resolution), max(y1, occ.origin[1] + oh * occ.resolution)
    W, H = int(np.ceil((x1 - x0) * px_per_m)), int(np.ceil((y1 - y0) * px_per_m))
    xs = x0 + (np.arange(W) + 0.5) / px_per_m
    ys = y0 + (np.arange(H) + 0.5) / px_per_m
    X, Y = np.meshgrid(xs, ys)
    img = np.full((H, W, 3), 205, np.uint8)
    if occ is not None:
        cls = occ.classes(X, Y)
        img[cls == 1] = 255
        img[cls == 2] = 0
    else:
        img[:] = 255
    t = hm.lookup(X, Y)
    known = np.isfinite(t)
    col = colourize(t, lo, hi)
    img[known] = (alpha * col[known] + (1 - alpha) * img[known]).astype(np.uint8)
    if samples_xy is not None and len(samples_xy):
        sc = ((np.asarray(samples_xy)[:, 0] - x0) * px_per_m).astype(int)
        sr = ((np.asarray(samples_xy)[:, 1] - y0) * px_per_m).astype(int)
        ok = (sc >= 0) & (sc < W) & (sr >= 0) & (sr < H)
        img[sr[ok], sc[ok]] = (60, 60, 60)
    img = np.ascontiguousarray(img[::-1])                       # y up
    # colour bar
    bar_w, pad = 24, 70
    out = np.full((max(H, 200) + 30, W + bar_w + pad + 20, 3), 255, np.uint8)
    out[30:30 + H, :W] = img
    bar_h = max(H, 200) - 20
    ramp = np.linspace(hi, lo, bar_h)[:, None].repeat(bar_w, axis=1)
    out[40:40 + bar_h, W + 15:W + 15 + bar_w] = colourize(ramp, lo, hi)
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        v = hi - frac * (hi - lo)
        yy = 40 + int(frac * (bar_h - 1))
        cv2.putText(out, f'{v:.1f} C', (W + 20 + bar_w, yy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1,
                    cv2.LINE_AA)
    if title:
        cv2.putText(out, title, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return out
