"""Camera-to-LaserScan core of the visual floor scan (no ROS imports, unit-testable).

Pipeline per frame (processed at a reduced resolution):
  1. FloorModel: Lab colour model (running mean + covariance) learned from a seed patch at the
     bottom centre of the image; pixels within a Mahalanobis threshold are floor; morphology
     cleans the mask; strong Canny edges are removed from it so the walk stops at them.
  2. first_obstacle_rows: for every column, walk up from the bottom to the first non-floor
     pixel (the base of the obstacle or wall).
  3. GroundLUT: every pixel below the horizon is back-projected through the intrinsics and the
     camera pose onto the plane z = 0 of the base frame, expressed as (range, bearing) in the
     scan frame. Precomputed once per camera pose / intrinsics change.
  4. to_scan: columns are binned by bearing into a fixed set of beams (min range per beam).
"""
from dataclasses import dataclass

import cv2
import numpy as np


# --------------------------------------------------------------------------- geometry
@dataclass
class GroundLUT:
    """Per-pixel ground-plane range/bearing (in the scan frame) at the processed resolution."""
    rng: np.ndarray        # (H, W) float32, inf where the ray does not hit the floor
    bearing: np.ndarray    # (H, W) float32, rad
    horizon_row: np.ndarray  # (W,) int: rows < horizon_row[c] cannot be floor

    @staticmethod
    def build(K: np.ndarray, src_size, proc_size, R_base_cam: np.ndarray, t_base_cam: np.ndarray,
              t_base_scan: np.ndarray, yaw_base_scan: float = 0.0) -> 'GroundLUT':
        """K: 3x3 intrinsics of the source image (src_size = (w, h)); proc_size = (w, h).
        R/t_base_cam: camera optical frame pose in the base frame (z up, floor at z = 0).
        t_base_scan / yaw_base_scan: scan frame pose in the base frame (assumed level)."""
        sw, sh = src_size
        pw, ph = proc_size
        sx, sy = sw / pw, sh / ph
        # pixel centres of the processed image in source pixel coordinates
        u = (np.arange(pw, dtype=np.float64) + 0.5) * sx - 0.5
        v = (np.arange(ph, dtype=np.float64) + 0.5) * sy - 0.5
        uu, vv = np.meshgrid(u, v)
        rays = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], axis=-1)
        d = rays @ R_base_cam.T                         # ray directions in the base frame
        dz = d[..., 2]
        hits = dz < -1e-6
        s = np.where(hits, -t_base_cam[2] / np.where(hits, dz, -1.0), 0.0)   # masked below
        px = t_base_cam[0] + s * d[..., 0] - t_base_scan[0]
        py = t_base_cam[1] + s * d[..., 1] - t_base_scan[1]
        c, sn = np.cos(-yaw_base_scan), np.sin(-yaw_base_scan)
        qx, qy = c * px - sn * py, sn * px + c * py
        rng = np.where(hits, np.hypot(qx, qy), np.inf).astype(np.float32)
        bearing = np.where(hits, np.arctan2(qy, qx), 0.0).astype(np.float32)
        # horizon: first row (from the top) where the ray hits the floor, per column
        horizon_row = np.where(hits.any(axis=0), hits.argmax(axis=0), ph).astype(np.int32)
        return GroundLUT(rng, bearing, horizon_row)


# --------------------------------------------------------------------------- floor colour
class FloorModel:
    """Running Gaussian model of the floor colour in (weighted) Lab.

    min_stddev_l / min_stddev_ab set a lower bound on the model spread (in 8-bit Lab units). They
    stand for the illumination change across the floor that a small seed patch cannot see (e.g.
    the floor getting darker with distance), and keep an untextured floor from collapsing the
    covariance to zero.
    """

    def __init__(self, threshold: float, learning_rate: float, luminance_weight: float,
                 min_stddev_l: float, min_stddev_ab: float, seed_inlier_fraction: float, init_frames: int):
        self.threshold = threshold
        self.alpha = learning_rate
        self.weights = np.array([luminance_weight, 1.0, 1.0], np.float32)
        self.min_cov = np.diag((self.weights * [min_stddev_l, min_stddev_ab, min_stddev_ab]) ** 2)
        self.seed_inlier_fraction = seed_inlier_fraction
        self.init_frames = init_frames
        self.mean = None
        self.cov = None
        self.frames = 0

    def features(self, bgr: np.ndarray) -> np.ndarray:
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
        return lab * self.weights

    def mahalanobis_sq(self, feat: np.ndarray) -> np.ndarray:
        diff = feat.reshape(-1, 3) - self.mean
        inv = np.linalg.inv(self.cov)
        return np.einsum('ij,jk,ik->i', diff, inv, diff).reshape(feat.shape[:2])

    def update(self, seed: np.ndarray) -> bool:
        """Update from seed-patch features (N x 3). Returns True if the patch was accepted."""
        mean = seed.mean(axis=0)
        cov = np.cov(seed.T) + self.min_cov
        if self.mean is None or self.frames < self.init_frames:
            a = 1.0 if self.mean is None else 1.0 / (self.frames + 1)
        else:
            # only learn when the patch still looks like floor (e.g. not an obstacle up close)
            d2 = self.mahalanobis_sq(seed.reshape(-1, 1, 3)).ravel()
            if np.mean(d2 < self.threshold ** 2) < self.seed_inlier_fraction:
                return False
            a = self.alpha
        if self.mean is None:
            self.mean, self.cov = mean, cov
        else:
            self.mean = (1 - a) * self.mean + a * mean
            self.cov = (1 - a) * self.cov + a * cov
        self.frames += 1
        return True


def seed_slice(h: int, w: int, width_frac: float, height_frac: float):
    sw, shh = max(2, int(w * width_frac)), max(2, int(h * height_frac))
    c0 = (w - sw) // 2
    return slice(h - shh, h), slice(c0, c0 + sw)


def floor_mask(model: FloorModel, feat: np.ndarray, gray: np.ndarray, morph_kernel: int,
               canny_low: float, canny_high: float, use_edges: bool) -> np.ndarray:
    mask = (model.mahalanobis_sq(feat) < model.threshold ** 2).astype(np.uint8)
    if morph_kernel > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    if use_edges:
        edges = cv2.Canny(gray, canny_low, canny_high)
        mask[edges > 0] = 0
    return mask.astype(bool)


def first_obstacle_rows(floor: np.ndarray, horizon_row: np.ndarray, min_obstacle_px: int):
    """Row of the floor/obstacle boundary per column, walking up from the bottom.

    Returns (rows, found): rows[c] is the lowest non-floor row that starts a run of at least
    min_obstacle_px non-floor pixels; found[c] is False when the column is floor all the way up
    to the horizon (no obstacle).
    """
    h, w = floor.shape
    non = ~floor
    if min_obstacle_px > 1:
        # a pixel counts only if the next (min_obstacle_px - 1) pixels above it are non-floor too
        run = non.copy()
        for k in range(1, min_obstacle_px):
            shifted = np.zeros_like(non)
            shifted[k:, :] = non[:-k, :]          # pixel k rows above
            run &= shifted
        non = run
    rows_idx = np.arange(h)[:, None]
    below_horizon = rows_idx >= horizon_row[None, :]
    non &= below_horizon
    flipped = non[::-1, :]
    found = flipped.any(axis=0)
    rows = h - 1 - flipped.argmax(axis=0)
    return rows, found


@dataclass
class ScanGeometry:
    angle_min: float
    angle_max: float
    num_beams: int
    range_min: float
    range_max: float

    @property
    def increment(self) -> float:
        return (self.angle_max - self.angle_min) / (self.num_beams - 1)


def to_scan(lut: GroundLUT, rows: np.ndarray, found: np.ndarray, geo: ScanGeometry,
            boundary_offset_px: float = 1.0) -> np.ndarray:
    """Bin column results into beams. inf = looked and saw no obstacle within range_max,
    nan = the beam was not observed by any column.

    The floor/obstacle boundary lies on the lower edge of the first non-floor pixel
    (row + 0.5). Blur and the Canny edge (removed from the floor mask) move the detected
    boundary down by about boundary_offset_px, which is added back here. The LUT is
    interpolated at the resulting fractional row.
    """
    h = lut.rng.shape[0]
    cols = np.arange(rows.shape[0])
    b_row = np.clip(rows + 0.5 - boundary_offset_px, lut.horizon_row, h - 1)
    r0 = np.floor(b_row).astype(int)
    r1 = np.minimum(r0 + 1, h - 1)
    frac = (b_row - r0).astype(np.float32)
    rng0, rng1 = lut.rng[r0, cols], lut.rng[r1, cols]
    rng_b = np.where(np.isfinite(rng0), (1 - frac) * rng0 + frac * rng1, rng1)
    brg_b = (1 - frac) * lut.bearing[r0, cols] + frac * lut.bearing[r1, cols]
    r = np.where(found, rng_b, np.inf)
    # bearing of the column at the boundary (or at the bottom row if no obstacle)
    b = np.where(found, brg_b, lut.bearing[-1, cols])
    r = np.where(r > geo.range_max, np.inf, r)
    r = np.where(r < geo.range_min, geo.range_min, r)   # closer than visible: clamp to the min

    idx = np.round((b - geo.angle_min) / geo.increment).astype(int)
    ok = (idx >= 0) & (idx < geo.num_beams)
    beams = np.full(geo.num_beams, np.inf, np.float32)
    np.minimum.at(beams, idx[ok], r[ok].astype(np.float32))
    seen = np.zeros(geo.num_beams, bool)
    seen[idx[ok]] = True
    return np.where(seen, beams, np.nan).astype(np.float32)
