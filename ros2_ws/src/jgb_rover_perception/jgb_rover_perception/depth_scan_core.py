"""Depth-image-to-LaserScan core for a depth camera (RealSense D435), no ROS imports.

Every valid depth pixel is back-projected through the intrinsics and the camera pose (from TF)
into the base frame (z up, floor at z = 0). Points whose height lies in [min_height, max_height]
are obstacles: floor points (below min_height) and overhead points (above max_height) are
dropped, so a pitched camera needs no special handling. Obstacle points are expressed as
(range, bearing) in the scan frame and binned by bearing into beams (nearest point per beam). A beam with no valid depth at all is unobserved (nan).
"""
from dataclasses import dataclass

import numpy as np

from jgb_rover_perception.floor_scan_core import ScanGeometry


@dataclass
class DepthRays:
    """Unit-depth ray directions of the strided pixel grid, in the base frame (z = 1 along the optical axis)."""
    dirs: np.ndarray       # (N, 3) base-frame direction for depth 1 m along the optical axis
    rows: np.ndarray       # (N,) source row of each ray
    cols: np.ndarray       # (N,) source column of each ray

    @staticmethod
    def build(K: np.ndarray, size, stride: int, R_base_cam: np.ndarray) -> 'DepthRays':
        w, h = size
        u = np.arange(stride // 2, w, stride)
        v = np.arange(stride // 2, h, stride)
        uu, vv = np.meshgrid(u, v)
        rays = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones(uu.shape)], axis=-1)
        return DepthRays((rays.reshape(-1, 3) @ R_base_cam.T), vv.ravel(), uu.ravel())


def depth_to_scan(depth: np.ndarray, rays: DepthRays, t_base_cam: np.ndarray, t_base_scan: np.ndarray,
                  yaw_base_scan: float, geo: ScanGeometry, min_height: float, max_height: float,
                  depth_min: float, depth_max: float) -> np.ndarray:
    """depth: (H, W) metres, 0 / nan = invalid. Returns per-beam ranges: nearest obstacle in
    range, inf = looked and saw none, nan = no valid depth in that beam (outside the field of view,
    or a hole in the depth image). Beams must be wider than the angular spacing of the strided rays."""
    z = depth[rays.rows, rays.cols].astype(np.float64)
    valid = np.isfinite(z) & (z >= depth_min) & (z <= depth_max)
    p = rays.dirs[valid] * z[valid, None] + t_base_cam
    height = p[:, 2]
    obstacle = (height >= min_height) & (height <= max_height)
    c, s = np.cos(-yaw_base_scan), np.sin(-yaw_base_scan)
    dx, dy = p[:, 0] - t_base_scan[0], p[:, 1] - t_base_scan[1]
    qx, qy = c * dx - s * dy, s * dx + c * dy
    bearing = np.arctan2(qy, qx)
    rng = np.hypot(qx, qy)

    idx = np.round((bearing - geo.angle_min) / geo.increment).astype(int)
    in_beams = (idx >= 0) & (idx < geo.num_beams)
    seen = np.zeros(geo.num_beams, bool)
    seen[idx[in_beams]] = True                                # any valid depth, floor included
    hit = obstacle & in_beams & (rng >= geo.range_min) & (rng <= geo.range_max)
    beams = np.full(geo.num_beams, np.inf, np.float32)
    np.minimum.at(beams, idx[hit], rng[hit].astype(np.float32))
    return np.where(seen, beams, np.nan).astype(np.float32)
