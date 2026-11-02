"""Unit tests for the depth-image-to-scan core (no ROS needed)."""
import math

import numpy as np

from jgb_rover_perception.depth_scan_core import DepthRays, depth_to_scan
from jgb_rover_perception.floor_scan_core import ScanGeometry

R_LINK_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # optical (z fwd) -> link (x fwd)


def pose(pitch, height, x=0.0):
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ R_LINK_OPT, np.array([x, 0.0, height])


def setup(pitch=0.0, height=0.246, w=64, h=48, hfov=1.5):
    f = (w / 2) / math.tan(hfov / 2)
    K = np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1]])
    R, t = pose(pitch, height)
    rays = DepthRays.build(K, (w, h), 1, R)
    geo = ScanGeometry(-hfov / 2, hfov / 2, 41, 0.15, 2.5)
    return K, R, t, rays, geo, (w, h)


def scan(depth, t, rays, geo):
    return depth_to_scan(depth, rays, t, np.zeros(3), 0.0, geo, 0.03, 0.30, 0.2, 3.0)


def test_wall_straight_ahead_is_found_at_its_range():
    K, R, t, rays, geo, (w, h) = setup()
    depth = np.full((h, w), 1.2, np.float32)                  # a wall 1.2 m away, filling the view
    r = scan(depth, t, rays, geo)
    mid = geo.num_beams // 2
    # the wall is seen where its height lies in the band (rows around/above the camera height)
    assert abs(r[mid] - 1.2) < 0.02
    assert np.isfinite(r).all() and (r >= 1.19).all()          # every beam saw the wall, nearest >= its range


def test_floor_is_ignored():
    K, R, t, rays, geo, (w, h) = setup(pitch=0.4)
    # depth of a flat floor seen by the pitched camera: z_opt = height / (down-component of the ray)
    d = rays.dirs                                             # base-frame directions of unit optical depth
    with np.errstate(divide='ignore', invalid='ignore'):
        z = np.where(d[:, 2] < -1e-3, -t[2] / d[:, 2], 0.0)
    depth = np.zeros((h, w), np.float32)
    depth[rays.rows, rays.cols] = z
    r = scan(depth, t, rays, geo)
    assert np.isinf(r).all()                                   # floor everywhere: no obstacle


def test_only_the_height_band_counts():
    K, R, t, rays, geo, (w, h) = setup()
    wall = np.full((h, w), 1.0, np.float32)
    args = (rays, t, np.zeros(3), 0.0, geo)
    assert np.isinf(depth_to_scan(wall, *args, 2.0, 3.0, 0.2, 3.0)).all()      # everything below the band
    assert np.isinf(depth_to_scan(wall, *args, -3.0, -2.0, 0.2, 3.0)).all()    # everything above the band
    assert np.isfinite(depth_to_scan(wall, *args, 0.0, 0.5, 0.2, 3.0)).all()   # the wall crosses the band


def test_range_limits():
    K, R, t, rays, geo, (w, h) = setup()
    too_near = np.full((h, w), 0.1, np.float32)               # below depth_min: invalid, not free space
    beyond = np.full((h, w), 2.8, np.float32)                 # valid but beyond range_max (2.5): free
    assert np.isnan(scan(too_near, t, rays, geo)).all()
    assert np.isinf(scan(beyond, t, rays, geo)).all()


def test_bearing_of_an_obstacle_on_the_left():
    K, R, t, rays, geo, (w, h) = setup()
    depth = np.zeros((h, w), np.float32)
    depth[:, :8] = 1.0                                        # left image columns = +y = positive bearing
    r = scan(depth, t, rays, geo)
    hit = np.where(np.isfinite(r))[0]
    assert hit.size and (geo.angle_min + geo.increment * hit).min() > 0.3


def test_beams_outside_the_field_of_view_are_unobserved():
    K, R, t, rays, geo, (w, h) = setup(hfov=1.5)
    wide = ScanGeometry(-1.5, 1.5, 81, 0.15, 2.5)
    r = depth_to_scan(np.full((h, w), 1.2, np.float32), rays, t, np.zeros(3), 0.0, wide, 0.03, 0.30, 0.2, 3.0)
    ang = wide.angle_min + wide.increment * np.arange(wide.num_beams)
    assert np.isfinite(r[np.abs(ang) < 0.7]).all() and np.isnan(r[np.abs(ang) > 0.9]).all()
