"""Unit tests for the visual floor scan geometry and column walk (no ROS needed)."""
import math

import numpy as np

from jgb_rover_perception.floor_scan_core import GroundLUT, ScanGeometry, first_obstacle_rows, to_scan


def camera(pitch, height, x=0.0):
    """Optical-frame pose in a z-up base frame: looking along +x, pitched down by `pitch`."""
    # optical axes in camera_link (x fwd): z_opt = x, x_opt = -y, y_opt = -z
    R_link_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    c, s = math.cos(pitch), math.sin(pitch)
    R_base_link = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])      # rotation about +y
    return R_base_link @ R_link_opt, np.array([x, 0.0, height])


def K_of(w, h, hfov):
    f = (w / 2) / math.tan(hfov / 2)
    return np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1]])


def test_centre_ray_hits_floor_at_expected_distance():
    pitch, height = 0.26, 0.2605
    R, t = camera(pitch, height, x=0.113)
    w, h = 64, 48
    lut = GroundLUT.build(K_of(w, h, 1.745), (w, h), (w, h), R, t, np.array([0.113, 0, 0]))
    # optical centre lies between the two middle rows/columns
    r = 0.5 * (lut.rng[h // 2 - 1, w // 2 - 1] + lut.rng[h // 2, w // 2])
    assert abs(r - height / math.tan(pitch)) < 0.03
    assert abs(lut.bearing[h - 1, w // 2] - 0.0) < 0.05
    assert lut.rng[0, w // 2] == np.inf            # top row looks above the horizon


def test_column_walk_and_scan():
    pitch, height = 0.26, 0.2605
    R, t = camera(pitch, height)
    w, h = 32, 24
    lut = GroundLUT.build(K_of(w, h, 1.745), (w, h), (w, h), R, t, np.zeros(3))
    floor = np.ones((h, w), bool)
    floor[:15, :8] = False             # obstacle in the left 8 columns from row 14 upward
    rows, found = first_obstacle_rows(floor, lut.horizon_row, min_obstacle_px=2)
    assert found[:8].all() and not found[8:].any()
    assert (rows[:8] == 14).all()
    geo = ScanGeometry(-0.87, 0.87, 20, 0.15, 2.5)
    # without offset the boundary is the lower edge of row 14: halfway between rows 14 and 15
    ranges = to_scan(lut, rows, found, geo, boundary_offset_px=0.0)
    finite = ranges[np.isfinite(ranges)]
    expected = (0.5 * (lut.rng[14, :8] + lut.rng[15, :8])).min()
    assert finite.size > 0 and abs(finite.min() - expected) < 1e-4
    # a 1 px offset moves the boundary up one row: farther away
    shifted = to_scan(lut, rows, found, geo, boundary_offset_px=1.0)
    assert np.nanmin(shifted[np.isfinite(shifted)]) > finite.min()
    # the left side of the image is +y (positive bearing): obstacles must be at positive angles
    angles = geo.angle_min + geo.increment * np.nonzero(np.isfinite(ranges))[0]
    assert (angles > 0).all()


def test_distortion_zero_equals_pinhole_and_barrel_moves_edge_rays():
    R, t = camera(0.26, 0.2605)
    w, h = 64, 48
    K = K_of(w, h, 1.745)
    plain = GroundLUT.build(K, (w, h), (w, h), R, t, np.zeros(3))
    zeros = GroundLUT.build(K, (w, h), (w, h), R, t, np.zeros(3), D=np.zeros(5))
    assert np.allclose(plain.rng[np.isfinite(plain.rng)], zeros.rng[np.isfinite(zeros.rng)])
    barrel = GroundLUT.build(K, (w, h), (w, h), R, t, np.zeros(3), D=np.array([-0.3, 0.1, 0, 0, 0]))
    # barrel distortion: a raw corner pixel really looks further out than the pinhole model says
    assert abs(barrel.bearing[h - 1, 0]) > abs(plain.bearing[h - 1, 0])
