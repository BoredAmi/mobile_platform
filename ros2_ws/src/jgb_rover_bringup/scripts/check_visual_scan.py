#!/usr/bin/env python3
"""Compare /visual_scan with the true ranges ray-cast in the world SDF from the ground-truth pose.

Optionally spins the robot in place (--spin-rate) so all directions are seen. For every beam:
  both finite      -> range error (reported as median / p90 absolute error)
  scan finite only -> false obstacle (scan sees something where the world has nothing in range)
  truth finite only -> missed obstacle
Ground truth is time-matched to each scan (nearest /ground_truth/odom message).
"""
import argparse
import math
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from jgb_rover_bringup.drive_tools import DriveHarness
from jgb_rover_bringup.world_geometry import load_world, raycast


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--world', default='apartment')
    ap.add_argument('--duration', type=float, default=15.0, help='s of sim time to record')
    ap.add_argument('--spin-rate', type=float, default=0.4, help='rad/s, 0 = stand still')
    ap.add_argument('--range-max', type=float, default=2.5)
    args = ap.parse_args()

    world = os.path.join(get_package_share_directory('jgb_rover_gazebo'), 'worlds', f'{args.world}.sdf')
    boxes, circles = load_world(world)

    rclpy.init()
    h = DriveHarness({'gt': '/ground_truth/odom'}, name='check_visual_scan')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, h)
    scans = []
    h.create_subscription(LaserScan, '/visual_scan', scans.append, 20)
    h.wait_ready()
    while rclpy.ok() and not tf_buffer.can_transform('base_footprint', 'visual_scan_link', Time()):
        rclpy.spin_once(h, timeout_sec=0.1)
    tf = tf_buffer.lookup_transform('base_footprint', 'visual_scan_link', Time()).transform.translation
    h.spin_for(1.0)
    scans.clear()
    h.spin_for(args.duration, 0.0, args.spin_rate)
    h.spin_for(0.5)

    gt_t, gt_x, gt_y, gt_yaw = h.tracks['gt'].arrays()
    errs, n_both, n_false, n_miss, n_free_ok, n_nan = [], 0, 0, 0, 0, 0
    for s in scans:
        t = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
        i = int(np.argmin(np.abs(gt_t - t)))
        yaw = gt_yaw[i]
        ox = gt_x[i] + math.cos(yaw) * tf.x - math.sin(yaw) * tf.y
        oy = gt_y[i] + math.sin(yaw) * tf.x + math.cos(yaw) * tf.y
        ang = s.angle_min + s.angle_increment * np.arange(len(s.ranges))
        truth = raycast(boxes, circles, ox, oy, ang + yaw, args.range_max)
        r = np.asarray(s.ranges, dtype=float)
        r[r > args.range_max] = np.inf               # "no obstacle" encodings (inf or range_max+)
        nan = np.isnan(r) | (r < s.range_min)        # unobserved beams
        n_nan += nan.sum()
        fin_s, fin_t = np.isfinite(r), np.isfinite(truth)
        both = fin_s & fin_t
        errs.extend(np.abs(r[both] - truth[both]))
        n_both += both.sum()
        n_false += (fin_s & ~fin_t & ~nan).sum()
        n_miss += (~fin_s & fin_t & ~nan).sum()
        n_free_ok += (~fin_s & ~fin_t & ~nan).sum()
    h.destroy_node()
    rclpy.shutdown()

    errs = np.asarray(errs)
    total = n_both + n_false + n_miss + n_free_ok
    print(f'scans: {len(scans)} ({len(scans) / args.duration:.1f} Hz), beams evaluated: {total}, '
          f'unobserved (nan): {n_nan}')
    if len(errs):
        print(f'range error where both see an obstacle ({n_both} beams): median {100 * np.median(errs):.1f} cm, '
              f'p90 {100 * np.percentile(errs, 90):.1f} cm, within 5 cm: {100 * np.mean(errs < 0.05):.1f} %, '
              f'within 10 cm: {100 * np.mean(errs < 0.10):.1f} %')
    print(f'free space correctly reported: {n_free_ok} beams')
    print(f'false obstacles: {n_false} beams ({100 * n_false / max(total, 1):.1f} %)')
    print(f'missed obstacles: {n_miss} beams ({100 * n_miss / max(total, 1):.1f} %)')


if __name__ == '__main__':
    main()
