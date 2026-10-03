#!/usr/bin/env python3
"""Phase 1 acceptance: does the robot settle on all 4 contacts within 1 s, without jitter or tipping?

Logs /ground_truth/odom (base_footprint in world) for --duration seconds of sim time after the
first message, then reports settle time, jitter in the last second and the height of each
contact point (wheel bottoms, caster sphere bottoms) computed from the pose and robot_spec.yaml.
"""
import argparse
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(roll), math.sin(roll), math.cos(pitch),
                                               math.sin(pitch), math.cos(yaw), math.sin(yaw)))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class SpawnCheck(Node):
    def __init__(self, duration):
        super().__init__('check_spawn', parameter_overrides=[rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.duration = duration
        self.samples = []          # t, z, roll, pitch, yaw
        self.t0 = None
        self.done = False
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb, 50)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        r, p, y = quat_to_rpy(msg.pose.pose.orientation)
        pos = msg.pose.pose.position
        self.samples.append((t - self.t0, pos.x, pos.y, pos.z, r, p, y))
        if t - self.t0 >= self.duration:
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=3.0)
    ap.add_argument('--settle-tol-z', type=float, default=0.0005, help='m')
    ap.add_argument('--settle-tol-ang', type=float, default=math.radians(0.2), help='rad')
    args = ap.parse_args()

    spec = yaml.safe_load(open(os.path.join(get_package_share_directory('jgb_rover_description'),
                                            'config', 'robot_spec.yaml')))
    rclpy.init()
    node = SpawnCheck(args.duration)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    s = np.array(node.samples)
    node.destroy_node()
    rclpy.shutdown()

    t, x, y, z, roll, pitch, yaw = s.T
    final = s[-1]
    dev = np.maximum(np.abs(z - final[3]) / args.settle_tol_z,
                     np.maximum(np.abs(roll - final[4]), np.abs(pitch - final[5])) / args.settle_tol_ang)
    unsettled = np.nonzero(dev > 1.0)[0]
    settle_t = 0.0 if len(unsettled) == 0 else t[min(unsettled[-1] + 1, len(t) - 1)]
    last = t >= t[-1] - 1.0

    print(f'samples: {len(t)} over {t[-1]:.2f} s sim time')
    print(f'settle time (|dz|<{args.settle_tol_z * 1e3:.1f} mm, |droll|,|dpitch|<'
          f'{math.degrees(args.settle_tol_ang):.1f} deg): {settle_t:.3f} s')
    print(f'final pose: x={final[1]*1e3:+.2f} mm y={final[2]*1e3:+.2f} mm z={final[3]*1e3:+.2f} mm '
          f'roll={math.degrees(final[4]):+.3f} deg pitch={math.degrees(final[5]):+.3f} deg '
          f'yaw={math.degrees(final[6]):+.3f} deg')
    print(f'jitter in last 1 s (peak-to-peak): z={np.ptp(z[last])*1e6:.1f} um '
          f'roll={math.degrees(np.ptp(roll[last])):.4f} deg pitch={math.degrees(np.ptp(pitch[last])):.4f} deg '
          f'xy drift={math.hypot(np.ptp(x[last]), np.ptp(y[last]))*1e6:.1f} um')
    print(f'max |roll|, |pitch| during whole run: {math.degrees(np.abs(roll).max()):.3f}, '
          f'{math.degrees(np.abs(pitch).max()):.3f} deg')

    # contact points (lowest point of each wheel / caster sphere) in base_footprint
    R = rot(final[4], final[5], final[6])
    p0 = final[1:4]
    cr = spec['casters']['wheel_radius']
    contacts = {}
    for side in ('left', 'right'):
        c = np.array(spec['drive'][f'{side}_wheel_centre'], dtype=float)
        centre_w = p0 + R @ c
        contacts[f'{side}_wheel'] = centre_w[2] - spec['drive']['wheel_radius']
    for name in ('front', 'rear'):
        xy = spec['casters'][f'{name}_swivel_axis_xy']
        centre_w = p0 + R @ np.array([xy[0], xy[1], cr])
        contacts[f'caster_{name}'] = centre_w[2] - cr
    for k, v in contacts.items():
        print(f'  contact {k:13s} lowest point z = {v*1e3:+.2f} mm')
    ok = (settle_t <= 1.0 and all(abs(v) < 0.002 for v in contacts.values())
          and math.degrees(max(abs(final[4]), abs(final[5]))) < 0.5 and np.ptp(z[last]) < 1e-4)
    print('RESULT:', 'PASS' if ok else 'FAIL')


if __name__ == '__main__':
    main()
