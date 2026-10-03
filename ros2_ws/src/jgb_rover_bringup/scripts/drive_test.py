#!/usr/bin/env python3
"""Phase 2 acceptance: straight-line and spin-in-place accuracy of the wheel odometry.

1. Drive --distance m straight at --speed m/s (closed loop on ground truth), stop, compare the
   distance travelled according to /wheel/odom with the ground truth.
2. Spin 360 deg at --spin-rate rad/s, compare the yaw change of /wheel/odom with ground truth.
"""
import argparse
import math

import rclpy

from jgb_rover_bringup.drive_tools import DriveHarness


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--distance', type=float, default=1.0)
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--spin-rate', type=float, default=1.0)
    ap.add_argument('--tolerance', type=float, default=0.03, help='max relative distance error')
    args = ap.parse_args()

    rclpy.init()
    h = DriveHarness({'gt': '/ground_truth/odom', 'wheel': '/wheel/odom'}, name='drive_test')
    h.wait_ready()
    h.spin_for(0.5)

    snap = lambda: {k: h.tracks[k].last for k in h.tracks}     # noqa: E731
    a = snap()
    h.straight(args.distance, args.speed, ref='gt')
    b = snap()
    d = {k: math.hypot(b[k][1] - a[k][1], b[k][2] - a[k][2]) for k in a}
    dyaw = {k: b[k][3] - a[k][3] for k in a}
    rel = abs(d['wheel'] - d['gt']) / d['gt']
    print(f'STRAIGHT {args.distance:.2f} m @ {args.speed:.2f} m/s')
    print(f'  ground truth : {d["gt"]:.4f} m, heading change {math.degrees(dyaw["gt"]):+.2f} deg')
    print(f'  wheel odom   : {d["wheel"]:.4f} m, heading change {math.degrees(dyaw["wheel"]):+.2f} deg')
    print(f'  distance error {100 * rel:.2f} % (limit {100 * args.tolerance:.0f} %) -> '
          f'{"PASS" if rel <= args.tolerance else "FAIL"}')

    a = snap()
    h.turn(2 * math.pi, args.spin_rate, ref='gt')
    b = snap()
    dyaw = {k: b[k][3] - a[k][3] for k in a}
    drift = {k: math.hypot(b[k][1] - a[k][1], b[k][2] - a[k][2]) for k in a}
    print(f'SPIN 360 deg @ {args.spin_rate:.2f} rad/s')
    print(f'  ground truth : {math.degrees(dyaw["gt"]):.2f} deg, position drift {1e3 * drift["gt"]:.1f} mm')
    print(f'  wheel odom   : {math.degrees(dyaw["wheel"]):.2f} deg')
    print(f'  wheel-odometry yaw error {math.degrees(dyaw["wheel"] - dyaw["gt"]):+.2f} deg '
          f'({100 * (dyaw["wheel"] - dyaw["gt"]) / dyaw["gt"]:+.2f} %)')
    h.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
