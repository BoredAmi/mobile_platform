#!/usr/bin/env python3
"""Drive a mapping tour through both rooms of the apartment world.

Goes to each waypoint (rotate towards it, then drive with heading correction) and, where marked,
spins 360 deg in place so the narrow camera scan sees the whole surroundings. The tour driver
navigates on the ground truth (it is a test driver, not part of the robot); SLAM only gets the
camera scan and the EKF odometry. Ends where it started, so slam_toolbox can close the loop.
"""
import argparse
import math

import rclpy

from jgb_rover_bringup.drive_tools import DriveHarness, wrap

# (x, y, spin) in the world frame; robot spawns at the origin facing +x
TOUR = [(0.0, 0.0, True), (-0.5, -1.4, True), (-2.3, -0.8, True), (-2.4, 0.3, True),
        (-0.5, 0.4, False), (0.5, 1.8, True), (2.2, 0.6, True), (2.6, 0.0, False),
        (4.1, 0.0, True), (2.6, 0.0, False), (2.4, -0.6, False), (0.8, -0.9, True), (0.0, 0.0, True)]
# back through the doorway centre: a straight line from (4.1, 0) to (2.4, -0.6) clips the door edge


def goto(h: DriveHarness, x, y, speed, turn_rate, tol=0.05, k_heading=2.0, rate=50.0, time_factor=3.0):
    """Drive to (x, y). Returns False if not reached within time_factor x the nominal time (+ 20 s)."""
    _, x0, y0, yaw = h.tracks['gt'].last
    deadline = h.now_s() + time_factor * math.hypot(x - x0, y - y0) / speed + 20.0
    bearing = math.atan2(y - y0, x - x0)
    if math.hypot(x - x0, y - y0) < tol:
        return True
    err = wrap(bearing - yaw)
    if abs(err) > 0.15:
        h.turn(err, turn_rate)
    next_send = 0.0
    while rclpy.ok():
        _, cx, cy, cyaw = h.tracks['gt'].last
        dist = math.hypot(x - cx, y - cy)
        if dist < tol:
            break
        if h.now_s() > deadline:
            h.spin_for(0.4)
            return False
        err = wrap(math.atan2(y - cy, x - cx) - cyaw)
        v = min(speed, 1.5 * dist) * max(0.0, math.cos(err))
        if h.now_s() >= next_send:
            h.send(v, max(-turn_rate, min(turn_rate, k_heading * err)))
            next_send = h.now_s() + 1.0 / rate
        rclpy.spin_once(h, timeout_sec=0.005)
    h.spin_for(0.4)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--turn-rate', type=float, default=0.6)
    ap.add_argument('--spin-rate', type=float, default=0.4)
    args = ap.parse_args()
    rclpy.init()
    h = DriveHarness({'gt': '/ground_truth/odom'}, name='waypoint_tour')
    h.wait_ready()
    h.spin_for(4.0)                         # IMU bias calibration needs the robot still
    t0 = h.now_s()
    for i, (x, y, spin) in enumerate(TOUR):
        if not goto(h, x, y, args.speed, args.turn_rate):
            print(f'waypoint {i + 1} ({x:+.1f}, {y:+.1f}) NOT reached (blocked?), skipping', flush=True)
            continue
        if spin:
            h.turn(2 * math.pi, args.spin_rate)
        _, cx, cy, _ = h.tracks['gt'].last
        print(f'waypoint {i + 1}/{len(TOUR)} ({x:+.1f}, {y:+.1f}) reached at ({cx:+.2f}, {cy:+.2f}), '
              f't = {h.now_s() - t0:.0f} s', flush=True)
    h.spin_for(1.0)
    h.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
