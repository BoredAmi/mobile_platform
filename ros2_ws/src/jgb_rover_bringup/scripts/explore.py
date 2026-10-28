#!/usr/bin/env python3
"""Wander around on the visual floor scan so slam_toolbox can map the room (real robot or sim).

Reactive, no planner: spin 360 deg in place (the camera sees only ~66 deg), then drive forward while
a corridor as wide as the robot is free on /visual_scan, steering towards the more open side; when it
is blocked, turn in place towards the more open side until it is free again. Spins again every
--spin-every seconds of driving.

The camera does not see the floor closer than ~0.37 m in front of the lens and nothing at the sides,
so obstacle points are remembered in odom for --memory seconds and checked against the robot outline
(robot_spec.yaml footprint + --margin): no steering while something is beside the body, and before
turning in place it backs up along its own track until the corners' turning circle is clear.
Stay next to the robot anyway. Ctrl-C stops it at any moment; it also stops when the scan goes stale.
Run with bringup_real.launch.py slow:=true (or bringup_sim.launch.py and --sim).
"""
import argparse
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan

from jgb_rover_bringup.drive_tools import DriveHarness, MotionTimeout, load_spec


class Explorer:
    def __init__(self, h: DriveHarness, args):
        self.h, self.a = h, args
        spec = load_spec()
        fp = spec['footprint']
        self.half_width = fp['overall_width_incl_wheels'] / 2 + args.margin
        self.rear = -fp['length_x'] / 2 - args.margin             # base_footprint is under the axle, mid-body
        self.front = fp['length_x'] / 2 + args.margin
        self.scan_x = spec['camera']['origin_in_base_footprint'][0]  # scan origin: ground under the lens
        # corners sweep this circle when turning in place
        self.turn_radius = math.hypot(self.half_width, max(self.front, -self.rear))
        self.scan = None
        self.scan_time = 0.0
        self.memory = deque()                                      # (time, xs, ys) obstacle points in odom
        h.create_subscription(LaserScan, '/visual_scan', self.scan_cb, 10)

    def scan_cb(self, msg):
        self.scan, self.scan_time = msg, time.monotonic()
        if not self.h.tracks['ekf'].t:
            return
        r = np.asarray(msg.ranges)
        ang = msg.angle_min + msg.angle_increment * np.arange(len(r))
        hit = (r > msg.range_min) & (r <= msg.range_max)           # free / unobserved beams dropped
        x = self.scan_x + r[hit] * np.cos(ang[hit])
        y = r[hit] * np.sin(ang[hit])
        _, px, py, yaw = self.h.tracks['ekf'].last
        c, s = math.cos(yaw), math.sin(yaw)
        self.memory.append((self.scan_time, px + c * x - s * y, py + s * x + c * y))
        while self.memory and self.scan_time - self.memory[0][0] > self.a.memory:
            self.memory.popleft()

    def fresh(self) -> bool:
        return self.scan is not None and time.monotonic() - self.scan_time < self.a.stale_after

    def points(self):
        """Remembered obstacle points (x forward, y left) in base_footprint, so obstacles that left the
        camera view (beside the wheels, after a turn) still count."""
        if not self.memory:
            return np.empty(0), np.empty(0)
        ox = np.concatenate([m[1] for m in self.memory])
        oy = np.concatenate([m[2] for m in self.memory])
        _, px, py, yaw = self.h.tracks['ekf'].last
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = ox - px, oy - py
        return c * dx + s * dy, -s * dx + c * dy

    def blocked(self) -> bool:
        """Something in the corridor ahead (clearance in front of the lens) or beside the body."""
        x, y = self.points()
        return bool(np.any((np.abs(y) < self.half_width) & (x > self.rear) & (x < self.scan_x + self.a.clearance)))

    def can_turn(self) -> bool:
        x, y = self.points()
        return not np.any(np.hypot(x, y) < self.turn_radius)

    def side_close(self) -> bool:
        """Something next to the body: drive straight, a steering correction would swing into it."""
        x, y = self.points()
        return bool(np.any((np.abs(y) < self.half_width + self.a.margin) & (x > self.rear) & (x < self.front)))

    def open_side(self) -> float:
        """> 0: more room on the left. Mean range per half, a beam with no obstacle counts as range_max."""
        s = self.scan
        r = np.asarray(s.ranges)
        r = np.where(r > s.range_max, s.range_max, r)
        r = np.where(r <= s.range_min, np.nan, r)         # unobserved
        ang = s.angle_min + s.angle_increment * np.arange(len(r))
        with np.errstate(all='ignore'):
            diff = np.nanmean(r[ang > 0]) - np.nanmean(r[ang < 0])
        return float(np.nan_to_num(diff))

    def make_room(self) -> bool:
        """Back up along the path it came (driven through, so free) until it can turn; False if it can't."""
        h, a = self.h, self.a
        if self.can_turn():
            return True
        print('  too close to turn: backing up')
        _, x0, y0, _ = h.tracks['ekf'].last
        while rclpy.ok() and not self.can_turn():
            _, x, y, _ = h.tracks['ekf'].last
            if math.hypot(x - x0, y - y0) > a.max_backup:
                h.spin_for(0.6)
                return False
            h.spin_for(0.1, -a.backup_speed, 0.0)
        h.spin_for(0.6)                                    # brake
        return True

    def spin(self, angle: float):
        if not self.make_room():
            print('  no room for the look-around, skipped')
            return
        print(f'  look around ({math.degrees(angle):.0f} deg)')
        self.h.turn(angle, self.a.spin_rate, ref='ekf', timeout=abs(angle) / self.a.spin_rate * 3 + 10)

    def run(self):
        h, a = self.h, self.a
        t_end = time.monotonic() + a.duration
        self.spin(2 * math.pi)
        last_spin = time.monotonic()
        turn_dir, turn_start = 0.0, 0.0
        max_turn_time = 2 * math.pi / a.turn_rate + 5.0     # a full turn without finding a way out
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(h, timeout_sec=0.0)
            if not self.fresh():
                h.send(0.0, 0.0)
                print('  no fresh /visual_scan: waiting')
                h.spin_for(1.0)
                continue
            if self.blocked():
                if turn_dir == 0.0:
                    h.spin_for(0.6)                         # brake
                    if not self.make_room():
                        print('stuck: no room to turn and backing up did not help, stopped')
                        return
                    turn_dir, turn_start = (1.0 if self.open_side() >= 0 else -1.0), time.monotonic()
                    print(f'  blocked: turning {"left" if turn_dir > 0 else "right"}')
                elif time.monotonic() - turn_start > max_turn_time:
                    h.spin_for(1.0)
                    print('no free direction found after a full turn, stopped')
                    return
                elif not self.can_turn():
                    h.spin_for(0.6)
                    if not self.make_room():
                        print('stuck: no room to turn and backing up did not help, stopped')
                        return
                h.spin_for(0.1, 0.0, turn_dir * a.turn_rate)
                continue
            turn_dir = 0.0
            if a.spin_every > 0 and time.monotonic() - last_spin > a.spin_every:
                h.spin_for(0.6)                             # brake
                self.spin(2 * math.pi)
                last_spin = time.monotonic()
                continue
            w = 0.0 if self.side_close() else float(np.clip(a.steer_gain * self.open_side(), -a.turn_rate, a.turn_rate))
            h.spin_for(0.1, a.speed, w)
        h.spin_for(1.0)
        print('time is up, stopped')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=180.0, help='[s] then stop')
    ap.add_argument('--speed', type=float, default=0.12, help='[m/s]')
    ap.add_argument('--turn-rate', type=float, default=0.4, help='[rad/s] turning away from obstacles')
    ap.add_argument('--spin-rate', type=float, default=0.4, help='[rad/s] 360 deg look-around')
    ap.add_argument('--spin-every', type=float, default=25.0, help='[s] of driving between look-arounds, 0 = never')
    ap.add_argument('--clearance', type=float, default=0.55, help='[m] free corridor needed in front of the lens')
    ap.add_argument('--margin', type=float, default=0.10, help='[m] added around the robot outline')
    ap.add_argument('--memory', type=float, default=6.0, help='[s] obstacles stay remembered after leaving the view')
    ap.add_argument('--backup-speed', type=float, default=0.06, help='[m/s] backing up to make room for a turn')
    ap.add_argument('--max-backup', type=float, default=0.25, help='[m] at most, then stop')
    ap.add_argument('--steer-gain', type=float, default=0.3, help='[rad/s per m] of left-right range difference')
    ap.add_argument('--stale-after', type=float, default=0.5, help='[s] without /visual_scan = stop')
    ap.add_argument('--sim', action='store_true', help='simulation: sim time')
    ap.add_argument('--yes', action='store_true', help='do not ask before driving')
    args = ap.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl-C -> KeyboardInterrupt, node stays usable
    h = DriveHarness({'ekf': '/odometry/filtered'}, name='explore', use_sim_time=args.sim)
    ex = Explorer(h, args)
    try:
        print('waiting for odometry and /visual_scan (is the bringup running?) ...')
        h.wait_ready(timeout=30.0)
        got_imu = []
        sub = h.create_subscription(Imu, '/imu/data', lambda m: got_imu.append(m), 10)
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and not (got_imu and ex.fresh()) and time.monotonic() < deadline:
            rclpy.spin_once(h, timeout_sec=0.1)
        h.destroy_subscription(sub)
        if not ex.fresh():
            print('no /visual_scan: is the camera / perception running?')
            return
        if not args.yes:
            input(f'\nthe robot will explore on its own for {args.duration:.0f} s at {args.speed:.2f} m/s, '
                  f'starting with a 360 deg turn.\nStay next to it. Press Enter to start, Ctrl-C to stop ... ')
        ex.run()
    except KeyboardInterrupt:
        print('\nstopped')
    except MotionTimeout as e:
        print(f'stopped: {e} (wheel stuck?)')
    finally:
        for _ in range(10):
            h.send(0.0, 0.0)
            time.sleep(0.02)
        h.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
