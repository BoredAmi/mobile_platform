#!/usr/bin/env python3
"""First drive test of the real robot: 1 m out, turn 180 deg, 1 m back, turn 180 deg.

Needs the drive stack only (no camera):
    ros2 launch jgb_rover_bringup bringup_real.launch.py camera:=false
    ros2 run jgb_rover_bringup out_and_back.py

The motions are closed loop on the EKF (/odometry/filtered: wheels + gyro), so the turns follow the
gyro. Afterwards it prints, per leg, what the EKF and the wheel odometry measured, where each thinks
the robot ended relative to the start, and calibration hints:
  - turns: wheel-odometry angle vs gyro angle -> effective wheel separation
  - --straight-only: drive one straight leg, then type the distance you measured with a tape
    -> effective wheel radius
Both go into robot_spec.yaml (drive.wheel_separation / drive.wheel_radius); rebuild afterwards.

Mark the start: a piece of tape at the centre between the two drive wheels and one at the front.
Ctrl-C stops the robot at any time. Each motion also has a timeout.
Also runs in the simulation: add --sim (then the ground truth is reported too).
"""
import argparse
import math
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu

from jgb_rover_bringup.drive_tools import DriveHarness, MotionTimeout, load_spec


def pose_in_start_frame(start, now):
    """(dx forward, dy left, dyaw) of pose `now` in the frame of pose `start`; poses are (t, x, y, yaw)."""
    _, x0, y0, yaw0 = start
    _, x, y, yaw = now
    c, s = math.cos(yaw0), math.sin(yaw0)
    dx, dy = x - x0, y - y0
    return c * dx + s * dy, -s * dx + c * dy, yaw - yaw0


def wait_for_imu(node, timeout):
    """imu_bias_calibration publishes /imu/data only once the gyro bias is calibrated."""
    got = []
    sub = node.create_subscription(Imu, '/imu/data', lambda m: got.append(m), 10)
    end = time.monotonic() + timeout
    while rclpy.ok() and not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return bool(got)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--distance', type=float, default=1.0, help='length of each straight leg [m]')
    ap.add_argument('--speed', type=float, default=0.15, help='[m/s]')
    ap.add_argument('--turn-rate', type=float, default=0.8, help='[rad/s]')
    ap.add_argument('--straight-only', action='store_true',
                    help='one straight leg, then ask for the tape-measured distance (wheel radius check)')
    ap.add_argument('--sim', action='store_true', help='simulation: sim time, ground truth reported too')
    ap.add_argument('--yes', action='store_true', help='do not ask before driving')
    args = ap.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl-C -> KeyboardInterrupt, node stays usable
    sources = {'ekf': '/odometry/filtered', 'wheel': '/wheel/odom'}
    if args.sim:
        sources['gt'] = '/ground_truth/odom'
    h = DriveHarness(sources, name='out_and_back', use_sim_time=args.sim)
    spec = load_spec()
    sep = spec['drive']['wheel_separation']
    radius = spec['drive']['wheel_radius']

    try:
        print('waiting for odometry (is bringup_real.launch.py running?) ...')
        h.wait_ready(timeout=30.0)
        print('waiting for the gyro calibration (keep the robot still) ...')
        if not wait_for_imu(h, 20.0):
            print('WARNING: no /imu/data: the EKF turns on wheel odometry only (IMU missing or the robot was '
                  'moved during calibration)')
        if not args.yes:
            plan = (f'{args.distance:.2f} m straight' if args.straight_only else
                    f'{args.distance:.2f} m out, turn 180 deg, {args.distance:.2f} m back, turn 180 deg')
            input(f'\nthe robot will drive {plan} at {args.speed:.2f} m/s.\n'
                  f'Clear {args.distance + 0.3:.1f} m in front of it. Press Enter to start, Ctrl-C to abort ... ')
        h.spin_for(0.5)

        drive_time = 2.0 * args.distance / args.speed + 5.0
        turn_time = 2.0 * math.pi / args.turn_rate + 5.0
        snap = lambda: {k: h.tracks[k].last for k in h.tracks}  # noqa: E731
        start = snap()

        if args.straight_only:
            h.straight(args.distance, args.speed, ref='ekf', timeout=drive_time)
            end = snap()
            d = {k: math.hypot(end[k][1] - start[k][1], end[k][2] - start[k][2]) for k in end}
            print('\nSTRAIGHT')
            for k in d:
                print(f'  {k:5s}: {d[k]:.3f} m, heading change {math.degrees(end[k][3] - start[k][3]):+.1f} deg')
            text = input('measured distance with a tape [m] (Enter to skip): ').strip()
            if text:
                real = float(text)
                print(f'  wheel odometry is {100 * (d["wheel"] - real) / real:+.1f} % off. Effective wheel radius '
                      f'{radius * real / d["wheel"]:.4f} m (robot_spec.yaml drive.wheel_radius, now {radius})')
            return

        legs = []
        for name, motion in (('out', lambda: h.straight(args.distance, args.speed, ref='ekf', timeout=drive_time)),
                             ('turn 1', lambda: h.turn(math.pi, args.turn_rate, ref='ekf', timeout=turn_time)),
                             ('back', lambda: h.straight(args.distance, args.speed, ref='ekf', timeout=drive_time)),
                             ('turn 2', lambda: h.turn(math.pi, args.turn_rate, ref='ekf', timeout=turn_time))):
            a = snap()
            print(f'{name} ...')
            motion()
            legs.append((name, a, snap()))
        end = snap()

        print('\nLEGS                       ' + ''.join(f'{k:>18s}' for k in h.tracks))
        for name, a, b in legs:
            if name.startswith('turn'):
                cells = [f'{math.degrees(b[k][3] - a[k][3]):+15.1f} deg' for k in h.tracks]
            else:
                cells = [f'{math.hypot(b[k][1] - a[k][1], b[k][2] - a[k][2]):15.3f} m  ' for k in h.tracks]
            print(f'  {name:25s}' + ''.join(cells))

        print('\nEND RELATIVE TO START (forward, left, heading; 0 = back where it started)')
        for k in h.tracks:
            dx, dy, dyaw = pose_in_start_frame(start[k], end[k])
            dyaw = math.degrees(dyaw) - 360.0
            print(f'  {k:5s}: {100 * dx:+6.1f} cm, {100 * dy:+6.1f} cm, {dyaw:+6.1f} deg')
        print('  The EKF steered, so it reports ~0 by design. Compare with your tape marks: the real gap is the '
              'error of the whole chain.')

        turned_ekf = sum(b['ekf'][3] - a['ekf'][3] for n, a, b in legs if n.startswith('turn'))
        turned_wheel = sum(b['wheel'][3] - a['wheel'][3] for n, a, b in legs if n.startswith('turn'))
        ratio = turned_wheel / turned_ekf
        print(f'\nTURNS: wheel odometry {math.degrees(turned_wheel):.1f} deg vs gyro/EKF {math.degrees(turned_ekf):.1f} deg '
              f'({100 * (ratio - 1):+.1f} %)')
        if abs(ratio - 1) > 0.02:
            print(f'  effective wheel separation ~ {sep * ratio:.3f} m (robot_spec.yaml drive.wheel_separation, now '
                  f'{sep}); repeat a few times before changing it')
        else:
            print('  wheel separation OK (within 2 %)')
        print('Wheel radius: run again with --straight-only and a tape measure.')
    except MotionTimeout as e:
        print(f'\nSTOPPED: {e}. Wheel stuck or blocked, odometry stopped, or the robot cannot reach the speed '
              '(check the wheels turn: firmware/tools/rover_cli.py vel 3 3).')
    except TimeoutError as e:
        print(f'\nSTOPPED: {e}')
    except KeyboardInterrupt:
        print('\naborted')
    finally:
        if rclpy.ok():
            h.spin_for(0.5)  # zero velocity
        h.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
