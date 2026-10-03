#!/usr/bin/env python3
"""Phase 3 evaluation: wheel-only odometry vs EKF (wheel + IMU) against ground truth.

Drives a square (--side m) --squares times, then a figure-8 (two circles of --fig8-radius),
closed loop on the ground truth so the robot really drives those shapes. Logs
/ground_truth/odom, /wheel/odom and /odometry/filtered, then prints the position RMSE and the
final yaw error of both estimates and saves a plot.

Needs bringup_sim.launch.py running. Waits for /imu/data (bias calibration finished) first.
"""
import argparse
import json
import math
import os
import time

import matplotlib
import numpy as np
import rclpy
from sensor_msgs.msg import Imu

from jgb_rover_bringup.drive_tools import DriveHarness, start_relative

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

SOURCES = {'gt': '/ground_truth/odom', 'wheel': '/wheel/odom', 'ekf': '/odometry/filtered'}
LABELS = {'gt': 'ground truth', 'wheel': 'wheel odometry', 'ekf': 'EKF (wheel + IMU)'}
COLOURS = {'gt': '#222222', 'wheel': '#d95f02', 'ekf': '#1b9e77'}


def wait_for_imu(node, timeout=60.0):
    got = []
    sub = node.create_subscription(Imu, '/imu/data', lambda m: got.append(1), 10)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            raise TimeoutError('no /imu/data: is the bias calibration node running and the robot still?')
    node.destroy_subscription(sub)


def evaluate(tracks, t0, t1):
    """Align every track at t0 and compare with the ground truth on the gt timestamps in [t0, t1]."""
    out = {}
    aligned = {}
    for k, tr in tracks.items():
        t = np.asarray(tr.t)
        i0 = int(np.searchsorted(t, t0))
        aligned[k] = start_relative(tr, i0)
    tg, xg, yg, yawg = aligned['gt']
    m = (tg >= t0) & (tg <= t1)
    tg, xg, yg, yawg = tg[m], xg[m], yg[m], yawg[m]
    series = {'t': tg - t0}
    for k in ('wheel', 'ekf'):
        t, x, y, yaw = aligned[k]
        xi, yi, yawi = (np.interp(tg, t, v) for v in (x, y, yaw))
        pos_err = np.hypot(xi - xg, yi - yg)
        yaw_err = np.degrees(np.arctan2(np.sin(yawi - yawg), np.cos(yawi - yawg)))
        out[k] = {'pos_rmse_m': float(np.sqrt(np.mean(pos_err ** 2))),
                  'pos_max_m': float(pos_err.max()),
                  'pos_final_m': float(pos_err[-1]),
                  'yaw_final_deg': float(yaw_err[-1]),
                  'yaw_rmse_deg': float(np.sqrt(np.mean(yaw_err ** 2)))}
        series[k] = (xi, yi, pos_err, yaw_err)
    series['gt'] = (xg, yg)
    return out, series


def plot(series, results, title, path):
    fig, ax = plt.subplots(1, 3, figsize=(16, 5.2))
    xg, yg = series['gt']
    ax[0].plot(xg, yg, color=COLOURS['gt'], lw=2.2, label=LABELS['gt'])
    for k in ('wheel', 'ekf'):
        xi, yi, pos_err, yaw_err = series[k]
        ax[0].plot(xi, yi, color=COLOURS[k], lw=1.4, label=LABELS[k])
        ax[1].plot(series['t'], 100 * pos_err, color=COLOURS[k], lw=1.4,
                   label=f'{LABELS[k]} (RMSE {100 * results[k]["pos_rmse_m"]:.1f} cm)')
        ax[2].plot(series['t'], yaw_err, color=COLOURS[k], lw=1.4,
                   label=f'{LABELS[k]} (final {results[k]["yaw_final_deg"]:+.2f} deg)')
    ax[0].set_aspect('equal')
    ax[0].set_xlabel('x [m] (start frame)')
    ax[0].set_ylabel('y [m]')
    ax[0].set_title('Path')
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('position error [cm]')
    ax[1].set_title('Position error')
    ax[2].set_xlabel('time [s]')
    ax[2].set_ylabel('yaw error [deg]')
    ax[2].set_title('Yaw error')
    ax[2].axhline(0, color='#999999', lw=0.8)
    for a in ax:
        a.grid(True, color='#dddddd', lw=0.6)
        a.legend(fontsize=8, frameon=False)
        for s in ('top', 'right'):
            a.spines[s].set_visible(False)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--squares', type=int, default=3)
    ap.add_argument('--side', type=float, default=1.0)
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--turn-rate', type=float, default=0.8)
    ap.add_argument('--fig8-radius', type=float, default=0.5)
    ap.add_argument('--label', default='run', help='name used in the output files and the plot title')
    ap.add_argument('--out-dir', default='.')
    args = ap.parse_args()

    rclpy.init()
    h = DriveHarness(SOURCES, name='eval_odometry')
    print('waiting for odometry and /imu/data (bias calibration) ...', flush=True)
    h.wait_ready()
    wait_for_imu(h)
    h.spin_for(0.5)
    t0 = h.tracks['gt'].last[0]

    for i in range(args.squares):
        for _ in range(4):
            h.straight(args.side, args.speed)
            h.turn(math.pi / 2, args.turn_rate)
        print(f'square {i + 1}/{args.squares} done', flush=True)
    h.arc(2 * math.pi, args.fig8_radius, args.speed)
    h.arc(-2 * math.pi, args.fig8_radius, args.speed)
    print('figure-8 done', flush=True)
    h.spin_for(1.0)
    t1 = h.tracks['gt'].last[0]
    h.recording = False

    results, series = evaluate(h.tracks, t0, t1)
    os.makedirs(args.out_dir, exist_ok=True)
    png = os.path.join(args.out_dir, f'eval_odometry_{args.label}.png')
    plot(series, results, f'{args.label}: {args.squares} x {args.side:.1f} m square + figure-8 '
                          f'({t1 - t0:.0f} s)', png)
    print(f'\n=== {args.label}  ({t1 - t0:.0f} s, {len(series["t"])} samples) ===')
    print(f'{"":18s} {"pos RMSE":>10s} {"pos max":>10s} {"pos final":>10s} {"yaw final":>10s} {"yaw RMSE":>10s}')
    for k in ('wheel', 'ekf'):
        r = results[k]
        print(f'{LABELS[k]:18s} {100 * r["pos_rmse_m"]:8.2f}cm {100 * r["pos_max_m"]:8.2f}cm '
              f'{100 * r["pos_final_m"]:8.2f}cm {r["yaw_final_deg"]:+8.2f}deg {r["yaw_rmse_deg"]:8.2f}deg')
    print(f'plot: {png}')
    with open(os.path.join(args.out_dir, f'eval_odometry_{args.label}.json'), 'w') as f:
        json.dump(results, f, indent=2)
    h.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
