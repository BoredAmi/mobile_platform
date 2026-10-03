"""Shared helpers for the evaluation scripts: record odometry tracks and drive simple motions.

All timing uses sim time. Motions are closed loop on one chosen odometry source
(normally the ground truth, so the robot really drives the requested shape and the other
sources are judged against it).
"""
import math
import os
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


def load_spec() -> dict:
    path = os.path.join(get_package_share_directory('jgb_rover_description'), 'config', 'robot_spec.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Track:
    """Odometry samples with an unwrapped (continuous) yaw."""
    t: list = field(default_factory=list)
    x: list = field(default_factory=list)
    y: list = field(default_factory=list)
    yaw: list = field(default_factory=list)       # unwrapped

    def add(self, t, x, y, yaw):
        if self.yaw:
            yaw = self.yaw[-1] + wrap(yaw - self.yaw[-1])
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.yaw.append(yaw)

    @property
    def last(self):
        return self.t[-1], self.x[-1], self.y[-1], self.yaw[-1]

    def arrays(self):
        return tuple(np.asarray(v) for v in (self.t, self.x, self.y, self.yaw))


class DriveHarness(Node):
    def __init__(self, sources: dict, name='drive_harness'):
        """sources: {label: odometry topic}. 'gt' should be the ground truth."""
        super().__init__(name, parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        spec = load_spec()
        lim = spec['drive']['suggested_limits']
        self.lin_acc = lim['linear_accel']
        self.ang_acc = lim['angular_accel']
        self.tracks = {k: Track() for k in sources}
        self.recording = True
        for label, topic in sources.items():
            self.create_subscription(Odometry, topic, lambda m, k=label: self._odom_cb(k, m), 50)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    # ------------------------------------------------------------------ io
    def _odom_cb(self, label, msg):
        if not self.recording:
            return
        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tracks[label].add(t, p.position.x, p.position.y, yaw_of(p.orientation))

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, v: float, w: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def wait_ready(self, labels=None, timeout=60.0):
        labels = labels or list(self.tracks)
        deadline = time.monotonic() + timeout          # wall clock: sim time may not run yet
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.tracks[k].t for k in labels):
                return
            if time.monotonic() > deadline:
                missing = [k for k in labels if not self.tracks[k].t]
                raise TimeoutError(f'no odometry from {missing}')

    def spin_for(self, seconds: float, v: float = 0.0, w: float = 0.0, rate: float = 50.0):
        """Keep sending (v, w) for `seconds` of sim time."""
        t_end = self.now_s() + seconds
        next_send = 0.0
        while rclpy.ok() and self.now_s() < t_end:
            if self.now_s() >= next_send:
                self.send(v, w)
                next_send = self.now_s() + 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.005)

    # ------------------------------------------------------------------ motions
    def straight(self, distance: float, speed: float, ref: str = 'gt', rate: float = 50.0):
        """Drive `distance` m (sign = direction) measured by source `ref`, then stop."""
        _, x0, y0, _ = self.tracks[ref].last
        brake = speed ** 2 / (2 * self.lin_acc)
        v = math.copysign(abs(speed), distance)
        next_send = 0.0
        while rclpy.ok():
            _, x, y, _ = self.tracks[ref].last
            if math.hypot(x - x0, y - y0) >= abs(distance) - brake:
                break
            if self.now_s() >= next_send:
                self.send(v, 0.0)
                next_send = self.now_s() + 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.005)
        self.spin_for(abs(speed) / self.lin_acc + 0.3)      # brake + settle

    def turn(self, angle: float, rate_rad: float, ref: str = 'gt', rate: float = 50.0):
        """Turn in place by `angle` rad (sign = direction) measured by source `ref`, then stop."""
        yaw0 = self.tracks[ref].last[3]
        brake = rate_rad ** 2 / (2 * self.ang_acc)
        w = math.copysign(abs(rate_rad), angle)
        next_send = 0.0
        while rclpy.ok():
            if abs(self.tracks[ref].last[3] - yaw0) >= abs(angle) - brake:
                break
            if self.now_s() >= next_send:
                self.send(0.0, w)
                next_send = self.now_s() + 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.005)
        self.spin_for(abs(rate_rad) / self.ang_acc + 0.3)

    def arc(self, angle: float, radius: float, speed: float, ref: str = 'gt', rate: float = 50.0):
        """Drive an arc of `radius` m through `angle` rad of heading (sign = left/right)."""
        yaw0 = self.tracks[ref].last[3]
        w = math.copysign(abs(speed) / radius, angle)
        brake = w ** 2 / (2 * self.ang_acc)
        next_send = 0.0
        while rclpy.ok():
            if abs(self.tracks[ref].last[3] - yaw0) >= abs(angle) - brake:
                break
            if self.now_s() >= next_send:
                self.send(abs(speed), w)
                next_send = self.now_s() + 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.005)
        self.spin_for(max(abs(speed) / self.lin_acc, abs(w) / self.ang_acc) + 0.3)


def start_relative(track: Track, i0: int = 0):
    """Express a track in its own pose at sample i0 (so different odometry frames compare)."""
    t, x, y, yaw = track.arrays()
    c, s = math.cos(yaw[i0]), math.sin(yaw[i0])
    dx, dy = x - x[i0], y - y[i0]
    return t, c * dx + s * dy, -s * dx + c * dy, yaw - yaw[i0]
