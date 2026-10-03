"""Startup gyro-bias calibration for the MPU6050 (same node in simulation and on the real robot).

While the robot is commanded to stand still, average the gyro for `calibration_duration`
seconds, then republish every IMU message with the bias removed. Afterwards, every time the
robot stands still again (zero command and wheels not turning for `zupt_settle_time`), the
bias estimate keeps being refined (zero-velocity update), which also follows slow thermal drift. The orientation is marked
unknown (the MPU6050 has no magnetometer), and covariances are filled from parameters.

Subscribes: imu_in (sensor_msgs/Imu, default /imu/data_raw), cmd_vel (TwistStamped or Twist),
            wheel_odom (nav_msgs/Odometry, optional stillness check)
Publishes:  imu_out (sensor_msgs/Imu, default /imu/data)
Services:   ~/recalibrate (std_srvs/Trigger) - restart the calibration (robot must stand still)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class ImuBiasCalibration(Node):
    def __init__(self):
        super().__init__('imu_bias_calibration')
        p = self.declare_parameter
        self.imu_in = p('imu_in', '/imu/data_raw').value
        self.imu_out = p('imu_out', '/imu/data').value
        self.cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value
        self.cmd_vel_stamped = p('cmd_vel_stamped', True).value
        self.wheel_odom_topic = p('wheel_odom_topic', '/wheel/odom').value     # '' = disabled
        self.duration = p('calibration_duration', 3.0).value                 # s
        self.min_samples = p('min_samples', 100).value
        self.cmd_timeout = p('cmd_vel_timeout', 0.5).value                   # s, older cmd = zero
        self.cmd_eps = p('cmd_zero_threshold', 1e-3).value                   # m/s and rad/s
        self.odom_eps = p('wheel_still_threshold', 0.005).value              # m/s and rad/s
        self.max_gyro_std = p('max_gyro_stddev', 0.02).value                 # rad/s, reject if shaken
        self.publish_uncalibrated = p('publish_before_calibrated', False).value
        self.gyro_stddev = p('gyro_stddev', 0.0009).value                    # rad/s
        self.accel_stddev = p('accel_stddev', 0.03).value                    # m/s^2
        self.frame_override = p('frame_id', '').value                        # '' = keep input frame
        self.zupt_enabled = p('zupt_enabled', True).value
        self.zupt_settle = p('zupt_settle_time', 0.3).value                  # s still before using samples
        self.zupt_max_samples = p('zupt_max_samples', 6000).value            # mean memory (6000 = 60 s @100 Hz)
        self.zupt_max_rate = p('zupt_max_rate', 0.05).value                  # rad/s, larger = being moved

        self.bias = np.zeros(3)
        self.calibrated = False
        self.samples = []
        self.window_start = None
        self.cmd_nonzero = False              # value of the latest command
        self.last_cmd_time = None             # ROS time [s] of the latest command
        self.still_since = self.now_s()       # ROS time [s] since the wheels are not turning
        self.n_bias = 0                       # samples in the running bias mean
        self.n_zupt = 0

        self.pub = self.create_publisher(Imu, self.imu_out, 20)   # reliable: works with any subscriber
        self.create_subscription(Imu, self.imu_in, self.imu_cb, qos_profile_sensor_data)
        if self.cmd_vel_stamped:
            self.create_subscription(TwistStamped, self.cmd_vel_topic, lambda m: self.cmd_cb(m.twist), 10)
        else:
            self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_cb, 10)
        if self.wheel_odom_topic:
            self.create_subscription(Odometry, self.wheel_odom_topic, self.odom_cb, 10)
        self.create_service(Trigger, '~/recalibrate', self.recalibrate_cb)
        self.get_logger().info(
            f'calibrating gyro bias: hold still for {self.duration:.1f} s ({self.imu_in} -> {self.imu_out})')

    # ------------------------------------------------------------------ inputs
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd_cb(self, twist: Twist):
        self.cmd_nonzero = (abs(twist.linear.x) > self.cmd_eps or abs(twist.linear.y) > self.cmd_eps
                            or abs(twist.angular.z) > self.cmd_eps)
        self.last_cmd_time = self.now_s()
        if self.cmd_nonzero and not self.wheel_odom_topic:
            self.still_since = None

    def odom_cb(self, msg: Odometry):
        t = msg.twist.twist
        if abs(t.linear.x) > self.odom_eps or abs(t.angular.z) > self.odom_eps:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = self.now_s()

    def recalibrate_cb(self, _req, resp):
        self.calibrated = False
        self.reset_window()
        resp.success = True
        resp.message = f'recalibrating, hold still for {self.duration:.1f} s'
        self.get_logger().info(resp.message)
        return resp

    def robot_should_be_still(self, settle: float = 0.0) -> bool:
        """Commanded to stand still and (if wheel odometry is available) not moving for `settle` s."""
        now = self.now_s()
        cmd_active = (self.cmd_nonzero and self.last_cmd_time is not None
                      and now - self.last_cmd_time < self.cmd_timeout)
        if cmd_active:
            return False
        if self.still_since is None:
            if self.wheel_odom_topic:
                return False
            self.still_since = now                  # no odometry: command stopped -> start settling
        return now - self.still_since >= settle

    def reset_window(self):
        self.samples = []
        self.window_start = None

    # ------------------------------------------------------------------ main
    def imu_cb(self, msg: Imu):
        w = msg.angular_velocity
        if not self.calibrated:
            self.collect(stamp_s(msg), (w.x, w.y, w.z))
            if not self.calibrated and not self.publish_uncalibrated:
                return
        elif self.zupt_enabled:
            self.zero_velocity_update(np.array((w.x, w.y, w.z)))
        out = Imu()
        out.header = msg.header
        if self.frame_override:
            out.header.frame_id = self.frame_override
        out.orientation.w = 1.0
        out.orientation_covariance[0] = -1.0           # orientation unknown (no magnetometer)
        out.angular_velocity.x = w.x - self.bias[0]
        out.angular_velocity.y = w.y - self.bias[1]
        out.angular_velocity.z = w.z - self.bias[2]
        out.linear_acceleration = msg.linear_acceleration
        g2, a2 = self.gyro_stddev ** 2, self.accel_stddev ** 2
        out.angular_velocity_covariance = [g2, 0.0, 0.0, 0.0, g2, 0.0, 0.0, 0.0, g2]
        out.linear_acceleration_covariance = [a2, 0.0, 0.0, 0.0, a2, 0.0, 0.0, 0.0, a2]
        self.pub.publish(out)

    def zero_velocity_update(self, gyro: np.ndarray):
        """Refine the bias with a capped running mean while the robot stands still."""
        if not self.robot_should_be_still(self.zupt_settle):
            return
        if np.any(np.abs(gyro - self.bias) > self.zupt_max_rate):
            return                                      # picked up / bumped: not a zero-rate sample
        self.n_bias = min(self.n_bias + 1, self.zupt_max_samples)
        self.bias += (gyro - self.bias) / self.n_bias
        self.n_zupt += 1
        if self.n_zupt % 1000 == 0:
            self.get_logger().info(
                f'zero-velocity update: {self.n_zupt} samples, bias z={self.bias[2]:+.5f} rad/s')

    def collect(self, t: float, gyro):
        if not self.robot_should_be_still():
            if self.samples:
                self.get_logger().info('robot moving: calibration window restarted')
            self.reset_window()
            return
        if self.window_start is None:
            self.window_start = t
        self.samples.append(gyro)
        if t - self.window_start < self.duration or len(self.samples) < self.min_samples:
            return
        data = np.asarray(self.samples)
        std = data.std(axis=0)
        if np.any(std > self.max_gyro_std):
            self.get_logger().warn(f'gyro too noisy for calibration (std {std}), retrying')
            self.reset_window()
            return
        self.bias = data.mean(axis=0)
        self.n_bias = len(data)
        self.calibrated = True
        self.reset_window()
        self.get_logger().info(
            f'gyro bias [rad/s] x={self.bias[0]:+.5f} y={self.bias[1]:+.5f} z={self.bias[2]:+.5f} '
            f'(noise std z={std[2]:.5f}, {len(data)} samples, '
            f'{math.degrees(self.bias[2]) * 60:.2f} deg/min yaw drift removed)')


def main():
    rclpy.init()
    node = ImuBiasCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
