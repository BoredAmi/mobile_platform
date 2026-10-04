"""sim_mcp9808: simulated MCP9808, a drop-in for mcp9808_driver.

Samples the world's temperature field (layout yaml 'temperature' section, world frame) at the
sensor position from the Gazebo ground truth pose, and models the sensor: first-order lag
(time_constant_s), a constant offset drawn once (bias_stddev_c), noise and quantisation to
resolution_c. Publishes the same sensor_msgs/Temperature as the real driver.
"""
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Temperature

from jgb_rover_control.spec_params import load_spec
from jgb_rover_temperature.field import TemperatureField


class SimMcp9808(Node):
    def __init__(self):
        super().__init__('mcp9808_driver')
        p = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731
        self.topic = p('topic', '/temperature')
        gt_topic = p('ground_truth_topic', '/ground_truth/odom')
        layout = p('layout_file', '')                    # '' = constant default_ambient_c
        seed = int(p('seed', 0))

        spec = load_spec()['temperature_sensor']
        sim = yaml.safe_load(open(os.path.join(get_package_share_directory('jgb_rover_description'),
                                               'config', 'sim_assumptions.yaml')))['temperature_sensor']
        self.frame_id = spec['frame']
        self.offset_xy = spec['origin_in_base_footprint'][:2]
        self.resolution = spec['resolution_c']
        self.tau = spec['time_constant_s']
        self.variance = spec['accuracy_c'] ** 2
        self.noise = sim['noise_stddev_c']
        self.rng = np.random.default_rng(seed)
        self.bias = float(self.rng.normal(0.0, sim['bias_stddev_c']))
        self.field = (TemperatureField.from_layout(layout, sim['default_ambient_c']) if layout
                      else TemperatureField(None, sim['default_ambient_c']))
        self.state = None                                  # lagged sensor temperature
        self.last_t = None

        self.pub = self.create_publisher(Temperature, self.topic, 10)
        self.create_subscription(Odometry, gt_topic, self.gt_cb, 10)
        self.create_timer(1.0 / spec['rate_hz'], self.tick)
        self.get_logger().info(f'simulated MCP9808: tau {self.tau} s, offset {self.bias:+.2f} C, '
                               f'{len(self.field.sources)} field sources -> {self.topic}')

    def gt_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        ox, oy = self.offset_xy
        x = msg.pose.pose.position.x + math.cos(yaw) * ox - math.sin(yaw) * oy
        y = msg.pose.pose.position.y + math.sin(yaw) * ox + math.cos(yaw) * oy
        air = float(self.field(x, y))
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.state is None:
            self.state = air                               # sensor settled at the start pose
        elif t > self.last_t:
            self.state += (1.0 - math.exp(-(t - self.last_t) / self.tau)) * (air - self.state)
        self.last_t = t

    def tick(self):
        if self.state is None:
            return
        value = self.state + self.bias + self.rng.normal(0.0, self.noise)
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.temperature = round(value / self.resolution) * self.resolution
        msg.variance = self.variance
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimMcp9808()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
