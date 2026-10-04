"""temperature_mapper: build a temperature heatmap in the map frame from a point sensor.

Subscribes: temperature_topic (sensor_msgs/Temperature), map_topic (nav_msgs/OccupancyGrid, optional),
            TF map_frame <- the message frame (temp_sensor_link)
Publishes:  heatmap_topic (nav_msgs/OccupancyGrid, values 1..98 = t_min..t_max, show with the
            RViz Map display, colour scheme 'costmap'), heatmap_topic/legend (visualization_msgs/Marker)
Service:    ~/save (std_srvs/Trigger): writes temperature_samples.csv, temperature_heatmap.png and
            temperature_heatmap.npz to output_dir

Each reading is lag-compensated (heatmap_core.LagCompensator), placed at the sensor position at
the compensated time (TF lookup), then all samples are gridded and smoothed (build_heatmap).
"""
import csv
import os
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Temperature
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from jgb_rover_temperature.heatmap_core import (LagCompensator, OccMap, build_heatmap, colour_range,
                                                mask_with_map, render, to_grid_values)


class TemperatureMapper(Node):
    def __init__(self):
        super().__init__('temperature_mapper')
        p = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731
        self.map_frame = p('map_frame', 'map')
        temp_topic = p('temperature_topic', '/temperature')
        map_topic = p('map_topic', '/map')                       # '' = do not mask with the SLAM map
        self.heatmap_topic = p('heatmap_topic', '/temperature_map')
        self.resolution = p('resolution', 0.10)                  # m per heatmap cell
        self.sigma = p('smoothing_sigma', 0.25)                  # m, interpolation kernel
        self.max_distance = p('max_distance', 0.6)               # m, farther from any sample = unknown
        tau = p('sensor_time_constant_s', 0.0)                   # 0 = no lag compensation
        window = p('lag_window_s', 4.0)
        t_min, t_max = p('t_min', 0.0), p('t_max', 0.0)          # colour range, equal = automatic
        self.t_range = (t_min, t_max) if t_max > t_min else (None, None)
        p('output_dir', '~/.ros/temperature_map')              # read at save time, can be changed live
        period = p('publish_period', 2.0)

        self.lag = LagCompensator(tau, window)
        self.samples = []                                        # (t, x, y, T_air, T_raw)
        self.occ = None
        self.missed_tf = 0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.grid_pub = self.create_publisher(OccupancyGrid, self.heatmap_topic, latched)
        self.legend_pub = self.create_publisher(Marker, self.heatmap_topic + '/legend', latched)
        self.create_subscription(Temperature, temp_topic, self.temp_cb, 20)
        if map_topic:
            self.create_subscription(OccupancyGrid, map_topic, self.map_cb, 1)   # volatile: works with any publisher
        self.create_service(Trigger, '~/save', self.save_cb)
        self.create_timer(period, self.publish)
        self.get_logger().info(f'{temp_topic} -> {self.heatmap_topic} (lag compensation tau {tau} s, '
                               f'window {window} s)')

    # ------------------------------------------------------------------ input
    def map_cb(self, msg):
        q = msg.info.origin.orientation
        if abs(q.z) > 1e-6:
            self.get_logger().warn('map with a rotated origin is not supported, not masking', once=True)
            return
        data = np.asarray(msg.data, np.int8).reshape(msg.info.height, msg.info.width)
        self.occ = OccMap(data, np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
                          msg.info.resolution)

    def temp_cb(self, msg):
        t_raw = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        est = self.lag.add(t_raw, msg.temperature)
        if est is None:
            return
        t_est, air = est
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id,
                                                 Time(nanoseconds=int(t_est * 1e9)))
        except TransformException as e:
            self.missed_tf += 1
            self.get_logger().warn(f'no {self.map_frame} pose for the sample: {e}', throttle_duration_sec=10.0)
            return
        tr = tf.transform.translation
        self.samples.append((t_est, tr.x, tr.y, air, msg.temperature))

    # ------------------------------------------------------------------ output
    def heatmap(self):
        if len(self.samples) < 2:
            return None
        a = np.asarray(self.samples)
        hm = build_heatmap(a[:, 1:3], a[:, 3], self.resolution, self.sigma, self.max_distance)
        return mask_with_map(hm, self.occ) if self.occ is not None else hm

    def publish(self):
        hm = self.heatmap()
        if hm is None:
            return
        lo, hi = colour_range(hm.values, *self.t_range)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = float(hm.resolution)
        msg.info.height, msg.info.width = hm.values.shape
        msg.info.origin.position.x, msg.info.origin.position.y = float(hm.origin[0]), float(hm.origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = to_grid_values(hm.values, lo, hi).ravel().tolist()
        self.grid_pub.publish(msg)

        legend = Marker()
        legend.header = msg.header
        legend.ns, legend.id = 'temperature_legend', 0
        legend.type, legend.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        legend.pose.position.x = float(hm.origin[0])
        legend.pose.position.y = float(hm.origin[1] + hm.values.shape[0] * hm.resolution)
        legend.pose.position.z = 0.5
        legend.pose.orientation.w = 1.0
        legend.scale.z = 0.2
        legend.color.r = legend.color.g = legend.color.b = legend.color.a = 1.0
        legend.text = f'temperature: blue {lo:.1f} C .. red {hi:.1f} C ({len(self.samples)} samples)'
        self.legend_pub.publish(legend)

    def save_cb(self, request, response):
        hm = self.heatmap()
        if hm is None:
            response.success, response.message = False, 'no samples yet'
            return response
        out_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        os.makedirs(out_dir, exist_ok=True)
        a = np.asarray(self.samples)
        with open(os.path.join(out_dir, 'temperature_samples.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'temperature_c', 'raw_reading_c'])
            w.writerows(a.tolist())
        np.savez(os.path.join(out_dir, 'temperature_heatmap.npz'), values=hm.values,
                 origin=hm.origin, resolution=hm.resolution, samples=a, frame=self.map_frame)
        img = render(hm, self.occ, a[:, 1:3], *self.t_range,
                     title=f'temperature heatmap, {len(a)} samples, {time.strftime("%Y-%m-%d %H:%M")}')
        png = os.path.join(out_dir, 'temperature_heatmap.png')
        cv2.imwrite(png, img)
        response.success, response.message = True, f'saved {len(a)} samples and {png}'
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = TemperatureMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
