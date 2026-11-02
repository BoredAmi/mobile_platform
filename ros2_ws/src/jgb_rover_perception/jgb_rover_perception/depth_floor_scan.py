"""depth_floor_scan: depth camera (RealSense D435) -> LaserScan on the same topic as visual_floor_scan.

Subscribes: depth_topic (sensor_msgs/Image, 16UC1 mm or 32FC1 m), camera_info_topic, TF
Publishes:  scan_topic (sensor_msgs/LaserScan, frame scan_frame)

Camera pose from tf2 (base_frame <- depth image frame), never hard-coded. See depth_scan_core.py.
"""
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from jgb_rover_perception.depth_scan_core import DepthRays, depth_to_scan
from jgb_rover_perception.floor_scan_core import ScanGeometry
from jgb_rover_perception.visual_floor_scan import quat_to_matrix


class DepthFloorScan(Node):
    def __init__(self):
        super().__init__('depth_floor_scan')
        p = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731
        self.depth_topic = p('depth_topic', '/camera/depth/image_rect_raw')
        self.info_topic = p('camera_info_topic', '/camera/depth/camera_info')
        self.scan_topic = p('scan_topic', '/visual_scan')
        self.base_frame = p('base_frame', 'base_footprint')
        self.scan_frame = p('scan_frame', 'visual_scan_link')
        self.max_rate = p('max_rate', 10.0)                       # Hz, frames above this are dropped
        self.stride = p('pixel_stride', 4)                        # use every n-th pixel in both directions
        self.num_beams = p('num_beams', 90)
        self.range_min = p('range_min', 0.15)
        self.range_max = p('range_max', 2.5)
        self.min_height = p('min_obstacle_height', 0.03)          # m above the floor: below is floor (and noise)
        self.max_height = p('max_obstacle_height', 0.30)          # m: the robot's height, above it can be passed
        self.depth_min = p('depth_min', 0.2)                      # m along the optical axis (D435 min ~0.2 m)
        self.depth_max = p('depth_max', 3.0)
        self.no_obstacle_margin = p('no_obstacle_margin', 0.05)
        self.unobserved_value = p('unobserved_value', 0.0)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.K = None
        self.size = None
        self.setup_key = None
        self.last_proc = 0.0
        self.n_frames = 0
        self.proc_ms = []
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.create_subscription(CameraInfo, self.info_topic, self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)

    def info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] > 0:
            self.K, self.size = K, (msg.width, msg.height)

    def ensure_setup(self, frame_id: str) -> bool:
        if self.K is None:
            self.get_logger().warn('waiting for camera_info', throttle_duration_sec=5.0)
            return False
        try:
            cam = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
            scan = self.tf_buffer.lookup_transform(self.base_frame, self.scan_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'waiting for TF: {e}', throttle_duration_sec=5.0)
            return False
        tc, ts = cam.transform.translation, scan.transform.translation
        qs = scan.transform.rotation
        R = quat_to_matrix(cam.transform.rotation)
        t = np.array([tc.x, tc.y, tc.z])
        t_scan = np.array([ts.x, ts.y, ts.z])
        yaw = float(np.arctan2(2 * (qs.w * qs.z + qs.x * qs.y), 1 - 2 * (qs.y ** 2 + qs.z ** 2)))
        key = (np.round(R, 5).tobytes(), np.round(t, 4).tobytes(), np.round(t_scan, 4).tobytes(),
               round(yaw, 4), self.K.tobytes(), self.size)
        if key == self.setup_key:
            return True
        self.setup_key = key
        self.R, self.t, self.t_scan, self.yaw = R, t, t_scan, yaw
        self.rays = DepthRays.build(self.K, self.size, self.stride, R)
        hfov = 2.0 * np.arctan(self.size[0] / (2.0 * self.K[0, 0]))
        self.geo = ScanGeometry(-hfov / 2, hfov / 2, self.num_beams, self.range_min, self.range_max)
        self.get_logger().info(
            f'depth scan set up: camera at z={t[2]:.3f} m, pitch {np.degrees(np.arcsin(-R[2, 2])):.1f} deg '
            f'down, scan span {np.degrees(hfov):.0f} deg, {self.rays.rows.size} rays')
        return True

    def depth_cb(self, msg: Image):
        now = time.monotonic()
        if now - self.last_proc < 1.0 / self.max_rate:
            return
        self.last_proc = now
        if not self.ensure_setup(msg.header.frame_id):
            return
        t0 = time.perf_counter()
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 1e-3               # RealSense 16UC1 is millimetres
        ranges = depth_to_scan(depth, self.rays, self.t, self.t_scan, self.yaw, self.geo, self.min_height,
                               self.max_height, self.depth_min, self.depth_max)
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = float(self.geo.angle_min)
        scan.angle_max = float(self.geo.angle_max)
        scan.angle_increment = float(self.geo.increment)
        scan.scan_time = float(1.0 / self.max_rate)
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max + 2 * self.no_obstacle_margin)
        ranges[np.isinf(ranges)] = self.range_max + self.no_obstacle_margin   # 'max_plus': free space for slam
        ranges[np.isnan(ranges)] = self.unobserved_value
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)
        self.proc_ms.append(1e3 * (time.perf_counter() - t0))
        self.n_frames += 1

    def report(self):
        if self.proc_ms:
            a = np.asarray(self.proc_ms)
            self.get_logger().info(f'{len(a) / 10.0:.1f} Hz, processing {a.mean():.1f} ms mean / '
                                   f'{np.percentile(a, 95):.1f} ms p95 per frame')
            self.proc_ms = []


def main():
    rclpy.init()
    node = DepthFloorScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
