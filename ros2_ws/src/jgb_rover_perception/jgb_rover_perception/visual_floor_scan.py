"""visual_floor_scan: turn the monocular camera into a fake laser scan (flat-floor assumption).

Subscribes: image_topic (sensor_msgs/Image), camera_info_topic (sensor_msgs/CameraInfo), TF
Publishes:  scan_topic (sensor_msgs/LaserScan, frame scan_frame), debug_topic (sensor_msgs/Image)

The camera pose comes from tf2 (base_frame <- image frame), never from hard-coded numbers, so a
different camera pitch or a re-mounted camera needs no change here. See floor_scan_core.py for
the algorithm. Processing is skipped (not queued) when frames arrive faster than max_rate.
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

import cv2

from jgb_rover_perception.floor_scan_core import (FloorModel, GroundLUT, ScanGeometry, first_obstacle_rows,
                                                  floor_mask, seed_slice, to_scan)


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class VisualFloorScan(Node):
    def __init__(self):
        super().__init__('visual_floor_scan')
        p = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731
        self.image_topic = p('image_topic', '/camera/image_raw')
        self.info_topic = p('camera_info_topic', '/camera/camera_info')
        self.scan_topic = p('scan_topic', '/visual_scan')
        self.debug_topic = p('debug_topic', '/visual_scan/debug')
        self.base_frame = p('base_frame', 'base_footprint')
        self.scan_frame = p('scan_frame', 'visual_scan_link')
        self.proc_w = p('proc_width', 320)
        self.proc_h = p('proc_height', 240)
        self.max_rate = p('max_rate', 15.0)                       # Hz, frames above this are dropped
        # scan geometry
        self.hfov = p('horizontal_fov', 0.0)                      # rad; 0 = from camera_info
        self.num_beams = p('num_beams', 160)
        self.range_min = p('range_min', 0.15)
        self.range_max = p('range_max', 2.5)
        # How "looked and saw no obstacle" is encoded:
        #   'max_plus': range_max + no_obstacle_margin (msg.range_max = range_max + 2 margins), which
        #               slam_toolbox/Karto ray-traces as free space up to its max_laser_range
        #   'inf':      +inf (REP-117), for consumers that handle it
        self.no_obstacle_mode = p('no_obstacle_mode', 'max_plus')
        self.no_obstacle_margin = p('no_obstacle_margin', 0.05)
        # beams no image column covered; 0.0 is below range_min, so consumers skip them
        self.unobserved_value = p('unobserved_value', 0.0)
        # floor model
        self.seed_w = p('seed_width_fraction', 0.25)
        self.seed_h = p('seed_height_fraction', 0.08)
        self.model = FloorModel(threshold=p('mahalanobis_threshold', 4.0),
                                learning_rate=p('learning_rate', 0.02),
                                luminance_weight=p('luminance_weight', 1.0),
                                min_stddev_l=p('min_stddev_l', 8.0),
                                min_stddev_ab=p('min_stddev_ab', 3.0),
                                seed_inlier_fraction=p('seed_inlier_fraction', 0.8),
                                init_frames=p('init_frames', 10))
        self.morph_kernel = p('morph_kernel', 5)
        self.use_edges = p('use_canny_edges', True)
        self.canny_low = p('canny_low', 60.0)
        self.canny_high = p('canny_high', 150.0)
        self.blur_kernel = p('blur_kernel', 3)
        self.min_obstacle_px = p('min_obstacle_px', 3)
        self.boundary_offset = p('boundary_offset_px', 1.0)        # px the edge/blur moves the boundary
        self.debug_every = p('debug_every_n', 2)                  # publish debug image every n frames

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.K = None
        self.D = None
        self.src_size = None
        self.lut = None
        self.lut_key = None
        self.geo = None
        self.last_proc = 0.0
        self.n_frames = 0
        self.proc_ms = []

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 2)
        self.create_subscription(CameraInfo, self.info_topic, self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)

    # ------------------------------------------------------------------ inputs
    def info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] <= 0:
            return
        self.K = K
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) else None
        self.src_size = (msg.width, msg.height)

    def camera_pose(self, frame_id: str):
        """(R, t) of the image frame and (t, yaw) of the scan frame in the base frame, or None."""
        try:
            cam = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
            scan = self.tf_buffer.lookup_transform(self.base_frame, self.scan_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'waiting for TF: {e}', throttle_duration_sec=5.0)
            return None
        tc, qc = cam.transform.translation, cam.transform.rotation
        ts, qs = scan.transform.translation, scan.transform.rotation
        yaw = np.arctan2(2 * (qs.w * qs.z + qs.x * qs.y), 1 - 2 * (qs.y ** 2 + qs.z ** 2))
        return (quat_to_matrix(qc), np.array([tc.x, tc.y, tc.z]),
                np.array([ts.x, ts.y, ts.z]), float(yaw))

    def ensure_lut(self, frame_id: str) -> bool:
        if self.K is None:
            self.get_logger().warn('waiting for camera_info', throttle_duration_sec=5.0)
            return False
        pose = self.camera_pose(frame_id)
        if pose is None:
            return False
        R, t, ts, yaw = pose
        key = (np.round(R, 5).tobytes(), np.round(t, 4).tobytes(), np.round(ts, 4).tobytes(),
               round(yaw, 4), self.K.tobytes(), None if self.D is None else self.D.tobytes(), self.src_size)
        if key == self.lut_key:
            return True
        self.lut = GroundLUT.build(self.K, self.src_size, (self.proc_w, self.proc_h), R, t, ts, yaw, self.D)
        self.lut_key = key
        hfov = self.hfov or 2.0 * np.arctan(self.src_size[0] / (2.0 * self.K[0, 0]))
        self.geo = ScanGeometry(-hfov / 2, hfov / 2, self.num_beams, self.range_min, self.range_max)
        visible = self.lut.rng[np.isfinite(self.lut.rng)]
        self.get_logger().info(
            f'ground LUT built: camera at z={t[2]:.3f} m, pitch {np.degrees(np.arcsin(-R[2, 2])):.1f} deg '
            f'down, nearest visible floor {visible.min():.3f} m, scan span {np.degrees(hfov):.0f} deg')
        return True

    # ------------------------------------------------------------------ main
    def image_cb(self, msg: Image):
        now = time.monotonic()
        if now - self.last_proc < 1.0 / self.max_rate:
            return
        self.last_proc = now
        if not self.ensure_lut(msg.header.frame_id):
            return
        t0 = time.perf_counter()
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        small = cv2.resize(bgr, (self.proc_w, self.proc_h), interpolation=cv2.INTER_AREA)
        if self.blur_kernel > 1:
            small = cv2.GaussianBlur(small, (self.blur_kernel, self.blur_kernel), 0)
        feat = self.model.features(small)
        rs, cs = seed_slice(self.proc_h, self.proc_w, self.seed_w, self.seed_h)
        self.model.update(feat[rs, cs].reshape(-1, 3))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        floor = floor_mask(self.model, feat, gray, self.morph_kernel, self.canny_low, self.canny_high,
                           self.use_edges)
        rows, found = first_obstacle_rows(floor, self.lut.horizon_row, self.min_obstacle_px)
        ranges = to_scan(self.lut, rows, found, self.geo, self.boundary_offset)
        self.publish_scan(msg, ranges)
        self.proc_ms.append(1e3 * (time.perf_counter() - t0))
        self.n_frames += 1
        if self.n_frames % self.debug_every == 0 and self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(msg, small, floor, rows, found, (rs, cs))

    def publish_scan(self, img_msg: Image, ranges: np.ndarray):
        scan = LaserScan()
        scan.header.stamp = img_msg.header.stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = float(self.geo.angle_min)
        scan.angle_max = float(self.geo.angle_max)
        scan.angle_increment = float(self.geo.increment)
        scan.scan_time = float(1.0 / self.max_rate)
        scan.range_min = float(self.range_min)
        r = ranges.copy()
        if self.no_obstacle_mode == 'max_plus':
            r[np.isinf(r)] = self.range_max + self.no_obstacle_margin
            scan.range_max = float(self.range_max + 2 * self.no_obstacle_margin)
        else:
            scan.range_max = float(self.range_max)
        r[np.isnan(r)] = self.unobserved_value
        scan.ranges = r.tolist()
        self.scan_pub.publish(scan)

    def publish_debug(self, img_msg, small, floor, rows, found, seed):
        dbg = small.copy()
        green = np.zeros_like(dbg)
        green[..., 1] = 255
        dbg[floor] = (0.6 * dbg[floor] + 0.4 * green[floor]).astype(np.uint8)
        hr = self.lut.horizon_row
        for c in range(0, self.proc_w):
            if hr[c] < self.proc_h:
                dbg[hr[c], c] = (255, 0, 255)                       # horizon: magenta
            if found[c]:
                r = self.lut.rng[min(rows[c] + 1, self.proc_h - 1), c]
                colour = (0, 0, 255) if r <= self.range_max else (0, 165, 255)  # red / orange: beyond max
                cv2.circle(dbg, (c, int(rows[c])), 1, colour, -1)
        rs, cs = seed
        cv2.rectangle(dbg, (cs.start, rs.start), (cs.stop - 1, rs.stop - 1), (255, 255, 0), 1)
        out = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        out.header = img_msg.header
        self.debug_pub.publish(out)

    def report(self):
        if self.proc_ms:
            a = np.asarray(self.proc_ms)
            self.get_logger().info(f'{len(a) / 10.0:.1f} Hz, processing {a.mean():.1f} ms mean / '
                                   f'{np.percentile(a, 95):.1f} ms p95 per frame')
            self.proc_ms = []


def main():
    rclpy.init()
    node = VisualFloorScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
