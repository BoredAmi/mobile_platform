"""aruco_detector: detect ArUco markers, publish their poses, and check them against the map.

Subscribes: image_topic, camera_info_topic, TF
Publishes:  poses_topic (geometry_msgs/PoseArray, in the image frame), TF <frame_prefix><id>
Mapping-quality check (optional): if ground_truth_file is set (the world layout yaml), every
detection is transformed into map_frame (TF at the image time) and compared with the marker's
true position, with the world -> map transform given by the robot spawn pose. A summary per
marker is logged every report_period seconds.
"""
import math
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def aruco_dictionary(name: str):
    dict_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


class ArucoDetector:
    """OpenCV 4.5 (function API) and >= 4.7 (ArucoDetector class) compatible detection."""

    def __init__(self, dictionary_name: str):
        d = aruco_dictionary(dictionary_name)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
            self.detect = lambda gray: self.detector.detectMarkers(gray)[:2]
        else:
            params = cv2.aruco.DetectorParameters_create()
            self.detect = lambda gray: cv2.aruco.detectMarkers(gray, d, parameters=params)[:2]


def rvec_to_quat(rvec):
    R, _ = cv2.Rodrigues(rvec)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return x, y, z, w


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        p = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731
        self.image_topic = p('image_topic', '/camera/image_raw')
        self.info_topic = p('camera_info_topic', '/camera/camera_info')
        self.poses_topic = p('poses_topic', '/aruco/poses')
        self.dictionary = p('dictionary', 'DICT_4X4_50')
        self.marker_size = p('marker_size', 0.10)                   # m, black square edge
        self.max_rate = p('max_rate', 10.0)                         # Hz
        self.publish_tf = p('publish_tf', True)
        self.frame_prefix = p('frame_prefix', 'aruco_')
        self.max_distance = p('max_distance', 3.0)                  # m, ignore farther detections
        self.map_frame = p('map_frame', 'map')
        self.truth_file = p('ground_truth_file', '')                # world layout yaml, '' = off
        self.spawn = p('spawn_pose', [0.0, 0.0, 0.0])               # x, y, yaw of map origin in world
        self.report_period = p('report_period', 10.0)

        self.detector = ArucoDetector(self.dictionary)
        s = self.marker_size / 2.0
        # marker frame: x right, y up, z out of the marker (towards the camera)
        self.obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.last = 0.0
        self.truth = self.load_truth(self.truth_file) if self.truth_file else {}
        self.stats = {}                                             # id -> list of (err_xy, err_z, dist, yaw_err)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pub = self.create_publisher(PoseArray, self.poses_topic, 10)
        self.create_subscription(CameraInfo, self.info_topic, self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        if self.truth:
            self.create_timer(self.report_period, self.report)

    def load_truth(self, path):
        layout = yaml.safe_load(open(path))
        ar = layout['aruco']
        cx, cy, cyaw = self.spawn
        c, s = math.cos(-cyaw), math.sin(-cyaw)
        truth = {}
        for item in ar['items']:
            wx, wy = item['xy']
            dx, dy = wx - cx, wy - cy                               # world -> map
            truth[item['id']] = (c * dx - s * dy, s * dx + c * dy, ar['z_centre'], item['normal_yaw'] - cyaw)
        self.get_logger().info(f'checking {len(truth)} markers against {path}')
        return truth

    def info_cb(self, msg):
        self.K = np.array(msg.k, np.float64).reshape(3, 3)
        self.D = np.array(msg.d, np.float64) if len(msg.d) else np.zeros(5)

    def image_cb(self, msg):
        now = time.monotonic()
        if self.K is None or now - self.last < 1.0 / self.max_rate:
            return
        self.last = now
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        corners, ids = self.detector.detect(gray)
        out = PoseArray()
        out.header = msg.header
        if ids is not None:
            for c, mid in zip(corners, ids.ravel()):
                ok, rvec, tvec = cv2.solvePnP(self.obj, c.reshape(4, 2).astype(np.float64), self.K, self.D,
                                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok or np.linalg.norm(tvec) > self.max_distance:
                    continue
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (float(v) for v in tvec.ravel())
                q = rvec_to_quat(rvec)
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
                out.poses.append(pose)
                if self.publish_tf:
                    t = TransformStamped()
                    t.header = msg.header
                    t.child_frame_id = f'{self.frame_prefix}{int(mid)}'
                    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
                        pose.position.x, pose.position.y, pose.position.z
                    t.transform.rotation = pose.orientation
                    self.tf_pub.sendTransform(t)
                if int(mid) in self.truth:
                    self.compare(int(mid), msg.header, tvec.ravel(), rvec)
        self.pub.publish(out)

    def compare(self, mid, header, tvec, rvec):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, header.frame_id, Time.from_msg(header.stamp))
        except TransformException:
            return                                                  # map not there yet / too old
        R = quat_to_matrix(tf.transform.rotation)
        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        p_map = R @ tvec + t
        normal = R @ cv2.Rodrigues(rvec)[0][:, 2]                   # marker z axis (out of the marker)
        tx, ty, tz, tyaw = self.truth[mid]
        err_xy = math.hypot(p_map[0] - tx, p_map[1] - ty)
        yaw_err = math.degrees(math.atan2(math.sin(math.atan2(normal[1], normal[0]) - tyaw),
                                          math.cos(math.atan2(normal[1], normal[0]) - tyaw)))
        self.stats.setdefault(mid, []).append((err_xy, p_map[2] - tz, float(np.linalg.norm(tvec)), yaw_err))

    def report(self):
        if not self.stats:
            return
        lines = []
        all_err = []
        for mid in sorted(self.stats):
            a = np.asarray(self.stats[mid])
            all_err.extend(a[:, 0])
            lines.append(f'id {mid}: n={len(a)} xy err median {100 * np.median(a[:, 0]):.1f} cm '
                         f'(z {100 * np.median(a[:, 1]):+.1f} cm, facing {np.median(a[:, 3]):+.0f} deg, '
                         f'at {np.median(a[:, 2]):.1f} m)')
        self.get_logger().info(f'marker check, {len(self.stats)} markers, overall median '
                               f'{100 * np.median(all_err):.1f} cm:\n  ' + '\n  '.join(lines))


def main():
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
