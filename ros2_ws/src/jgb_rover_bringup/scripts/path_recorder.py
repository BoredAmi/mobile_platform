#!/usr/bin/env python3
"""Publish nav_msgs/Path histories of the EKF estimate and the ground truth for RViz.

/odometry/filtered (frame odom)        -> /path/ekf
/ground_truth/odom (frame world, sim)  -> /path/ground_truth, re-expressed in map_frame through the
                                          spawn pose (the map frame starts at the spawn pose)
A pose is appended when the robot moved more than min_step m or turned more than min_turn rad.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        self.min_step = self.declare_parameter('min_step', 0.02).value          # m
        self.min_turn = self.declare_parameter('min_turn', 0.05).value          # rad
        self.max_poses = self.declare_parameter('max_poses', 20000).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.spawn = self.declare_parameter('spawn_pose', [0.0, 0.0, 0.0]).value  # x, y, yaw in world
        self.paths = {}
        self.pubs = {}
        for name, topic, out in (('ekf', '/odometry/filtered', '/path/ekf'),
                                 ('gt', '/ground_truth/odom', '/path/ground_truth')):
            self.pubs[name] = self.create_publisher(Path, out, 1)
            self.create_subscription(Odometry, topic, lambda m, n=name: self.cb(n, m), 10)

    def cb(self, name, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        if name == 'gt':                                   # world -> map (spawn pose)
            sx, sy, syaw = self.spawn
            dx, dy = ps.pose.position.x - sx, ps.pose.position.y - sy
            c, s = math.cos(-syaw), math.sin(-syaw)
            ps.pose.position.x, ps.pose.position.y = c * dx - s * dy, s * dx + c * dy
            q = ps.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) - syaw
            q.x = q.y = 0.0
            q.z, q.w = math.sin(yaw / 2), math.cos(yaw / 2)
            ps.header.frame_id = self.map_frame
        path = self.paths.setdefault(name, Path())
        path.header = ps.header
        if path.poses:
            last = path.poses[-1].pose
            step = math.hypot(ps.pose.position.x - last.position.x, ps.pose.position.y - last.position.y)
            turn = abs(2 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w)
                       - 2 * math.atan2(last.orientation.z, last.orientation.w))
            if step < self.min_step and min(turn, 2 * math.pi - turn) < self.min_turn:
                return
        path.poses.append(ps)
        del path.poses[:-self.max_poses]
        self.pubs[name].publish(path)


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
