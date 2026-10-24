#!/usr/bin/env python3
"""Visualization-only paths for the LiDAR-free RGB-D demo.

The first temporally confirmed visual map candidate anchors the local RGB-D
odometry trajectory into the saved map frame. This is only a display alignment:
the node publishes no TF, /pose, estimator correction, or control output.
"""
from collections import deque
import copy
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix, quaternion_matrix


def pose_to_matrix(pose):
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        return None
    T = quaternion_matrix(q)
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(T).all():
        return None
    return T


def matrix_to_pose_stamped(stamp, T, frame_id):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T[:3, 3])
    q = quaternion_from_matrix(T)
    msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
    msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
    return msg


class VisualizationPathBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.max_poses = max(100, int(rospy.get_param('~max_path_poses', 3000)))
        self.max_anchor_skew = float(rospy.get_param('~max_anchor_skew_sec', 0.25))
        self.odom_buffer = deque(maxlen=2000)
        self.pending_anchor = None
        self.T_map_local = None
        self.anchor_stamp = None
        self.last_odom_stamp = -1.0
        self.global_poses = deque(maxlen=self.max_poses)
        self.odom_poses = deque(maxlen=self.max_poses)

        self.global_path_pub = rospy.Publisher(
            '/failsafe_rviz/global_visual_path', Path, queue_size=1, latch=True)
        self.odom_path_pub = rospy.Publisher(
            '/failsafe_rviz/visual_odom_path', Path, queue_size=1, latch=True)
        self.aligned_odom_pub = rospy.Publisher(
            '/failsafe_rviz/visual_odom_map_candidate', Odometry, queue_size=10)
        self.status_pub = rospy.Publisher(
            '/failsafe_rviz/alignment_status', String, queue_size=2, latch=True)

        rospy.Subscriber('/fast_livo2/visual_global_candidate',
                         PoseWithCovarianceStamped, self.on_global, queue_size=20)
        rospy.Subscriber('/fast_livo2/fallback/camera_imu_odom_candidate',
                         Odometry, self.on_odom, queue_size=100)
        self.status_pub.publish(String(data='WAITING_FOR_VISUAL_MAP_CANDIDATE'))

    @staticmethod
    def make_path(stamp, poses):
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.poses = list(poses)
        return msg

    def on_global(self, msg):
        T_map_body = pose_to_matrix(msg.pose.pose)
        if T_map_body is None:
            return
        with self.lock:
            pose_msg = matrix_to_pose_stamped(msg.header.stamp, T_map_body, 'map')
            if self.global_poses and msg.header.stamp <= self.global_poses[-1].header.stamp:
                return
            self.global_poses.append(pose_msg)
            self.global_path_pub.publish(self.make_path(msg.header.stamp, self.global_poses))
            if self.T_map_local is None:
                self.pending_anchor = (msg.header.stamp.to_sec(), T_map_body)
                self.try_set_anchor()

    def on_odom(self, msg):
        T_local_body = pose_to_matrix(msg.pose.pose)
        if T_local_body is None:
            return
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            if not np.isfinite(stamp) or stamp <= self.last_odom_stamp:
                return
            self.last_odom_stamp = stamp
            self.odom_buffer.append((stamp, T_local_body))
            if self.T_map_local is None:
                self.try_set_anchor()
            if self.T_map_local is not None and stamp >= self.anchor_stamp:
                self.publish_aligned_odom(msg, T_local_body)

    def try_set_anchor(self):
        if self.T_map_local is not None or self.pending_anchor is None or not self.odom_buffer:
            return
        map_stamp, T_map_body = self.pending_anchor
        stamp, T_local_body = min(
            self.odom_buffer, key=lambda item: abs(item[0] - map_stamp))
        skew = abs(stamp - map_stamp)
        if skew > self.max_anchor_skew:
            self.status_pub.publish(String(data=(
                'WAITING_FOR_TIME_ALIGNED_RGBD_ODOMETRY skew_sec=%.3f' % skew)))
            return
        self.T_map_local = T_map_body @ np.linalg.inv(T_local_body)
        self.anchor_stamp = map_stamp
        self.status_pub.publish(String(data=(
            'VISUALIZATION_ONLY_ANCHOR_SET skew_sec=%.3f; NO_TF_OR_POSE_AUTHORITY' % skew)))
        rospy.logwarn('RViz-only map alignment set from the first confirmed visual candidate; '
                      'this does not change odometry or grant pose authority')
        # Include buffered odometry after the visual anchor, then publish the
        # accumulated path as subsequent frames arrive.
        for buffered_stamp, buffered_T in self.odom_buffer:
            if buffered_stamp >= self.anchor_stamp:
                self.append_aligned_pose(buffered_stamp, buffered_T)

    def append_aligned_pose(self, stamp, T_local_body):
        if self.T_map_local is None or stamp <= self.anchor_stamp:
            return
        if self.odom_poses and stamp <= self.odom_poses[-1].header.stamp.to_sec():
            return
        T_map_body = self.T_map_local @ T_local_body
        self.odom_poses.append(
            matrix_to_pose_stamped(rospy.Time.from_sec(stamp), T_map_body, 'map'))
        self.odom_path_pub.publish(
            self.make_path(rospy.Time.from_sec(stamp), self.odom_poses))

    def publish_aligned_odom(self, source, T_local_body):
        stamp = source.header.stamp.to_sec()
        T_map_body = self.T_map_local @ T_local_body
        out = copy.deepcopy(source)
        out.header.frame_id = 'map'
        out.child_frame_id = 'failsafe_demo_visual_body_candidate'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(
            float, T_map_body[:3, 3])
        q = quaternion_from_matrix(T_map_body)
        out.pose.pose.orientation.x = float(q[0])
        out.pose.pose.orientation.y = float(q[1])
        out.pose.pose.orientation.z = float(q[2])
        out.pose.pose.orientation.w = float(q[3])
        self.aligned_odom_pub.publish(out)
        self.append_aligned_pose(stamp, T_local_body)


if __name__ == '__main__':
    rospy.init_node('rviz_visualization_path_bridge')
    VisualizationPathBridge()
    rospy.spin()
