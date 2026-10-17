#!/usr/bin/env python3
"""Anchor FAST-LIVO2's local frame to the already matched PCD pose.

The paused FLOAM start localizer owns ``map -> base_link``.  FAST-LIVO2 starts
its own local ``camera_init -> aft_mapped`` frame after the bag is resumed.  On
the first FAST odometry sample, compose the two poses so RViz can display the
FAST path and registered cloud in the PCD's map frame without a second global
initializer competing for /pose or /tf.
"""

import threading

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


def pose_matrix(pose):
    matrix = tf.transformations.quaternion_matrix([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w])
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix


def pose_from_matrix(matrix):
    pose = PoseStamped().pose
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]
    q = tf.transformations.quaternion_from_matrix(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


class FastLivo2MapAnchor:
    def __init__(self):
        self.lock = threading.Lock()
        self.global_pose = None
        self.anchor = None  # map <- camera_init
        self.anchor_sent = False
        self.path_pub = rospy.Publisher(
            "/fast_livo2/global_path", Path, queue_size=2, latch=True)
        self.pose_pub = rospy.Publisher(
            "/fast_livo2/global_pose", PoseStamped, queue_size=4, latch=True)
        self.status_pub = rospy.Publisher(
            "/fast_livo2/global_anchor_status", String, queue_size=1, latch=True)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.path = Path()
        self.path.header.frame_id = "map"
        rospy.Subscriber("/pose", PoseStamped, self.global_pose_cb, queue_size=1)
        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/path", Path, self.path_cb, queue_size=1)
        self.publish_status("WAITING_FOR_PCD_POSE_AND_FAST_ODOM")

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))
        rospy.loginfo("FAST-LIVO2 map anchor: %s", text)

    def global_pose_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != "map":
            rospy.logwarn("Ignoring PCD pose in frame %s", msg.header.frame_id)
            return
        with self.lock:
            self.global_pose = msg
            # A new PCD localization is a deliberate reset. Re-anchor FAST on
            # its next local odometry message.
            self.anchor = None
            self.anchor_sent = False
            self.path = Path()
            self.path.header.frame_id = "map"
        self.publish_status("PCD_POSE_READY_WAITING_FOR_FAST_ODOM")

    def send_anchor(self, matrix, stamp):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time(0)
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera_init"
        transform.transform.translation.x = matrix[0, 3]
        transform.transform.translation.y = matrix[1, 3]
        transform.transform.translation.z = matrix[2, 3]
        q = tf.transformations.quaternion_from_matrix(matrix)
        transform.transform.rotation.x, transform.transform.rotation.y = q[0], q[1]
        transform.transform.rotation.z, transform.transform.rotation.w = q[2], q[3]
        self.static_broadcaster.sendTransform(transform)

    def odom_cb(self, msg):
        local = pose_matrix(msg.pose.pose)
        with self.lock:
            if self.global_pose is None:
                return
            if self.anchor is None:
                # FAST's aft_mapped is the IMU/body state. The recorded-cart
                # calibration has zero base/IMU translation and identical
                # LiDAR-to-base and LiDAR-to-IMU rotations.
                self.anchor = pose_matrix(self.global_pose.pose) @ tf.transformations.inverse_matrix(local)
                self.send_anchor(self.anchor, msg.header.stamp)
                self.anchor_sent = True
                self.publish_status("ANCHORED_MAP_TO_CAMERA_INIT")
            global_matrix = self.anchor @ local
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "map"
        pose.pose = pose_from_matrix(global_matrix)
        self.pose_pub.publish(pose)

    def path_cb(self, msg):
        with self.lock:
            if self.anchor is None:
                return
            anchor = self.anchor.copy()
        path = Path()
        path.header = msg.header
        path.header.frame_id = "map"
        for item in msg.poses:
            transformed = PoseStamped()
            transformed.header = item.header
            transformed.header.frame_id = "map"
            transformed.pose = pose_from_matrix(anchor @ pose_matrix(item.pose))
            path.poses.append(transformed)
        self.path_pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("fast_livo2_map_anchor")
    FastLivo2MapAnchor()
    rospy.spin()
