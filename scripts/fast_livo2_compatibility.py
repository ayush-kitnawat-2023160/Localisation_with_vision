#!/usr/bin/env python3
"""Publish an honestly framed copy of FAST-LIVO2 local odometry.

FAST-LIVO2 owns the native ``camera_init -> aft_mapped`` local transform.  This
bridge never relabels that local pose as ``map -> base_link`` and never
publishes TF. FLOAM publishes the global measurement topics and the genuine
optimized ``map -> base_link`` transform directly.
"""

import copy

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class CompatibilityBridge:
    def __init__(self):
        native_odom = rospy.get_param("~native_odom_topic", "/aft_mapped_to_init")
        odom_topic = rospy.get_param("~odom_topic", "/fast_livo2/local_odom")
        pose_topic = rospy.get_param("~pose_topic", "/fast_livo2/local_pose")
        path_topic = rospy.get_param("~path_topic", "/fast_livo2/local_trajectory")

        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=20)
        self.pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=20)
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=5)
        self.native_path_pub = rospy.Publisher("/fast_livo2/native_path", Path, queue_size=5)

        rospy.Subscriber(native_odom, Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/path", Path, self.path_cb, queue_size=5)
        rospy.loginfo("FAST-LIVO2 local compatibility: %s -> %s and %s (no TF)",
                      native_odom, odom_topic, pose_topic)

    def odom_cb(self, native):
        odom = copy.deepcopy(native)
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = copy.deepcopy(odom.header)
        pose.pose = copy.deepcopy(odom.pose.pose)
        self.pose_pub.publish(pose)

    def path_cb(self, native):
        path = copy.deepcopy(native)
        self.path_pub.publish(path)
        self.native_path_pub.publish(native)


if __name__ == "__main__":
    rospy.init_node("fast_livo2_compatibility")
    CompatibilityBridge()
    rospy.spin()
