#!/usr/bin/env python3
"""Record FAST-LIVO2 odometry and current failsafe status to a CSV file."""

import csv
import os
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Recorder:
    def __init__(self):
        path = rospy.get_param("~output", "/tmp/fast_livo2_trajectory.csv")
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw",
                              "failsafe_mode", "sensor_health"])
        self.mode = "UNKNOWN"
        self.health = "UNKNOWN"
        odom_topic = rospy.get_param("~odom_topic", "/aft_mapped_to_init")
        rospy.Subscriber("/fast_livo2/failsafe_mode", String, self._mode, queue_size=10)
        rospy.Subscriber("/fast_livo2/sensor_health", String, self._health, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self._odom, queue_size=100)
        rospy.on_shutdown(self.close)

    def _mode(self, msg):
        self.mode = msg.data

    def _health(self, msg):
        self.health = msg.data

    def _odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.writer.writerow([msg.header.stamp.to_sec(), p.x, p.y, p.z,
                              q.x, q.y, q.z, q.w, self.mode, self.health])
        self.file.flush()

    def close(self):
        if not self.file.closed:
            self.file.close()


if __name__ == "__main__":
    rospy.init_node("fast_livo2_failsafe_trajectory_recorder")
    Recorder()
    rospy.spin()
