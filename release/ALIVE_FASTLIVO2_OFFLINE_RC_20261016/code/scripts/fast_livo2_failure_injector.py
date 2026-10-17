#!/usr/bin/env python3
"""Runtime single-sensor dropout relay for FAST-LIVO2-failsafe tests.

Messages are forwarded unchanged.  The selected stream is suppressed only
inside [drop_start, drop_start + drop_duration), measured in ROS time from the
first relayed sensor message.  This tests live failover/recovery rather than
launch-time sensor disabling.
"""

import rospy
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String


class FailureInjector:
    def __init__(self):
        self.drop_sensor = str(rospy.get_param("~drop_sensor", "none")).lower()
        if self.drop_sensor not in ("none", "camera", "lidar", "imu"):
            raise rospy.ROSInitException("drop_sensor must be none|camera|lidar|imu")
        self.drop_start = float(rospy.get_param("~drop_start", 30.0))
        self.drop_duration = float(rospy.get_param("~drop_duration", 10.0))
        self.t0 = None
        self.was_dropping = False

        lidar_in = rospy.get_param("~source_lidar_topic", "/fast_livo2/source/lidar")
        imu_in = rospy.get_param("~source_imu_topic", "/fast_livo2/source/imu")
        camera_in = rospy.get_param("~source_camera_topic", "/fast_livo2/source/image")
        lidar_out = rospy.get_param("~output_lidar_topic", "/fast_livo2/input/lidar")
        imu_out = rospy.get_param("~output_imu_topic", "/fast_livo2/input/imu")
        camera_out = rospy.get_param("~output_camera_topic", "/fast_livo2/input/image")

        self.lidar_pub = rospy.Publisher(lidar_out, PointCloud2, queue_size=8)
        self.imu_pub = rospy.Publisher(imu_out, Imu, queue_size=400)
        self.camera_pub = rospy.Publisher(camera_out, Image, queue_size=12)
        self.status_pub = rospy.Publisher("/fast_livo2/failure_injector_status", String,
                                          queue_size=4, latch=True)

        rospy.Subscriber(lidar_in, PointCloud2, self._lidar, queue_size=8,
                         buff_size=8 * 1024 * 1024)
        rospy.Subscriber(imu_in, Imu, self._imu, queue_size=400)
        rospy.Subscriber(camera_in, Image, self._camera, queue_size=12,
                         buff_size=16 * 1024 * 1024)
        self._publish_status("armed")
        rospy.loginfo("[INJECTOR] sensor=%s start=%.3fs duration=%.3fs",
                      self.drop_sensor, self.drop_start, self.drop_duration)

    def _elapsed(self):
        now = rospy.Time.now().to_sec()
        if now <= 0.0:
            return None
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def _publish_status(self, state):
        self.status_pub.publish(String(data=(
            "sensor={},state={},drop_start={:.3f},drop_duration={:.3f}".format(
                self.drop_sensor, state, self.drop_start, self.drop_duration))))

    def _drop(self, sensor):
        elapsed = self._elapsed()
        active = (elapsed is not None and self.drop_sensor == sensor and
                  self.drop_duration > 0.0 and
                  self.drop_start <= elapsed < self.drop_start + self.drop_duration)
        if active and not self.was_dropping:
            self.was_dropping = True
            rospy.logwarn("[INJECTOR] %s DROP START at %.3f s", sensor, elapsed)
            self._publish_status("dropping")
        elif not active and self.was_dropping and self.drop_sensor == sensor:
            self.was_dropping = False
            rospy.logwarn("[INJECTOR] %s DROP END at %.3f s", sensor, elapsed)
            self._publish_status("recovered")
        return active

    def _lidar(self, msg):
        if not self._drop("lidar"):
            self.lidar_pub.publish(msg)

    def _imu(self, msg):
        if not self._drop("imu"):
            self.imu_pub.publish(msg)

    def _camera(self, msg):
        if not self._drop("camera"):
            self.camera_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("fast_livo2_failure_injector")
    FailureInjector()
    rospy.spin()
