#!/usr/bin/env python3
"""Adapt ALIVE ROS1 sensor messages to the copied FAST-LIVO2 estimator.

The estimator's Velodyne preprocessor stores per-point time in milliseconds
after reading a field expressed in microseconds.  The supplied ALIVE bags store
that field in seconds, so this node scales only the point-time field and leaves
all other PointCloud2 bytes untouched.

The full archived bag also contains camera header stamps from a different
clock epoch.  ``camera_timestamp_mode=rebase`` preserves image timing while
anchoring the first image to the current ROS clock.  Live mode must use
``preserve``.
"""

import copy

import numpy as np
import rospy
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField


class SensorAdapter:
    def __init__(self):
        self.lidar_time_scale = float(rospy.get_param("~lidar_time_scale", 1.0e6))
        self.camera_mode = str(rospy.get_param("~camera_timestamp_mode", "preserve")).lower()
        self.max_camera_hz = float(rospy.get_param("~max_camera_hz", 0.0))
        self.last_camera_source_stamp = None
        if self.camera_mode not in ("preserve", "rebase"):
            raise rospy.ROSInitException("camera_timestamp_mode must be preserve or rebase")
        if self.camera_mode == "rebase" and not rospy.get_param("/use_sim_time", False):
            raise rospy.ROSInitException(
                "camera_timestamp_mode=rebase requires use_sim_time=true and rosbag play --clock")
        self.camera_offset = None
        self.last_lidar_stamp = None
        self.lidar_frame = rospy.get_param(
            "~lidar_frame", rospy.get_param("/calibration/lidar_frame", "velodyne"))
        self.camera_frame = rospy.get_param(
            "~camera_frame",
            rospy.get_param("/calibration/camera_frame", "camera_color_optical_frame"))

        lidar_in = rospy.get_param("~source_lidar_topic", "/lidar103/velodyne_points")
        imu_in = rospy.get_param("~source_imu_topic", "/mavros/imu/data")
        camera_in = rospy.get_param("~source_camera_topic", "/camera/color/image_raw")
        lidar_out = rospy.get_param("~output_lidar_topic", "/fast_livo2/input/lidar")
        imu_out = rospy.get_param("~output_imu_topic", "/fast_livo2/input/imu")
        camera_out = rospy.get_param("~output_camera_topic", "/fast_livo2/input/image")

        self.lidar_pub = rospy.Publisher(lidar_out, PointCloud2, queue_size=4)
        self.imu_pub = rospy.Publisher(imu_out, Imu, queue_size=200)
        self.camera_pub = rospy.Publisher(camera_out, Image, queue_size=8)

        rospy.Subscriber(lidar_in, PointCloud2, self.lidar_cb, queue_size=8,
                         buff_size=8 * 1024 * 1024)
        rospy.Subscriber(imu_in, Imu, self.imu_cb, queue_size=400)
        rospy.Subscriber(camera_in, Image, self.camera_cb, queue_size=12,
                         buff_size=16 * 1024 * 1024)
        rospy.loginfo("FAST-LIVO2 adapter: %s -> %s, %s -> %s, %s -> %s; camera clock=%s max_hz=%.2f",
                      lidar_in, lidar_out, imu_in, imu_out, camera_in, camera_out,
                      self.camera_mode, self.max_camera_hz)

    @staticmethod
    def find_field(msg, name):
        return next((field for field in msg.fields if field.name == name), None)

    @staticmethod
    def field_view(raw, msg, field, dtype):
        itemsize = np.dtype(dtype).itemsize
        if msg.width <= 0 or msg.height <= 0:
            return None
        needed = (field.offset + msg.row_step * (msg.height - 1)
                  + msg.point_step * (msg.width - 1) + itemsize)
        if raw.size < needed:
            return None
        return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=raw,
                          offset=field.offset,
                          strides=(msg.row_step, msg.point_step))

    def lidar_cb(self, msg):
        if msg.is_bigendian:
            rospy.logerr_throttle(5.0, "Big-endian PointCloud2 is not supported by the adapter")
            return
        time_field = self.find_field(msg, "time")
        ring_field = self.find_field(msg, "ring")
        if time_field is None or time_field.datatype not in (
                PointField.FLOAT32, PointField.FLOAT64):
            rospy.logerr_throttle(5.0, "FAST-LIVO2 requires floating PointCloud2 field 'time'")
            return
        if ring_field is None:
            rospy.logerr_throttle(5.0, "FAST-LIVO2 Velodyne input requires PointCloud2 field 'ring'")
            return

        dtype = np.float32 if time_field.datatype == PointField.FLOAT32 else np.float64
        source = np.frombuffer(msg.data, dtype=np.uint8)
        values = self.field_view(source, msg, time_field, dtype)
        if values is None:
            rospy.logerr_throttle(5.0, "Invalid PointCloud2 layout for field 'time'")
            return

        finite = np.isfinite(values)
        sample = values[finite]
        # DEV-ONLY optimisation: ROS image/cloud payload bytes are immutable
        # in this callback. Copy the message shell and its header, not the
        # potentially multi-megabyte data buffer. LiDAR time bytes are still
        # copied below before any mutation; never write into msg.data.
        out = copy.copy(msg)
        out.header = copy.copy(msg.header)
        if not isinstance(msg.data, bytes):
            # Protect callers that provide a mutable Python payload instead
            # of ROS's normal deserialized immutable bytes object.
            out.data = copy.deepcopy(msg.data)
        out.header.frame_id = self.lidar_frame

        # A driver that already publishes microseconds must not be scaled again.
        #
        # The ALIVE VLP-16 bags stamp each cloud close to the end of the scan,
        # so their per-point times are approximately [-0.100, +0.001] seconds.
        # FAST-LIVO2 requires non-negative offsets from the scan-begin stamp.
        # Move the header to the earliest point and shift the point offsets
        # before converting seconds to microseconds.  This is the same adapter
        # convention used by the verified NEW_CART deployment.
        if sample.size and float(np.max(np.abs(sample))) < 1.0e3:
            raw = bytearray(out.data)
            target = np.frombuffer(raw, dtype=np.uint8)
            target_values = self.field_view(target, out, time_field, dtype)
            target_finite = np.isfinite(target_values)
            if target_finite.any():
                minimum = float(np.min(target_values[target_finite]))
                if minimum < -1.0e-6:
                    target_values[target_finite] -= minimum
                    out.header.stamp += rospy.Duration.from_sec(minimum)
            target_values[target_finite] *= self.lidar_time_scale
            out.data = bytes(raw)

        self.last_lidar_stamp = out.header.stamp
        self.lidar_pub.publish(out)

    def imu_cb(self, msg):
        self.imu_pub.publish(msg)

    def camera_cb(self, msg):
        if self.max_camera_hz > 0.0 and not msg.header.stamp.is_zero():
            if self.last_camera_source_stamp is not None:
                dt = (msg.header.stamp - self.last_camera_source_stamp).to_sec()
                if dt >= 0.0 and dt < (1.0 / self.max_camera_hz):
                    return
            self.last_camera_source_stamp = msg.header.stamp
        # Keep RGB pixels byte-identical, and allocate a new header before
        # changing its stamp/frame. Do not copy the whole image payload.
        out = copy.copy(msg)
        out.header = copy.copy(msg.header)
        if not isinstance(msg.data, bytes):
            out.data = copy.deepcopy(msg.data)
        if self.camera_mode == "rebase":
            if self.camera_offset is None:
                now = rospy.Time.now()
                if now.is_zero():
                    rospy.logerr_throttle(
                        5.0, "Waiting for a non-zero /clock before rebasing camera timestamps")
                    return
                self.camera_offset = now - out.header.stamp
                # Expose the ONE camera-hardware -> ROS/bag-time offset so all
                # RealSense streams (RGB, depth) use exactly the same clock
                # transform.  Independently estimating depth offset from Python
                # callback time introduced nondeterministic 90-113 ms skew.
                rospy.set_param("/fast_livo2/camera_clock_offset_sec",
                                self.camera_offset.to_sec())
                rospy.loginfo("FAST-LIVO2 camera clock rebased by %.6f s",
                              self.camera_offset.to_sec())
            out.header.stamp = out.header.stamp + self.camera_offset
        elif self.last_lidar_stamp is not None:
            clock_error = abs((out.header.stamp - self.last_lidar_stamp).to_sec())
            if clock_error > 5.0:
                rospy.logerr_throttle(
                    5.0,
                    "Camera/LiDAR clocks differ by %.3f s in preserve mode; use the bag launch for archived data",
                    clock_error)
                return
        out.header.frame_id = self.camera_frame
        self.camera_pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("fast_livo2_sensor_adapter")
    SensorAdapter()
    rospy.spin()
