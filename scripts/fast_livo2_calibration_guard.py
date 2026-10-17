#!/usr/bin/env python3
"""Validate one calibration profile and own the base->LiDAR static TF."""

import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix


def vector(name, size):
    value = rospy.get_param(name, None)
    if not isinstance(value, list) or len(value) != size:
        raise RuntimeError(f"{name} must contain exactly {size} numbers")
    result = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(result)):
        raise RuntimeError(f"{name} contains non-finite values")
    return result


def rotation(name):
    result = vector(name, 9).reshape(3, 3)
    determinant = float(np.linalg.det(result))
    error = float(np.linalg.norm(result.T @ result - np.eye(3)))
    if abs(determinant - 1.0) > 1.0e-3 or error > 1.0e-3:
        raise RuntimeError(
            f"{name} is not a proper rotation (det={determinant:.9f}, orth={error:.3e})")
    return result


def require_non_placeholder(rotation_matrix, translation, label):
    if (np.max(np.abs(rotation_matrix - np.eye(3))) < 1.0e-8
            and np.max(np.abs(translation)) < 1.0e-8):
        raise RuntimeError(f"{label} is still the identity/zero placeholder")


def main():
    rospy.init_node("fast_livo2_calibration_guard")
    if not bool(rospy.get_param("/calibration/ready", False)):
        raise RuntimeError(
            "NEW-CART CALIBRATION IS LOCKED: fill config/calibration/"
            "new_cart_measured.yaml from physical measurements, then set calibration/ready=true")

    measurement_id = str(rospy.get_param("/calibration/measurement_id", ""))
    if not measurement_id or "REPLACE_ME" in measurement_id:
        raise RuntimeError("calibration/measurement_id is missing or still a placeholder")

    t_imu_lidar = vector("/calibration/lidar_to_imu_translation", 3)
    r_imu_lidar = rotation("/calibration/lidar_to_imu_rotation")
    t_base_lidar = vector("/calibration/lidar_to_base_translation", 3)
    r_base_lidar = rotation("/calibration/lidar_to_base_rotation")
    fast_t = vector("/extrin_calib/extrinsic_T", 3)
    fast_r = rotation("/extrin_calib/extrinsic_R")
    p_camera_lidar = vector("/extrin_calib/Pcl", 3)
    r_camera_lidar = rotation("/extrin_calib/Rcl")

    if np.max(np.abs(fast_t - t_imu_lidar)) > 1.0e-8 or \
            np.max(np.abs(fast_r - r_imu_lidar)) > 1.0e-8:
        raise RuntimeError(
            "FAST extrin_calib and calibration/lidar_to_imu do not match")
    require_non_placeholder(r_imu_lidar, t_imu_lidar, "LiDAR->IMU")
    require_non_placeholder(r_base_lidar, t_base_lidar, "LiDAR->base_link")
    require_non_placeholder(r_camera_lidar, p_camera_lidar, "LiDAR->camera")

    sensor_height = float(rospy.get_param("/calibration/sensor_height_m", 0.0))
    if not math.isfinite(sensor_height) or not 0.3 <= sensor_height <= 4.0:
        raise RuntimeError("calibration/sensor_height_m must be a measured value in [0.3, 4.0] m")

    width = int(rospy.get_param("/laserMapping/cam_width", 0))
    height = int(rospy.get_param("/laserMapping/cam_height", 0))
    camera_values = {
        name: float(rospy.get_param(f"/laserMapping/{name}", float("nan")))
        for name in ("cam_fx", "cam_fy", "cam_cx", "cam_cy",
                     "cam_d0", "cam_d1", "cam_d2", "cam_d3", "cam_d4")
    }
    if width <= 0 or height <= 0 or not all(
            math.isfinite(value) for value in camera_values.values()) \
            or camera_values["cam_fx"] <= 0.0 or camera_values["cam_fy"] <= 0.0 \
            or camera_values["cam_cx"] <= 0.0 or camera_values["cam_cy"] <= 0.0:
        raise RuntimeError("camera dimensions/intrinsics/distortion are not calibrated")

    for name in ("imu_time_offset", "lidar_time_offset", "img_time_offset"):
        value = float(rospy.get_param(f"/time_offset/{name}", float("nan")))
        if not math.isfinite(value):
            raise RuntimeError(f"time_offset/{name} is not finite")
    fast_floam_offset = float(rospy.get_param(
        "/calibration/fast_to_floam_time_offset_sec", float("nan")))
    if not math.isfinite(fast_floam_offset):
        raise RuntimeError("calibration/fast_to_floam_time_offset_sec is not finite")

    rospy.loginfo("Calibration guard passed: %s, LiDAR height %.3f m",
                  measurement_id, sensor_height)

    if bool(rospy.get_param("~publish_lidar_tf", False)):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = str(
            rospy.get_param("/calibration/base_frame", "base_link"))
        transform.child_frame_id = str(
            rospy.get_param("/calibration/lidar_frame", "velodyne"))
        transform.transform.translation.x = float(t_base_lidar[0])
        transform.transform.translation.y = float(t_base_lidar[1])
        transform.transform.translation.z = float(t_base_lidar[2])
        homogeneous = np.eye(4)
        homogeneous[:3, :3] = r_base_lidar
        quaternion = quaternion_from_matrix(homogeneous)
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(transform)
        rospy.loginfo("Calibration node owns static TF %s -> %s",
                      transform.header.frame_id, transform.child_frame_id)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        try:
            rospy.logfatal("%s", exc)
        except Exception:
            print(f"FATAL: {exc}")
        raise SystemExit(2)
