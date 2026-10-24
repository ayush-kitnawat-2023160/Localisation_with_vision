#!/usr/bin/env python3
"""Check camera<-LiDAR TF, FAST-LIVO2 extrinsics and optional 3D->2D reprojection.

Example:
  python3 scripts/verify_zedx_calibration.py \
    --tf config/calibration/zedx_lidar_tf_source_20261019.yaml \
    --profile config/calibration/zedx_current.yaml \
    --object-points /path/to/object_points.txt \
    --image-points /path/to/image_points.txt
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml


def rotation_from_rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1]
    ap.add_argument("--tf", type=Path, default=root / "config/calibration/zedx_lidar_tf_source_20261019.yaml")
    ap.add_argument("--profile", type=Path, default=root / "config/calibration/zedx_current.yaml")
    ap.add_argument("--image-points", type=Path,
                    default=root / "config/calibration/measurements_20261019/image_points.txt")
    ap.add_argument("--object-points", type=Path,
                    default=root / "config/calibration/measurements_20261019/object_points.txt")
    args = ap.parse_args()

    with args.tf.open() as f:
        tf = yaml.safe_load(f)["zedx_left_camera_optical_frame"]["velodyne"]
    with args.profile.open() as f:
        profile = yaml.safe_load(f)

    R = rotation_from_rpy(tf["roll"], tf["pitch"], tf["yaw"])
    t = np.array([tf["x"], tf["y"], tf["z"]], dtype=float)
    Rc = np.asarray(profile["extrin_calib"]["Rcl"], dtype=float).reshape(3, 3)
    tc = np.asarray(profile["extrin_calib"]["Pcl"], dtype=float)
    rotation_error = np.linalg.norm(R - Rc)
    translation_error = np.linalg.norm(t - tc)
    assert abs(np.linalg.det(Rc) - 1.0) < 1e-6, "Rcl is not a valid rotation"
    assert rotation_error < 2e-7, f"TF/profile rotation differs by {rotation_error}"
    assert translation_error < 1e-8, f"TF/profile translation differs by {translation_error}"

    print("TF/profile extrinsics: PASS")
    print("Convention: p_camera = Rcl @ p_velodyne + Pcl")
    print("Rcl: ", Rc.tolist())
    print("Pcl: ", tc.tolist())

    if bool(args.image_points) != bool(args.object_points):
        ap.error("Provide BOTH --image-points and --object-points, or neither")
    if args.image_points:
        image = np.atleast_2d(np.loadtxt(args.image_points))
        obj = np.atleast_2d(np.loadtxt(args.object_points))
        if image.shape[1] != 2 or obj.shape[1] != 3 or len(image) != len(obj):
            raise ValueError("Expected equally sized Nx2 image and Nx3 LiDAR point arrays")
        cfg = profile["laserMapping"]
        camera = obj @ Rc.T + tc
        if np.any(camera[:, 2] <= 0):
            raise ValueError("Some transformed landmarks are behind the camera")
        projected = np.column_stack([
            float(cfg["cam_fx"]) * camera[:, 0] / camera[:, 2] + float(cfg["cam_cx"]),
            float(cfg["cam_fy"]) * camera[:, 1] / camera[:, 2] + float(cfg["cam_cy"]),
        ])
        errors = np.linalg.norm(projected - image, axis=1)
        print(f"Reprojection: {len(image)} correspondences, "
              f"mean={errors.mean():.3f}px median={np.median(errors):.3f}px "
              f"p95={np.percentile(errors, 95):.3f}px max={errors.max():.3f}px")
        print("NOTE: Reprojection consistency is not proof of IMU/base calibration.")
    else:
        print("No correspondence files provided; reprojection check skipped.")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyError, AssertionError, ValueError, OSError) as exc:
        sys.exit(f"Calibration check FAILED: {exc}")
