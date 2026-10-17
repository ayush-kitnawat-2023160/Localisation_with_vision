#!/usr/bin/env python3
"""Small, auditable evidence bundle. NEVER package rosbags, PCD, or secrets.

Bundle is deliberately named REJECTED while safety checks are not met.
The complete raw metrics and logs remain in their original result directories.
"""
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED

root=Path(__file__).resolve().parents[1]
dev=root/'results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015'
hold=root/'results/18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015'
slope=root/'results/camera_lidar_imu_gps/SLOPE_RECOVERY_VERIFIED_DEV_20261015'
files=[]
for path in [
    root/'reports/HEALTHY_COMPUTE_VALIDATION_20261015.md',
    root/'reports/HEALTHY_LOOP_MOTION_BRIDGE_VALIDATION_20261015.md',
    root/'reports/FOUR_MODE_FINAL_VALIDATION_20261015.md',
    dev/'frozen_validation_manifest.json',
    dev/'four_modes_and_holdout_status.txt',
    dev/'four_mode_readiness_audit.json',
    dev/'COMPARISON_BRIDGE/comparison.csv',
    dev/'COMPARISON_BRIDGE/comparison.json',
    dev/'COMPARISON_BRIDGE/01_four_mode_pcd_overlays.png',
    dev/'COMPARISON_BRIDGE/02_four_mode_reference_disagreement.png',
    hold/'reference_aligned_historical_comparisons.json',
    hold/'COMPARISON/comparison.csv',
    hold/'COMPARISON/summary.json',
    hold/'COMPARISON/all_four_modes_disagreement_vs_time.png',
    hold/'COMPARISON/healthy_pcd_overlay.png',
    hold/'COMPARISON/camera_failed_pcd_overlay.png',
    hold/'COMPARISON/lidar_failed_pcd_overlay.png',
    hold/'COMPARISON/imu_failed_pcd_overlay.png',
    slope/'validation_status.txt',
    slope/'LIDAR_FAIL_SLOPE_220S/metrics_exact.txt',
    slope/'LIDAR_FAIL_SLOPE_220S/plots/07_orange_trajectory_on_pcd_clean.png',
]:
    if path.is_file():files.append(path)
for base,suffix,cases in [
    (dev,'_BRIDGE_290S',['HEALTHY','CAMERA_FAIL','LIDAR_FAIL','IMU_FAIL']),
    (hold,'_HOLDOUT_220S',['HEALTHY','CAMERA_FAIL','LIDAR_FAIL','IMU_FAIL'])
]:
    for case in cases:
        folder=base/(case+suffix)
        for rel in ['metrics_exact.txt','run_metadata.txt',
                    'plots/07_orange_trajectory_on_pcd_clean.png',
                    'plots/06_global_pose_on_pcd_xy_zoom_loop.png',
                    'plots/03_pose_step_continuity.png']:
            p=folder/rel
            if p.is_file():files.append(p)
output=root/'results/VALIDATION_EVIDENCE_20261015_NOT_DEPLOYABLE.zip'
with ZipFile(output,'w',compression=ZIP_DEFLATED,compresslevel=5) as z:
    z.writestr('READ_ME_FIRST.txt',
       'Experimental localisation validation — 15 October 2026.\n'
       'NOT DEPLOYABLE: LiDAR failover diverges, jumps remain, target 0.5m not verified.\n'
       'All RMSE values compare against same-bag unvalidated diagnostic trajectory proxies, NOT surveyed ground truth.\n'
       'See reports/FOUR_MODE_FINAL_VALIDATION_20261015.md.\n')
    for p in sorted(set(files)):
        z.write(p,arcname=p.relative_to(root).as_posix())
print(output,'files',len(set(files)),'bytes',output.stat().st_size)
