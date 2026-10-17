#!/usr/bin/env python3
"""Portable evidence bundle for the still-FAILED localisation experiments.

Only whitelist code diffs, scored trial summaries, and small PCD overlays.
Never bundle bag files, PCD databases, credentials, full ROS logs or models.
"""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED

root=Path(__file__).resolve().parents[1]
included=[
 'reports/PENDING_FAILSAFE_ENGINEERING_20261015.md',
 'reports/FOUR_MODE_FINAL_VALIDATION_20261015.md',
 'scripts/build_isolated_failclosed_recovery_20261015.py',
 'scripts/build_isolated_camera_recovery_20261015.py',
 'scripts/build_isolated_watchdog_overlay_20261015.py',
 'scripts/localization_trust_watchdog.py',
 'scripts/test_localization_trust_watchdog.py',
 'scripts/test_localization_trust_watchdog_ros_20261015.py',
 'scripts/score_failclosed_recovery_trial_20261015.py',
 'launch/fast_livo2_step3_global_failclosed_recovery_dev.launch',
 'launch/fast_livo2_step3_global_failclosed_temporal_dev.launch',
 'launch/fast_livo2_step3_global_failclosed_watchdog_dev.launch',
 'step3_global/failclosed_recovery_dev_20261015/isolated_change.patch',
 'step3_global/camera_return_handoff_dev_20261015/isolated_change.patch',
 'results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/failclosed_vs_previous_lidar_160s.json',
 'results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/failclosed_v2_vs_previous_lidar_155s.json',
 'results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/temporal_vs_failclosed_v2_155s.json',
 'results/camera_lidar_imu_gps/CAMERA_RETURN_HANDOFF_DEV_20261015/camera_return_vs_frozen_190s.json',
 'results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_FAILCLOSED_V2_160S/metrics_exact.txt',
 'results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_TEMPORAL_160S/metrics_exact.txt',
 'results/camera_lidar_imu_gps/CAMERA_RETURN_HANDOFF_DEV_20261015/CAMERA_FAIL_RETURN_195S/metrics_exact.txt',
 'results/camera_lidar_imu_gps/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_FAILCLOSED_195S/metrics_exact.txt',
 'results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_FAILCLOSED_V2_160S/plots/07_orange_trajectory_on_pcd_clean.png',
 'results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_TEMPORAL_160S/plots/07_orange_trajectory_on_pcd_clean.png',
 'results/camera_lidar_imu_gps/CAMERA_RETURN_HANDOFF_DEV_20261015/CAMERA_FAIL_RETURN_195S/plots/07_orange_trajectory_on_pcd_clean.png',
 'results/camera_lidar_imu_gps/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_FAILCLOSED_195S/plots/07_orange_trajectory_on_pcd_clean.png',
]
paths=[root/x for x in included]
missing=[str(p.relative_to(root)) for p in paths if not p.is_file()]
if missing:
    print('Missing optional/expected evidence:',missing)
    paths=[p for p in paths if p.is_file()]

manifest={'label':'EXPERIMENTAL - NOT DEPLOYABLE',
          'accuracy_ground_truth_independent':False,
          'requested_0p5m_accurate_and_all_failures_accepted':False,
          'live_cart_and_braking_tested':False,
          'notes':'Files from 15 Oct 2026. Reference trajectories are diagnostic proxies. Never treat stop_required Bool as a real DBW stop.',
          'files':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}
out=root/'results/PENDING_FAILSAFE_EVIDENCE_20261015_NOT_DEPLOYABLE.zip'
with ZipFile(out,'w',ZIP_DEFLATED,compresslevel=5) as z:
    z.writestr('DO_NOT_DEPLOY.txt',
      'NOT FOR GOLF CART DEPLOYMENT.\n\nLiDAR outage recovery FAILED in both bags, '
      'and 0.5m accuracy NOT independently verified.\n'
      'The ROS watchdog is NOT connected to DBW/brakes.\n'
      'Start with reports/PENDING_FAILSAFE_ENGINEERING_20261015.md.\n')
    z.writestr('validation_manifest.json',json.dumps(manifest,indent=2)+'\n')
    for p in paths:z.write(p,p.relative_to(root).as_posix())
with ZipFile(out) as z:
    assert z.testzip() is None
print('Evidence bundle',out,'entries',len(paths)+2,'size_MB',round(out.stat().st_size/1e6,2))
