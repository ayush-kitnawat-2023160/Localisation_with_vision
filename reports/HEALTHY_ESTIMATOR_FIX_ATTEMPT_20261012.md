# Healthy FAST-LIVO2 repair attempts — 12 October 2026

## Status: NOT FIXED / DO NOT DEPLOY EXPERIMENTS

All experiments use only `camera_lidar_imu_gps.bag` at 1x playback. No retuning or testing on the held-out September 18 bag. Independent surveyed ground truth is unavailable. Diagnostics are raw-trajectory disagreement with same-bag direct LiDAR registration to the existing saved PCD, after one SE(2) alignment over 4–10 seconds; these numbers are proxy disagreements and cannot prove absolute localization accuracy.

| Development candidate | Raw XY disagreement, t=25 s | t=60 s | Conclusion |
|---|---:|---:|---|
| Original healthy FAST-LIVO2 | 7.82 m | 17.63 m | Fails |
| Archived standalone FAST-LIVO2 map builder | 8.03 m | 18.17 m | Same underlying drift |
| Zero-yaw calibration trial | 7.82 m | 17.66 m | Rejected, unmeasured extrinsic |
| LiDAR–IMU translation +/-2.3 m | 7.59–7.98 m | 17.49–17.70 m | Rejected, unmeasured lever arm |
| Continuous photometric camera pose fusion | 7.67 m | 16.26 m | Unstable: 6.21 m published step; rejected |
| Local LiDAR voxel 0.8 m | 7.75 m | 17.44 m | Not meaningful |
| Remove visual bootstrap | 7.96 m | 17.78 m | Rejected |
| PCD-derived gyro-bias ESIKF measurement | 7.76 m | 17.49 m | Mechanically applied, not effective |
| Dual prior global geometric voxel map | 7.76 m | 17.95 m | Prior support decays; not effective |
| Fixed-initial-anchor PCD measurement into ESIKF | 7.75 m | 17.36 m | 67 bounded map corrections accepted but raw drift persists |
| Gyro/accel covariance reduced 10x | 8.06 m | 18.05 m | Rejected |
| Gyro/accel covariance reduced 100x | 8.02 m | 18.14 m | Rejected |
| Replace live local voxels with immutable PCD | **55.25 m** | Not completed | **Stopped early**: loss of scan-to-map constraints, 0 matched features during sustained periods |

## Diagnostic conclusions

1. LiDAR per-point timing: recorded [-0.0995, +0.0013] seconds over ~100 ms scan; adapter correctly shifts to scan start. IMU rate ~100Hz and LiDAR ~10Hz. No gross timing-unit mistake found.
2. Recorded `T_base_link_velodyne` zero translation comes from a static TF, **not** calibrated sensor centers. Physical LiDAR-to-IMU 6-DoF transform remains unverified. Guessed offsets cannot be deployed.
3. Relative yaw disagreement to verified PCD geometry grows to ~8 degrees at 20s and ~14 degrees near 30s; MAVROS IMU orientation is closer early but itself diverges during major turns. The saved PCD registration is a proxy, not absolute truth.
4. Raw FAST-LIVO2 and archived standalone map-builder drift similarly. This is not simply a failsafe plotting or map-frame anchor bug.
5. When injecting verified PCD pose corrections directly into the ESIKF while leaving the accumulated live local voxel map unchanged, the next local scan-to-map optimization pulls the filter back toward its existing self-consistent but drifting geometry. Updating the output anchor alone cannot repair raw odometry. Full immutable-map replacement currently lacks adequate per-scan plane/voxel correspondences and diverges, so blindly switching maps is unsafe.
6. A truly stable solution must keep the estimator and geometric map internally consistent, use verified physical extrinsics, and enforce localization confidence / stop-driving behavior when the map geometry cannot support global registration. Exact zero error is physically unrealistic with noisy sensors; use grounded safety and error bounds.

## Software and safety

- Stable Docker tags `fast_livo2_failsafe:dev` and `:step4-physicalreacquire-20261008` remain unchanged.
- Experimental tags include `experimental-pcd-gyro-bias-20261012` and `experimental-fixed-anchor-esikf-20261012`. The experimental measurement paths are **disabled by default** and MUST NOT be used on the vehicle.
- Replay runner now supports explicit calibration and prior-map file mounting, with configuration provenance preserved.
- Raw-odometry quality gate in `scripts/score_healthy_raw_development.py` remains available; it rejects all tested variants against 2m @25s / 3m @60s development-proxy thresholds.
- Dangerous replacement-prior 90s test was terminated early following 55m proxy error and zero map constraints.

## Required next engineering change

Identify a calibrated sensor geometry and external trusted motion reference, then implement map-consistent external measurement fusion (including the voxel-map coordinate correction or rebuilding appropriate local geometry) and evaluate raw motion/heading and published map pose over full 290-second development runs, with strict continuity and no recovery gaps. Do not validate held-out September 18 until a development candidate passes.
