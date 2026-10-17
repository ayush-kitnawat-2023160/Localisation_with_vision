# Curated validation results

**Critical reassessment (2026-10-12): NOT deployment-qualified.** The 290 s
map-frame agreement figures below mask significant drift in the unanchored
FAST-LIVO2 estimator. On the ORIGINAL `camera_lidar_imu_gps.bag`, the healthy
raw trajectory differs from direct saved-PCD/GICP registrations by roughly
7.8 m at 25 s and 17.6 m at 60 s after aligning 4–10 s. A preserved standalone
FAST-LIVO2 map-builder run shows similarly large raw disagreement. These are
proxy comparisons, **not independent ground truth**, but they invalidate the
previous description of the healthy mode as a stable localization baseline.
See `../healthy_sensor_diagnostics_20261012/`,
`../../../reports/HEALTHY_ESTIMATOR_DIAGNOSIS_20261012.md`, and the full
experimental failure report
`../../../reports/HEALTHY_ESTIMATOR_FIX_ATTEMPT_20261012.md`.

This directory intentionally contains only the current accepted results, the most useful LiDAR baseline/milestone, and the latest cross-bag diagnostics. Failed tuning sweeps and duplicate historical runs were removed during repository cleanup.

## Primary 290 s validation

| Run | Overall 3D RMSE | Overall max | Failure-window RMSE | Failure-window max | Final error |
| --- | ---: | ---: | ---: | ---: | ---: |
| `FINAL290_HEALTHY_BOUNDEDPLACE_20261006` | 1.384 m | 5.142 m | — | — | 0.462 m |
| `FINAL290_CAMERA_FAIL_BOUNDEDPLACE_20261006` | 1.279 m | 4.954 m | 0.997 m | 2.975 m | 1.240 m |
| `FINAL290_LIDAR_FAIL_BOUNDEDPLACE_20261006` | 1.631 m | 4.195 m | 2.194 m | 3.859 m | 0.484 m |
| `FINAL290_IMU_FAIL_BOUNDEDPLACE_20261006` | 2.742 m | 16.651 m | 1.475 m | 4.673 m | 0.881 m |

The primary `camera_lidar_imu_gps.bag` Step-1/2/3 validation is the accepted current result set. For failure runs the injected outage is 125–165 s.

## LiDAR-failure development evidence

| Run | Failure RMSE | Failure max | Failure-end error |
| --- | ---: | ---: | ---: |
| `LIDARFIX_V9_SMOOTHRATE_FULLAUTH_180S_20261005` | 2.945 m | 6.991 m | 4.207 m |
| `LIDARFIX_BOUNDEDPLACE_180S_20261006` | 2.592 m | 3.942 m | 2.782 m |

`LIDAR_FAILURE_COMPARISON_20261006.txt` contains the detailed V9-to-bounded-place comparison.

## Cross-bag diagnostics

The retained `SECOND_18SEP_*` runs are **not** accepted Step-4 validation. They are kept because they are the latest evidence of what still fails on another bag.

| Run | Overall 3D RMSE | Overall max | Failure RMSE | Final error |
| --- | ---: | ---: | ---: | ---: |
| `SECOND_18SEP_HEALTHY_220S_20261006` | 16.789 m | 36.149 m | — | 32.432 m |
| `SECOND_18SEP_CAMERA_FAIL_220S_20261006` | 1.157 m | 9.620 m | 0.391 m | 0.046 m |
| `SECOND_18SEP_LIDAR_FAIL_220S_20261006` | 5.675 m | 24.343 m | 5.061 m | 0.089 m |
| `SECOND_18SEP_IMU_FAIL_220S_20261006` | 7.866 m | 33.804 m | 3.113 m | 0.089 m |

Each retained run includes `metrics_exact.txt`, raw CSV/log outputs, and plots. The presentation-friendly trajectory is `plots/07_orange_trajectory_on_pcd_clean.png`.
