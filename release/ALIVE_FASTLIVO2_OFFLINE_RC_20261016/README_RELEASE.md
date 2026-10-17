# ALiVE FAST-LIVO2 clean code snapshot + offline verification (October 16, 2026)

Single-image release candidate. NEVER autonomous deployment approved.
Source code/launch/config files copied from the exact frozen Docker image and runtime mounts.
Every mode was replayed in ROS at 1x using the same Docker image and parameter setup.
Both references are saved-map proxies. They are NOT independent surveyed ground truth.

| Recording | Mode | New proxy 3D RMSE | Hist. cherry-picked best | New outage RMSE | New recovery RMSE | Max step | Max gap |
|---|---|---:|---:|---:|---:|---:|---:|
| main | HEALTHY | 1.374m | 1.388m | — | — | 0.737m | 0.200s |
| main | CAMERA_FAIL | 1.408m | 1.568m | 1.0267 | 1.0217 | 0.852m | 0.200s |
| main | LIDAR_FAIL | 2.000m | 1.534m | 3.2239 | 1.929 | 1.934m | 0.201s |
| main | IMU_FAIL | 2.149m | 1.594m | 3.4939 | 2.3166 | 1.539m | 0.200s |
| sept18 | HEALTHY | 1.925m | 1.237m | — | — | 0.627m | 0.234s |
| sept18 | CAMERA_FAIL | 1.326m | 1.194m | 1.6134 | 0.5273 | 0.777m | 0.234s |
| sept18 | LIDAR_FAIL | 1.611m | 1.591m | 3.9423 | 1.3698 | 1.565m | 0.403s |
| sept18 | IMU_FAIL | 1.457m | 1.374m | 2.5572 | 0.7655 | 0.703m | 0.403s |

Gate failures: 14.
Original best-so-far 8 runs remain untouched.
This snapshot EXCLUDES external bag files, raw saved map, 800MB visual image keyframes.
Asset paths and Docker hash are in manifest. Full raw runs remain in results/FINAL_SINGLE_IMAGE_RC_20261016.
Do not use this offline localisation candidate as a validated real-time navigation output.
The corrected September 18 baseline has no artificial right-side triangle.
