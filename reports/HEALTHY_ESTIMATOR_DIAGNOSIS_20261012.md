# Healthy FAST-LIVO2 root-cause investigation — 2026-10-12

**Result: NOT FIXED / NOT DEPLOYMENT-READY.** Controlled tests on **`camera_lidar_imu_gps.bag` only**. The 18 September holdout was not used for development or tuning.

## Method

Run FAST-LIVO2 at **1× bag playback** using the original 8 September 2026 cart recording and existing saved PCD. Score the **RAW** FAST-LIVO2 local trajectory, rigidly aligned just once over seconds 4–10, against time-matched direct GICP registrations to the fixed saved PCD. This prevents the separate `map<-FAST` anchor, periodic reacquisition, and orange published trajectory smoothing from hiding local-estimator drift. The scores are **not independent surveyed ground truth** and GICP can also be wrong where geometry is ambiguous.

## Raw horizontal pose disagreement (meters)

| Experiment | 20 s | 25 s | 40 s | 60 s | 75 s |
|---|---:|---:|---:|---:|---:|
| Baseline healthy failsafe | 5 | 7 | 5 | 1 | 1 |
| Archived standalone FAST-LIVO2 | 5 | 8 | 6 | 1 | 1 |
| Trial LiDAR–IMU T_x=+2.3m (unmeasured) | 5 | 7 | 5 | 1 | 1 |
| Trial LiDAR–IMU T_x=-2.3m (unmeasured) | 5 | 7 | 6 | 1 | 1 |
| Trial LiDAR yaw=0° (unmeasured) | 5 | 7 | 5 | 1 | 1 |
| Continuous healthy visual fusion | 3 | 7 | 4 | 1 | 1 |
| Disable early visual bootstrap | 5 | 7 | 5 | 1 | 1 |
| Online LiDAR voxel 0.8m | 5 | 7 | 5 | 1 | 1 |
| Opt-in PCD relative-heading gyro-bias ESIKF observation | 5 | 7 | 5 | 1 | 1 |

**Every tested variant fails a strict raw-odometry acceptance criterion** of <=2m at 25s and <=3m at 60s. Minor 0.1–0.3m changes are not meaningful corrections of the 8–18m drift, particularly against a map-derived proxy. The custom gyro-bias ESIKF observation compiled, ran and was logged, but did **not** materially improve raw results, so it remains opt-in and disabled by default.

## Strong diagnostic evidence

- Original, independently archived FAST-LIVO2 map-builder raw poses show essentially the same drift as the failsafe fork: **8.03m vs 7.82m at 25s**, **18.17m vs 17.62m at 60s**. Thus the failures are not solely introduced by the failsafe wrapper.
- Direct saved-map GICP heading minus single-initialization-aligned FAST local heading diverges by approximately **5° at 15s, 8° at 20s, 14° at 30s**, then changes again along the route. This is a **heading-estimation disagreement**, explaining the rapid lateral position divergence. GICP headings are themselves geometric proxies.
- Bag sensor times checked directly: LiDAR ~10Hz, IMU ~100Hz; original LiDAR per-point times about `[-0.0995, +0.0013]s`, scan header around the scan end. Adapter correctly shifts header to scan start and rescales point time to microseconds; inspected scans have their latest point in the last position. **No gross timing-unit bug verified.**
- The recorded `T_base_link_velodyne` TF has **zero translation** even though the cart sensors are physically separated. This reflects recorded TF, **not a surveyed 3D sensor extrinsic**. Neither trial of +/-2.3m along X improves matching meaningfully, so do not treat a guessed lever arm as a correction. Full calibrated 6-DOF transform is still needed.
- The original YAML includes LiDAR pitch 20° and yaw 10° from recorded TF; forcing yaw 0° also fails, and MUST NOT replace a measurement.
- In healthy mode the production failsafe configuration disables photometric visual state fusion after bootstrap. Re-enabling it at 1× causes intermittent loss of output: the 90s test has only 439 poses, 6.21m maximum pose step and 8.86m full diagnostic disagreement. This is NOT a safe shortcut to three-sensor pose fusion.
- The 0.8m online voxel and no-visual-bootstrap tests also fail to fix the underlying raw drift.

## Source / safety decisions

- `scripts/run_step3_global_case.sh`: supports explicit calibration YAML mounting (`STEP3_CALIBRATION_FILE`) and logs provenance; gyro-feedback flag is explicitly opt-in (`STEP3_GYRO_BIAS_FEEDBACK_ENABLE`).
- `scripts/score_healthy_raw_development.py`: computes **raw local** trajectory disagreement to either development loop-closed reference or verified development GICP poses; optional hard acceptance gates prevent map-frame RMSE from falsely certifying raw odometry.
- `src/step3_global_acquirer.cpp`, `src/LIVMapper.cpp`, `include/LIVMapper.h`: experimental **opt-in** PCD-heading-derived gyro-bias pseudo-observation into the existing ESIKF, default OFF; compilation and 90s replay passed, raw quality gate FAILED. This is not a production-calibrated covariance-consistent physical bias observation, and must not be promoted without further independent validation.
- The prior good image `fast_livo2_failsafe:step4-physicalreacquire-20261008` and `:dev` are unchanged. Only experimental Docker tag `fast_livo2_failsafe:experimental-pcd-gyro-bias-20261012` was built.
- **No September 18 holdout reruns or retuning**, because the development healthy acceptance gate is failed.

## What is needed for an actual fix

1. Verify the **physical LiDAR-to-IMU 6-DOF extrinsic** and IMU gyro frame/time synchronization from calibrated measurements, rather than trusting the recorded static TF's zero translation.
2. Make the local yaw observable against trustworthy geometric features: improve LiDAR scan-to-map yaw constraints and implement a verified, physically modeled map-heading measurement in the same persistent ESIKF with proper residual covariance and voxel map consistency. Do not manually overwrite trajectory/pose.
3. Validate repeated development runs, raw yaw/relative-motion error and PCD lock continuity with strict thresholds before even checking the held-out September 18 recording. Exact zero drift is not achievable with noisy unaided dead-reckoning; require bounded stable localization, not a coincidentally low end RMSE.

Original experiment outputs, exact configs, raw trajectories, logs and plot PNGs:
`results/camera_lidar_imu_gps/healthy_sensor_diagnostics_20261012/`.
