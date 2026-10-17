# ALiVE FAST-LIVO2 — experimental low-latency global pose output

**17 October 2026 | Implemented and tested in the development repository | NOT car-ready or promoted to `/pose`**

## Implemented

A separate ROS1 localization node, `fast_livo2_highrate_global_pose.py`, subscribes to the native **single FAST-LIVO2 ESIKF's existing IMU-propagated local odometry** (`/LIVO2/imu_propagate`), its ordinary LIO/VIO state (`/aft_mapped_to_init`), and the existing Step3 corrected global pose (`/fast_livo2/step3_global_pose`). It constructs `T_map<-local = T_map<-body(timestamp) * inverse(T_local<-body(same timestamp))` and publishes `T_map<-local * T_local<-body(imu timestamp)` as `/fast_livo2/highrate_global_pose`. This does **not** add GICP work or a second ESIKF, and preserves the incoming original sensor timestamp. It does not touch FLOAM, the cart planner/bridge, speed, braking or ROS controller code.

**Fail-closed controls:** no map lock = no output; timestamp-matched ESIKF/global anchor required; current IMU state must agree numerically with native ESIKF at co-timed points (frame-name alias alone is not sufficient); maximum source/global/local age; plausible relative velocity/turn; no output in LV_IMU_FAILED/IMU_ONLY_BRIDGE; on recovery clear all pre-failure state and require strictly new source timestamps and two fresh ESIKF comparisons; bounded subscriber/outgoing queues. No stale pose is relabelled `now`.

**Sources (host relative to FAST-LIVO2-failsafe):**
- First implementation + real healthy/camera/IMU trials: `step3_global/highrate_global_pose_dev_20261017/`
- Rate-limit floating timestamp tolerance (14 tests, real camera failure replay): `step3_global/highrate_global_pose_rate_fix_dev_20261017/`
- Hardened IMU-recovery source-time barrier (15 tests, actual IMU-failure replay): `step3_global/highrate_global_pose_mode_barrier_dev_20261017/`
- Launch `launch/fast_livo2_highrate_global_pose_dev_20261017.launch`, dev-only config `step3_global/highrate_global_pose_dev_20261017/config_highrate_rosbag.yaml` turns on native uav/imu_rate_odom so the original 1x replay can drive this node.
- Read-only independent ROS topic age monitor: `scripts/measure_pose_freshness.py`. Evaluation: `scripts/evaluate_highrate_pose_20261017.py`.

## Measured 1x main-bag results (in ROS /clock container, NOT remote NUC/planner latency)

| Scenario | Original Stage3 map pose age p50/p95 | New high-rate pose age p50/p95 | New output rate | Max contiguous pose step |
| --- | --- | --- | --- | --- |
| Healthy first source build, 90s | 44.6/65.1 ms | 10.0/48.8 ms | 36.5 Hz | 0.389 m (max gap 0.090s) |
| Camera fail, with rate rounding fix, 82s | 44.4/67.2 ms | 10.2/46.6 ms | 62.7 Hz | 1.773 m (max gap 0.260s) |
| IMU failure + hardened time barrier, 105s | 35.3/56.1 ms | 10.5/40.0 ms when available | Not continuous | Withheld for full IMU failure |

Healthy 90s: 3200 published high-rate observations, no steps above 0.85m, Step3 original map reference remained in its ordinary independent topic, per-process extra high-rate Python CPU approximately 3.48% of one logical core. Closely matched the 10Hz source at time-matched poses: median difference 0.029m, p95 0.108m. This compares two outputs of the same system, not survey GT.

Camera-fail (35s to 55s injector, actual mode transitions logged): `LIO_CAMERA_FAILED` published 879 actual messages through the outage, but the higher-rate dev variant still recorded **6 steps >0.85m**, including 1.773m. Most followed ~0.2s blank intervals and large local corrections. Extra publisher CPU ~3.67% of one logical core. **Fails continuity acceptance; cannot feed planner.** Increasing Hz alone cannot repair local ESIKF correction aliasing.

IMU fail 40s..65s injector: **zero poses with source timestamps inside the actual `[FULL_LIVO -> LV_IMU_FAILED, LV_IMU_FAILED -> FULL_LIVO)` interval** (audited from ROS mode transitions), and no pre-recovery IMU source stamps escaped after recovery. High-rate output is intentionally absent for ~25s: it is NOT a functioning IMU-independent planner localization output. Large discontinuity across the gap reflects vehicle motion while no pose was published; in addition, short-interval jumps remain possible. Step3's own failure estimator was unreliable in some replays and cannot be corrected by transport acceleration.

## Performance architecture decision

The FAST-LIVO2 ESIKF remains the sole state estimator, ~1Hz GICP verified map fixes remain asynchronous, and map-frame high-rate **prediction** works in healthy and camera-outage cases at the observed 36-63Hz. A source-clock recency test and cross-frame checks prevent false freshness. This is a functional offline prototype, not a deployable control input.

Before assigning authoritative `/pose`: constrain/correct sudden ESIKF/map-anchor handoffs in degraded modes, provide verified map-based fallback through IMU failure without lying about current pose, prove full healthy + three outage modes on BOTH old and new-camera bags, test CPU and synchronized sensor-to-planner age on actual dual NUCs, measure physical extrinsics and independent cart error. New camera calibration and time stamps cannot be guessed. Cart configuration remains locked.

**Preserved:** immutable image `fast_livo2_failsafe:single-build-rc-20261016` (sha256:9fda983a4dd504c4e52d672f257d3e4ad703fd72e8e8c9ce24bfc0aeb33abe48); actual `latest_repo/alive-dev` Git status clean; no control code edits; no automations or physical driving commands. Final disposition **DEV PROTOTYPE, NOT PROMOTED**.
