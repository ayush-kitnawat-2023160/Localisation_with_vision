# FAST-LIVO2 Healthy: upper-loop fix and CPU optimisation trials — 15 Oct 2026

## Status

- **Upper-right-loop GICP dropout improved** with an isolated, opt-in C++ change. The prior frozen healthy Docker image and ordinary launch have NOT been promoted or overwritten.
- 90-second and full 290-second healthy replays finished at **1× rosbag rate** on `camera_lidar_imu_gps.bag`.
- The reported RMSE is **disagreement against an unvalidated same-bag loop-closed reference trajectory**, NOT independent ground truth. The requested 0.5 m absolute accuracy has **not** been shown.
- **No sensor-failure regression tests** or September-18 holdout validation have yet passed for this candidate. Do not deploy this experimental image to the cart.

## Root cause of the problematic ~48–51-second matching gap

At the overlapping saved-map loop, a legitimate-looking candidate is physically nearby and GICP has excellent fitness, but its stored map-keyframe ID is far from the previous ID. The previous relaxed yaw gate required nearby **IDs** even though the path loops spatially, so the candidate at approximately 49 seconds was rejected when yaw innovation was around **10.9°**, just above the normal 10° limit. Additional GICP attempts and map-reacquisition created a 2.5–3.5-second accepted-fix gap, followed by a large residual correction.

### The tested change

Generated from the frozen 2026-10-14 healthy image using `scripts/build_isolated_healthy_loop_bridge_20261015.py` (source patch saved under `step3_global/healthy_loop_motion_bridge_dev_20261015/isolated_change.patch`). Image tag:

`fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015`

The gate `tracking_healthy_loop_motion_bridge_enable` defaults **false** inside C++; only `launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch` enables it. It requires ALL of the following before allowing a GICP match across a large saved-keyframe ID jump:

1. Healthy sensor mode, valid recent independent FAST pose + prior verified map fix, no local instability or failure/recovery mode.
2. Very strong GICP geometry (fitness at most 0.095), global-prediction innovation at most **1.25 m**, yaw innovation at most **14°**.
3. The previous verified map fix is between **0.3 and 1.8 seconds** old.
4. Relative vehicle motion measured between last/current map-fixed body poses agrees with the corresponding **raw FAST-LIVO2 relative motion**, within **1.10 m XY and 8° yaw**.
5. Existing downstream accepted-match motion checks, anchor smoothing and publication limits are unchanged. No loop-closed evaluation reference or ground-truth file is read at runtime.

**Safety distinction:** This is not a global yaw-threshold increase. Geometry, small innovation and independent relative-motion agreement are all mandatory. At the original problem turn, this accepted ID 87 with GICP fitness 0.0807–0.0821 and 10.9° innovation; accepted matches continued through the next scans.

## Measured quality: 1× healthy replay

| Metric | Cached previous healthy (90s) | Loop fix (90s) | Cached previous healthy (290s) | Loop fix (290s) |
| --- | ---: | ---: | ---: | ---: |
| 3D proxy RMSE | 1.393 m | **1.365 m** | 1.463 m | **1.388 m** |
| 40–60 s loop proxy RMSE | 1.229 m | **0.973 m** | 1.228 m | **0.968 m** |
| Worst accepted-GICP gap | 3.530 s | **1.009 s** | 3.530 s | **1.815 s** |
| Max published pose step | 0.736 m | 0.747 m | 0.745 m | **0.710 m** |
| 95th percentile acceleration | 7.805 m/s² | 8.027 m/s² | 8.502 m/s² | **8.432 m/s²** |
| Accepted GICP fixes | 85 | 88 | 283 | 284 |
| Mean container CPU | 157.4% | 157.0% | 158.5% | 156.3% |

Comparison with the *older* previous-best full replay (`HEALTHY_GAP_HANDOFF_FULL_290S`) also passes: its 3D proxy RMSE was 1.453 m, worst gap 2.521 s, and maximum pose step 0.755 m. The new full run has 1.388 m / 1.815 s / 0.710 m, respectively.

Full-run worst gap moved to **279.39–281.21 seconds**, where weak geometry and yaw limits led to extra candidate checks and a verified fallback. Its downstream correction was smoothed; avoid blindly relaxing the established safety thresholds to force additional gains.

Saved evidence:

- `results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/loop_bridge_90s_score.json`
- `results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/loop_bridge_290s_score.json`
- `results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/HEALTHY_BRIDGE_290S/plots/07_orange_trajectory_on_pcd_clean.png`
- `.../HEALTHY_BRIDGE_290S/plots/06_global_pose_on_pcd_xy_zoom_loop.png`
- Raw `global_pose.csv`, `gicp_map_pose.csv`, `trajectory.csv`, `launch.log`, and `metrics_exact.txt` in both candidate folders.

## Separate CPU experiments (do not promote)

**Exact ROS sensor-adapter shallow-copy variant:** `scripts/fast_livo2_sensor_adapter_shallow_dev_20261015.py` avoids copying immutable camera and LiDAR message payloads while preserving copied headers and separate mutable LiDAR data. Exact ROS serialized bytes matched baseline for both cloud and image, and inputs were unchanged (`scripts/test_sensor_adapter_shallow_equivalence_20261015.py`). The microbenchmark of **160 messages** took **0.003 → 0.001 s for RGB**, **0.014 → 0.007 s for LiDAR**. The absolute saving is too small to claim meaningful steady-state pipeline CPU improvement. **No deployment or replay acceptance.** The original adapter remains default.

**Eight-slot LRU PCD target cache** in `launch/fast_livo2_step3_global_healthy_loop_bridge_cache_dev.launch`: 90-second replay preserved GICP gap (1.009 s), accepted-fix count (88), and maximum published step (~0.747 m). It reduced measured mean container CPU **157.0% → 153.1%** in one run (~2.5%), below the meaningful saving threshold and susceptible to scheduling variation. **Leave disabled** until repeated 1× runs show a robust gain; earlier combined 12-slot cache/affinity tuning degraded quality.

The full 290-second bridge run itself reduced container CPU by only approximately **1.4%** relative to the cached baseline; the main benefit is fewer expensive retries and improved GICP continuity in the problematic loop. Actual large sustained CPU reduction remains unachieved.

## Next validation gates before deployment

1. Repeat 290-second healthy at 1× on additional seeds/CPU loads and independently check the loop branch against map and vehicle motion.
2. Retain original Step-3/FAST ESIKF estimator semantics. Profile GICP and FAST-LIVO2 estimator hot paths before attempting any new solver/feature-quality reductions.
3. Validate camera-failed, LiDAR-failed and IMU-failed replays (full 290 s with 125–165 s outages), including transition/recovery jerk and starvation checks. This is **not** covered by healthy-only tests.
4. Validate on the independent Sept-18 holdout bag with the correct recorded calibration, and measure real wall-time / callback latency on the cart NUC before considering deployment.
