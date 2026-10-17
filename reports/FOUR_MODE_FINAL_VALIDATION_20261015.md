# FAST-LIVO2 localisation failsafe — four-mode regression and holdout audit

**Date:** 15 October 2026, Asia/Kolkata.  **Status:** Experimental. **NOT approved for deployment.**

## Exact frozen candidate tested

- Docker image: `fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015`
- Launch: `launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch`
- Original `camera_lidar_imu_gps.bag` and saved 15 cm visual / LiDAR map; original recorded-cart calibration.
- 1× playback, 290 seconds per mode, 40-second injected sensor outages at 125–165 seconds.
- Opt-in exact ORB database cache is built once from 615 immutable source map images, matched byte-for-byte against the original ORB extraction; original FAST-LIVO2 ESIKF preserved.
- `Step3` cross-keyframe loop-motion gate is opt-in and isolated. Healthy output only. No production image / runtime launch overwritten.
- Metrics are disagreement against a **same-bag loop-closed trajectory proxy**, not independently surveyed ground truth. Independent `0.5 m` accuracy is neither demonstrated nor verifiable from these data.

## Full development results (1×, 290 s each)

| Mode | Full-run 3D proxy RMSE | 40 s failure proxy RMSE | 20 s post-failure proxy RMSE | Largest published step | Result |
|---|---:|---:|---:|---:|---|
| Healthy | **1.388 m** | Not applicable | Not applicable | **0.710 m** | Healthy quality gate passed vs previous stable healthy, but not 0.5 m target |
| Camera failed | 1.568 m | 1.035 m | 1.426 m | 0.837 m | Failure/recovery works; camera recovery disagreement regressed against prior baseline |
| LiDAR failed | **22.398 m** | **6.142 m** | **14.713 m** | **1.831 m** | **FAILED: excessive drift, blocked global reacquisition** |
| IMU failed | 1.594 m | 1.818 m | **1.102 m** | 1.193 m | Recovery improved vs October 13, but abrupt pose steps remain |

The full runs preserved pose messages through each outage. However, a message stream remaining alive is **not** proof of a trustworthy global pose. All-mode validation and vehicle safety gates fail; do not publish these as a successful sensor-robustness benchmark.

## Root cause: LiDAR dropout

1. The simulator confirms `FULL_LIVO -> VI_LIDAR_FAILED` at 125 s, then `VI_LIDAR_FAILED -> IMU_ONLY_BRIDGE` after approximately **5 s** when visual confidence becomes weak, despite available camera/depth sensor data. Earlier October-13 preserved code held `VI_LIDAR_FAILED` around **15 s**, and its failure-window RMSE was **2.17 m**, though it also had excessive jumps.
2. During the 40-second outage, weak visual relative odometry and IMU-only propagation accumulate increasing horizontal/vertical drift. Depth-to-saved-map matches alone fail to pull the publisher back to a verified trajectory; the error grows beyond 10 m by the end of the outage.
3. On LiDAR recovery the filter resumes `FULL_LIVO`, but Step-3 global reacquisition repeatedly rejects plausible geometry because the difference in vertical map position is approximately **8–9 m** while the current physical gate allows **3 m**. The global output never recovers properly within the full run.
4. Prior experiments which simply relax the vertical gate (`RECOVERY_MAPZ_DEV_20261014`, 3D proxy RMSE ~17 m) also fail. **Do not remove the gate or overwrite poses with unverified ICP matches.** Fixing this requires confidence-aware relocalisation with temporal/geometric verification, proper estimator reset/anchor continuity, and a bounded published correction or safe stop.

## Root cause: camera/IMU remaining jumps

- Camera failure correctly switches to `LIO_CAMERA_FAILED` and back; immediately after recovery the map matcher observes ~20–22° yaw innovation, so correction is distributed over time. Post-recovery proxy RMSE **1.426 m**, versus **0.823 m** on October-13 baseline, is a regression.
- IMU failure correctly switches to `LV_IMU_FAILED` and back. Full-run RMSE **1.594 m** improved versus October-13 **1.779 m**, recovery window **1.102 m** versus **3.776 m**, but maximum published step **1.193 m** does not satisfy a strict smoothness gate.
- Goal remains smooth, bounded, geometrically consistent global pose and automatic initialisation without reintroducing manual offsets or synthetic plot shifts.

## Runtime compute: measured limits

- Exact startup ORB cache: **14.86 → 0.03 s** for the saved-image ORB database build; **not** a steady-state whole-pipeline speedup.
- Isolated descriptor norm-caching: ~**2.1× matcher-only speedup** but around **1% whole-pipeline CPU reduction**, rejected for production complexity.
- Eight-slot PCD target LRU: **157.0% → 153.1%** container CPU in one 90 s test; ~2.5% and sensitive to scheduler variability, **not promoted**.
- Current full healthy loop-motion gate run used ~**156.3%** CPU versus ~**158.5%** in paired cached healthy comparison, within repeated-run variability. Goal of meaningful lower CPU/latency has **not** been met.
- Earlier per-process CPU profile identifies FAST-LIVO2 mapping (~55% of one logical core) and Step-3 GICP (~35%) as larger costs than descriptor comparison. Any further estimator/GICP optimisation requires an isolated build, 1× full-loop tests, all failure modes, and runtime latency measurements on the NUC.

## Development evidence

- `results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/COMPARISON_BRIDGE/`
- `results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/four_mode_readiness_audit.json`
- Individual mode directories contain `metrics_exact.txt`, `launch.log`, `global_pose.csv`, `gicp_map_pose.csv`, PCD map overlays and step-continuity plots.
- `scripts/validate_loopbridge_four_modes_and_holdout_20261015.sh` runs all modes sequentially and refuses to delete old results outside its own experimental output folders. The terminal session controlling the batch disconnected during the final holdout run; the underlying Docker replay completed and the final IMU evaluation/plots and four-mode comparison were regenerated manually from the saved pose streams. This is documented rather than silently treated as an uninterrupted batch process.

## September 18 independent holdout

Running the exact frozen image on `18_sep_2026_combined.bag` with the saved October-9 **corner-fixed, held-out LiDAR/PCD** proxy, 1× playback, 220 seconds per mode, 90–130 s sensor outages. No holdout calibration, PCD map or loop-closed proxy is regenerated or tuned. **Holdout results to be appended only after complete outputs are verified.**

### Healthy holdout (completed at 01:58:43 IST)

- **2,141 published samples**, 217.13 s recorded pose duration; complete 220-second 1× rosbag run.
- 3D discrepancy to frozen corner-fixed LiDAR/PCD proxy **1.914 m RMSE**, maximum discrepancy **9.471 m** around 75.65 s, max consecutive published pose step **0.783 m**.
- At the problematic turn, a GICP candidate was rejected by motion consistency; after a gap, route reacquisition accepted a candidate with **9.07 m XY/3D innovation** and ~21° yaw innovation. The anchor published smoothed corrections, but the large transient remains and the 0.5 m target is not met.
- Fair comparison: October 8 `STEP4_PHYSICAL_HEALTHY_20261008` re-evaluated against the **same** corrected reference has **1.237 m RMSE** and max step **0.842 m**. Current candidate's published steps are slightly lower, but accuracy/generalisation is worse. October 7 `FINAL_HEALTHY_220S_20261007` was 5.258 m proxy RMSE; the choice of baseline matters.
- Independently recomputed historical comparisons are frozen under `results/18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/reference_aligned_historical_comparisons.json`.
- **Decision: holdout healthy localisation not accepted.** The September 18 reference remains a LiDAR/PCD proxy, not surveyed ground truth.

### Camera-failure holdout (completed at 02:02:44 IST)

- `CAMERA_FAIL_HOLDOUT_220S`: **2,142 samples**, 217.13 s pose duration, 3D proxy RMSE **1.194 m**, failure-window (90–130 s) RMSE **1.838 m**, first 20-second recovery-window RMSE **0.547 m**, max consecutive published step **0.815 m**.
- The ROS status changed `FULL_LIVO -> LIO_CAMERA_FAILED -> FULL_LIVO` at the intended injected outage/recovery points. The stream survived, but neither independent localisation accuracy nor a 0.5 m bound is established.
- **Repeatability warning:** even before the camera-failure window begins, the healthy and camera-failure replays diverged dramatically at the 40–80 s bend: healthy replay RMSE **3.343 m**, pre-camera-failure replay RMSE ~**0.320 m**. Both had all sensors healthy before 90 s. Therefore the runs are sensitive to scheduling/GICP matching timing; the lower overall camera-failure RMSE is NOT evidence that removing the camera improves the estimator. Repeated paired seeds, CPU and callback-latency tests are mandatory.

### LiDAR-failure holdout (completed at 02:06:46 IST): critical failure

- `LIDAR_FAIL_HOLDOUT_220S`: **1,906 samples**, 217.13 s pose duration, full 3D proxy RMSE **17.813 m**, failure-window 90–130 s **17.211 m**, maximum published step **10.683 m**; **REJECT**.
- The worst published step occurred **90.35–90.62 s**, immediately as LiDAR dropped, moving from approximately [71.60, −18.99, −3.58] m to [75.67, −9.25, −1.92] m in 0.27 s. This is a **failure-handoff discontinuity** rather than normal vehicle motion.
- Maximum proxy error exceeded **61 m** around 142.3 s, after LiDAR resumed. Recovery must not be considered successful just because `FULL_LIVO` resumes.
- Unlike the original development bag, visual mode **remained active for the entire 40 s LiDAR outage**, yet the map-frame trajectory diverged severely. This proves that modifying the IMU-only fallback timer or relaxing global reacquisition's vertical bound alone is insufficient. The visual-map confidence/registration and the motion handed to the global publisher also require independent geometric and temporal verification.
- Stage 1 safe completion requires a system-level **localization-untrusted / safe stop** handoff to planning/DBW when no consistent map position can be substantiated. Publishing smooth but incorrect global poses without a safety acknowledgement is not acceptable. No car DBW change has been made by these tests.

### IMU-failure holdout (completed at 02:10:45 IST)

- `IMU_FAIL_HOLDOUT_220S`: **2,137 pose samples**, 217.14 s publication duration; 3D proxy RMSE **1.374 m**, outage 90–130 s RMSE **2.323 m**, first 20 s after recovery RMSE **0.751 m**, largest published pose step **0.669 m**.
- Sensor transitions `FULL_LIVO -> LV_IMU_FAILED -> FULL_LIVO` occurred correctly, and the published trajectory was smoother than the LiDAR-failure case. However, the audit detected a pose-stream gap longer than 0.5 s; smoothness and availability gates therefore do NOT both pass.
- Same-corrected-proxy historical comparison: October 7 `FINAL_IMU_FAIL_220S_20261007` had **7.020 m** 3D RMSE and **2.991 m** max step; the current candidate improves materially but still misses 0.5 m target and complete reliability requirements.

### Complete September holdout quality matrix

| Holdout case | 3D proxy RMSE | Failure-window RMSE | Post-failure 20 s RMSE | Max step | Safety gate |
|---|---:|---:|---:|---:|---|
| Healthy | 1.914 m | — | — | 0.783 m | FAIL: proxy target / >9 m transient |
| Camera failed | 1.194 m | 1.838 m | 0.547 m | 0.815 m | FAIL: 0.5 m proxy target / replay nondeterminism |
| LiDAR failed | **17.813 m** | **17.211 m** | severe | **10.683 m** | **CRITICAL FAIL: localisation, jumps, pose gap** |
| IMU failed | 1.374 m | 2.323 m | 0.751 m | 0.669 m | FAIL: target / pose gap |

All 8 development + holdout replays have finished and have evaluations and PCD overlays. See
`results/18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/COMPARISON/`:
`comparison.csv`, `summary.json`, `all_four_modes_disagreement_vs_time.png` and one PCD overlay per mode. The final strict audit is at
`results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/four_mode_readiness_audit.json`, and explicitly marks `production_deployment_approved=false`.

### Additional isolated, slope-aware LiDAR recovery trial — NOT validated

- Built isolated image `fast_livo2_failsafe:slope-recovery-verified-dev-20261015`, from the **frozen loop-motion-bridge** image. Source and launch in `scripts/build_isolated_slope_recovery_dev_20261015.py`, `step3_global/slope_recovery_verified_dev_20261015/`, and `launch/fast_livo2_step3_global_slope_recovery_dev.launch`. The production image/launch remain untouched.
- Rationale: the development route's reference elevation changes about **3.5 m** over the 40-second LiDAR outage. A fixed 3 m vertical offset gate comparing to the PRE-outage map position can wrongly reject true map fixes. The new opt-in exception proposes a bounded **10% grade (+1 m tolerance, at most 12 m)** only after critical sensor recovery and when GICP fitness is at most **0.075**, keeping the independent XY/yaw checks and multi-scan temporal verification.
- Ran 220 s original bag at 1× with LiDAR outage 125–165 s: full 3D proxy RMSE **2.065 m**, outage RMSE **3.187 m**, first 20 s after recovery RMSE **3.507 m**, max published step **2.261 m**.
- **Critical controlled-test outcome:** logs contain **zero `STEP3 SLOPE-RECOVERY` acceptances**. This run retained `VI_LIDAR_FAILED` for the full 40 seconds rather than entering `IMU_ONLY_BRIDGE`, unlike the initial failed 290-second replay. Therefore this improvement in whole-run RMSE is attributable to different failover/timing behaviour, NOT demonstrated benefit from the slope gate. **REJECT / DO NOT PROMOTE**. The same unresolved trajectory jumps and weak visual-map reliability remain.
- Saved experimental evidence: `results/camera_lidar_imu_gps/SLOPE_RECOVERY_VERIFIED_DEV_20261015/LIDAR_FAIL_SLOPE_220S/` and `validation_status.txt`. Do not mix this with the frozen eight-run comparison.

### Final engineering disposition

**Safe status: NOT COMPLETE / NOT DEPLOYABLE.** The requested all-mode smoothness, <0.5 m absolute accuracy, sensor-failure correctness, independent repeatability and meaningful steady-state CPU reduction are not yet demonstrated. Do not merge any experimental C++ gate into the live golf-cart node or claim live sensor robustness. Existing production image/configuration remain unchanged. Required next stage is confidence/temporal verification of visual-map absolute poses, a fail-closed planner/DBW response to invalid localisation, legitimate bounded LiDAR relocalisation, repeated paired run latency measurements on the NUC, and independently validated ground truth for claimed accuracy.

## Deployment decision

**REJECT** until all of the following are independently verified: no unverified global pose during a prolonged loss of visual/LiDAR confidence; accurate re-localisation after 40-second outages; no large pose steps on recovery; real hardware replay latency and worst-case load; correctness on September 18 holdout; proper fail-safe stop/slowdown integration; and an independently validated reference if claiming an accuracy threshold such as 0.5 m.
