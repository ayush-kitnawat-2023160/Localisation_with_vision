# ALiVE FAST-LIVO2 localisation — CURRENT status (17 Oct 2026)

**Research/offline candidate only. NOT FINAL.**

## Preserved accepted code
- Frozen eight-case full 1x ROSBAG image:
  fast_livo2_failsafe:single-build-rc-20261016
- Clean source/result handoff:
  release/ALIVE_FASTLIVO2_OFFLINE_RC_20261016/
  ALIVE_FASTLIVO2_LOCALIZATION_ONLY_HANDOFF_20261016.zip

## 17 Oct new controlled software research
Three more *separately compiled* opt-in C++ localisation-only variants were
created and 1x ROS-replayed: recovery-map point budget, deferred sensor
time FIFO, and timestamp-bracketed SE(3) ESIKF/LiDAR alignment.

All failed at least one important cross-bag accuracy, pose jump, or timing
requirement and are NOT part of the accepted build. One identically rebuilt
frozen baseline repeat itself changed Sep18 recovery RMSE from 0.765 m
to 3.351 m, proving replay-sensitive observation/recovery behavior.

Audit / real result plots / C++ diffs:
  reports/LOCALIZATION_ONLY_COMPLETION_AUDIT_20261017.md
  results/LOCALIZATION_ONLY_COMPLETION_20261017/
  results/LOCALIZATION_ONLY_COMPLETION_20261017/
    FASTLIVO2_LOCALIZATION_RESEARCH_REJECTED_TRIALS_20261017.zip

## Unfinished engineering (do not claim otherwise)
- Incorrect/repeated-geometry depth-to-global-PCD associations.
- Variable real-time registration/recovery and FAST local pose gaps.
- All 8 full 1x sensor cases passing <0.5m diagnostic proxy RMSE,
  <0.85m steps, <0.3s gaps; independent surveyed truth.
- Automatic GPS/visual/LiDAR cold initialization.
- Real NUC runtime/calibration validation and integration as exactly
  one authoritative /pose source in the campus lanelet frame.

Actual runtime target:
  /home/devil/Desktop/car/alive-repo/latest_repo/alive-dev/

It still has a FLOAM-based global /pose localiser; it has NOT been
modified. DO NOT change cart brake, speed, steering, DBW or ROS1/ROS2
vehicle-control bridges as part of this localisation task.

## 17 Oct further IMU quarantine source investigation (not accepted)
- Isolated C++ Step-3 IMU-quarantine seed built and tested from frozen single-image base on **both recorded bags at 1×**.
- Main 190s outage proxy RMSE 3.494→1.064m, but no FAST-GUARD or seed code ever ran; **NOT a causal improvement**.
- Sep18 160s recovery20 proxy RMSE 0.765→2.229m and maximum pose gap 0.403→0.605s; **REJECTED**.
- Exact report: `reports/IMU_GUARD_SEED_TWO_BAG_REJECTION_20261017.md`.
- Archived source/diff/real ROS logs/results: `results/IMU_QUARANTINE_SEED_DEV_20261017/`.
- Live cart repo remains unmodified. The actual `config/calibration/new_cart_measured.yaml` remains locked (`calibration.ready: false`) pending physical measurements. No independent surveyed ground truth or NUC validation.
- No claim of final completion, sub-0.5m accuracy, or vehicle-readiness. The accepted offline baseline remains frozen.

## 17 Oct low-latency / compute priority (new-camera-bag follow-up)
- Priority shifted away from optimizing the currently unvalidated same-bag RMSE proxy to actual sensor-to-planner pose age, CPU/RAM, bounded callback queues and robust deterministic sensor handoff.
- Source audit: current actual live FLOAM `/pose` waits for Ceres scan-to-map optimization; destination estimator already uses queue 1 / tcpNoDelay. New Step3 publishes at FAST odometry cadence but has unbounded-seeming incoming odom backlog settings (500 msgs) and queue 5 latched dynamic pose.
- Baseline 75s 1× simulated ROS global pose age p95 67.44ms; opt-in skip unused debug work 68.05ms. CPU difference negligible.
- Opt-in Step3 queue 1/unlatched + odom queue 10/TCP_NODELAY replay (55s) p95 69.50ms versus 68.14ms baseline first 55s; no demonstrable low-load acceleration.
- Complete local test report: `reports/LOCALIZATION_LOW_LATENCY_COMPUTE_AUDIT_20261017.md`, machine metrics: `results/LOW_LATENCY_POSE_TRANSPORT_DEV_20261017/LATENCY_PROFILE_COMPARISON_20261017.json`. Probe `scripts/measure_pose_freshness.py` supports running on actual planner NUC (`--global-pose-topic /pose`) to include transport/queue lag.
- Both proposed optimizations remain dev-only. Do not merge blindly. Await next recording + synchronized actual NUC timing and physically measured extrinsics before selecting a fast global pose architecture.

## 17 Oct high-rate global pose development — offline prototype (not for cart)
- Implemented a distinct, timestamp-correct ROS1 `/fast_livo2/highrate_global_pose` publisher using the same native FAST-LIVO2 ESIKF's IMU-propagated state plus Step-3's validated map frame. **Not a second estimator; not connected to `/pose`**.
- 1× main healthy bag: 36.5 Hz high-rate, 10.0ms median/48.8ms p95 simulated-clock age versus Step3's 44.6ms/65.1ms; 0.389m max step, ~3.5% one-core CPU.
- Roundoff-safe rate limit reached ~62.7Hz on main camera-failure 82s replay (10.2ms p50/46.6ms p95). Maintained stream through camera failure, but 6 steps >0.85m (max 1.77m) during brief gaps: **continuity gate failed, not cart-ready**.
- Hardened IMU-failure recovery-time barrier: no pose stamped inside actual recorded IMU-outage interval, rejects any pre-recovery IMU source after mode clears. Output stops during 25s IMU outage: **not continuous localization**. Underlying map+ESIKF recovery remains an independent blocker.
- Report `reports/FASTLIVO2_HIGH_RATE_GLOBAL_POSE_VALIDATION_20261017.md`, status `results/HIGH_RATE_GLOBAL_POSE_DEV_20261017/FINAL_HIGH_RATE_DEV_VERDICT_20261017.json`; full code and replay evidence ZIP alongside it. 15 regression unit tests pass for hardened variant. Original frozen Docker baseline, actual cart runtime Git, planning/bridge/DBW and calibration lock unchanged.
