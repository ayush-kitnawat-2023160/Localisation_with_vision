# ALiVE FAST-LIVO2 — real-time localization / compute investigation (2026-10-17)

**Decision:** Preserve the frozen single-build baseline. Do **not** promote either opt-in modification based on the current data. Zero changes to the actual cart code/planning/control/braking. New bag + physical NUC/ROS bridge transport still untested.

## Why the old /pose was late

Read-only source inspection of `latest_repo/alive-dev/src/fast_livo2/src/floam_global/src/fast_livo2_floam_localizer.cpp` shows it publishes authoritative `/pose` only **after** performing its LiDAR scan-to-map/Ceres processing (the published header keeps the original scan time). Estimator already has `queue=1` plus `tcpNoDelay` for `/pose`, so it cannot repair an upstream late pose. This explains a plausible path for reported delay, but actual car round-trip milliseconds were **not measured**.

New Step-3 publishes global poses from FAST local odometry callbacks, independent of the ~1Hz scan matching callbacks (`AsyncSpinner(3)`). That can keep output responsive while GICP catches up, provided ESIKF/ROS queues have not accumulated and confidence remains sound.

## 1× replay measurements (diagnostic ROS /clock freshness, not car latency)

| Measure | 75s untouched baseline | 75s opt-in debug-work guard |
|---|---:|---:|
| Global pose age p50 | 45.6 ms | 44.7 ms |
| Global pose age p95 | 67.4 ms | 68.0 ms |
| FAST mapper CPU, one-core % | 49.26% | 48.53% |
| Global matcher CPU, one-core % | 34.04% | 33.75% |
| Max global pose output gap | 0.200 s | 0.200 s |
| Max published pose step | 0.757 m | 0.747 m |

Debug-work candidate (`step3_global/low_latency_debug_dev_20261017`) skips unused `/Laser_map`, `/cloud_registered`, `/rgb_img` conversion and caps/decimates RViz `/path` (opt-in only). Real ROS replay demonstrated no meaningful p95 age or CPU reduction, so **not promoted**. No changes to the ESIKF or source clocks.

| Measure | Baseline first 55s | 55s opt-in fresh-transport candidate |
|---|---:|---:|
| Global pose age p50 | 48.3 ms | 48.0 ms |
| Global pose age p95 | 68.1 ms | 69.5 ms |
| FAST mapper CPU, one-core % | 52.15% | 52.71% |
| Global matcher CPU, one-core % | 41.02% | 42.20% |
| Max global pose gap | 0.200 s | 0.200 s |
| Max published pose step | 0.757 m | 0.741 m |

Transport candidate (`step3_global/low_latency_pose_transport_dev_20261017`) sets `/fast_livo2/step3_global_pose` ROS publisher queue to 1, disables latch on dynamic pose, limits FAST odometry subscription queue to 10, and enables TCP_NODELAY. Actual ROS parameter and C++ runtime log confirmed enablement. **No improvement in an uncongested single-container run**; this patch only protects against backlogged ROS transports; benchmark on both physical NUCs before promotion. All 6 freshness probe unit tests passed; source compiled; replay saved and plotted.

## When the new camera/bags arrive

Do not optimize against reference RMSE first; compare throughput and age and keep diagnostic pose accuracy only as a no-regression check. Probe available:

```bash
# In a ROS1 Noetic environment attached to the right ROS_MASTER_URI.
# Run on the actual NUC/planner-side to measure the ROS network arrival too.
python3 scripts/measure_pose_freshness.py --pose-only --global-pose-topic /pose --csv /tmp/alive_pose_age.csv
# Stop with Ctrl-C and summarize:
python3 scripts/summarize_pose_freshness.py /tmp/alive_pose_age.csv --out /tmp/alive_pose_age_summary.json
```

For Step3 development use `--global-pose-topic /fast_livo2/step3_global_pose`; monitor FAST local odom and published global pose simultaneously. Both devices need chrony verified and consistent hardware stamps. The live config currently defaults camera to `rebase_live` (first packet ROS receipt as time offset), so apparent packet freshness may hide camera capture/network delay; verify physical clocks before interpreting ROS header ages.

**Design direction for actual responsiveness:** IMU-rate ESIKF prediction (50–100Hz) + asynchronously corrected low-rate globally verified map anchor, only with a bounded prediction horizon, transform calibration, covariance/staleness gating and failover handling. A faster pose topic must not re-stamp old geometry as current; must not claim localization when unobservable. Do not connect it to `/pose` until replay + actual NUC tests pass and exactly one publisher owns the map-frame pose.

**Initial on-device engineering targets, not achieved/safety-certified:** measure p50/p95/p99 age on the planner machine; aim p95 <100–150ms and detect/reject age >300ms; ensure no accumulating queues over 30 minutes including camera loss and IMU/LiDAR recoveries, stable CPU and memory, no delayed pose jumps. Test new high-res camera in FULL_LIVO and all three failures at 1×. Measure actual camera capture stamps, transmission, and FPS. Independent surveyed GT and physical cart calibration remain required for cart deployment.

**Status:** Measurement tooling ready. No actual NUC latency or newest-camera performance demonstrated. Frozen baseline and `latest_repo/alive-dev` are unchanged; the new-cart physical calibration file is still locked.
