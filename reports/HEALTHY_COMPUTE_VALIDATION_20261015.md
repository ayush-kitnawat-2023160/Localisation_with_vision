# FAST-LIVO2 healthy compute / continuity audit — 2026-10-15

## Status / scope

- The cart production launch and Docker tag were **not promoted or overwritten**. All changes are opt-in development work; these tests do not certify deployment.
- Dataset: `camera_lidar_imu_gps.bag` at **1× playback**.
- The saved loop-closed trajectory is a **same-bag diagnostic proxy**, NOT independent ground truth. Proxy RMSE cannot prove actual cart localisation accuracy.
- The `0.5 m` user target has **not** been reached or independently verified.

## 1. Descriptor norm-caching experiment

- Modified `src/step3_global_acquirer.cpp` to calculate scan-context sector norms once per map descriptor comparison, preserving circular yaw-shift selection logic and threshold semantics.
- Compiled direct production function + historical reference with `scripts/test_scan_descriptor_fastpath.py`.
- All **400** edge/random descriptor comparisons matched within `1e-12`; selected yaw shifts matched exactly.
- Isolated C++ microbenchmark: **257.5 ms → 121.9 ms (2.11×)** in the descriptor-matching component ONLY.
- Used `scripts/build_isolated_descriptor_normcache_20261015.py` to build image `fast_livo2_failsafe:scan-descriptor-isolated-dev-20261015` from the preserved healthy image. Source diff contains **only one matcher hunk**; a separate full-tree dev image was built but is NOT an admissible A/B control.
- Paired 90-second healthy replay on one session:

  | Metric | Preserved healthy image | Isolated cached-norm image |
  | --- | ---: | ---: |
  | 3D trajectory proxy disagreement | 1.414 m | 1.393 m |
  | 40–60 s loop proxy disagreement | 1.238 m | 1.225 m |
  | Max published pose step | 0.777 m | 0.755 m |
  | Longest GICP accepted-fix gap | 3.530 s | 3.631 s |
  | Accepted GICP fixes | 85 | 85 |
  | Average container CPU | 159.4% | 157.8% |

- **Decision: quality gate passed; 10% meaningful compute reduction gate failed. Do NOT promote**. A historical run's 220.7% CPU measurement did not reproduce in the immediate baseline rerun, proving strong scheduling/run-to-run variability. Saved detailed gate: `results/camera_lidar_imu_gps/HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/ab_score.json`.

## 2. Actual process-level CPU profiler

Added `scripts/profile_step3_container_cpu.py`, which samples real host CPU-time deltas for each Docker process without modifying the container's behavior. In a 90-second baseline run, active-window average CPU as percentage of one logical core:

| Process | CPU |
| --- | ---: |
| `fastlivo_mapping` (FAST-LIVO2 estimator / mapping) | 55.4% |
| `step3_global_acquirer` (PCD/GICP) | 34.6% |
| `fast_livo2_sensor_adapter.py` | 19.3% |
| `visual_place_hint.py` | 18.4% |
| Failsafe frontend | 8.6% |

See `results/camera_lidar_imu_gps/HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/HEALTHY_PROFILE_BASELINE_90S.process_cpu.csv`. Values are workload-specific and NOT a core-independent benchmark.

## 3. Startup-only exact ORB map cache

- Added opt-in param `orb_retrieval_cache_path` to `scripts/visual_place_hint.py`, leaving the original `cv2.ORB_create(nfeatures=700,fastThreshold=12)` computation path unchanged when no cache is configured.
- Built **615** saved-image descriptor records (**409,592** raw uint8 ORB vectors) with the frozen image's OpenCV **4.2.0**. Cache size **13 MB**. Cache stores filenames, sizes, modification times, ORB settings, OpenCV version, and offsets; stale metadata triggers fallback.
- `scripts/build_exact_orb_retrieval_cache.py --verify` passed exact byte comparison on 25 sampled keyframes; `scripts/benchmark_exact_orb_cache_startup.py` passed byte-for-byte matching for **all 615** keyframes and ordering.
- Actual database build **14.857 s without cache → 0.030 s with cache** in isolated cold-process benchmark (500× startup component). This is **NOT a FAST-LIVO2 runtime acceleration**.
- Opt-in mount in `scripts/run_step3_global_case.sh` via `STEP3_ORB_CACHE=absolute/path/to/cache.npz`; the healthy gap-handoff development launch alone exposes the private ROS param. The default uses the unmodified ORB extraction path.
- Live 90-second healthy cache replay passed trajectory/map-match short gate: 1.393 m 3D proxy, 0.736 m maximum pose step, 3.530 s maximum accepted GICP gap, 85 GICP fixes, zero FAST pose-gap warnings. Baseline replay: 1.414 m proxy, 0.777 m max pose step, 3.530 s GICP gap, 85 fixes.
- Confirmed log: `[FAILSAFE PLACE] loaded exact ORB map descriptor cache: 615 frames` followed by `ready: 615 visual keyframes`.
- Important scope: healthy dev launch has `visual_place_enable_on_healthy=false`; the cache improves startup readiness and potentially degradation handoff availability, not sustained healthy CPU. No LiDAR/IMU failure regression pass claimed.
- Saved output: `results/camera_lidar_imu_gps/HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/orb_cache_healthy_90s_gate.json`.

### Complete 290-second cache-enabled healthy run (finished at 01:00:16 IST)

Full 290-second 1× rosbag replay **completed** with its original frozen healthy image, exact ORB cache opt-in, trajectory CSVs, logs, and PCD overlays. Evaluated with `scripts/evaluate_healthy_smoothness.py` against the previous stable full healthy replay. Both use the same-bag unvalidated trajectory proxy:

| Metric | Previous stable healthy 290s | Cached healthy 290s |
| --- | ---: | ---: |
| Full trajectory 3D proxy RMSE | 1.453 m | 1.463 m |
| 40–60 s loop proxy RMSE | 1.166 m | 1.228 m |
| Maximum published pose step | 0.755 m | 0.745 m |
| Acceleration p95 | 8.413 m/s² | 8.502 m/s² |
| Acceleration maximum | 23.31 m/s² | 17.47 m/s² |
| Accepted GICP fixes | 282 | 283 |
| Longest accepted GICP gap | 2.521 s | 3.530 s |
| Final proxy displacement | 0.677 m | 0.473 m |

- The full run is complete and the trajectory visibly follows the saved PCD map, but **does not improve full-run accuracy or upper-loop continuity**. The cache is still validated as a fast-start opt-in; it is NOT promoted as better localisation.
- Exact saved full comparison: `results/camera_lidar_imu_gps/HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/healthy_orb_cache_290s_smoothness.json`.
- Saved PCD overlay: `results/camera_lidar_imu_gps/HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/HEALTHY_ORB_CACHE_290S/plots/07_orange_trajectory_on_pcd_clean.png`.
- No new sensor-failure-mode acceptance achieved in this run. 0.5 m whole-route accuracy target remains unmet.

## 4. Persistent upper-loop gap diagnosis

- The 47–51 s top-right loop GICP gap also occurred in the untouched baseline replay, so it cannot be assigned solely to the descriptor caching change.
- Logs around bag timestamp `1788852619.304772` show geometrically low-fitness GICP with yaw innovation about **10.9°**, but local tracking gates did not accept a consistent candidate. Another candidate was motion-gate rejected, after which an accepted map fix came roughly **3.5 s later** with approximately **2.71 m residual**.
- Do not simply raise global yaw/fitness limits: adjacent map traversals are physically close here, and blind relaxation risks accepting the wrong lane/branch. Next effort should require robust route continuity, independently verified motion consistency, and stable output under reacquisition.

## Next acceptance criteria

1. Preserve today's full **290 s healthy** cache replay and its trajectory, GICP gaps and acceleration plots as a diagnostic, without claiming it is an improvement over the prior baseline.
2. Profile Step-3 GICP and FAST-LIVO2 estimator separately; target true throughput bottlenecks without aggressively reducing features, map resolution, or registration quality.
3. Protect the upper-loop gap fix behind new dev-only guards and test repeated 1× bags against the preserved baseline.
4. Then separately validate complete **290 s healthy / camera-fail / LiDAR-fail / IMU-fail** modes on a frozen image before checking the untouched Sept-18 holdout and before any vehicle deployment.
