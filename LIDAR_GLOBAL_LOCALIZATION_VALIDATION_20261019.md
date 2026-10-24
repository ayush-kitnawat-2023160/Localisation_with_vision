# LiDAR map and held-out global localization check — 19 October 2026

## Result

The LiDAR-only map built from `localization_full.bag` produced an initial map lock on all three held-out recordings. Continuous tracking stayed locked for `localization.bag` and `localization_from2.bag`. `localization_3.bag` lost tracking, reacquired once, and ended in `RELOCALIZING`; it does not pass the continuous-localization check.

This is an offline matcher validation against a raw odometry map. It does not establish surveyed global accuracy or qualify `/pose` for driving.

## Map used

- Directory: `maps/campus_full_lio_226s_20261019/`
- Source: `localization_full.bag`, using the complete 225.52-second LiDAR-only FAST-LIVO2 trajectory (2,237 poses; no gaps over 0.5 seconds).
- Map database: 319 LiDAR keyframes, 407,918 points in the preview cloud, per-keyframe PCDs/poses, and cached Scan Context/local submaps.
- Matcher: LiDAR Scan Context candidate search followed by GICP, with FAST-LIVO2 odometry for local motion/tracking. Search and tracking are configured at 1 Hz; the initial lock requires two consistent registrations.
- Map coordinates are an arbitrary local frame. This map has no loop-closure/pose-graph optimization and was not aligned to surveyed coordinates.

The separate integrated camera-fusion replay of the full bag was not used to construct this LiDAR map. Its recorded trajectory had an 89.05 m/s pose-derived spike, so it is not a suitable map trajectory without further correction.

## Held-out replays

All tests used the same map and LiDAR-only acquisition/tracking configuration. Bags were played at 0.75x. The measured lock time is from the first delivered LiDAR scan to the first published global lock, so it includes scheduling and GICP work; it is not an isolated single-query benchmark.

| Recording | Bag duration | Replay wall time | First lock after first LiDAR | Bag time at lock | Follow-on accepted map fixes | Median / p95 GICP fitness | Final status |
|---|---:|---:|---:|---:|---:|---:|---|
| `localization.bag` | 99.54 s | 134 s | 3.17 s | 2.12 s | 96 | 0.062 / 0.097 | `GLOBAL_LOCKED` |
| `localization_from2.bag` | 88.33 s | 119 s | 3.41 s | 2.02 s | 85 | 0.067 / 0.098 | `GLOBAL_LOCKED` |
| `localization_3.bag` | 64.31 s | 87 s | 3.26 s | 2.12 s | 50 | 0.057 / 0.222 | `RELOCALIZING: verifying` |

The slowest initial lock was **3.41 seconds wall time** after the first LiDAR scan. All replays exited successfully. For the first two bags, the matcher accepted nearly one map update per second and reported no tracking rejections. Their final state remained `GLOBAL_LOCKED`.

On `localization_3.bag`, the matcher entered `RELOCALIZING: verifying` at 30.94 seconds of bag time, reported `GLOBAL_REACQUIRED_SMOOTH` at 34.05 seconds, and returned to `RELOCALIZING: verifying` at 63.79 seconds near the end. The log records six rejected tracking candidates, five route-continuous weak-geometry accepts, an 8.37-second maximum gap since a trusted map fix, and a recovery correction as large as 23.54 m / 134.8 degrees. The recovery publisher capped smoothing at 1.5 seconds. This is an ambiguity/continuity risk; without ground truth, the selected map segment cannot be certified as correct.

## Isolated loop-closure candidate

I tested repeated scan pairs before changing the map. Six scan-to-scan GICP constraints passed conservative overlap/correction gates. A separate SE(2) pose-graph candidate used those six loop edges plus the original sequential odometry edges; it adjusted XY/yaw only and kept the original raw map untouched. The candidate's median XY pose change was 0.084 m, 95th percentile 0.204 m, maximum 0.367 m; maximum yaw change was 0.390 degrees. The normalized graph residual fell from 0.0409 to 0.0149.

I rebuilt the candidate's Scan Context/local-submap cache and replayed all three holdouts against it. The first two stayed `GLOBAL_LOCKED`, with initial lock times of 3.07 s and 3.31 s. `localization_3.bag` still ended in `RELOCALIZING: verifying`; it entered relocalization twice, accepted 42 tracking updates, rejected nine candidates, and logged a maximum gap of 9.28 s. Its largest requested recovery ramp grew to 32.88 m / 157.6 degrees. The small pose-graph corrections did not resolve that recording's ambiguity, so the raw map remains the reference used for the primary results and the loop-constrained map remains experimental.

## GPS use

**The LiDAR global matcher is not using GPS as a coarse-location prior.** `step3_global_acquirer` subscribes to LiDAR, FAST odometry, failsafe mode, and the depth-map pose topic; it does not subscribe to `NavSatFix`. The bag player included the GPS topic for these checks, but the matcher ignored it. The map builder stores GPS samples as metadata only.

The full map bag contains 22,610 GPS samples, all with fix status `0` (no fix). Their horizontal covariance fields have a median of 2.87 m² and a 95th percentile of 9.10 m². GPS therefore cannot validate this map or provide a trustworthy coarse prior in these recordings.

## Geometry and limits

The LiDAR-only mapping trajectory spans 23.15 m vertically. Nearby XY revisits have a median absolute Z difference of 0.157 m and a 95th percentile of 0.773 m, which is an internal repeatability check only. The trajectory's relative endpoint altitude differs from the GPS relative altitude by about 14.05 m, but the GPS has no fix and is not a valid independent reference. The LiDAR–IMU/base mount values also remain physically unverified.

The matcher test supports map-frame relocalization on two bags and exposes a relocalization weakness on the third. Because the map is raw, unoptimized, and not georeferenced, successful Scan Context/GICP matches measure consistency with this map—not absolute position error. Record and inspect `/fast_livo2/step3_global_pose` during a follow-up recovery test before treating large map-anchor corrections as continuous output.

## Reproduction artifacts

- Map metadata and poses: `maps/campus_full_lio_226s_20261019/metadata.json`, `keyframes.csv`, `pose.json`.
- Mapping trajectory and vertical audit: `outputs/baseline_full_lio_only_20261019/trajectory.csv`, `outputs/looooo_integrity_20261019/localization_full_vertical.json`.
- Held-out logs and monitor events: `outputs/lidar_global_holdout_20261019/global_*_rate075_20261019/`.
- Aggregated metrics: `outputs/lidar_global_holdout_20261019/run_summary.json`; map-frame route overlay: `outputs/lidar_global_holdout_20261019/trajectory_overlay.png`.
- Separate loop-constrained candidate and its three-bag reruns: `maps/campus_full_lio_loop_candidate_20261019/pose_graph_candidate.json`, `outputs/lidar_global_candidate_validation_20261019/run_summary.json`.
- Runner: `scripts/run_lidar_global_holdout.sh`.
