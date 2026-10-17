# LiDAR-failure localisation experiments — 16 October 2026

**Outcome: all trial implementations REJECTED. Existing best retained. No deployment.**

All four trials used the same main rosbag at **1×**, with the same 40s LiDAR outage
(125–165s). The comparison window is [0,185]s for every variant.
Every RMSE is disagreement with the same-bag loop-closed trajectory **proxy**.
It is **not independent ground truth** and cannot establish field accuracy.

| Trial | Outage RMSE | Recovery RMSE | Max pose step | Decision |
| --- | ---: | ---: | ---: | --- |
| Frozen curated Oct 13 best | **2.170 m** | **1.317 m** | 2.276 m | Still NOT deployment-safe |
| Enable local tracker for 2.2s | 8.710 m | 8.630 m | 1.698 m | **REJECT** |
| ORB + RGB-D keyframe PnP | 3.909 m | 2.079 m | 1.763 m | **REJECT** |
| Visual voxel + LiDAR PCD XYZ blend | 3.570 m | 1.484 m | 2.181 m | **REJECT** |
| Visual voxel + LiDAR PCD veto only | 2.575 m | 1.030 m | 1.998 m | **REJECT** |

## Concrete findings

1. The old map-localizer config had `depth_tracker_max_age_sec: 0.0`,
   disabling its bounded local RGB-D saved-map tracker. Enabling a
   2.2-second window yielded some valid-looking local matches but
   suppressed independent global reacquisition and drifted badly.
2. Enabling opt-in depth-backed ORB/PnP produced hundreds of geometric
   image inliers but worse pose accuracy and recovery under this CPU load.
3. In the frozen Step-3 launch, both `map_path` and `visual_map_path`
   pointed to `/visual_map.pcd`. In the depth-failover-only frontend,
   the geometric `map_` was not loaded at all. A genuinely independent
   crossmap test required BOTH an isolated launch correction AND opt-in
   PCD loading in the frontend. These are fixed ONLY in dev artifacts.
4. Independent canonical saved-LiDAR-PCD GICP confirmed 29 visual-map
   fixes and vetoed 5 geometry disagreements in the dual-map blend
   trial, but injecting 75% of its XYZ made positional error worse.
5. The final veto-only crosscheck retained 35 matches, rejected 5,
   and improved post-recovery pose/step continuity, but had **worse
   outage RMSE**. It must not be promoted.
6. The earlier timestamped independent depth registration audit
   showed 34 accepted raw saved-map fixes with **2.014m** 3D RMSE
   during outage; median transport delay only **0.084s**. Repeated
   road/map geometry, inaccurate absolute matches, and insufficient
   map-geometry observability are now the limiting factors—not
   primarily a missing publication smoother or processing delay.

## Files and reproducibility

- JSON audit: `reports/LIDAR_FAILURE_REGISTRATION_TRIALS_AUDIT_20261016.json`
- Baseline: `results/BEST_SO_FAR_20261015/camera_lidar_imu_gps/LIDAR_FAILED/`
- Local trial: `step3_global/depth_temporal_tracker_trial_20261016/`
- Feature-PnP trial: `step3_global/depth_feature_pnp_trial_20261016/`
- Dual-map blend: `step3_global/depth_crossmap_trial_20261016/`
- Veto-only crossmap: `step3_global/depth_crossmap_veto_trial_20261016/`
- Isolated launch: `launch/fast_livo2_step3_global_dual_map_dev_20261016.launch`
- Isolated image: `fast_livo2_failsafe:depth-crossmap-veto-only-dev-20261016`
- Builders: `scripts/build_isolated_depth_crossmap_20261016.py` and
  `scripts/build_depth_crossmap_veto_only_20261016.py`
- Saved PCD maps and camera/IMU original calibration were not fitted
  to the reference; no production launch/config/source was changed.

## Next research/engineering milestone

Do not accept a map match from just GICP fitness, overlap, or PnP
inlier count; they are insufficient on repeated-road geometry.
Develop a confidence-calibrated, well-conditioned relative-to-absolute
RGB-D/IMU registration using distinctive, STATIC multi-frame landmarks
and independent 3D correspondence verification, plus explicit
observability diagnostics of weak along-road direction. Evaluate on
a separate mapping session, not the held-out Sep18 bag for fitting.
Before any cart run, connect a sustained untrusted localisation state
to a bench-tested planner/DBW fail-closed stop. The current offline
success criteria are not a tested safety control.

**None of this establishes a sub-0.5m result, stable four-mode sensor
failure operation, or NUC real-time capability.**
