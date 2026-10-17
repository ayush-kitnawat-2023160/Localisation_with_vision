# FAST-LIVO2 localisation: 17 October multi-frame registration trial

**Verdict: REJECTED / NOT DEPLOYABLE.** This experiment is isolated from the
frozen release and the live golf-cart repository. No vehicle-control code was
readied for deployment or modified. In particular, do NOT replace the actual
cart's unique `/pose` publisher with this rejected experiment.

## Hypothesis and implementation

Previous LiDAR-loss visual retrieval + RGB-D PnP + depth-map GICP trials
accepted physically plausible matches against repeated campus-road geometry.
GICP residual, ORB/PnP count and reprojection error alone are not independent
absolute location evidence. We implemented an isolated, opt-in ROS Python
frontend under `step3_global/multiframe_place_gate_dev_20261017/`, with:

1. Depth-backed 3D PnP inlier covariance-rank test; refuse strongly planar or
   otherwise underconstrained 3D feature distributions.
2. Timestamped `/aft_mapped_to_init` local ESIKF odometry converted to the
   LiDAR frame with recorded body<-LiDAR extrinsics. Refuse absent or stale
   (over 0.15 s apart) source-time odometry.
3. Compare the relative SE(3) motion between two consecutive independent
   RGB-D/archived-image map hypotheses to local ESIKF motion at those same
   timestamps, with bounded translation/yaw residual. Two consecutive
   observations are still required before publishing a place candidate.
4. ROS clock rewind clears historical observations; no source timestamps are
   relabelled. This check applies to LiDAR-loss feature-PnP only, not the
   healthy / camera-loss / IMU-loss C++ localisation paths.

The first ROS trial launch had an isolated helper-module import issue in the
catkin-generated executable; it was stopped promptly and its results are not
used. A mounted source-path import corrected the issue. The complete
`LIDAR_FAIL_MULTIFRAME_GATE_MAIN_1X_RERUN` was replayed with the opt-in gate
and feature-PnP enabled and no Python traceback. Nine deterministic Python
tests PASS, including a test showing that constant false absolute-map offsets
are mathematically *undetectable* by relative-motion checks alone.

## Real single-image 1x ROSBAG replay and score

Recording: `camera_lidar_imu_gps.bag`. Artificial LiDAR outage: 125-165 s.
Both baseline and trial scored over the first 190 seconds using the exact
same diagnostic, map-aligned, loop-closed proxy trajectory. These errors are
**NOT independent field localisation accuracy**. The source frozen image is
`fast_livo2_failsafe:single-build-rc-20261016`; the trial only mounts the
isolated Python frontend and opt-in ROS launch setting.

| Diagnostic metric | Frozen baseline 190s | Proposed gate 190s |
| --- | ---: | ---: |
| Full 3D proxy RMSE | 1.687634 m | **5.394043 m** |
| LiDAR-outage 3D proxy RMSE | 3.223926 m | **9.655119 m** |
| First 20s recovery 3D proxy RMSE | 1.929020 m | **11.581334 m** |
| Maximum pose step | 1.933858 m | 1.290151 m |
| Published pose coverage | 188.685 s | 187.676 s |

The smaller maximum pose step **does not** imply success. Positional drift
grew catastrophically in the outage and recovery windows. The trial log
records 11 rejected cross-frame motion disagreements and 10 rejected weak
3D-landmark-support hypotheses, plus 8 accepted `depth_saved_map_icp` place
fixes. Relative motion can detect a road-identity *switch*, not a systematic
offset onto an indistinguishable parallel/repeated road. Withholding absolute
corrections without a better observable source leaves unbounded growing
global error. Do not weaken gates or post hoc smooth this trajectory to
claim improved accuracy.

Evidence is preserved under:
`results/MULTIFRAME_PLACE_GATE_DEV_20261017/main/`
including candidate raw ROS poses, measurements, logs, diagnostic figures,
`metrics_exact.txt`, and read-only symlinks into the original frozen
release for matched-window baseline comparison. The frozen 8-case release is
unchanged; it remains the best cross-mode evidence, NOT a cart-ready release.

## Remaining blocker and required acceptance criteria

To genuinely solve this task, the map needs **independently distinctive,
static, 3D, cross-frame landmark correspondences** (with explicit map-place
uniqueness and confidence), not just a locally plausible single-frame GICP
registration, while preserving a single persistent FAST-LIVO2 ESIKF. Sensor
time/extrinsics/calibration and loaded NUC callback latency also need
independent validation. This diagnostic run provides no evidence for
0.5 m accuracy. Repeat all four modes on BOTH bags at 1x from ONE accepted
build with <=0.5m proxy RMSE, <=0.85m steps, and no >0.3s pose gaps before
promoting code; repeat runs to assess timing sensitivity. Surveyed independent
ground truth is still needed to claim field accuracy. The existing FLOAM
global `/pose` publisher in `latest_repo/alive-dev` remains authoritative.

**Release gate: FAILED. NO PROMOTION. NO CART AUTONOMOUS TEST.**
