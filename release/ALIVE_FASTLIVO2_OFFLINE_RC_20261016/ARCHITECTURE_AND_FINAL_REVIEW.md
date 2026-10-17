# ALiVE FAST-LIVO2 failsafe: code architecture and final engineering review

This source snapshot is from ONE frozen experimental Docker image. Do not
confuse it with the older BEST_SO_FAR_20261015 selections from mixed images.

## Actual estimator and processing graph

Raw LiDAR / RealSense RGB / RealSense Depth / rear GPS+IMU in original rosbag
 -> Sensor adapter (timestamp/frame normalization)
 -> Test-only failure injector (none, camera, LiDAR, IMU)
 -> FAST-LIVO2 HKU-MARS ESIKF/local LiDAR-vision-inertial odometry
 -> saved-camera RGB-D and saved-LiDAR PCD registration candidates
 -> Step-3 global acquirer (map<-local anchor, absolute pose publication)
 -> /fast_livo2/step3_global_pose (diagnostic ROS published pose)
 -> RViz and trajectory CSV/plots ONLY for this release.

The estimator is an ESIKF, not a conventional separate EKF.
The original cart runtime architecture is ROS1 Noetic on NUC with a
ROS1/ROS2 bridge to VAT/DBW; that hardware chain was not exercised here.

## Modes

| Injected failure | Expected ROS mode | Intended fallback |
|---|---|---|
| none | FULL_LIVO | LiDAR + camera + IMU |
| camera | LIO_CAMERA_FAILED | LiDAR+IMU scan/map matching |
| lidar | VI_LIDAR_FAILED or IMU_ONLY_BRIDGE on low visual confidence | RGB-D/vision+IMU and saved-map hints |
| imu | LV_IMU_FAILED | LiDAR + vision motion and saved-map corrections |

The finalizer demands evidence of an actual sensor-failure mode transition
and recovery for each failure-mode replay. Merely renaming plots does not pass.

## Core source files

- code/src/step3_global_acquirer.cpp: global PCD map-anchor geometry and
  published pose continuity logic
- code/src/failsafe_map_localizer.cpp: map/LiDAR registration and depth
  failover frontend
- code/src/lidar_relative_failsafe_frontend.cpp: degraded relative-motion frontend
- code/src/main.cpp, LIVMapper.cpp, IMU_Processing.cpp, vio.cpp and
  voxel_map.cpp: FAST-LIVO2 ESIKF and visual/LiDAR processing
- code/include, code/vendor, code/msg, code/config: build support,
  pointmap configuration, sensor extrinsics and ROS interfaces
- code/launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch:
  ROSBAG-only Step3 launch used for this image
- reproduction_tools: actual dataset runner, new reference builder,
  rescorer and artifact packaging
- optional_shadow_smoother: standalone, delayed RViz-only preview,
  explicitly not in autonomous pose source

## What has and has NOT been fixed

Done:
- November? NO. All dates and source IDs frozen on 2026-10-16.
- Rebuilt Sep18 false-triangle reference with saved-PCD-backed LiDAR matches
  and validated withheld map points; no artificial road-turn editing
- One frozen code/config/launch bundle and exact Docker digest for
  full-length 1x repeatable validation
- All four modes tested on each bag by a scripted, serialized suite
- Fresh full plots + raw published poses, with identity and reference hashes
- Offline localisation status/watchdog topic tests, which do not
  validate sensor/map accuracy on the physical cart.

Not done and therefore NOT to be claimed:
- Independent surveyed or RTK <0.5m ground truth
- Accurate, continuously trusted absolute localisation through every
  LiDAR/IMU loss interval
- Never exceeding 0.85m steps or 0.3s timestamp gaps
- GPU/CPU timing and ROS frame calibration signed off on either physical NUC
- Cold-start GPS coarse / vision fine / LiDAR precise initialization
- Low-speed autonomous cart deployment

## Cleanup strategy

Research evidence is preserved in its original directories. Instead of
deleting scripts that may be referenced in historical scientific results, we
exported clean source from the frozen Docker image into code/. Host-only
evaluation scripts are in reproduction_tools/, and the experimental shadow
smoother is in optional_shadow_smoother/.

The cart runtime alive-dev repo and BITBUCKET branches are NOT modified.
