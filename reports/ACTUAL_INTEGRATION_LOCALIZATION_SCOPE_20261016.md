# ALiVE localisation-only integration audit — 2026-10-16

Actual runtime repository (read-only):
/home/devil/Desktop/car/alive-repo/latest_repo/alive-dev/

No planner, ROS1/ROS2 bridge, drive-by-wire, brake, steering or control
files were edited.

## Existing localisation architecture and topic contract

1. src/fast_livo2/launch/fast_livo2_floam_compat_localization.launch
   starts FAST-LIVO2 ESIKF local odometry AND a copied FLOAM global
   frontend/localiser. This is NOT yet an all-FAST-LIVO2-saved-PCD failsafe.
2. FAST-LIVO2 sensor adapter accepts single front VLP16
   /lidar103/velodyne_points, /mavros/imu/data,
   /camera/color/image_raw, publishing to /fast_livo2/input/*.
3. The existing fast_livo2_floam_localizer owns global map-space /pose,
   while FAST-LIVO2's native local frame is camera_init -> aft_mapped.
   The estimator subscribes to /pose (src/estimator/src/estimator.cpp:115).
   NEVER publish a second competing /pose or duplicate TF.
4. The live launch defaults /use_sim_time=false, map frame=map,
   base frame=base_link, global TF off by default, with new_cart_measured.yaml.
   The experimental rosbag has different clock/extrinsic assumptions.
5. The compatibility launch defaults map to
   $(find fast_livo2)/maps/test.pcd, which is not present in the latest
   local checkout. Provide an explicit verified map path when integrated.
6. The frozen failsafe Step-3 stack still lives separately in
   /home/devil/Desktop/car/alive-repo/FAST-LIVO2-failsafe/,
   not in latest_repo/alive-dev/src/fast_livo2 today.

## Remaining localisation-specific integration gates

- Automatic GPS/visual/LiDAR initialization into saved campus-map frame
  without a manually published initial pose.
- Map<-fast<-base_link extrinsics, timestamp alignment, and exactly one
  authoritative /pose publisher in the same lanelet map frame.
- ESIKF dropout and recovery robustness, no alias-map registrations,
  and 1x NUC CPU/timing budgets.
- Same-image replay of healthy, camera, LiDAR and IMU-failure on both bags,
  with accurate Sep18 map proxy plus independent surveyed GT.
- The Oct16 fixed image still fails 14 quantitative continuity/accuracy
  checks and is not approved for live /pose replacement.

## Current isolated development

The IMU map-quarantine experimental C++ fix is packaged in its own Docker
image and ROSBAG launch. It is not in latest_repo/alive-dev or the cart
runtime. It must pass both bags before even offline promotion.

The golf cart has its own zero-velocity auto-braking behavior.
This project WILL NOT modify any motion/brake/deadman/bridge code.

**Status:** Read-only audit complete; live localization replacement and
physical NUC localisation validation pending.
