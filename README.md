# ALiVE FAST-LIVO2 Failsafe — Fresh ZED X ROS-bag Workspace

This checkout has been reset for **new recordings** (19 October 2026). Historical
PCD/Lanelet maps, frozen validation outputs, experiment reports, release ZIPs
and cached scan databases were removed from this working tree. The ROS1
FAST-LIVO2 estimator, RGB-D failsafe, map-localizer source, tests, Docker
build and experimental code are preserved.

**Fresh startup has NO global map, no global relocalization and no validated
map-frame /pose output.** It provides local odometry and sensor-failure
experimentation only. Do not connect this offline configuration to vehicle
planning or controls as a trusted global localization source.

## Calibration

Active new-bag profile: config/calibration/zedx_current.yaml

- Camera frame: zedx_left_camera_optical_frame.
- LiDAR frame: velodyne.
- Convention: p_camera = Rcl * p_velodyne + Pcl.
- Translation Pcl: [-0.027102, -0.526644, 0.003782] metres.
- Rotation: extrin_calib/Rcl in the active profile, derived from supplied
  roll=0.110776, pitch=-1.171075, yaw=1.476343 **radians**.
- The two user-provided TF files contained the same values; the source is
  retained at config/calibration/zedx_lidar_tf_source_20261019.yaml.
- With 40 supplied camera/LiDAR correspondences and intrinsics from the
  existing ZED X profile, reprojection errors were 3.502 px median,
  5.931 px 95th percentile. This is a consistency check, not surveyed truth.
- Camera intrinsics (960x600, fx=fy=371.695343, cx=477.192841,
  cy=294.337616) come from the EXISTING ZED X rosbag profile, not from
  the uploaded TF files. Verify against new bag CameraInfo.
- LiDAR-to-IMU, LiDAR-to-base and sensor height are **legacy inherited
  assumptions**, not newly measured values. The real-cart calibration file
  config/calibration/new_cart_measured.yaml stays LOCKED (ready: false).

The newly uploaded 40 image/object correspondence pairs are preserved
under config/calibration/measurements_20261019/ for reproducibility.
Check the transform AND those reprojections in this checkout:

    python3 scripts/verify_zedx_calibration.py

To test another calibration dataset instead, supply different files:

    python3 scripts/verify_zedx_calibration.py \
      --image-points /path/to/image_points.txt \
      --object-points /path/to/object_points.txt

ZED X aligned depth uses 32FC1 in metres with depth_scale=1.0 and native
sensor timestamps. The earlier RealSense bag path used 16UC1 millimetres;
do not mix the two profiles.

## Fresh ROS-bag replay (no saved map required)

New default entrypoint:

    launch/fast_livo2_zedx_fresh_rosbag.launch

Its legacy alias launch/fast_livo2_looooo_rosbag.launch uses the same
fresh-map-free settings. Both disable map_localizer_enable, visual place
recognition and saved-map recovery-lock requirements.

Example using the included sibling ROS-bag directory:

    ./docker/build.sh
    ./scripts/run_fresh_zedx_bag.sh looooo/localization.bag 1.0

Or, within an already configured ROS1 Noetic catkin environment, use
two terminals:

    roslaunch fast_livo2 fast_livo2_zedx_fresh_rosbag.launch

    rosbag play --clock /path/to/new_recording.bag

Default sensor topics (override these launch args if new recordings differ):

    lidar_topic  = /lidar103/velodyne_points
    imu_topic    = /mavros/imu/data
    camera_topic = /zedx/zed_node/left/image_rect_color
    depth_topic  = /zedx/zed_node/depth/depth_registered

The fresh local estimator publishes /aft_mapped_to_init; inspect
/fast_livo2/failsafe_mode and /fast_livo2/sensor_health for failures.
**Do not assume /pose exists or is global without an explicit, independently
verified map localizer.**

## Adding a brand-new map later

1. Record and validate new bag timestamps, sensor topics, CameraInfo,
   depth units, point-cloud point-time field and all physical extrinsics.
2. Build a new, internally consistent LiDAR map from these recordings.
   A FAST-LIVO2 odometry-based PCD export alone is not automatically
   loop-closed or globally registered.
3. Store new maps in maps/ (ignored by Git) and create a new corresponding
   saved-map database/visual-keyframe database under step3_global/ (ignored).
4. Only after map validation, explicitly enable map_localizer_enable and
   supply map_path, visual_map_path and a matching calibration. Do not
   re-enable map-lock-required sensor recovery until the global node is
   present and verified.
5. Evaluate healthy, camera-failure, LiDAR-failure, IMU-failure and recovery
   on each new bag. The existing source is NOT approved for cart operation.

## Cleanup boundaries

Removed historical data under this repository only:

- maps/full_campus.pcd and maps/full_campus_lanelet.osm;
- results/, results_archive/, step3_global/, release/, reports/;
- historical result-dependent README files and old diagnostic reference JSON;
- Python bytecode caches.

Preserved: source, build files, tests, calibration/launch configurations,
historical experiment scripts, and ../rosbag/ (the sibling folder, including
your four new ZED X recordings).

New results and map assets go under ignored paths such as outputs/, results/,
maps/ and step3_global/. **Git history still contains older tracked results**;
no commits were created, branches rewritten or remote history changed.
