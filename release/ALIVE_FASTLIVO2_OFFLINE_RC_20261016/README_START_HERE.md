# ALiVE FAST-LIVO2 clean OFFLINE RC — October 16, 2026

## Release status
THIS IS A SINGLE-IMAGE OFFLINE RELEASE CANDIDATE — **NOT VEHICLE DEPLOYABLE**.
It is a clean source snapshot captured from one frozen Docker image, with
the exact config/launch/sensor scripts mounted during the real 1× bag runs.
It is intentionally separate from older experiments and cherry-picked results.

## Layout
- code/: original source (C++ step3, map localizer, sensor bridge, ESIKF,
  build support, configs, launch files, research scripts)
- manifest/source_file_sha256.txt: exact portable relative checksums
- manifest/SOURCE_SNAPSHOT_TYPE.txt: tested image identity
- validation/: generated 8-case replay comparison plots after all runs complete
- results/FINAL_SINGLE_IMAGE_RC_20261016 in ORIGINAL workspace: all eight
  source global_pose.csv streams, GICP pose, run logs and exact plots

## Binary used for verification
  fast_livo2_failsafe:single-build-rc-20261016
  sha256:9fda983a4dd504c4e52d672f257d3e4ad703fd72e8e8c9ce24bfc0aeb33abe48

This Docker image is an exact tag of the Oct 15 healthy-loop-motion-bridge
experimental image; it was not recompiled for final validation.
Do not substitute fast_livo2_failsafe:dev or C++ from the current working
tree and then claim results belong to the same binary.

## External data (not packaged)
The user computer already has:
- /home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag
- /home/devil/Desktop/car/alive-repo/rosbag/18_sep_2026_combined.bag
- step3_global/map_db_sweeps_fast/cache/global_map.pcd
- step3_global/healthy_validation_assets_20261014/final_camera_rgb_lidar_voxel_15cm.pcd
- step3_global/healthy_validation_assets_20261014/visual_map_images/
- step3_global/healthy_validation_assets_20261014/orb_cache_dev_20261015/visual_orb_retrieval_exact.npz
- Main-bag 290s same-bag loopclosed reference (not GT)
- New Sept18 LiDAR/PCD same-bag proxy Oct16 (NOT GT)

The map and image keyframe DB exceed 900MB and are NOT included here.
This code snapshot depends on installed ROS Noetic, PCL, Eigen, catkin and
matching Docker runtime plus dataset. Do not claim this source ZIP alone
provides a deployable, built cart environment.

## Reproduction from original project workspace
  cd /home/devil/Desktop/car/alive-repo/FAST-LIVO2-failsafe
  bash scripts/run_final_single_image_suite_20261016.sh

IMPORTANT: This runner refuses to overwrite existing results. For a second
replication, change the suite output directory to a NEW destination and use
the same pinned image ID and asset checksum manifest.

## Why this is not autonomous release-ready
- Existing best 8 runs come from multiple images; hence new one-image suite.
- Requirement <0.5m RMSE not achieved against even proxy references.
- LiDAR-fail and IMU-fail jump/continuity issues persist.
- Independent surveyed GT/RTK proof is unavailable.
- The localisation output has not passed real-cart sensor and timing validation.
  Use the frozen candidate for offline evaluation, not autonomous driving.
- Previous fixed-lag smoother lives in separate dev-only ROS shadow/preview.
  It is deliberately NOT wired into the navigation topic in this release.

Old experimental images and results are intentionally NOT deleted. Source
cleanup is a clean isolated export, not destructive garbage collection.

## Critical ROSBAG-only launch settings
The frozen Step3 launch inherits use_sim_time=true and hardcoded taskset -c
0-11 groups for the Lenovo laptop; the two ALiVE NUCs have a different CPU
and ROS1/ROS2 networking environment. Do not directly launch this file on
the golf cart. A separate NUC-specific localisation launch must be validated with
actual sensor topics, RealSense calibration, live time and CPU headroom.

## Source-control note
The working FAST-LIVO2-failsafe directory presently has no .git metadata;
there is no reliable project Git commit ID to attach to these local runs.
This release is instead pinned by Docker image digest, per-file SHA256 and
sensor/map reference SHA256 manifests. Do not claim this was pushed to a
remote Bitbucket/GitHub branch. Import the vetted source into the ALiVE
repository only after its hardware acceptance gates pass.

