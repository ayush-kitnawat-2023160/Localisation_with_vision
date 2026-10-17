# FAST-LIVO2 Failsafe Localization

Robust ROS Noetic localization pipeline for the ALiVE golf cart. The current system keeps FAST-LIVO2 as the persistent estimator and adds health-aware degraded operation plus saved-map/global corrections rather than switching to unrelated SLAM stacks.

## Current status

The current `camera_lidar_imu_gps.bag` pipeline has completed Step 1–3 validation:

- Healthy FAST-LIVO2 remains alive over the full ~290 s run.
- Camera failure falls back to LiDAR + IMU.
- LiDAR failure uses RGB-D/visual motion with appearance-guided saved visual-voxel-map correction.
- IMU failure uses visual/LiDAR motion with saved-map correction.
- Step 3 global map/PCD correction is smoothed and uses bounded SE(2)-style map correction rather than hard pose jumps.
- LiDAR-failure map correction uses visual place as the global anchor and bounded depth GICP refinement. Pure local depth-only tracking was tested and rejected because repeated road geometry caused longitudinal drift.

Step 4 is under way. On `18_sep_2026_combined.bag`, healthy and camera-failure 220 s replays dated 2026-10-08 now regain PCD tracking after roughly 5.5 s rather than 32 s at the overlapping route section. The keyframe-ID-only rejection was replaced by map-space reachability, yaw/height plausibility, GICP geometry and two-scan confirmation. See `results/18_sep_2026_combined/README.md`. Cross-bag LiDAR/IMU failure retests, original-bag regression and independent absolute-accuracy checks are still pending.

## Canonical pipeline files

Only the canonical implementation is kept in the active source tree:

- `src/failsafe_map_localizer.cpp` — failure-aware saved-map correction and degraded-mode map localization.
- `src/step3_global_acquirer.cpp` — global PCD/map acquisition and correction authority.
- `src/lidar_relative_failsafe_frontend.cpp` — LiDAR relative-motion frontend.
- `scripts/rgbd_failsafe_vo.py` — RGB-D degraded relative motion.
- `scripts/visual_place_hint.py` — appearance retrieval + depth-map geometric verification.
- `scripts/fast_livo2_sensor_adapter.py` — sensor adaptation/health plumbing.
- `scripts/fast_livo2_failure_injector.py` — reproducible sensor-failure injection.
- `launch/fast_livo2_step3_global_rosbag.launch` — current Step-3 rosbag launch.
- `launch/fast_livo2_failsafe_rosbag.launch` — current failsafe launch.
- `config/failsafe_rosbag.yaml` — main bag configuration.
- `config/map_failsafe_recorded_cart.yaml` — current saved-map/failsafe tuning.
- `config/calibration/recorded_cart_bag.yaml` — recorded-cart calibration.

Historical `pre_*`, `v*`, and `lidarfix_*` source/config/launch copies were intentionally removed. Reproducibility is provided by retained result runs and Docker tags rather than duplicate source files in the active tree.

## Repository layout

```text
FAST-LIVO2-failsafe/
├── config/             ROS/calibration/failsafe configuration
├── docker/             reproducible ROS Noetic build/run environment
├── include/            C++ headers
├── launch/             canonical launch files
├── maps/               lanelet/static map assets used by compatibility launches
├── msg/                ROS messages
├── results/            curated validation evidence only
├── rviz/               RViz configuration
├── scripts/            runtime, validation and plotting tools
├── src/                FAST-LIVO2 + failsafe implementation
├── step3_global/       current fast global-localization DB + shared PCD keyframes
├── test/               tests
├── tools/              utilities
├── validation/         validation helpers/configuration
└── vendor/             vendored build dependencies
```

`step3_global/map_db_sweeps_fast/pcd` intentionally points to `step3_global/map_db_sweeps/pcd`. Do not delete the shared `map_db_sweeps/pcd` directory when pruning caches.

## Build

```bash
cd /home/devil/Desktop/car/alive-repo/FAST-LIVO2-failsafe
./docker/build.sh
```

This produces `fast_livo2_failsafe:dev` and `fast_livo2:dev`. The final bounded-place validation image is also tagged locally as:

```text
fast_livo2_failsafe:lidar-best-boundedplace-20261006
```

The validation runner defaults to the freshly built `fast_livo2_failsafe:dev`; override with `STEP3_IMAGE` when reproducing a frozen image.

The Step-4 global reacquisition fix is controlled by the ROS private parameter `tracking_global_reacquire_physical_gate` (enabled by default in `launch/fast_livo2_step3_global_rosbag.launch`). Set it to `false` for the historical keyframe-ID-guard behavior. Physical gate tuning uses `tracking_global_reacquire_position_slack_m`, `tracking_global_reacquire_vertical_slack_m` and `tracking_global_reacquire_predicted_pose_weight` in addition to the existing global reacquisition motion/yaw limits.

## Run a validation case

The main runner is:

```bash
./scripts/run_step3_global_case.sh <case_name> <none|camera|lidar|imu>
```

Example full LiDAR-failure test:

```bash
STEP3_VISUAL_PLACE_ENABLE=true \
PLAY_DURATION=290 \
DROP_START=125 \
DROP_DURATION=40 \
./scripts/run_step3_global_case.sh FINAL_LIDAR_TEST lidar
```

Important environment overrides supported by the runner include `STEP3_IMAGE`, `BAG_PATH`, `STEP3_MAP_DB`, `STEP3_VISUAL_MAP`, `STEP3_VISUAL_MAP_IMAGES`, `STEP3_VISUAL_MAP_TRAJ`, `STEP3_MAP_LOCALIZER_CONFIG`, `STEP3_REFERENCE`, `PLAY_DURATION`, `DROP_START`, and `DROP_DURATION`.

The default recorded-cart validation currently expects external assets under `/home/devil/Desktop/car/alive-repo/`, including the rosbag and final visual voxel-map/reference trajectory. The current fast global DB itself is stored under this repo at `step3_global/map_db_sweeps_fast`.

## Curated results

See `results/camera_lidar_imu_gps/failsafe_v2_continuous_map/README.md` for the retained result set and metrics.

The accepted primary 290 s runs are:

- `FINAL290_HEALTHY_BOUNDEDPLACE_20261006`
- `FINAL290_CAMERA_FAIL_BOUNDEDPLACE_20261006`
- `FINAL290_LIDAR_FAIL_BOUNDEDPLACE_20261006`
- `FINAL290_IMU_FAIL_BOUNDEDPLACE_20261006`

Useful LiDAR-failure comparison runs retained are:

- `LIDARFIX_V9_SMOOTHRATE_FULLAUTH_180S_20261005` — previous baseline.
- `LIDARFIX_BOUNDEDPLACE_180S_20261006` — bounded-place milestone.

For every final run, the clean map overlay is `plots/07_orange_trajectory_on_pcd_clean.png`.

## Generated-data policy

Large generated results and Step-3 databases are intentionally ignored by `.gitignore` and `.dockerignore`. The Docker build context should contain code/config only; runtime map DBs and validation results are mounted separately. Do not commit rosbag files, generated result folders, or temporary build directories into the source history.
