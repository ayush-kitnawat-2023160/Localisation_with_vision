#!/usr/bin/env bash
set -euo pipefail

# Reproducible single-sensor failure run for camera_lidar_imu_gps.bag.
# Usage: ./scripts/run_geometric_failsafe_case.sh <case_name> <none|camera|lidar|imu>

CASE="${1:-baseline}"
SENSOR="${2:-none}"
PLAY_DURATION="${PLAY_DURATION:-55}"
FASTLIVO_IMAGE="${FASTLIVO_IMAGE:-fast_livo2_failsafe:dev}"
MAP_LOCALIZER_ENABLE="${MAP_LOCALIZER_ENABLE:-true}"
FAILSAFE_FRONTENDS_ENABLE="${FAILSAFE_FRONTENDS_ENABLE:-true}"
VISUAL_PLACE_ENABLE="${VISUAL_PLACE_ENABLE:-true}"
MAP_PRIMARY_ON_IMU_FAILURE="${MAP_PRIMARY_ON_IMU_FAILURE:-$MAP_LOCALIZER_ENABLE}"
MAP_PRIMARY_ON_LIDAR_FAILURE="${MAP_PRIMARY_ON_LIDAR_FAILURE:-$MAP_LOCALIZER_ENABLE}"
MAPPER_LAUNCH_PREFIX="${MAPPER_LAUNCH_PREFIX:-taskset -c 0-7}"
MAP_LOCALIZER_LAUNCH_PREFIX="${MAP_LOCALIZER_LAUNCH_PREFIX:-taskset -c 8-11 nice -n 12}"
VISUAL_PLACE_LAUNCH_PREFIX="${VISUAL_PLACE_LAUNCH_PREFIX:-taskset -c 8-11 nice -n 12}"
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$PACKAGE_DIR/.." && pwd)"
OUT="$PACKAGE_DIR/results/camera_lidar_imu_gps/failsafe_v2_continuous_map/$CASE"
MAP_DIR="$ROOT/temp_fast_livo_2/final_failsafe_maps"
VISUAL_DIR="$ROOT/temp_fast_livo_2/output_image3d_map_complete_v3/livo/image"
VISUAL_TRAJ="$ROOT/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv"

rm -rf "$OUT"
mkdir -p "$OUT"

# All benchmark cases use --net=host and the same ROS node names. A stale
# benchmark container therefore corrupts the next run even if its case name is
# different. Remove only our local fastlivo_geo_* test containers; never touch
# an external/cart ROS master.
STALE_GEO_CONTAINERS="$(docker ps -a --format '{{.Names}}' | grep '^fastlivo_geo_' || true)"
if [[ -n "$STALE_GEO_CONTAINERS" ]]; then
  echo "$STALE_GEO_CONTAINERS" | xargs -r docker rm -f >/dev/null 2>&1 || true
  sleep 1
fi

docker rm -f "fastlivo_geo_$CASE" >/dev/null 2>&1 || true
docker run --rm --net=host --name "fastlivo_geo_$CASE" \
  -v "$ROOT/rosbag:/bags:ro" \
  -v "$MAP_DIR:/reference_map:ro" \
  -v "$VISUAL_DIR:/visual_map_images:ro" \
  -v "$VISUAL_TRAJ:/visual_map_trajectory.csv:ro" \
  -v "$OUT:/results" \
  -v "$PACKAGE_DIR/scripts/rgbd_failsafe_vo.py:/ws/src/Fast_Livo2/scripts/rgbd_failsafe_vo.py:ro" \
  -v "$PACKAGE_DIR/scripts/visual_place_hint.py:/ws/src/Fast_Livo2/scripts/visual_place_hint.py:ro" \
  -v "$PACKAGE_DIR/scripts/record_failsafe_trajectory.py:/ws/src/Fast_Livo2/scripts/record_failsafe_trajectory.py:ro" \
  -v "$PACKAGE_DIR/launch/fast_livo2_failsafe_rosbag.launch:/ws/src/Fast_Livo2/launch/fast_livo2_failsafe_rosbag.launch:ro" \
  -v "$PACKAGE_DIR/config/failsafe_rosbag.yaml:/ws/src/Fast_Livo2/config/failsafe_rosbag.yaml:ro" \
  -v "$PACKAGE_DIR/config/map_failsafe_recorded_cart.yaml:/ws/src/Fast_Livo2/config/map_failsafe_recorded_cart.yaml:ro" \
  "$FASTLIVO_IMAGE" bash -lc "
    set -e
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    roscore >/results/roscore.log 2>&1 &
    MPID=\$!
    sleep 2
    roslaunch fast_livo2 fast_livo2_failsafe_rosbag.launch \
      calibration_file:=/ws/src/Fast_Livo2/config/calibration/recorded_cart_bag.yaml \
      lidar_topic:=/lidar103/velodyne_points \
      imu_topic:=/mavros/imu/data \
      camera_topic:=/camera/color/image_raw \
      depth_topic:=/camera/depth/image_rect_raw \
      depth_info_topic:=/camera/depth/camera_info \
      map_localizer_enable:=$MAP_LOCALIZER_ENABLE \
      failsafe_frontends_enable:=$FAILSAFE_FRONTENDS_ENABLE \
      map_path:=/reference_map/final_lidar_geometric_voxel_15cm.pcd \
      visual_voxel_map_path:=/reference_map/final_camera_rgb_lidar_voxel_15cm.pcd \
      visual_place_enable:=$VISUAL_PLACE_ENABLE \
      visual_map_image_dir:=/visual_map_images \
      visual_map_trajectory:=/visual_map_trajectory.csv \
      map_primary_on_imu_failure:=$MAP_PRIMARY_ON_IMU_FAILURE \
      map_primary_on_lidar_failure:=$MAP_PRIMARY_ON_LIDAR_FAILURE \
      mapper_launch_prefix:='$MAPPER_LAUNCH_PREFIX' \
      map_localizer_launch_prefix:='$MAP_LOCALIZER_LAUNCH_PREFIX' \
      visual_place_launch_prefix:='$VISUAL_PLACE_LAUNCH_PREFIX' \
      drop_sensor:=$SENSOR \
      drop_start:=15.0 \
      drop_duration:=20.0 \
      trajectory_output:=/results/trajectory.csv \
      >/results/launch.log 2>&1 &
    LPID=\$!
    # Do not start bag playback until the exact saved visual-map database is
    # loaded. Otherwise a sensor failure can occur before its fallback is armed.
    if [ '$VISUAL_PLACE_ENABLE' = 'true' ]; then
      for i in \$(seq 1 90); do
        rosparam get /fast_livo2_visual_place_hint/ready 2>/dev/null | grep -qx true && break
        sleep 1
      done
      if ! rosparam get /fast_livo2_visual_place_hint/ready 2>/dev/null | grep -qx true; then
        echo 'ERROR: visual place database did not become ready' >>/results/launch.log
        kill -INT \$LPID 2>/dev/null || true
        kill \$MPID 2>/dev/null || true
        exit 3
      fi
    fi

    RPID=''
    if [ '$MAP_LOCALIZER_ENABLE' = 'true' ]; then
      rosrun fast_livo2 record_failsafe_trajectory.py \
        __name:=fast_livo2_map_frame_recorder \
        _output:=/results/map_trajectory.csv \
        _odom_topic:=/fast_livo2/map_odom \
        >/results/map_recorder.log 2>&1 &
      RPID=\$!
    fi

    # Verify that the global map-frame interface used by the Lanelet side is
    # actually alive during the run, not merely declared in source code.
    WATCH=''; POSEWATCH=''; STATUSWATCH=''; STATUSSTREAM=''; RELHINTSTREAM=''; TFWATCH=''
    if [ '$MAP_LOCALIZER_ENABLE' = 'true' ]; then
      timeout 50s rostopic echo -n 1 /fast_livo2/map_odom \
        >/results/map_odom_sample.txt 2>/results/map_odom_sample.err & WATCH=\$!
      timeout 50s rostopic echo -n 1 /pose \
        >/results/pose_sample.txt 2>/results/pose_sample.err & POSEWATCH=\$!
      timeout 50s rostopic echo -n 1 /fast_livo2/localization_status \
        >/results/localization_status_sample.txt 2>/results/localization_status_sample.err & STATUSWATCH=\$!
      rostopic echo /fast_livo2/localization_status \
        >/results/localization_status_stream.log 2>/results/localization_status_stream.err & STATUSSTREAM=\$!
      timeout 50s rosrun tf tf_echo map base_link \
        >/results/map_to_base_tf_sample.txt 2>/results/map_to_base_tf_sample.err & TFWATCH=\$!
    fi
    if [ '$FAILSAFE_FRONTENDS_ENABLE' = 'true' ]; then
      rostopic echo -p /fast_livo2/relative_motion_hint \
        >/results/relative_motion_hint.csv 2>/results/relative_motion_hint.err & RELHINTSTREAM=\$!
    fi

    set +e
    timeout --signal=INT ${PLAY_DURATION}s rosbag play --clock -r 1.0 \
      /bags/camera_lidar_imu_gps.bag >/results/rosbag.log 2>&1
    set -e
    sleep 3
    [ -n "\$WATCH" ] && kill \$WATCH 2>/dev/null || true
    [ -n "\$POSEWATCH" ] && kill \$POSEWATCH 2>/dev/null || true
    [ -n "\$STATUSWATCH" ] && kill \$STATUSWATCH 2>/dev/null || true
    [ -n "\$STATUSSTREAM" ] && kill -INT \$STATUSSTREAM 2>/dev/null || true
    [ -n "\$RELHINTSTREAM" ] && kill -INT \$RELHINTSTREAM 2>/dev/null || true
    [ -n "\$TFWATCH" ] && kill \$TFWATCH 2>/dev/null || true
    [ -n "\$RPID" ] && kill -INT \$RPID 2>/dev/null || true
    kill -INT \$LPID 2>/dev/null || true
    sleep 2
    kill \$MPID 2>/dev/null || true
    true
  "

# Every completed run must leave self-contained plots + loop-closed metrics.
# Plotting is post-processing only and never feeds the estimator.
if [[ -s "$OUT/map_trajectory.csv" ]]; then
  python3 "$PACKAGE_DIR/scripts/evaluate_loopclosed_run.py" \
    "$OUT/map_trajectory.csv" --reference "$VISUAL_TRAJ" --duration 45 \
    >"$OUT/metrics_vs_loopclosed.txt" 2>&1 || true
fi
python3 "$PACKAGE_DIR/scripts/plot_run_results.py" \
  "$OUT" --reference "$VISUAL_TRAJ" --sensor "$SENSOR" \
  >"$OUT/plot_generation.log" 2>&1 || {
    echo "WARNING: automatic plot generation failed; see $OUT/plot_generation.log" >&2
  }

echo "CASE $CASE ($SENSOR)"
wc -l "$OUT/trajectory.csv" || true
echo '--- localization status sample ---'
cat "$OUT/localization_status_sample.txt" 2>/dev/null || true
grep -E "Fixed startup anchor|Healthy map<-ESIKF anchor|DROP START|DROP END|FAILSAFE MAP| -> |relocalization LOCKED|Waiting for fresh" \
  "$OUT/launch.log" | tail -120 || true
