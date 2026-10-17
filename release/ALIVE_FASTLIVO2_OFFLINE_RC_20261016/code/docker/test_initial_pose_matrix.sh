#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROSBAG_DIR="${ROSBAG_DIR:-/home/ayush/Desktop/faculty_loop_reindexed}"
MAP_DIR="${MAP_DIR:-$PACKAGE_DIR/maps}"
BAG="${BAG:-/data/rosbag/bags/faculty_loop.bag}"
BAG_DURATION="${BAG_DURATION:-8}"
BAG_RATE="${BAG_RATE:-1.0}"
BASE_X="${BASE_X:--409.695}"
BASE_Y="${BASE_Y:-447.199}"
BASE_YAW="${BASE_YAW:-2.613637}"

CASES=(
  "exact 0.0 0.0 0.0"
  "x_plus_0p5 0.5 0.0 0.0"
  "x_minus_0p5 -0.5 0.0 0.0"
  "y_plus_0p5 0.0 0.5 0.0"
  "yaw_plus_5deg 0.0 0.0 0.0872664626"
  "yaw_minus_5deg 0.0 0.0 -0.0872664626"
  "combined_1m_8deg 1.0 -1.0 0.1396263402"
  "combined_2m_15deg 2.0 -2.0 0.2617993878"
)

for case_spec in "${CASES[@]}"; do
  read -r case_name dx dy dyaw <<<"$case_spec"
  read -r initial_x initial_y initial_yaw initial_qz initial_qw < <(
    python3 -c 'import math,sys; x=float(sys.argv[1])+float(sys.argv[4]); y=float(sys.argv[2])+float(sys.argv[5]); yaw=float(sys.argv[3])+float(sys.argv[6]); print(x,y,yaw,math.sin(yaw/2.0),math.cos(yaw/2.0))' \
      "$BASE_X" "$BASE_Y" "$BASE_YAW" "$dx" "$dy" "$dyaw"
  )

  docker rm -f fast_livo2_pose_test >/dev/null 2>&1 || true
  echo "CASE $case_name seed=($initial_x,$initial_y,$initial_yaw)"
  docker run --rm --name fast_livo2_pose_test \
    --volume "$(cd "$ROSBAG_DIR" && pwd):/data/rosbag:ro" \
    --volume "$(cd "$MAP_DIR" && pwd):/data/maps:ro" \
    fast_livo2:dev bash -lc '
      set -euo pipefail
      roscore >/tmp/roscore.log 2>&1 &
      master_pid=$!
      launch_pid=""
      pose_pid=""
      cost_pid=""
      cleanup() {
        [[ -n "$pose_pid" ]] && kill "$pose_pid" >/dev/null 2>&1 || true
        [[ -n "$cost_pid" ]] && kill "$cost_pid" >/dev/null 2>&1 || true
        [[ -n "$launch_pid" ]] && kill "$launch_pid" >/dev/null 2>&1 || true
        kill "$master_pid" >/dev/null 2>&1 || true
      }
      trap cleanup EXIT
      sleep 2
      roslaunch fast_livo2 fast_livo2_full_rosbag.launch \
        rviz:=false \
        map_path:=/data/maps/full_campus.pcd \
        >/tmp/launch.log 2>&1 &
      launch_pid=$!
      sleep 7
      rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
        "{header: {frame_id: map}, pose: {pose: {position: {x: '"$initial_x"', y: '"$initial_y"', z: 0.0}, orientation: {z: '"$initial_qz"', w: '"$initial_qw"'}}}}" \
        >/tmp/initialpose.log 2>&1 || true
      rostopic echo -p /pose >/tmp/poses.csv 2>/tmp/poses.err &
      pose_pid=$!
      rostopic echo -p /ceres_cost >/tmp/cost.csv 2>/tmp/cost.err &
      cost_pid=$!
      timeout --signal=INT 20 rosbag play --quiet --clock \
        --duration='"$BAG_DURATION"' -r '"$BAG_RATE"' '"$BAG"' \
        >/tmp/play.log 2>&1 || play_status=$?
      sleep 2
      kill "$pose_pid" "$cost_pid" >/dev/null 2>&1 || true
      python3 - <<"PY"
import csv
import math

with open("/tmp/poses.csv", newline="") as stream:
    rows = list(csv.DictReader(stream))
if not rows:
    print("RESULT no_global_pose")
else:
    row = rows[-1]
    def value(name):
        return float(row[name])
    x = value("field.pose.position.x")
    y = value("field.pose.position.y")
    z = value("field.pose.position.z")
    qx = value("field.pose.orientation.x")
    qy = value("field.pose.orientation.y")
    qz = value("field.pose.orientation.z")
    qw = value("field.pose.orientation.w")
    yaw = math.atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0 * (qy*qy + qz*qz))
    stamp = value("field.header.stamp") / 1.0e9
    print("RESULT poses=%d stamp=%.6f xyz=(%.6f,%.6f,%.6f) yaw=%.6f" %
          (len(rows), stamp, x, y, z, yaw))

try:
    with open("/tmp/cost.csv", newline="") as stream:
        costs = list(csv.DictReader(stream))
    if costs:
        print("COST messages=%d final=%.6f average=%.6f" %
              (len(costs), float(costs[-1]["field.data"]),
               float(costs[-1]["field.data2"])))
    else:
        print("COST no_messages")
except (FileNotFoundError, KeyError, ValueError):
    print("COST unavailable")
PY
      printf "RESIDUAL_WARNINGS "
      grep -Ec "No edges or surface residuals|Not enough correct points" /tmp/launch.log || true
      grep -E "Map road plane|FLOAM global pose initialized|FLOAM global map initialized|No FAST-LIVO2 pose" /tmp/launch.log | tail -n 5 || true
      if [[ "${play_status:-0}" -ne 0 && "${play_status:-0}" -ne 124 && "${play_status:-0}" -ne 130 ]]; then
        exit "$play_status"
      fi
    '
done
