# REPRODUCTION-ONLY TOOLS (separate from frozen single-image code)

These orchestration/evaluation scripts are NOT compiled or executed as ROS
nodes in the frozen core C++ image. The runner uses them on the host to
mount configuration, sensor adapters, play the rosbag at 1x and score pose CSV.

The *core* code/ snapshot is the tested image's source tree except these
RUNTIME-MOUNTED host files (they are exact from the 1x validation):
- launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch
- launch/fast_livo2_failsafe_rosbag.launch
- scripts/fast_livo2_sensor_adapter.py
- scripts/rgbd_failsafe_vo.py
- scripts/visual_place_hint.py
- config/failsafe_rosbag.yaml
- config/calibration/recorded_cart_bag.yaml
- config/map_failsafe_recorded_cart.yaml

This folder includes the new Oct16 reference builder and metrics packaging.
Do not treat the offline results or ROS stop Boolean as deployment approval.
