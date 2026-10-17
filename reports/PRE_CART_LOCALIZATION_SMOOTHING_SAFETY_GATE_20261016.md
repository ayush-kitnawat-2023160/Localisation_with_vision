# ALiVE localisation: smoothing and pre-cart readiness — 16 Oct 2026

**NO-GO for moving autonomous cart tests. Stationary, actuator-disabled sensor/ROS checks are the appropriate next test.**

## Real replay evidence

All errors are disagreement from a loop-closed diagnostic trajectory, NOT independent surveyed ground truth. Each run used the real camera_lidar_imu_gps bag at 1x with a 40-second LiDAR blackout (125–165s). The candidate comparisons use the first 185s of the historical 290s baseline.

| Version | LiDAR-outage 3D proxy RMSE | 20s recovery RMSE | Worst published step | Worst timestamp gap |
| --- | ---: | ---: | ---: | ---: |
| Preserved Oct13 best | 2.170m | 1.317m | 2.276m | 0.201s |
| Newer Step3 C++ causal smoother | 10.527m | 15.954m | 6.153m | 4.942s |
| Original frozen Oct13 + causal smoother across return | 17.659m | 41.708m | 4.891m | 5.245s |

The frozen candidate bounded each step INSIDE the LiDAR outage to 0.80m, but developed huge positional errors and large recovery jumps. Do NOT mistake a smooth-looking orange line for a valid navigation estimate. Both experimental changes were rejected; eight curated best runs and production container are unchanged.

We discovered a reporting bug in the first trial: its unsafe Bool could briefly be false even while smoothed and internal map positions differed by over 15m. The code was fixed to latch unsafe after any LiDAR outage, and to check absolute map backlog on every frame. Corrected isolated C++ image COMPILED, but its updated safety flag was NOT rerun on a full rosbag; the evidence PNG shows the original, pre-flag-fix trajectory (the pose algorithm is unchanged).

The separate ROS watchdog now hard-latches a stop-required status on receiving this unsafe flag. Unit tests and a ROS1 synthetic publisher/subscriber test PASS, including resistance to a subsequent false flag. These are **status messages, not tested cart braking**. There is no confirmed consumer for localization_stop_required in the planner/drive-by-wire controller; therefore STOP IS NOT PHYSICALLY GUARANTEED. The baseline Oct13 source also has no verified independent trust publisher; only experimental branches contain one.

## Files

- Actual trajectory comparison: results/CART_TEST_READINESS_20261016/ACTUAL_SMOOTHING_VS_BASELINE_PCD_20261016.png
- Machine-readable go/no-go: results/CART_TEST_READINESS_20261016/readiness_manifest.json
- Baseline vs frozen trial: step3_global/outage_causal_frozen_oct13_dev_20261016/frozen_oct13_smoother_vs_curated_185s.json
- Actual pose and error plots: step3_global/outage_causal_frozen_oct13_dev_20261016/MAIN_FROZEN_OCT13_CAUSAL_190S/plots/
- Updated isolated dev build script: scripts/build_outage_causal_frozen_oct13_20261016.py
- Safety watchdog / tests: scripts/localization_trust_watchdog.py and scripts/test_watchdog_smoother_ros_20261016.py
- Preserved original eight best runs: results/BEST_SO_FAR_20261015/

## Before any actual cart test

### Phase A — permitted: stationary rig, actuators DISABLED
Inspect mounted LiDAR103, RealSense camera/depth, rear IMU/GPS, current golf-cart dimensions and TF base_link. Verify camera/depth sync, camera-to-LiDAR extrinsics, LiDAR down-tilt, PCD map frame, stable chrony between the NUCs, live sensor timestamps and topic rates. Confirm ROS1 master, Noetic/Jazzy bridge, CPU/RAM and no real-time lag. RViz localisation-only, record a labelled bag. Keep motors/steering/braking commands disabled. Do not send experimental motion commands.

Useful ROS1 read-only commands, after confirming active topic names:
    rostopic info /fast_livo2/step3_global_pose
    rostopic info /fast_livo2/step3_pose_trusted
    rostopic info /fast_livo2/localization_stop_required
    rostopic hz /lidar103/velodyne_points
    rostopic hz /camera/color/image_raw
    rostopic hz /camera/depth/image_rect_raw
    rostopic hz /mavros/imu/data
    chronyc tracking
    chronyc sources -v

### Phase B — mandatory bench tests before any autonomous movement
Wire a *genuine independent* map-trust heartbeat and the localization_stop_required signal into the active planner-to-VAT drive-by-wire command arbitration. Verify actual measured speed reaches zero and braking engages on startup untrusted, LiDAR loss, camera loss, IMU loss, delayed pose, stale trust, jump, ROS process/network failure, yaw discontinuity and manual E-stop. Preserve manual brake/steering override and require explicit human re-arm. Test only when propulsive motion is physically prevented. A Boolean ROS message alone does not count as braking.

### Phase C — must pass before closed-course motion
Fix 3D depth-to-saved-PCD registration ambiguity and heading drift at the source rather than smoothing the wrong coordinates. Replay healthy + camera loss + LiDAR loss + IMU loss from ONE approved image at 1x on both original and independent September holdout bag; no tuning on holdout. Targets: <=0.5m diagnostic RMSE, <=0.85m pose step, <=0.30s stream gap, no sustained failure drift, no false map trust, CPU headroom on actual NUC, and real-world landmark check against independently surveyed positions.

### Phase D — later, only after full verification
Empty, geofenced, closed-course test at physical walking speed with a trained human operator and spotter, hardwired E-stop, proven DBW brake on localization failure, manual steering and brake override, and active limits in the vehicle command arbiter. First evaluate operator-driven localization with recording; only then permit narrow autonomous trials.

**Current readiness: stationary sensing and RViz only. No approved smoother, no autonomous drive-by-wire deployment.**
