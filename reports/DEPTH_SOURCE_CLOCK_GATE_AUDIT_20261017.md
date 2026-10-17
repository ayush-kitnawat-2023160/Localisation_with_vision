# FAST-LIVO2 depth-GICP source-time gating experiment (17 October 2026)

Experimental Docker image:
fast_livo2_failsafe:depth-source-clock-gate-localization-dev-20261017

Based on immutable frozen 8-case image:
fast_livo2_failsafe:single-build-rc-20261016

## Hypothesis and source edit

Original failsafe_map_localizer.cpp throttles depth-to-map registrations
using ros::Time::now() from the scheduling of each callback. At 1x ROSBAG
replay, GICP/ORB CPU time changes which camera frames get processed.
Source-time selection is more reproducible across different CPU load.

The isolated opt-in parameter depth_source_time_gate_enable=true instead
throttles against normalized depth message header timestamps and rejects
out-of-order timestamps. This DOES NOT change map correspondences, ICP
acceptance thresholds, saved map, ESIKF state or publish smoothing.

Deterministic sampling unit tests PASS, and the ROS1 replay emitted
[DEPTH SOURCE CLOCK] logs with normalized RealSense depth timestamps.

## Corrected September 18 LiDAR-fail holdout

Outage 3D reference-proxy RMSE:
  frozen = 3.942 m, source-time trial = 3.925 m
Recovery first 20 seconds:
  frozen = 1.370 m, source-time trial = **3.421 m**
Largest published pose step:
  frozen = 1.564 m, source-time trial = **1.838 m**

Logs show 18 geometrically accepted visual-depth map poses in the
source-time trial versus 16 in the frozen test. Several accepted poses
show low GICP fitness 0.034–0.105 but remain in locally plausible,
globally WRONG portions of repeated campus geometry. The new source
gate merely changes the input selection; it cannot make geometric
aliases observable, hence worse recovery can occur.

**Decision: source-time gate NOT PROMOTED.**
It is an experiment whose deterministic sampler passed but whose
live localisation accuracy/recovery did not.

This note includes ONLY localisation code. latest_repo/alive-dev and
golf-cart movement/braking/bridge code were NOT modified.

## Main-bag LiDAR-loss check (same opt-in Docker image)

Outage 3D diagnostic proxy RMSE:
  frozen = 3.224 m, source-time trial = 2.850 m
Recovery first 20 seconds:
  frozen = 1.929 m, source-time trial = **3.806 m**
Largest published pose step:
  frozen = 1.934 m, source-time trial = **2.093 m**
Largest global proxy disagreement: trial **8.564 m**.

There were 36 log-emitted accepted depth-map measurements versus 32 in
frozen baseline. That does not establish their correct absolute location.
The main experimental replay did not exhibit an explicit
VI_LIDAR_FAILED -> FULL_LIVO transition in the scorer's log search.
It cannot be counted as a verified healthy recovery without additional
mode-evidence; regardless, the pose error and jump gates fail.

**Final outcome: BOTH recordings fail promotion.** The deterministic
selection improvement was syntactically and operationally validated, but
its effect on 3D localisation was a regression during recovery on both
bags. Do not replace the frozen image or remap the live /pose topic.
