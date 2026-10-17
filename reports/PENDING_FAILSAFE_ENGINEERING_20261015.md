# FAST-LIVO2 failsafe: pending engineering tasks — 2026-10-15

## Deployment disposition

**NOT READY FOR CART DEPLOYMENT.** All builds here are isolated, opt-in development
images; the preserved golf-cart production launch and DBW controls were not
modified. The existing accuracy metrics are **differences from saved,
unvalidated loop-closed/PCD reference trajectories**, NOT independent surveyed
ground truth. A 0.5 m absolute localization RMSE cannot be certified from them.

## Baseline and experiment design

Frozen accepted *healthy-only* reference:
`fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015`.

Historical Oct-15 full checks with 40-second single-sensor outages:

| Bag / mode | 3D reference discrepancy | Largest published step |
| --- | ---: | ---: |
| Main 290s, healthy | 1.388 m | 0.710 m |
| Main 290s, camera loss | 1.568 m | 0.837 m |
| Main 290s, LiDAR loss | **22.398 m** | **1.831 m** |
| Main 290s, IMU loss | 1.594 m | 1.193 m |
| Sep-18 220s, healthy | 1.914 m | 0.783 m |
| Sep-18 220s, camera loss | 1.194 m | 0.815 m |
| Sep-18 220s, LiDAR loss | **17.813 m** | **10.683 m** |
| Sep-18 220s, IMU loss | 1.374 m | 0.669 m |

## A. Fixed specific false-recovery and queue integrity bugs (dev only)

Builder: `scripts/build_isolated_failclosed_recovery_20261015.py`.
Image: `fast_livo2_failsafe:failclosed-recovery-dev-20261015`.
Dev launch: `launch/fast_livo2_step3_global_failclosed_recovery_dev.launch`.

Changes are limited to Step-3 map-pose trust/recovery and do NOT overwrite
FAST-LIVO2 ESIKF, LiDAR visual mapping, or production cart commands.

1. RGB-D map pose is not marked verified until FAST odometry is actually
   available at the same measurement timestamp.
2. Deferred/in-flight LiDAR registration cannot replace the map anchor after
   the state has transitioned to `VI_LIDAR_FAILED` or `IMU_ONLY_BRIDGE`.
3. On LiDAR return, the local bridge is allowed only with a **recent verified
   independent saved-map depth registration**; without it, no fake updated
   last-good timestamp, map-ID reset, or local rebase is allowed. A separate
   global reacquisition must pass multiple GICP geometrically consistent
   matches before relock.
4. Step-3 optionally publishes `/fast_livo2/step3_pose_trusted` (`std_msgs/Bool`),
   based on recent independently verified GICP or depth-map poses and local
   FAST motion plausibility. A false status correctly appears during an
   unverified LiDAR outage. A true flag is still only a **heuristic health
   estimate**, not evidence of global accuracy or a stop command.

**Sep-18 LiDAR-outage 160s development comparison:**

| First 155 seconds | Frozen bridge (old) | Fail-closed v1 | Fail-closed v2 |
| --- | ---: | ---: | ---: |
| Full 3D proxy RMSE | 21.811 m | 1.550 m | 2.175 m |
| LiDAR-outage proxy RMSE | 17.211 m | 2.949 m | 4.133 m |
| 20 s post-outage proxy RMSE | 50.672 m | 2.693 m | 2.375 m |
| Largest published step | 10.683 m | 1.866 m | 1.670 m |
| Worst pose-timestamp gap | 0.706 s | 0.504 s | 0.605 s |

**Quality gate FAILED for both v1/v2.** A large amount of the apparent error
change occurs *before* sensor failure, when all sensors are present: the old
single run disagreed with the proxy by 13.5 m over 60–90s, while v1 was
0.33 m and v2 2.46 m. Thus these numbers **do not prove causal improvement**
from the patch: bag callbacks/GICP matching are scheduling-sensitive. None of
the variants meets the 0.85 m published step or 0.5 s pose-gap criteria.

**Main-bag independent check of v2: FAILED.** On
`camera_lidar_imu_gps.bag` at 1×, 195s with LiDAR outage 125–165s,
`results/camera_lidar_imu_gps/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_FAILCLOSED_195S`
recorded **27.755 m** full 3D proxy RMSE, **6.584 m** during outage,
**50.258 m** in the first 20 seconds after LiDAR return, and **2.084 m**
maximum published step. The false bridge was refused correctly, but verified
global reacquisition still failed to restore accurate publication. Thus this
build is not a robust LiDAR-failure solution on the main bag either.

Exact score files under
`results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/`:
`failclosed_vs_previous_lidar_160s.json`,
`failclosed_v2_vs_previous_lidar_155s.json` (see actual filenames).

## B. Temporal, geometry-verified depth relock (dev-only)

Existing `failsafe_map_localizer.cpp` feature, normally off, was enabled in
`launch/fast_livo2_step3_global_failclosed_temporal_dev.launch`. Strong
independent live-depth registrations were required across at least **three**
consecutive, physically consistent observations, with GICP fit at most 0.09,
map overlap at least 0.95, max speed/acceleration and course-change gates.

The log confirmed a verified streak of 3/3 followed by several validated
depth-map absolute poses. This is a real sensor-processing pathway, not a
post-hoc plot correction. The continuity/safety acceptance gates still apply.

Metrics and approval decision are in
`results/18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/temporal_vs_failclosed_v2_155s.json`.

**Actual 155s trial outcome:** 3/3 independently registered, physically
consistent live-depth hypotheses were confirmed in the log, and map poses
were indeed published from the verified depth pathway. Comparing the same
frozen reference and first 155s with the fail-closed v2 run:

| Metric | Fail-closed v2 | With three-fix temporal relock |
| --- | ---: | ---: |
| 3D proxy RMSE | 2.175 m | 1.838 m |
| During LiDAR outage | 4.133 m | 3.916 m |
| 20s after LiDAR return | **2.375 m** | **2.760 m** |
| Maximum published step | **1.670 m** | **1.686 m** |
| Worst pose-message gap | 0.605 s | 0.605 s |

The improved full-run proxy RMSE coincided with a much better *pre-outage*
path (2.459 m → 0.290 m), demonstrating replay timing variability again.
Post-failure recovery **regressed**, and the high jumps / gaps remain.
**Gate FAILED. Do not promote.**

## C. Offline fail-closed safety watchdog (not yet connected to DBW)

`scripts/localization_trust_watchdog.py` consumes fresh pose+map-trust ROS
signals and publishes `/fast_livo2/localization_stop_required` as Bool,
defaulting to TRUE for startup, stale trust, pose gaps, invalid motion, or
missing pose. It requires five successive validated samples before clearing a
normal transient trip. Physically impossible map jumps cause an **operator
restart required** hard fault, not time-based auto-clear.

Pure deterministic unit test:
`python3 scripts/test_localization_trust_watchdog.py`: **PASS** for startup,
rearming, stale inputs, trust loss, implausible pose steps, invalid frame gaps,
and hard-fault latch. A false trust status cannot silently increment the
rearming counter. Hard faults do **not** auto-clear merely because later
frames stay near a possibly incorrect map branch. This watchdog is **not wired to planning or DBW**;
publishing `localization_stop_required` alone will NOT physically stop the
golf cart. A closed-loop actuator-safe integration + bench test is essential.

**Actual ROS1 integration test also PASS:** separate opt-in sidecar image
`fast_livo2_failsafe:failclosed-watchdog-dev-20261015` was built using
`scripts/build_isolated_watchdog_overlay_20261015.py`, alongside development
launch `launch/fast_livo2_step3_global_failclosed_watchdog_dev.launch`.
`scripts/test_localization_trust_watchdog_ros_20261015.py` ran against a
real isolated ROS1 master and the actual watchdog publisher/subscriber node.
It verified startup stop=TRUE, correct healthy re-arming, timeout stops,
explicit trust-loss stops, and large position jumps causing non-clearing hard
stops. Pure deterministic tests additionally check unrealistic yaw jumps.
No CAN or drive-by-wire hardware was exercised: it is **signalling only**.

## D. Camera-recovery development test

Builder `scripts/build_isolated_camera_recovery_20261015.py` changes only
the recovery-epoch recognition in Step-3 so the camera-return event can use
the existing bounded handoff logic. A separate replay measures the actual
accuracy, latency, and jump gates before deciding whether this is useful.

**195s 1× main-bag test with camera outage 125–165s** saved at
`results/camera_lidar_imu_gps/CAMERA_RETURN_HANDOFF_DEV_20261015/`.
Against the same frozen historical camera-failure run, both scored over the
common first **190s**:

| Metric | Frozen camera-fail | Camera-return candidate |
| --- | ---: | ---: |
| Full proxy RMSE | 1.256 m | 1.122 m |
| Outage proxy RMSE | 1.035 m | 0.909 m |
| 20s post-return proxy RMSE | 1.426 m | **0.893 m** |
| Largest published pose step | **0.837 m** | **0.889 m** |
| Largest pose stream gap | 0.200 s | 0.200 s |

Although recovery proxy error improved in this single replay, the maximum
step **regressed past the 0.85m smoothness limit** (and pre-failure trajectories
also differed). Thus the strict combined quality gate **FAILED**; this
candidate is not promoted. Exact comparison saved as
`CAMERA_RETURN_HANDOFF_DEV_20261015/camera_return_vs_frozen_190s.json`.

## E. CPU and final acceptance blockers

Previous workload-specific process profile: FAST-LIVO2 mapping ~55% of one
logical core, Step-3 GICP ~35%, sensor adapter ~19%, visual place node ~18%.
ORB map startup database loading fell **14.86 to 0.03 seconds** with exact
descriptor caching, but **steady-state pipeline CPU improved only ~1–2.5%**
in controlled candidates and was not accepted. Reducing estimator/GICP
iterations or feature quality previously damaged map matching and smoothness.

Still required before live cart:

- Consistent LiDAR-off visual map registration and physically continuous
  motion across BOTH bags, with no unverified global-pose control authority.
- No post-camera / post-IMU recovery jumps; validated repeated 1× replay runs.
- Proven planner/DBW actuator-level braking on stale/false trust (watchdog
  heartbeat, braking dynamics, sign-change and ROS master failure cases).
- Real NUC profiling of latency, CPU/RAM saturation, and mapping under load.
- Independently verified GNSS/RTK/motion-capture/landmark reference to support
  a genuine 0.5 m 3D RMSE claim, not only saved-loop PCD proxies.

**No experimental image is deployed or promoted.**
