# ALiVE FAST-LIVO2 localisation — independent release readiness audit
2026-10-17, after 2026-10-16 full ROSBAG validation.

## Scope
Only FAST-LIVO2 ESIKF, Step3 map registration, relative-motion
fallback, pose continuity, reference/proxy evaluation and localization
integration were inspected or edited. NO live ALiVE planner, bridge, brake,
steering, DBW or velocity-control source was changed. The intended ALiVE
integration repo is:
  /home/devil/Desktop/car/alive-repo/latest_repo/alive-dev/
Its Git tree is clean after the tests.

## Verified baseline
  fast_livo2_failsafe:single-build-rc-20261016
Eight full 1x ROS bag replays were completed from that single image, and all
historical/mixed-build best-so-far results and source bags were left intact.

| Recording / mode | Full 3D proxy RMSE | Outage 3D proxy RMSE |
|---|---:|---:|
| main healthy | 1.374m | -- |
| main camera fail | 1.408m | 1.027m |
| main LiDAR fail | 2.000m | 3.224m |
| main IMU fail | 2.149m | 3.494m |
| Sep18 healthy | 1.925m | -- |
| Sep18 camera fail | 1.326m | 1.613m |
| Sep18 LiDAR fail | 1.611m | 3.942m |
| Sep18 IMU fail | 1.457m | 2.557m |

These diagnostic loopclosed/LiDAR-PCD references are NOT independently
surveyed truth. The wrong right-side Sep18 triangle was already removed in
the new Oct16 reference.

## Concrete, newly discovered IMU-failure bug

At 131-134 seconds on the main bag, the published map pose remained near
[20.4,-170.9] even though the cart and accepted 1Hz PCD registrations kept
moving. The original read-only logs show FAST-GUARD quarantining an
implausible local 15.7m/s update. Instead of following the latest independent
saved-map registrations, the global publisher froze and produced up to 14.16m
instantaneous map-proxy disagreement.

The accepted GICP measurements during main IMU loss are substantially
better than the published global output (about 0.698m 3D proxy RMSE
versus 3.494m). So output authority is part of the bug. However the
Sep18 GICP measurements during IMU loss are ~2.656m from its proxy,
versus 2.557m for the published pose; blindly following GICP alone
is NOT a general solution.

## New C++ localization-only experimental variants, intentionally NOT promoted

V1: fast_livo2_failsafe:imu-map-quarantine-localization-dev-20261016
 - Adds opt-in fresh, time-aligned, independent PCD map-motion following
   only during ESIKF IMU-loss quarantine, with physical speed and yaw cap.
 - Main IMU outage 3.494 -> 2.142m, recovery20 2.317 -> 1.770m;
   worst step 1.539 -> 1.411m.
 - Sep18 outage 2.557 -> 2.172m, BUT recovery20 0.765 -> 1.383m,
   and full 160s RMSE 1.409 -> 1.541m. No quarantine code branch
   activated on Sep18, so the numerical difference is NOT causal proof.

V2: fast_livo2_failsafe:imu-map-quarantine-bounded-v2-dev-20261016
 - Extends map-motion age to 2.5s and explicitly caps the older
   map-fallback step during IMU-loss quarantine.
 - Main IMU outage 1.102m, recovery20 1.307m, max step 1.104m.
 - Sep18 outage 2.213m, BUT recovery20 becomes **11.377m**,
   160s full RMSE becomes **4.468m**, a **2.723-second** published
   pose gap occurs between 135.15-137.88s, and the largest absolute
   reference disagreement reaches **19.05m** around 141.31s.
 - At 135-138s the FAST-guard reports a 2.72s local pose gap and
   quarantine on the recovered IMU state. The IMU-specific optional
   map follower did NOT activate on Sep18; that suggests this is a
   run-time timing/estimator recovery instability, not a directly
   demonstrated effect of the new IMU handoff patch.
 - V2 did not trigger the newly added IMU-specific quarantine branch
   on the main run either, so its unusually good main score cannot be
   attributed to that code fix.

**Decision: both variants REJECTED for release.**
The freeze mechanism is diagnosed and a local remedy prototyped, but
the two-bag evidence fails the robustness/continuity gates.
The original frozen single-image core remains unchanged. No live
vehicle output was remapped.

## Source measurement root cause is still not solved

The LiDAR-failure depth/visual-map GICP accepted 34 absolute poses with
2.014m diagnostic 3D RMSE in the original controlled 1x experiment.
Previous ORB/PnP feature registration, local depth tracker, dual-map veto,
seed caps and publication smoothing did not reach 0.5m or preserve
cross-bag holdout performance. The repeated campus road is ambiguous;
GICP fitness/visual inlier counts alone cannot establish absolute XY.
Sub-0.5m accuracy cannot honestly be claimed by smoothing output or
fitting to the same bag loopclosed reference.

## Integration into actual latest_repo/alive-dev

Read-only inspection shows its existing ROS localisation launch still
uses FAST-LIVO2 local ESIKF plus a copied FLOAM global localiser
publishing /pose. The ALiVE estimator expects /pose in map frame.
Therefore the planned full FAST-LIVO2 global/failsafe replacement is
NOT integrated into latest_repo and should not overwrite its
single /pose publisher without a validated map transform and end-to-end
ROS/NUC test. See ACTUAL_INTEGRATION_LOCALIZATION_SCOPE_20261016.md.
Do not touch the vehicle-control stack.

## Remaining acceptance requirements

1. Distinctive static, multi-frame RGB-D/PCD landmark association and
   independently verified camera-LiDAR-IMU calibration, not just
   repeated-road GICP fitness. Do NOT train to Sep18 reference.
2. Deterministic source-time sensor processing with measured runtime
   CPU headroom. Investigate 2.72s ESIKF pose gaps under 1x replay
   and differences in the number of map matches/faults between runs.
3. A continuous error-bounded, physically plausible published map pose
   through each outage, including 20s reacquisition, while remaining
   aligned to the saved PCD lanelet map.
4. Repeat all 8 cases from ONE accepted image with no >0.85m steps,
   no >0.3s output holes and <=0.5m reference-proxy 3D RMSE; then
   validate independently surveyed points before claiming physical accuracy.
5. Only thereafter integrate a unique map-frame /pose publisher into
   latest_repo/alive-dev as localisation-only change.

**Completion verdict: PARTIAL. Offline source/replay/diagnosis complete;
required accuracy, robustness and runtime stability NOT achieved.
No fabricated improvement, edited trajectory plot or per-bag GT fit.**
