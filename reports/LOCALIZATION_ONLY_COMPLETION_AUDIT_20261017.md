# FAST-LIVO2 robust localisation: 17 October 2026 engineering completion audit

## Verdict

**Not final and NOT deployed.** The saved-map localisation source and
sensor-failure estimator remain inaccurate beyond stated acceptance limits.
The original eight-case, one-frozen-build rosbag baseline is preserved,
and the active ALiVE cart repository has not been edited. All code developed
in this evaluation is isolated, opt-in and not merged.

This report concerns localisation only: FAST-LIVO2 HKU-MARS ESIKF,
saved-map RGB-D / LiDAR GICP, Step-3 global map anchor and publisher,
failure recovery and CPU-limited map construction. There were NO edits to
brake, speed, steering, planner, ROS1/ROS2 vehicle bridges or DBW.

## Baseline retained from 16 October

Frozen image: fast_livo2_failsafe:single-build-rc-20261016
Original single-build full 1x metrics live under
results/FINAL_SINGLE_IMAGE_RC_20261016/.

Outage 3D disagreement against bag-specific diagnostic reference:
- Main LiDAR loss 125-165 s: 3.224 m RMSE.
- Main IMU loss 125-165 s: 3.494 m RMSE.
- Sep18 LiDAR loss 90-130 s: 3.942 m RMSE.
- Sep18 IMU loss 90-130 s: 2.557 m RMSE.

References are saved-PCD / loopclosed trajectories, NOT surveyed or RTK
ground truth. The invalid Sep18 false-right-triangle reference was replaced
before the Oct16 baseline was scored. The original baseline passes
neither sub-0.5 m accuracy nor all continuity requirements.

## What was actually implemented/tested 17 October

### Recovery local voxel-map budget (C++ opt-in, compiled)

Image: fast_livo2_failsafe:recovery-voxel-budget-localization-dev-20261017

The original ESIKF warm-up repeatedly inserted ~6,900-8,100 points per
recovery scan into the local voxel map. Experimental mapper limited ONLY
the recovery first scan and 4 additional warm-up insertions to about
1,170-1,390 points each. No normal estimator map matching was modified.

1x IMU outage / recovery:
- Main outage 3.494 -> 1.395 m, BUT recovery20 2.317 -> 4.229 m.
- Sep18 outage 2.557 -> 2.167 m, BUT recovery20 0.765 -> 5.091 m.
- Sep18 max pose gap 0.504 s, 1 step >0.85 m; no reliable target pass.

**REJECTED**. Fewer map points weakened critical recovery geometry. A
single run's reduced runtime gap is not proof of deterministic CPU safety.

### Frozen-image exact repeatability experiment

Without changing any source, reran the frozen single-build image on the
corrected Sep18 bag at the same 1x playback, IMU loss 90-130 s.

- Oct16 frozen run on 0-160s: IMU-outage 2.557 m; recovery20 0.765 m;
  max pose gap 0.403 s; max pose step 0.703 m.
- Oct17 same-image replay: IMU-outage 2.160 m; recovery20 **3.351 m**;
  max gap 0.403 s; max pose step 0.759 m.
- Trusted GICP accepts in the first 160s: 152 -> 144.
- GICP accepts during 40s IMU outage: 37 -> 31.
- GICP accepts during 20s recovery: 18 -> 17.

This establishes material **run-to-run performance variation despite
identical estimator and map code**. Therefore any one-bag one-run gain
must not be automatically attributed to the candidate C++ changes.

### Step-3 sensor timestamp FIFO (C++ opt-in, compiled)

Image: fast_livo2_failsafe:step3-deferred-fifo-localization-dev-20261017

Previously the async deferred-LiDAR queue selected the newest
FAST-odometry-aligned scan. This candidate selected the earliest
eligible odometry-aligned sensor timestamp in each 1 Hz window.

- The change actually chose a non-newest scan in only 1 of 171 logged
  main callbacks and 3 of 154 logged Sep18 callbacks.
- Main IMU outage 1.944 m, recovery20 0.979 m, but largest step
  1.803 m (above 0.85 m).
- Sep18 IMU outage 2.435 m, recovery20 1.712 m, largest step
  0.946 m, maximum gap 0.403 s.

**REJECTED**. The attempted scheduling mechanism almost never activated;
main-bag improvements are confounded by the observed replay variability.

### Step-3 SE(3) time-bracketed FAST odometry (C++ opt-in, compiled)

Image: fast_livo2_failsafe:step3-odom-interpolated-localization-dev-20261017

The frozen code returned the closest FAST-LIVO2 odometry sample within
0.15 s of the LiDAR measurement timestamp. At about 5 m/s this could
correspond to up to 0.75 m of physical motion. The candidate used a
time-bracketed, already-received ESIKF pose pair and SE(3) interpolation,
with a tight 0.045s nearest-sample grace. It never used reference points,
future state extrapolation or any vehicle actuator command.

Real 1x results:
- Main IMU outage 2.708 m, recovery20 2.478 m, max step 1.509 m.
- Sep18 IMU outage 2.504 m, recovery20 1.658 m, max step 0.820 m,
  max timestamp gap 0.504 s.
- Both still fail >=1 target. Sep18 recovery worse than Oct16 frozen
  baseline; main recovery remains worse.

**NOT PROMOTED**. The interpolation formula is physically principled,
but the limited test does not establish system-level improvement or
cross-bag robustness, and the same frozen image is not replay-deterministic.

## Structural source-measurement limitations

Earlier controlled trials found LiDAR-loss absolute visual-depth-to-saved
voxel map matches with 34 accepted 40s-window observations and 2.014 m
3D diagnostic reference discrepancy, despite low GICP fitness and
median measurement/publication age ~0.084 s. Repeated campus-road
geometry and uncertain heading/landmark distinctiveness make such
matches unreliable as ground-truth anchors. Merely smoothing or reducing
map correction gains cannot remove this source error.

During IMU loss, raw accepted LiDAR GICP matches were ~0.698 m RMSE
on the main recording but ~2.656 m on Sep18; blindly following GICP
would likewise overfit one recording.

## Actual target repository status

The user's real target is:
  /home/devil/Desktop/car/alive-repo/latest_repo/alive-dev/

Its present FAST-LIVO2 ROS launch still uses a copied FLOAM global
localiser as the sole publisher of /pose, which the ALiVE estimator
consumes in map frame. The unvalidated experimental global Step-3
publisher must not be copied over /pose or create duplicate TF.

The target Git working tree and all non-localisation control code
remain unchanged. No actual golf-cart CPU, frame transform, live
camera timing, initialization, or surveyed global accuracy was tested.

## Required remaining engineering work

1. Implement an observable, distinctive, static multi-frame
   RGB-D/PCD landmark registration (not just GICP fitness or image
   inlier count), with independent camera-LiDAR-IMU calibration checks.
2. Fix the cross-thread sensor/FAST-odom queue and recovery CPU stalls
   so repeated 1x and live NUC executions produce consistent accepted
   map observations, poses and timestamps.
3. Maintain a single persistent ESIKF through failures, with
   physically plausible global output and independently verified
   map re-acquisition. Do not use predictive smoothing to hide a bad
   absolute pose.
4. Repeat healthy, camera fail, LiDAR fail and IMU fail on both bags
   using ONE immutable codebase, all 1x, and show <=0.5m RMSE against
   the diagnostic proxy, <=0.85m pose steps and <=0.3s pose gaps.
   Independently survey GT before claiming field accuracy.
5. Only after passing should the validated global /pose replace
   FLOAM in latest_repo/alive-dev, without touching the car controls.

**Current user deliverables:** frozen offline baseline + precise
C++ experiments, actual ROS result CSVs, plots, source diffs and
normalized measurement counts. It would be misleading to call
localisation "complete" or claim autonomous-cart readiness.
