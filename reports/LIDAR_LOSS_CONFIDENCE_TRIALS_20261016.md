# LiDAR-loss localisation continuity and RGB-D confidence trials

**2026-10-16, India local time. Result: NO EXPERIMENT ACCEPTED. Production unchanged.**

## Starting point

Dataset `/rosbag/camera_lidar_imu_gps.bag`, 1× replay. Historical best
`results/BEST_SO_FAR_20261015/camera_lidar_imu_gps/LIDAR_FAILED/` retains
**1.534 m** same-bag 3D reference discrepancy over the complete **290s** and
a **2.276 m** largest published position step. The score is *not independent
ground truth*. No 0.5m RMSE claim can be inferred.

This audit uses **exactly the same first 190s** for every candidate, with
LiDAR artificially unavailable from 125–165s, to prevent comparing partial
195s trials with a full 290s result.

## Three isolated experiments (plus frozen curated run)

| 190s trial | 3D proxy RMSE | LiDAR-outage 40s RMSE | 20s recovery RMSE | Max published step | Steps above 0.85m | VI → IMU transitions |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| **Frozen curated Oct 13 baseline** | **1.483 m** | **2.170 m** | **1.317 m** | 2.276 m | 83 | 1 |
| Slower Step-3 map correction ramp | 2.286 m | 3.430 m | 4.797 m | **1.711 m** | 56 | 2 |
| Three-delta RGB-D confidence vote | 1.521 m | 2.716 m | 1.907 m | 2.168 m | 77 | 11 |
| RGB-D vote with 0.55/0.85 s bounded hysteresis | 2.549 m | 2.814 m | 5.484 m | 2.158 m | 69 | **0** |

Detailed numeric audit and saved trajectory plots:

- `step3_global/rgbd_mode_vote_hysteresis_trial_20261016/ALL_LIDAR_TRIALS_AUDIT_20261016.json`
- `step3_global/depth_ramp_rate_trial_20261016/LIDAR_FAIL_SLOW_RAMP_195S/`
- `step3_global/rgbd_mode_vote_trial_20261016/LIDAR_FAIL_RGBD_MODE_VOTE_195S/`
- `step3_global/rgbd_mode_vote_hysteresis_trial_20261016/LIDAR_FAIL_RGBD_VOTE_HYST_195S/`

All three complete 1× replays have an original global-pose CSV, ROS launch
log, evaluation, and plot of the orange pose on the saved PCD map.

## Engineering changes isolated and tested

**Correction ramp:** `scripts/build_depth_ramp_rate_trial_20261016.py` created
an opt-in launch that reduced allowed map-correction rate from 6.0 to 2.5m/s
and extended smoothing from 1.50 to 3.00s. No estimator code changed.
Result: reduced abrupt steps but **increased map-frame drift and delayed
return**; not retained.

**RGB-D relative geometry vote:**
`scripts/build_isolated_rgbd_mode_vote_20261016.py` built
`fast_livo2_failsafe:rgbd-mode-vote-dev-20261016`, changing only the LiDAR-loss
mode selection and tracking of accepted, bounded `rgbd_relative_delta`
events within the existing FAST-LIVO2 ESIKF. Three recent deltas and a maximum
age of 0.55s could keep `VI_LIDAR_FAILED` active when the LiDAR-dependent
photometric confidence temporarily collapsed. Data remained in sensor time.
The logging confirmed the vote was exercised 3 times, **but repeated
callback gaps caused 11 brief drops back to IMU_ONLY_BRIDGE**. This increased
mode chatter, not robustness.

**Bounded vote hysteresis:**
`scripts/build_isolated_rgbd_vote_hysteresis_20261016.py` built
`fast_livo2_failsafe:rgbd-mode-vote-hysteresis-dev-20261016` from the first
isolated image, requiring three fresh samples to enter and allowing only a
0.85s maximum sensor-time age once the vote was established. The original
IMU-only fallback remained mandatory when stale. In this replay, there were
zero `VI → IMU` switches **and zero positive RGB-D vote events**. Therefore
the lack of switches occurred *without the new mechanism intervening*; this
is direct evidence of scheduling sensitivity, NOT proof the new hysteresis
worked. Error was worse than the baseline.

### Root-cause evidence

During the prior matched-run LiDAR outage, the RGB-D PnP pipeline reported
99–245 valid inliers and filtered motion around 4.4–4.7m/s while the
LiDAR-aided native FAST camera confidence declined and the failure FSM
entered IMU-only mode. These are not contradictory measurements: an RGB-D
relative motion estimate can be locally good while an independent absolute
map match becomes stale. However, **we cannot trust a relative VO stream alone
as a global map position**, so manipulating the mode selector will not solve
saved-map reacquisition or the 0.5m target.

Repeated PCD corrections during LiDAR loss also caused fast moving public
anchor estimates and large steps. Reducing only the correction speed pushed
the cart's pose away from the actual PCD trajectory. A naive pose clip at
the 125–165s window likewise increased drift and induced a larger catch-up
jump at the failure boundary, so it was discarded without creating a patch.

## Decision and next technically justified work

**REJECT all three; keep the curated Oct 13 LiDAR-failure run unchanged.**
None meets the strict discontinuity threshold (0.85m); no consistent
improvement was shown in outage and post-outage RMSE; and replay-to-replay
variation is substantial. There was no real-car or NUC latency test. The
results are NOT surveyed position ground truth.

Future work must instrument timestamped independent **depth-to-saved-PCD
absolute registrations** (fitness, overlap, landmark ambiguity, serial
consistency), compare them to the actual ESIKF RGB-D relative increments,
avoid applying unverified map anchors, and ensure bounded, confidence-aware
reacquisition. A fail-closed planner/DBW connection must be bench-tested on
real hardware before any autonomous cart deployment. Lowering thresholds or
silencing the IMU fallback is specifically **not** an acceptable fix.

The `results/BEST_SO_FAR_20261015` eight primary full runs, all archived
historical evidence, production launch and Docker images remain unmodified.
