# ALiVE FAST-LIVO2 localisation — verified status, 17 Oct 2026

## Accepted code
The current frozen, eight-case-tested OFFLINE reference implementation remains:
  fast_livo2_failsafe:single-build-rc-20261016

The previous 8-case 1x data and plots remain at:
  results/FINAL_SINGLE_IMAGE_RC_20261016/

The **corrected 18 September LiDAR/PCD reference** is:
  results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/

Original independent surveyed GT is unavailable, so error is always
distance to diagnostic map-aligned reference, not absolute field truth.

Actual ROS1 vehicle integration target:
  /home/devil/Desktop/car/alive-repo/latest_repo/alive-dev/
The Git working tree is clean. It still uses a copied FLOAM global map
localizer for the map-frame /pose topic. Do NOT overwrite that topic
with an unvalidated experimental trajectory.

## 17 Oct experiments

1. Recovery voxel work budget, opt-in:
   - Original ~7-8k downsampled points per scan -> 1200 recovery seed
     points; recovery warm-up 5 -> 3 scans. Normal ESIKF and saved-map
     GICP are unaffected.
   - Sep18 IMU max published pose gap 0.403 -> 0.234 seconds.
   - BUT Sep18 LiDAR recovery 20s proxy RMSE 1.370 -> 2.335m,
     main LiDAR recovery RMSE 1.929 -> 2.339m.
   - IMU main outage improvement before recovery is likely sensitive to
     replay timing and is not causal proof of this change.
   - REJECTED for whole-stack promotion.

2. Depth source-clock registration frame limiter, opt-in:
   - Previous depth-GICP sampling selected frames by callback arrival
     ROS time. New code selects depth frames by normalized camera
     message timestamp. Pure deterministic clock unit tests PASSED.
   - Sep18 visual-depth saved-map fixes: 16 -> 18 logged accepted,
     but recovery error 1.370 -> 3.421m and max step 1.838m.
   - Main fixes: 32 -> 36 accepted, but recovery error
     1.929 -> 3.806m and max step 2.093m.
   - Therefore changing sample selection is NOT a fix for incorrect
     absolute map correspondences. REJECTED.

Both experiments were compiled from the same frozen baseline image,
ran at 1x on the recorded bags, and were not fitted to the references.
Full source, exact metrics, trajectory plots and source diffs archived:
  results/LOCALIZATION_ONLY_COMPLETION_20261017/
  LOCALIZATION_EXPERIMENTS_REJECTED_WITH_REAL_1X_EVIDENCE_20261017.zip

## Remaining algorithmic source problem

Depth and LiDAR GICP can produce low residuals against the WRONG
repeated campus road segment. Fitness numbers ~0.034–0.105 did not
guarantee absolute XY correctness under LiDAR outage. Once an
ambiguous saved-map correction is accepted, the persistent local
ESIKF/map-anchor recovery can move in the wrong direction.

The next justified change is to build a **measurement-level,
multi-frame place-verification criterion** which requires an
independent distinctive visual landmark signature (or another true
independent absolute observation), tracks place identity across time
and verifies the relative SE(3) movement between candidate matches.
When no unambiguous absolute observation exists, the estimator must
represent growing positional uncertainty rather than pretend the map
registration is valid. A post hoc low-pass or speed cap cannot
reconstruct missing absolute information.

Then investigate IMU recovery by replaying identical 1x bags with
instrumentation of estimator callback CPU latency and per-sensor
timestamps. Repeated good/bad runs demonstrate scheduler sensitivity,
and isolated freeze fixes previously did not generalize.

**Acceptance gate before integration**: one image, healthy plus camera,
LiDAR and IMU failure on BOTH bags, 0.5m proxy 3D RMSE, max pose step
<=0.85m and no >0.3s output gaps, with no holdout fitting. A field
accuracy claim also needs independent surveyed ground truth.

## Explicit release verdict

NOT COMPLETE / NOT PROMOTED. No vehicle-control or actual runtime
repository files changed. The original frozen research baseline is
the only reproducible full eight-case source image.
