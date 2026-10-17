# LiDAR-loss error: source measurement diagnosis

**Decision:** 2+ metres is still unacceptable. Do not deploy or promote the
diagnostic image. Retain all eight curated best runs as they were.

## What was tested

Independent diagnostic-only Docker image
`fast_livo2_failsafe:depth-timing-diagnostic-20261016` built from the frozen
Oct 13 experiment. Original FAST-LIVO2 ESIKF, map points, ICP constraints,
Step-3 gain, failure windows and publication logic **unchanged**. Replay used
`camera_lidar_imu_gps.bag` at 1x through 180 s, with LiDAR withheld
125–165 s. Output and PCD plots:
`step3_global/lidar_fail_depth_timing_diag_20261016/DEPTH_TIMING_180S/`.

The loop-closed trajectory was used **only to score**, not to initialize or
correct anything. It is a proxy, not an independent surveyed ground truth.

## Finding: the raw depth map matches are already inaccurate

Recomputed every accepted absolute RGB-D-to-saved-voxel-map GICP pose at its
**actual corrected PoseStamped measurement timestamp** (not the later ROS
log time). That prevents accusing legitimate sensor/compute delay of being
registration error.

| LiDAR-outage window | Accepted depth matches | XY proxy RMS | 3D proxy RMS |
| --- | ---: | ---: | ---: |
| 125–135s | 9 | 1.612m | 1.820m |
| 135–145s | 8 | 1.670m | 1.703m |
| 145–155s | 8 | 1.841m | 1.982m |
| 155–165s | 9 | 2.310m | 2.438m |
| **125–165s total** | **34** | **1.885m** | **2.014m** |

Median actual publication age **0.084s**, 95th percentile **0.194s**,
maximum **0.201s**; all 34 measurements show latency compensation. Median
GICP fitness **0.0726**; low fitness/overlap checks are not sufficient to
guarantee a correct global XY position in repeated campus-road geometry.

This is strong evidence that tightening the publisher while continuing to
trust the SAME noisy absolute poses cannot reliably achieve 0.5m. A causal
offline experiment blending these actual measurements with existing FAST
odometry also did **not** improve the 2m-scale error. That trial was not
promoted or made into a new localisation output.

## Separate issue: dormant helper anchor does not match live Step-3 anchor

At onset of LiDAR failure the map-localizer predictor and its old anchor
initially agree. After its first depth registration, logs show about
**19–21 degrees** difference between the candidate and localizer's internal
`map<-fast` frame. Reported ~50–60m *anchor transform translation* is **not a
50m cart-position jump**. At later points the corresponding candidate and
localizer map positions differ ~6m in XYZ, much of it vertical. The physical
discontinuity gate correctly rejects these targets. Never relax this gate
solely to remove the warning. Step-3 remains the global-pose authority;
the depth failover node is a geometric measurement frontend, not a second
localisation publisher.

## What an actual accuracy improvement requires

Fix the **source map registration**, with trustworthy map-landmark constraints
and an event-time-consistent RGB-D/IMU motion prior. Reject or explicitly
mark ambiguous saved-voxel matches (similar corridor/road structures),
require independent inter-frame/visual-feature consistency, assess pitch and
depth-to-LiDAR extrinsics on the same coordinate frame, and improve geometric
conditioning. Preserve fail-closed output when map observability is low.
Once source matches approach the desired accuracy, re-run full 290s 1x healthy
and all failure modes on the primary bag, then confirm **without fitting on**
`18_sep_2026_combined.bag`.

**Current operational status:** still unresolved; no sub-0.5m result was
claimed, no deployment approval, no production change. Root-cause data:
`step3_global/lidar_fail_depth_timing_diag_20261016/RAW_DEPTH_VS_REFERENCE_TIMESTAMP_CORRECTED_20261016.json`.
