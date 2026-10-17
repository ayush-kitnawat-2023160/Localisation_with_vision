# FAST-LIVO2 recovery map processing budget — 17 Oct 2026

**LOCALISATION-ONLY experimental image:**
fast_livo2_failsafe:recovery-voxel-budget-localization-dev-20261017

**Status: REJECTED FOR PROMOTION**. The source change exists only in
step3_global/recovery_voxel_budget_localization_dev_20261017/ with a
separate ROSBAG YAML/launch. latest_repo/alive-dev and the original
cart controls are unchanged.

## Actual source change

- Modified only LIVMapper.cpp + LIVMapper.h from the immutable frozen
  single-build-rc-20261016 Docker image.
- Under a recovery-only guard, build the local voxel map from 1200
  deterministically indexed body/world points rather than 6900–8043
  already-downsampled points per scan. Ordinary LiDAR, visual map,
  saved-map registration and ESIKF state propagation are NOT altered.
- Recovery map warm-up is three scans instead of five. Normal scanning
  outside recovery retains the original clouds and estimator.
- Source parameter failsafe/recovery_voxel_seed_point_budget defaults
  to zero (disabled); only the experimental YAML enables 1200 points.

Measured local map-build durations: Sept18 285–386 ms, main
113–135 ms over three samples. No instrumented baseline build durations
exist, so this is NOT a per-build speedup claim.

## Actual 1x ROSBAG replay (3D diagnostic map proxy, NOT surveyed GT)

| Bag / failure | Metric | Frozen baseline | New recovery budget |
|---|---|---:|---:|
| Sep18 IMU | Outage RMSE | 2.557 m | 2.476 m |
| Sep18 IMU | Recovery 20s RMSE | 0.765 m | 0.778 m |
| Sep18 IMU | Largest pose gap | 0.403 s | 0.234 s |
| Sep18 IMU | Largest pose step | 0.703 m | 0.790 m |
| Main IMU | Outage RMSE | 3.494 m | 1.081 m |
| Main IMU | Recovery 20s RMSE | 2.317 m | 1.349 m |
| Main IMU | Largest pose step | 1.539 m | 0.988 m |
| Sep18 LiDAR | Outage RMSE | 3.942 m | 4.038 m |
| Sep18 LiDAR | Recovery 20s RMSE | 1.370 m | **2.335 m** |
| Sep18 LiDAR | Largest pose step | 1.564 m | 1.158 m |
| Sep18 LiDAR | Largest pose gap | 0.403 s | 0.268 s |
| Main LiDAR | Outage RMSE | 3.224 m | 2.939 m |
| Main LiDAR | Recovery 20s RMSE | 1.929 m | **2.339 m** |
| Main LiDAR | Largest pose step | 1.934 m | 1.827 m |

The big IMU-main *outage* improvement starts before the recovery-only
code executes; replay timing/match variability is a plausible explanation
and cannot be attributed to this patch.

Acceptance: Both bags and both LiDAR/IMU modes must preserve/improve
outage and recovery accuracy, keep steps <=0.85m, gaps <=0.3s, and
full-route 3D proxy RMSE <=0.5m. Candidate FAILS on LiDAR recovery
and global accuracy/continuity requirements.

LiDAR depth/PCD correspondence ambiguity remains the primary unresolved
source error. Independent surveyed GT is unavailable. The actual
latest_repo/alive-dev still uses copied FLOAM for map-frame /pose.

NEVER merge this isolated experimental source into the cart runtime.
