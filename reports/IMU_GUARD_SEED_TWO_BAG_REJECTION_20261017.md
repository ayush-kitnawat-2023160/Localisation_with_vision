# FAST-LIVO2 — controlled IMU-quarantine source trial (17 Oct 2026)

**VERDICT: REJECTED, NOT integrated, NOT autonomous-cart-ready.**

## Implementation

Opt-in C++ pre-seed for Step-3 map publisher on the *first* implausible FAST-LIVO2 ESIKF increment during IMU loss. A recent (<1.0s) independent saved LiDAR/PCD map observation and verified map motion are required. When no such observation is fresh, the existing hold/reacquisition path is preserved. The patch never modifies ESIKF or the cart planner/bridges/DBW, and has zero changes to healthy/camera-loss/LiDAR-loss logic.

C++ source and exact diff: `step3_global/imu_guard_verified_seed_dev_20261017/`. Compiled Docker image: `fast_livo2_failsafe:imu-guard-verified-map-seed-dev-20261017` (`sha256:ee7a6e3a5b5ca6f2116ae0f6c8ebad35528d74a696a64c2dc165eb951e9ca0f3`). Launch: `launch/fast_livo2_imu_guard_verified_seed_dev_20261017.launch`.

## Measured real 1× ROSBAG replays (same scoring windows)

| Proxy diagnostic | Main 190s baseline → trial | Sep18 160s baseline → trial |
|---|---:|---:|
| Full 3D RMSE | 2.052 → 1.199 m | 1.409 → 1.727 m |
| 40s IMU-outage RMSE | 3.494 → 1.064 m | 2.557 → 2.562 m |
| First 20s recovery RMSE | 2.317 → 1.296 m | 0.765 → 2.229 m |
| Maximum published step | 1.539 → 1.279 m | 0.703 → 0.843 m |
| Largest pose gap | 0.200 → 0.200 s | 0.403 → 0.605 s |

**Neither 1× run triggered a FAST-GUARD event or the new IMU-QUARANTINE-SEED branch.** Any numerical gain on the main recording is scheduler/replay variability, not evidence that this patch worked. Sep18 recovery disagreement increased 0.765 to 2.229 m; its largest output timestamp gap grew 0.403 to 0.605 s. The candidate fails the required 0.5m proxy RMSE, 0.85m max step and 0.30s max gap gates. Results are diagnostic *map-aligned loopclosed proxies*, **NOT independently surveyed physical accuracy**. No fitting or reference-based source modification occurred.

## Preserved baseline and strict readiness conditions

- Frozen single-image eight-case 1× baseline (Oct16): `results/FINAL_SINGLE_IMAGE_RC_20261016/`; it did not pass all target gates.
- Current rejected experiment: `results/IMU_QUARANTINE_SEED_DEV_20261017/`. Candidate log, source, builds and plots are retained, not merged.
- Actual runtime repo: `latest_repo/alive-dev/`, Git tree clean; its existing FLOAM global `/pose` remains the only live authority. No vehicle-controller source was modified.
- Actual cart calibration `config/calibration/new_cart_measured.yaml` remains locked, with placeholder transforms: real LiDAR/IMU/camera and base_link offsets must be measured, never guessed from the rosbag.
- Before a release: fix distinguishable global map associations and callback timing, meet eight fixed-code replays (both bags × four modes), show <=0.5m proxy RMSE, <=0.85m steps and <=0.3s max output gaps; obtain independent surveyed accuracy and validate real NUC sensor timing/CPU + correct unique map frame publisher. Do not remap experimental `/pose` before acceptance.

**Not a completed localization release. No vehicle driving authorization.**
