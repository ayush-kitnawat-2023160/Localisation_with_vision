# GPS-free FAST-LIVO2 + LiDAR/visual global localization + KLT fallback (ROS1)

**Status: offline integrated; shadow only. NOT cleared for vehicle motion or unattended autonomous deployment.**

This is the current, conservative integration branch in `FailSafe_FastLivo2`. Do not use the older `fast_livo2_step3_*_dev.launch` experiment variants as a deployment launcher. Do not reset/clean the working tree: it contains unrelated user experiments and uncommitted work.

## Single selected architecture

```text
Live source LiDAR + MAVROS IMU (+ 5 Hz ZED left image)
  -> sensor adapter (explicit point-time units, timestamp verification)
  -> ONE FAST-LIVO2 LiDAR/visual/IMU ESIKF
  -> /fast_livo2/deployment/fast_odom_candidate   [LOCAL ONLY]
           |
           +-> Step3 Scan Context -> GICP multi-hypothesis -> LiDAR map hypotheses
           |   (GPS retrieval OFF; gyro/map pose feedback into FAST OFF)
           |
ZED X left RGB -> CPU XFeat worker + PnP -> VISUAL global map hypotheses
ZED X left RGB + registered depth + gyro -> CPU KLT + PnP+gyro -> LOCAL camera VO
           |
FAST+KLT healthy overlap -> time-aligned KLT continuation candidate
           (only during *observed* raw LiDAR timeout, max 15s, fail-closed)
           |
Sensor-health monitor: hardware/source freshness, RGB-depth time skew,
    independent pose consistency, source changes, permanent manual re-arm latch
           |
SHADOW-only fused map pose + review/diagnostics. No navigation /pose or TF.
```

### ROS topics (all non-authoritative)

| Topic | Meaning |
| --- | --- |
| `/fast_livo2/deployment/fast_odom_candidate` | One running FAST-LIVO2's actual local odometry, separately namespaced |
| `/fast_livo2/deployment/klt_local_odom_candidate` | Independent CPU KLT + PnP + gyro VO in its initial body frame |
| `/fast_livo2/deployment/klt_continuation_candidate` | KLT pose aligned to the *previous* FAST local frame during LiDAR loss |
| `/fast_livo2/deployment/step3_gicp_map_pose_candidate` | LiDAR Scan Context/GICP map matching candidate |
| `/fast_livo2/deployment/visual_global_candidate` | GPS-free XFeat visual place candidate (not a map lock) |
| `/fast_livo2/shadow_fused_map_pose` | Rate-limited map pose hypothesis, withheld on untrusted Step3 |
| `/fast_livo2/deployment/tf_candidate_only` | FAST mapper TF quarantined from the standard `/tf` channel |
| `/fast_livo2/deployment_health_status` | JSON sensor health and stop/rearm advisory (not a brake command) |
| `/fast_livo2/deployment_stop_advisory` | String advisory only; **not a physical brake or safety PLC** |
| `/fast_livo2/deployment/klt_continuation_status` | KLT alignment/readiness/fault state; no automatic handover |

**Forbidden in this deployment graph:** writing `/pose`, `/aft_mapped_to_init`, `/tf`, `/path`, `/cmd_vel`, `/initialpose`, legacy fused map pose or FAST ESIKF corrections from Step3 global candidates. All real estimator outputs and Step3 feedback publications are namespaced under `/fast_livo2/deployment/` or another explicit shadow topic. A validation probe subscribes to these forbidden topics as `AnyMsg` and fails replay if any messages appear.

## Hardware connections and prerequisites

- ROS1 Noetic on the NUC; Jetson Orin runs the ZED X camera driver and generates registered depth; LiDAR and MAVROS IMU must have genuinely synchronized timestamps. Merely receiving the ROS topics is not proof of timing accuracy.
- Explicitly verify `PointCloud2.time` units (`seconds` or `microseconds`); there is **no default live point-time unit**. Some archived LiDAR messages use negative seconds from end-of-scan; adapter shifts to scan-start before conversion. A wrongly declared hardware driver is rejected rather than heuristically guessed.
- Use a real **physically measured** LiDAR-to-base, LiDAR-to-IMU and ZED camera-to-body + ZED IMU-to-optical calibration, including the as-installed camera orientation. Current `config/calibration/zedx_current.yaml` is **OFFLINE ONLY**, inherited and not physically verified. The live guard refuses it.
- The stored maps `maps/campus_full_lio_226s_20261019` and `maps/campus_full_visual_rgbd_226s_20261020` were built from `localization_full.bag` using raw LiDAR odometry, **not surveyed/loop closed**. Global map accuracy, wrong-map rate and seasonal robustness remain unverified.
- The XFeat worker must be running on a localhost-only endpoint; preflight hashes the learned and companion ORB keyframe CSVs and calibration YAML, validates retrieval mode `learned_float_flann`, backend `xfeat_mnn`, image size, readiness and `gps_used=false`. The entire descriptor collection is not cryptographically bound by those hashes (future hardening).
- Independently wired, authenticated vehicle motion inhibit and brake acknowledgement must exist before a future authority-changing handoff; this shadow launch does not actuate a brake.

## OFFLINE end-to-end tests (safe to rerun, no overwrite)

To review the saved results:

```bash
cd ~/Desktop/ALIVE_N/FailSafe_FastLivo2
cat outputs/deployment_audit_20261021/FINAL_GPSFREE_SHADOW_REPORT_20261021.md
cat outputs/deployment_audit_20261021/HARD_CODE_BAG_LEAK_AUDIT_20261021.md
```

To execute an *isolated*, full FAST-LIVO2 + Step3 + KLT + health replay with no external worker:

```bash
BAG=localization_from2 RUN=unique_user_generated_id DURATION=40 RATE=0.75 VISUAL=false \
  ./scripts/run_deployment_gpsfree_shadow_rosbag_20261021.sh
```

For XFeat, first start its independent CPU worker against the exact learned map / ORB map / calibration (e.g., existing `fast_livo2:learned_visual_cpu_20261020` Docker image using `scripts/serve_learned_visual_global.py`). Then confirm `curl -s http://127.0.0.1:18091/health` returns a ready, GPS-disabled `xfeat_mnn` worker. Replay with `VISUAL=true` and **a unique run name**; the runner requires matching map fingerprints and refuses other running host ROS masters.

Raw LiDAR blackout ONLY in a clearly labeled offline experiment:

```bash
BAG=localization_from2 RUN=unique_outage_experiment DURATION=40 RATE=0.75 VISUAL=false \
  LIDAR_FAULT_START=20 LIDAR_FAULT_END=29 \
  ./scripts/run_deployment_gpsfree_shadow_rosbag_20261021.sh
```

This mask applies only to the raw LiDAR stream. It does **not** replay precomputed FAST-LIVO2 odometry, simulate an authoritative stop, or inject a known correct global pose. Fault parameters appear in `experiment_manifest.env`. The monitor records trajectories and the final `runtime_postconditions.json` gates missing estimator streams and forbidden publishers. An odometry candidate is required for a named LiDAR-blackout test to count as successfully *produced*, but even that doesn't prove safe driving.

Post-hoc estimate of relative drift against a **separate uninterrupted LiDAR-odometry bag** (reference is not used by the estimator):

```bash
# Needs ROS Noetic python, normally inside the fast_livo2 Docker environment
python3 scripts/compare_klt_continuation_posthoc_20261021.py \
  --run outputs/deployment_audit_20261021/YOUR_UNIQUE_FAULT_RUN \
  --lidar-only-odometry-bag outputs/lidar_global_holdout_20261019/localization_from2_lidar_only_odom.bag \
  --output outputs/deployment_audit_20261021/YOUR_UNIQUE_FAULT_RUN
```

### Existing experimental evidence before final topic quarantine

- Full `localization.bag`, 65s: 609 FAST odom, 814 KLT, 61 XFeat, 57 Step3 map, 568 fused shadow; `GLOBAL_LOCKED`, zero `/pose`.
- Full `localization_from2.bag`, 65s: 605 FAST, 826 KLT, 61 XFeat, 57 Step3; `GLOBAL_LOCKED`, zero `/pose`.
- Difficult `localization_3.bag`, 65s: 603 FAST, 797 KLT, 50 XFeat, 19 Step3 map; `MH_HANDOFF_REQUIRED`; fused shadow stops, zero `/pose`.
- With Step3->FAST ESIKF feedback explicitly disabled, 40s `localization_from2` repeat: 357 FAST, 508 KLT, 37 XFeat, 31 Step3; still `GLOBAL_LOCKED`, zero prohibited navigation topics. Freshest folder: `shadow_isolated_global_from2_40s_20261021`.
- LiDAR blackout `[20,29]` (before final topic quarantine): 110 continuing KLT candidate poses over 8.28s. Against held-out uninterrupted LiDAR odometry, relative disagreement RMSE **0.496m**, endpoint **0.830m**, 5s translation RPE median **0.460m**. These values are **NOT surveyed ground truth** and require repetition after final graph isolation.

**No data is scored as successful merely because a replay exits 0.** Static safety rules and runtime forbidden-topic gates have separate tests.

## Live shadow preparation (dry-run by default)

```bash
CALIBRATION=/absolute/physically_measured.yaml \
MAP_DB=/absolute/verified_map_db \
VISUAL_MAP_DIR=/absolute/companion_visual_map \
LEARNED_MAP_DIR=/absolute/learned_xfeat_map \
LIDAR_POINT_TIME_UNIT=seconds VISUAL=true \
  ./scripts/start_deployment_gpsfree_live_shadow_20261021.sh
```

**Do not set `EXECUTE=true` until** the required physical calibrations, LiDAR point-time convention, observed ZED/IMU clock skew, independent vehicle motion-inhibit/stop path and on-target Orin/NUC resource validation are completed. The current older calibration will correctly fail preflight. Even after a successful live **shadow** check, no independent nav authority or automatic rejoin is enabled.

### Definition of done for supervised live fallback (NOT yet achieved)

1. Physical interlocks: independent authenticated wheel-speed-zero, brake engaged, motion-controller inhibited and planner stopped, with watchdog that can actually inhibit motion even if ROS dies.
2. True sensor health: IMU/LiDAR hardware-origin time, extrinsics, camera depth validity, actual ROS scheduling jitter and Orin->NUC transport under total CPU/GPU load.
3. Recovery: sufficient independently corroborated LiDAR-visual matches, stable estimated local/map transform, wrong-branch rejection and a persistent map/odom epoch transaction with acks from all consumers; never resume from stale retained global pose.
4. Blind quality: fresh campus recordings from unseen routes / days / weather with independent measurements, no bag-specific tuning once thresholds are frozen.
5. Failure injection: LiDAR and camera dropouts, simultaneous loss, stale odom, replayed candidate, process crashes and stop/rejoin conditions; controlled hardware tests with a supervising operator.

**No map reset, wheel command or vehicle pose authority is implemented in this shadow stack.**
