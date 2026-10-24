# October 19, 2026 — ZED X / VLP-16 ROS bag integrity and FAST-LIVO2 sanity report

## Answer

**No evidence of ROS-bag file corruption.** All four recorded bags were read end-to-end using `rosbag.Bag.read_messages(raw=True)`, and each bag's serialized-message count matched its index exactly. The original LiDAR / MAVROS IMU / ZED registered RGB-D sensor-header timestamps are monotonic with normal data rates and no long missing intervals.

**But camera-enabled FAST-LIVO2 is NOT reliable on every bag.** Running normal FAST-LIVO2 without map-failsafe or global map nodes shows that LIO alone is temporally continuous on all three shorter bags. A genuine visual-enabled run is unstable on `localization.bag` despite intact sensor streams. Accuracy against surveyed geometry remains unverified for all trajectories.

## Full serialized-message integrity audit

| Recording | Duration (s) | Indexed msgs / fully read msgs | LiDAR frames | MAVROS IMU | ZED RGB | ZED registered depth | Result |
|---|---:|---:|---:|---:|---:|---:|---|
| `localization.bag` | 99.54 | 55,549 / 55,549 | 987 | 9,955 | 1,290 | 1,290 | PASS |
| `localization_from2.bag` | 88.33 | 49,474 / 49,474 | 876 | 8,833 | 1,156 | 1,155 | PASS |
| `localization_3.bag` | 64.31 | 36,059 / 36,059 | 638 | 6,432 | 846 | 846 | PASS |
| `localization_full.bag` | 226.09 | 126,606 / 126,606 | 2,241 | 22,610 | 2,949 | 2,948 | PASS |

All files collectively: **267,688 of 267,688 serialized messages read**. No indexed count mismatches and no malformed standard sensor headers. No negative (out-of-order) timestamp steps on the four key sensor topics.

Key observed timestamp properties:

- Maximum LiDAR header-stamp gap approximately **0.102 s** in all four recordings.
- Maximum MAVROS IMU header-stamp gap approximately **0.011 s** in all four recordings.
- Median nearest LiDAR-to-IMU stamp offset: **2.45–2.53 ms** across the bags; this is *sample spacing*, not a calibrated clock-offset estimate.
- RGB and registered-depth header timestamps: **0 ms median/p95 nearest-frame separation**.
- Median bag arrival minus sensor-header stamp: **~0.15–0.16 s for ZED RGB/depth**, **~0.001–0.002 s for LiDAR/IMU**. This signals pipeline/recording latency, not necessarily an actual time-base mismatch.
- The RGB/depth counts differ by one frame in `localization_from2` and `localization_full`; this is not evidence of corruption. Timestamp distribution and pairing should be taken into account at processing time.

**What PASS does not mean:** raw chunk readability and standard ROS headers do not independently certify that every LiDAR point, camera pixel, physical mounting transform or GPS reading is correct. This is a structural and timestamp audit, not ground-truth validation.

## Independent local-odometry baselines on the THREE OTHER bags

All runs used the existing `fast_livo2:dev` Docker image and current ZED X calibration, **one FAST-LIVO2 estimator**, no map matcher, global pose correction or failure-injection node. The sensor adapter preserved the recorded camera timestamps.

- **LIO** (`/common/img_en=0`, `failsafe/enable=false`): replay rate **1.0×**.
- **Native visual-enabled LIVO** (`/common/img_en=1`, `failsafe/enable=false`): replay rate **0.75×**. `/common/img_en=1` was directly verified in ROS for the corrected visual runs.

| Bag | LIO poses / max gap / maximum speed / Z span | Visual-on LIVO poses / max gap / maximum speed / Z span |
|---|---|---|
| `localization.bag` | **983 / 0.102 s / 2.99 m/s / 6.41 m** | **399 / 0.333 s / 28.06 m/s / 94.19 m — FAIL** |
| `localization_from2.bag` | **872 / 0.103 s / 2.87 m/s / 3.52 m** | **350 / 0.333 s / 4.80 m/s / 9.46 m — continuous, vertical consistency questionable** |
| `localization_3.bag` | **634 / 0.102 s / 2.77 m/s / 4.37 m** | **258 / 0.333 s / 2.70 m/s / 5.78 m — continuous** |

The measured intervals in native-visual mode are approximately **0.25 s median** rather than LiDAR-only's approximately **0.101 s**, giving ~4 visual-mode pose messages per sensor second. Different frame-processing costs and update scheduling are possible explanations. The observation alone does not identify the slow operation.

### Matched-rate control on `localization.bag`

A second **camera-OFF** replay was conducted at **0.75×**, the *same* rate as the truly camera-ON replay:

- LIO @1.0×: **983 poses**, span **99.04 s**, max gap **0.102 s**, max estimated speed **2.99 m/s**, Z span **6.41 m**.
- LIO @0.75×: **983 poses**, span **99.04 s**, max gap **0.102 s**, max estimated speed **2.99 m/s**, Z span **6.41 m**.
- LIVO @0.75×, visual actually ON: **399 poses**, span **98.50 s**, max gap **0.333 s**, max estimated speed **28.06 m/s**, Z span **94.19 m**.

The camera-enabled run first exceeds **5 m/s at approximately 56.9 s** into the LiDAR header timeline; **155 inter-pose steps exceed 5 m/s**. Its final estimated Z is approximately **−89.53 m**, versus **+6.29 m** on the continuous LIO control. Such behavior cannot be attributed to the known file-index/timestamp integrity checks, and rate alone does not explain this particular case. It is a strong indicator of a faulty **visual-enabled estimator computation/configuration/timing path** that requires narrower experiments.

### CRITICAL test-harness correction

An earlier launch expression used the Python-bool/string comparison:

    $(eval 1 if arg('enable_visual') == 'true' else 0)

This silently set `/common/img_en=0` even when `enable_visual:=true`. Therefore **the previous `baseline_full_20261019` run and the first runs in `sanity_*_livo_20261019` folders are NOT valid visual-on tests**. The launcher has been corrected to use native ROS launch `if`/`unless` groups and now **asserts the actual `/common/img_en` ROS parameter before starting replay**. Invalidly labeled folders have been marked; the report above uses only the **`*_livo_verified_20261019`** folders for visual-on claims. A *new* full-bag visual-on replay has NOT been performed in this comparison.

This also means the previous full-bag catastrophe at 0.75×, though real, **cannot** be causally blamed on camera updates from that initial mislabeled experiment. The full bag's LiDAR-only replay at 1.0× completed end to end with 2,237 poses, but showed significant vertical variation. The earlier map-building run covered only ~127.8 s because its initial pose was delayed to ~37.25 s and its playback stopped at ~165.19 s; neither phenomenon proves the bag is truncated.

## What to investigate next

1. **Time-bound the visual path**: profile image callback arrival, deserialization, sparse-direct feature matching, visual-map update, and ESIKF residual/state correction. Monitor raw image stamp versus actual processing stamp; do NOT equate the ~155 ms arrival lag with a true sensor time offset.
2. **Disentangle CPU starvation from an incorrect visual update**: run images enabled with visual state correction disabled, then selectively enable state corrections with strict innovation/outlier gating. Keep the same bag and playback rate.
3. **Audit frames and sensor extrinsics**: check ZED optical-frame conventions and LiDAR–IMU / LiDAR–camera transforms, especially for vertical drift. Cross-check start/end height against independent physical landmarks, not merely GPS altitude.
4. **Reassess the long full-bag timing** at matched playback rates after confirming the image-enable launch parameter. A simple success/failure on shorter bags cannot certify the full map trajectory.
5. **Only after the odometry trajectory is credible**, rebuild the LiDAR and visual map from optimized poses and commence held-out global-localization tests.

## Reproduce and visualize

From `~/Desktop/ALIVE_N/FailSafe_FastLivo2`:

```bash
python3 scripts/report_looooo_sanity.py
cat outputs/looooo_integrity_20261019/sanity_report.md
xdg-open outputs/looooo_integrity_20261019/localization_xy.png
xdg-open outputs/looooo_integrity_20261019/localization_z.png
xdg-open outputs/looooo_integrity_20261019/localization_speed.png
xdg-open outputs/looooo_integrity_20261019/localization_from2_xy.png
xdg-open outputs/looooo_integrity_20261019/localization_3_xy.png
```

Inspect per-bag structural audits:

```bash
cat outputs/looooo_integrity_20261019/localization.json
cat outputs/looooo_integrity_20261019/localization_from2.json
cat outputs/looooo_integrity_20261019/localization_3.json
cat outputs/looooo_integrity_20261019/localization_full.json
```

Diagnostic code: `scripts/audit_looooo_rosbags.py`, `scripts/run_baseline_zedx_full.sh`, `launch/fast_livo2_baseline_zedx_rosbag.launch`, and `scripts/report_looooo_sanity.py`. All source bags were mounted read-only. Neither the maps nor `alive_dev` were modified by these checks.
