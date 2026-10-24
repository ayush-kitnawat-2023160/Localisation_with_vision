# Camera scheduler investigation — 19 October 2026

## Finding

The native LIVO synchronizer changes the LiDAR estimation schedule when images are enabled. In the failing configuration, a 5 Hz camera cap admits approximately 4 images/s and also batches LiDAR into approximately 4 registration updates/s, instead of the approximately 10 scans/s used by LiDAR-only. This changes IMU propagation intervals and point-cloud deskew/registration; it is not merely a different published odometry frequency.

Disabling photometric state corrections while retaining image tracking and visual map maintenance does **not** remove the failure. Instrumentation verifies zero visual position/velocity/bias corrections in that control. Thus a bad photometric correction is not necessary to reproduce this failure. The largest estimator position corrections occur in the batched LiDAR updates.

A candidate processes scan boundaries and image timestamps in chronological order, splitting and rebasing pending points correctly and sorting their timestamps before deskew. It retains camera tracking and, in the fusion case, photometric corrections. Candidate replay results are substantially more consistent with LiDAR-only. This demonstrates a local odometry improvement, not surveyed accuracy or global localization.

## Matched comparison

Input: `localization.bag`, rate **0.75×**, camera cap **5 Hz**. Same isolated source snapshot, calibration, map-free launch and diagnostic instrumentation. No saved-map/global anchoring or synthetic sensor failure is enabled.

| Mode | Published poses | Max pose-derived speed (m/s) | Local Z span (m) | Endpoint XYZ (m) |
|---|---:|---:|---:|---|
| LiDAR + IMU | 983 | 2.99 | 6.41 | 89.44, -33.13, 6.29 |
| Original camera fusion | 399 | 2.87 | 16.50 | 87.87, -28.75, 16.17 |
| Original tracking, no visual correction | 399 | 13.48 | 15.47 | 9.59, -70.60, -15.38 |
| Candidate camera fusion | 1378 | 3.02 | 5.66 | 88.99, -32.88, 5.65 |
| Candidate tracking, no visual correction | 1378 | 3.02 | 5.70 | 89.29, -32.54, 5.69 |
| Sorting only, original scheduler | 399 | 2.85 | 15.51 | 87.36, -27.75, 14.89 |

Original native camera runs have 401 LIO events versus 987 in LiDAR-only. The candidate has approximately 1,380 LIO events: scan boundaries plus additional partial-scan updates at image timestamps. Pose counts alone must not be interpreted as LiDAR scan rates. Both primary candidate trajectories have finite positions and strictly increasing timestamps.

The original fusion run shown here has a 16.50 m Z span but no extreme speed spike. Other session/replay results differ, including the previously reported 94 m Z failure. Exact divergence severity is not deterministic across all fusion runs. Crucially, the independent session reproduced the **tracking-only** failure to numerical precision (13.48 m/s and 15.47 m Z span), and our candidate removes that failure on this bag.

The LiDAR-only replay consumed the full bag and produced 983 poses, matching the independent session's completed control. Its original harness encountered a shell EOF error after playback; its status file therefore lacks the completion marker. Do not silently interpret that missing flag as a successfully completed harness. The corrected, frozen `run_case_v2.sh` completed both primary candidate runs normally. The first cadence prototype failed initialization and was aborted; `cadence_camera_075.INVALID.txt` excludes it from analysis.

## Sorting ablation

Sorting points by relative acquisition time while retaining the original image-driven LIO schedule gives maximum pose-derived speed 2.85 m/s and Z span 15.51 m. Sorting changes the failure substantially but leaves serious vertical drift; it is not sufficient to reproduce the combined candidate's improvement. Consequently, the result supports preserving scan-boundary estimation as well as sorting, and does not establish how much either change alone contributes under all calibrations. A cadence-only, unsorted ablation was not run.

The Velodyne preprocessing path does not sort its points and the native IMU deskew loop traverses them as though time-ordered. Raw clouds contain small timestamp reversals (~0.077 ms in the initial sample). The sorting-only ablation demonstrates sensitivity to this path; it does not establish that those tiny reversals alone explain the full divergence.

## Cost and delay

Original camera-enabled run, measured wall-clock work per event:

| Stage | Median | p95 | Maximum |
|---|---:|---:|---:|
| Camera conversion/copy/enqueue | 0.59 ms | 1.45 ms | 4.30 ms |
| Complete VIO event | 8.74 ms | 14.05 ms | 26.81 ms |
| Batched LIO event | 97.28 ms | 221.36 ms | 309.30 ms |
| LiDAR-only LIO event | 52.21 ms | 86.02 ms | 127.84 ms |
| Candidate camera-enabled LIO event | 36.08 ms | 81.69 ms | 101.12 ms |
| Candidate VIO event | 4.46 ms | 7.85 ms | 10.66 ms |

The expensive work is the larger batched LiDAR registration, not image conversion or the photometric update. Median LIO input grows from 13,295 points without images to 32,517 with the original image-timed schedule; the candidate uses 11,146 points per event. Early baseline timing overlapped another diagnostic session; candidate checks ran after its replay stopped. These costs identify where work occurs but are not an isolated CPU benchmark or a real-time worst-case guarantee.

Camera callback ROS-time header age in the original fusion run is median **163.5 ms**, maximum **191.0 ms**. That is receipt age on replay, not a proven camera–LiDAR exposure clock offset. Original VIO processing age reaches approximately **400 ms**. The candidate still waits for images, with observed VIO age up to approximately **390 ms** and LIO event age up to approximately **490 ms**. It fixes estimation cadence, not delayed-image arrival. Estimator timestamps were logged to ~10 ms precision; these age values are approximate. Replay clock scaling and callback scheduling affect them. Do not put the transport lag into `img_time_offset` as a calibration value. No unbounded queue growth was observed; the first fusion run's image queue stayed at most one and LIO-observed LiDAR queue at most one.

## Calibration checks

- The supplied optical-camera←LiDAR transform reprojects 40 attached correspondences with median 3.50 px and p95 5.93 px error. CameraInfo intrinsics vary by less than 0.05 px in focal length across the four bags; no gross intrinsics mismatch was found.
- IMU/base←LiDAR calibration is inherited: RPY approximately `[0,20,10]` degrees, zero translation, height 1.5 m. The supplied attachments do not calibrate that chain.
- Combining the supplied camera←LiDAR transform with recorded TF yields base←LiDAR RPY approximately `[0.475,22.795,-2.282]` degrees and translation `[-0.0062,0.0871,0.5426]` m: a **12.77° / 0.55 m** discrepancy from the inherited values. However, recorded base←ZED-base is identity and may be a wrapper default rather than a measured mounting transform. This is a consistency alarm, not authority to replace the calibration.
- Comparing MAVROS and ZED IMUs yields a roughly **4°** gravity-direction inconsistency with the inherited chain. Gyro fits also disagree, but the recordings excite mostly yaw; their weakly constrained full rotation is unsuitable as final calibration. The diagnostic best-fit inter-IMU shifts of roughly 5–10 ms are not camera exposure offsets.
- `tf_chain_calibration_experiment.yaml` is an offline hypothetical TF-chain profile, **not activated or validated**. Physical LiDAR–IMU/base measurement and sufficiently exciting calibration motion remain necessary.

## Vertical geometry of LiDAR-only

Local Z is not gravity-aligned in every recording. Using the initial two seconds of IMU acceleration as an approximate up direction:

| Bag | Raw Z span | Gravity-aligned height span |
|---|---:|---:|
| localization | 6.41 m | 5.78 m |
| localization_from2 | 3.52 m | 4.08 m |
| localization_3 | 4.37 m | 2.08 m |
| localization_full | 23.15 m | 23.08 m |

Initial attitude explains much of localization_3's raw Z, but not the full recording's 23 m excursion. Selected broad near-horizontal ground planes give median sensor-plane distances near **2.1 m**, inconsistent with the inherited 1.5 m assumption, subject to terrain/plane-selection uncertainty. `sensor_height_m` is not an input to the raw Mapper ESIKF; changing it alone will not fix native odometry divergence. Ground slope, actual terrain altitude and drift cannot be separated conclusively without surveyed/independent measurements. XY-near revisits do not prove physical loop closure. Do not label these trajectories globally accurate or flatten all Z changes.

## Visual update safety

The live candidate now rejects non-finite projections, points behind the camera, and patches whose full pyramid/gradient sampling neighborhood would cross the image border. It checks image and patch storage before pixel access, clears invalid measurement rows, and requires at least 24 safe projected patches before applying a visual correction. Camera confidence/support uses the safe projected count. Sparse frames still maintain map data where safe but skip state correction.

The final holdout replay completed without an out-of-bounds crash or image-size rejection. The throttled log recorded safe visual updates on 21 time intervals and low-support skips on 51; those log counts are lower bounds because the messages are throttled. This does not establish visual accuracy on all motion or lighting conditions.

The matched candidate keeps visual tracking active. Existing failure-mode transitions, camera-loss recovery, global anchoring, full-speed sustained replay and resource-limited deployment were **not** validated by these tests.

The cadence candidate still waits when the camera queue is empty. A freshness-first experiment that advanced LiDAR without waiting and dropped images older than the current estimator timestamp completed the primary bag, but nearly all camera frames were stale, no safe visual update was logged, and local Z span regressed to 12.59 m. That behavior is not enabled in the launch defaults. Correct use of delayed images requires a fixed-lag/out-of-sequence estimator update that accounts for already-applied LiDAR updates; merely applying an old image to the current pose is unsafe.

## Deliverables and integration

The tested scan-cadence, partial-scan, point-order and visual-safety changes are integrated in the live `FailSafe_FastLivo2` source. `alive-dev` and the source bags were untouched. No production Docker image was retagged.

Artifact directory: `outputs/camera_investigation_20261019_1533/`.

- `comparison.png`, `run_summary.json`, `experiment_manifest.json`: plots and machine-readable measurements.
- `source/`, `snapshot.json`: instrumented baseline and original source hashes.
- `cadence_source/`: original isolated candidate and test fixture.
- The live ZED X rosbag launches enable `vio/preserve_lidar_cadence=true`; the code default remains false for other launch profiles.
- `prepare_source.py`, `prepare_cadence_candidate.py`, `run_case_v2.sh`, `analyze_runs.py`: instrumentation, exact-reproducing candidate preparation, replay and analysis.
- `preserve_lidar_cadence.patch`: minimal scheduler changes against the isolated baseline, excluding diagnostic instrumentation.
- `preserve_lidar_cadence_live_tree.patch`: additive source change used for integration; the live tree also includes the VIO guards and ZED X launch parameter.
- Per-run CSV, full ROS logs, parameter dumps, metadata, statuses and build logs are retained.
- `geometry_audit.json`, `inertial_chain_audit.json`, `tf_chain_audit.json`, `vertical_audit.json` and corresponding scripts preserve the calibration/geometry calculations.

To repeat a candidate run in the preserved investigation container (choose a fresh output name):

```bash
docker start alive_camera_investigation
docker exec -e CADENCE=true alive_camera_investigation bash /investigation/run_case_v2.sh review_camera_075 true false false 5 .75 localization.bag
```

This command uses the isolated diagnostic ROS master and read-only bag mount. Stop the container after use.

This is a tested local-odometry improvement, not a declaration that the full sensor-failure stack or global localization is validated.

## Final live-tree validation

The integrated source was rebuilt in the ROS Noetic container and replayed on `localization.bag` at 0.75× with ZED X visual fusion enabled, camera cap 5 Hz, and `vio/preserve_lidar_cadence=true`. The replay completed with 1,378 poses over 98.74 s, local Z span **5.76 m**, and maximum pose-derived speed 3.21 m/s (100 ms resampled maximum 3.02 m/s). The LiDAR-only reference is 6.41 m Z span. The live replay log recorded 45 throttled safe-update intervals, 76 low-support skips, one stale startup image, and no crash or assertion. Thus the camera-enabled trajectory stayed near the LiDAR-only vertical span while supported visual updates remained active. These are local trajectory consistency metrics, not surveyed accuracy.

The no-wait/drop-stale experiment is deliberately not enabled in the shipped rosbag launch: it completed but produced a 12.59 m Z span and no logged safe visual update. Until a fixed-lag/out-of-sequence update is implemented and tested, the cadence candidate preserves valid image-time association and can wait for camera arrival.

## Holdout recording

`localization_3.bag`, camera fusion enabled, same 0.75× rate and candidate: 859 poses, 63.41 s trajectory span, local Z span 3.46 m; completed=True. Raw maximum pose-derived speed was 8.95 m/s across a 0.103 ms, 0.92 mm interval between nearly coincident events; the maximum after resampling at 100 ms was 2.85 m/s. The log recorded safe visual updates and no crash. This is a cross-recording sanity check, not an accuracy reference.

The additional freshness-first replay completed all 99.04 s of `localization.bag`, but its 711-pose trajectory had a 12.59 m local Z span and no applied safe visual update; 64 throttled stale-image warnings were logged. Treat this as evidence that stale-image dropping alone does not solve delayed-image fusion.

The calibration consistency audit read 300 near-stationary IMU samples and four ground-plane scans from `localization.bag`. Its median plane-vs-inherited-gravity angle was 2.45°, with fitted sensor-to-ground offsets near 1.99–2.10 m. The fit is explicitly biased by the inherited orientation and selected ground slab; it cannot identify LiDAR-to-IMU translation or prove sensor mounting. The camera-to-LiDAR transform passed its independent 40-point reprojection check. Do not substitute the plane fit for physical mounting measurement.
