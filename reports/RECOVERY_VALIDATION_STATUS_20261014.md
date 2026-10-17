# FAST-LIVO2 failsafe — 14 October 2026 validation status

**NOT approved for physical golf-cart deployment.** Original default ROS launch and production Docker tag were not promoted/overwritten. The original development reference is a loop-closed trajectory **proxy, not independent ground truth**.

## Reference / known-good experiments

- Dataset: `camera_lidar_imu_gps.bag`, 290 s, 1×; single injected sensor outage between 125–165 s.
- Preserved complete four-mode baseline: `results/camera_lidar_imu_gps/LATEST_20261013_1X_290S/`.
- Preserved experimental healthy-loop result: `results/camera_lidar_imu_gps/LOOP_BOUNDED_DEV_20261014/HEALTHY_TURN_FASTCATCH_290S/`; 42–60 s loop reference disagreement RMSE 0.937 m vs old 3.928 m; 18 verified PCD fixes vs 9, longest GICP gap 1.31 s vs 9.08 s; pass healthy *development proxy* quality gate. **Not independently surveyed.**
- Prior `CAMERA_FAIL_TURN_FASTCATCH_290S` passes comparison regression. Prior `LIDAR_FAIL_TURN_FASTCATCH_REPLAY_290S` and `IMU_FAIL_TURN_FASTCATCH_REPLAY_290S` fail it, particularly post-failure.

## New diagnoses

1. The turn-sensitive relative-motion limiter introduced for healthy-loop publication (`tracking_publish_relative_motion_limit_enable=true`, a 7°/s threshold and 8 s hold, 1.5 m/s correction during held turns) is absent from the older baseline launch. Late post-failure publication can accumulate metres of correction backlog despite current PCD/GICP anchors.
2. In long LiDAR outages, a fixed `tracking_global_reacquire_vertical_slack_m=3` checks candidate heights against a **stale** pre-outage map pose. Following a hill/grade, this can reject otherwise well-fitting global map candidates. An experimental capped elapsed-time vertical allowance is implemented, not certified.
3. A map-height-following experiment can prevent local-FAST vertical bias from influencing the output when verified PCD matches are fresh, but it must never be considered an independent localization measurement or used without appropriate safety monitoring.

## Full 290 s, 1× LiDAR-outage development replays

| Test results folder | Full proxy 3D RMSE | 125–165 s proxy RMSE | Final proxy error | Decision |
| --- | ---: | ---: | ---: | --- |
| `LATEST_20261013_1X_290S/LIDAR_FAIL` (old baseline) | 1.534 m | 2.170 m | 0.829 m | Reference baseline, still not deployable |
| `LOOP_BOUNDED_DEV_20261014/LIDAR_FAIL_TURN_FASTCATCH_REPLAY_290S` | 2.076 m | 2.394 m | 5.579 m | Rejected |
| `RECOVERY_CATCHUP_DEV_20261014/LIDAR_FAIL_CATCHUP_290S` (shorter turn hold) | 1.632 m | 2.376 m | 3.241 m | Rejected; also worse loop pose jump |
| `RECOVERY_Z_ONLY_DEV_20261014/LIDAR_FAIL_CATCHUP_290S` | 1.924 m | 2.587 m | 3.562 m | Rejected |
| `RECOVERY_MAPZ_DEV_20261014/LIDAR_FAIL_CATCHUP_290S` (map-height following) | 17.034 m | 5.874 m | 34.149 m | Rejected; lost global matching after outage |
| `RECOVERY_RELOCK_DEV_20261014/LIDAR_FAIL_CATCHUP_290S` (age-aware reacq) | 1.853 m | 2.769 m | 4.189 m | Rejected; map relock restored but trajectory worse |

- All above test data are stored under `results/camera_lidar_imu_gps/`. The last valid development proxy quality gates are recorded in each experiment directory. No older results were deleted.
- `POSTCRITICAL_BASELINE_DEV_20261014/LIDAR_FAIL_CATCHUP_290S` combined the healthy-loop publishing behavior with optional return to baseline publishing after a critical failure. **Incomplete**: 206.5 s of bag only, not 290 s. Docker events recorded two explicit `kill` events, and `die` exit code **137**. The cause/actor has not been established. Its limited-window RMSE for 185–205 s was 2.244 m vs old baseline 2.193 m; 125–165 s was 5.522 m vs old 2.170 m. Neither a quality pass nor a valid full-run result. Do not merge the truncated run into full-run comparisons.

## Source, build and gate audit

- Opt-in recovery logic in `src/step3_global_acquirer.cpp`, disabled by default unless a development launch explicitly enables it. Development launch files include `launch/fast_livo2_step3_global_outage_relock_dev.launch` and `launch/fast_livo2_step3_global_postcritical_baseline_dev.launch`.
- Built experimental Docker images: `fast_livo2_failsafe:recovery-z-catchup-dev-20261014`, `fast_livo2_failsafe:recovery-mapz-dev-20261014`, `fast_livo2_failsafe:recovery-relock-dev-20261014`, and `fast_livo2_failsafe:postcritical-baseline-dev-20261014`; build logs in matching result directories.
- Strict gate `scripts/evaluate_recovery_catchup_gate.py` checks actual outage/recovery, whole-run, outage, post-recovery, end-of-run, and maximum pose-step regressions. Current runs were checked and rejected automatically; no candidate was promoted.
- Sequential dev runners are saved as `scripts/run_recovery_*validation_20261014.sh` and `scripts/run_postcritical_baseline_validation_20261014.sh`.

## Remaining blockers / next decision

1. Prevent uncoordinated container kills and verify a new full replay can complete (use an isolated Docker run name and preserve the test output lock). A kill with exit 137 is not a localization accuracy result.
2. Debug `FAST` versus verified PCD motion in post-critical recovery, especially the 125–185 s high-error interval and turns near 260–290 s. Keep healthy-loop behavior protected by a separate gate; do not globally relax GICP fitness, yaw, or branch checks.
3. Require a complete 290-second pass for healthy, camera, LiDAR and IMU modes on the same frozen image, with saved PCD overlays and error curves. Only then evaluate the untouched `18_sep_2026_combined.bag` holdout. Neither the development nor holdout full acceptance criterion has been met today.
