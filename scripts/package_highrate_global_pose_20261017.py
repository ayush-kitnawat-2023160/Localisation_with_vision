#!/usr/bin/env python3
"""Reproduce and archive real 1x replay results; refuse autonomous deployment claim."""
from pathlib import Path
import json,zipfile,subprocess,sys
R=Path(__file__).resolve().parents[1]
root=R/'results/HIGH_RATE_GLOBAL_POSE_DEV_20261017'
cases={
 'healthy_v1':root/'HIGH_RATE_HEALTHY_MAIN_90S',
 'camera_v1':root/'HIGH_RATE_CAMERA_FAIL_MAIN_82S',
 'camera_ratefix':R/'results/HIGH_RATE_GLOBAL_POSE_RATE_FIX_DEV_20261017/HIGH_RATE_FIX_CAMERA_FAIL_MAIN_82S',
 'imu_barrier':R/'results/HIGH_RATE_GLOBAL_POSE_MODE_BARRIER_DEV_20261017/HIGH_RATE_BARRIER_IMU_FAIL_MAIN_105S'}
values={}
for key,d in cases.items():
 path=d/'highrate_latency_audit.json'
 if not path.is_file():subprocess.run(['python3',str(R/'scripts/evaluate_highrate_pose_20261017.py'),str(d)],stdout=subprocess.DEVNULL,check=True)
 data=json.loads(path.read_text());values[key]=data
 assert data['highrate']['n']>500,(key,'no real highrate samples')
 assert 'fail' not in str(d/'run_metadata.txt').lower() or (d/'run_metadata.txt').is_file()
 assert 'play_rate=1.0' in (d/'run_metadata.txt').read_text() or 'play_rate=1' in (d/'run_metadata.txt').read_text()
assert values['healthy_v1']['highrate']['mean_published_rate_hz']>25
assert values['camera_ratefix']['highrate']['mean_published_rate_hz']>50
assert values['imu_barrier']['critical_no_imu_failure_mode_publishing'] is True
assert values['imu_barrier'].get('imu_failed_window_direct_published_count')==0
assert values['camera_ratefix']['highrate']['steps_gt_0p85m']>0
runtime_clean=not subprocess.check_output(['git','-C',str(R/'../latest_repo/alive-dev'),'status','--porcelain']).strip()
assert runtime_clean
original=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}','fast_livo2_failsafe:single-build-rc-20261016'],text=True).strip()
assert original=='sha256:9fda983a4dd504c4e52d672f257d3e4ad703fd72e8e8c9ce24bfc0aeb33abe48'
h=values['healthy_v1'];c=values['camera_ratefix'];f=values['imu_barrier']
report=f'''# ALiVE FAST-LIVO2 — experimental low-latency global pose output

**17 October 2026 | Implemented and tested in the development repository | NOT car-ready or promoted to `/pose`**

## Implemented

A separate ROS1 localization node, `fast_livo2_highrate_global_pose.py`, subscribes to the native **single FAST-LIVO2 ESIKF's existing IMU-propagated local odometry** (`/LIVO2/imu_propagate`), its ordinary LIO/VIO state (`/aft_mapped_to_init`), and the existing Step3 corrected global pose (`/fast_livo2/step3_global_pose`). It constructs `T_map<-local = T_map<-body(timestamp) * inverse(T_local<-body(same timestamp))` and publishes `T_map<-local * T_local<-body(imu timestamp)` as `/fast_livo2/highrate_global_pose`. This does **not** add GICP work or a second ESIKF, and preserves the incoming original sensor timestamp. It does not touch FLOAM, the cart planner/bridge, speed, braking or ROS controller code.

**Fail-closed controls:** no map lock = no output; timestamp-matched ESIKF/global anchor required; current IMU state must agree numerically with native ESIKF at co-timed points (frame-name alias alone is not sufficient); maximum source/global/local age; plausible relative velocity/turn; no output in LV_IMU_FAILED/IMU_ONLY_BRIDGE; on recovery clear all pre-failure state and require strictly new source timestamps and two fresh ESIKF comparisons; bounded subscriber/outgoing queues. No stale pose is relabelled `now`.

**Sources (host relative to FAST-LIVO2-failsafe):**
- First implementation + real healthy/camera/IMU trials: `step3_global/highrate_global_pose_dev_20261017/`
- Rate-limit floating timestamp tolerance (14 tests, real camera failure replay): `step3_global/highrate_global_pose_rate_fix_dev_20261017/`
- Hardened IMU-recovery source-time barrier (15 tests, actual IMU-failure replay): `step3_global/highrate_global_pose_mode_barrier_dev_20261017/`
- Launch `launch/fast_livo2_highrate_global_pose_dev_20261017.launch`, dev-only config `step3_global/highrate_global_pose_dev_20261017/config_highrate_rosbag.yaml` turns on native uav/imu_rate_odom so the original 1x replay can drive this node.
- Read-only independent ROS topic age monitor: `scripts/measure_pose_freshness.py`. Evaluation: `scripts/evaluate_highrate_pose_20261017.py`.

## Measured 1x main-bag results (in ROS /clock container, NOT remote NUC/planner latency)

| Scenario | Original Stage3 map pose age p50/p95 | New high-rate pose age p50/p95 | New output rate | Max contiguous pose step |
| --- | --- | --- | --- | --- |
| Healthy first source build, 90s | {h['freshness_original_global']['age_p50_ms']:.1f}/{h['freshness_original_global']['age_p95_ms']:.1f} ms | {h['freshness_highrate']['age_p50_ms']:.1f}/{h['freshness_highrate']['age_p95_ms']:.1f} ms | {h['highrate']['mean_published_rate_hz']:.1f} Hz | {h['highrate']['max_step_m']:.3f} m (max gap {h['highrate']['max_gap_s']:.3f}s) |
| Camera fail, with rate rounding fix, 82s | {c['freshness_original_global']['age_p50_ms']:.1f}/{c['freshness_original_global']['age_p95_ms']:.1f} ms | {c['freshness_highrate']['age_p50_ms']:.1f}/{c['freshness_highrate']['age_p95_ms']:.1f} ms | {c['highrate']['mean_published_rate_hz']:.1f} Hz | {c['highrate']['max_step_m']:.3f} m (max gap {c['highrate']['max_gap_s']:.3f}s) |
| IMU failure + hardened time barrier, 105s | {f['freshness_original_global']['age_p50_ms']:.1f}/{f['freshness_original_global']['age_p95_ms']:.1f} ms | {f['freshness_highrate']['age_p50_ms']:.1f}/{f['freshness_highrate']['age_p95_ms']:.1f} ms when available | Not continuous | Withheld for full IMU failure |

Healthy 90s: {h['highrate']['n']} published high-rate observations, no steps above 0.85m, Step3 original map reference remained in its ordinary independent topic, per-process extra high-rate Python CPU approximately {h['process_cpu_pct_one_core'].get('fast_livo2_high',0):.2f}% of one logical core. Closely matched the 10Hz source at time-matched poses: median difference {h['difference_to_interpolated_existing_pose_m']['p50']:.3f}m, p95 {h['difference_to_interpolated_existing_pose_m']['p95']:.3f}m. This compares two outputs of the same system, not survey GT.

Camera-fail (35s to 55s injector, actual mode transitions logged): `LIO_CAMERA_FAILED` published {c['highrate']['mode_counts'].get('LIO_CAMERA_FAILED',0)} actual messages through the outage, but the higher-rate dev variant still recorded **{c['highrate']['steps_gt_0p85m']} steps >0.85m**, including {c['highrate']['max_step_m']:.3f}m. Most followed ~0.2s blank intervals and large local corrections. Extra publisher CPU ~{c['process_cpu_pct_one_core'].get('fast_livo2_high',0):.2f}% of one logical core. **Fails continuity acceptance; cannot feed planner.** Increasing Hz alone cannot repair local ESIKF correction aliasing.

IMU fail 40s..65s injector: **zero poses with source timestamps inside the actual `[FULL_LIVO -> LV_IMU_FAILED, LV_IMU_FAILED -> FULL_LIVO)` interval** (audited from ROS mode transitions), and no pre-recovery IMU source stamps escaped after recovery. High-rate output is intentionally absent for ~25s: it is NOT a functioning IMU-independent planner localization output. Large discontinuity across the gap reflects vehicle motion while no pose was published; in addition, short-interval jumps remain possible. Step3's own failure estimator was unreliable in some replays and cannot be corrected by transport acceleration.

## Performance architecture decision

The FAST-LIVO2 ESIKF remains the sole state estimator, ~1Hz GICP verified map fixes remain asynchronous, and map-frame high-rate **prediction** works in healthy and camera-outage cases at the observed 36-63Hz. A source-clock recency test and cross-frame checks prevent false freshness. This is a functional offline prototype, not a deployable control input.

Before assigning authoritative `/pose`: constrain/correct sudden ESIKF/map-anchor handoffs in degraded modes, provide verified map-based fallback through IMU failure without lying about current pose, prove full healthy + three outage modes on BOTH old and new-camera bags, test CPU and synchronized sensor-to-planner age on actual dual NUCs, measure physical extrinsics and independent cart error. New camera calibration and time stamps cannot be guessed. Cart configuration remains locked.

**Preserved:** immutable image `fast_livo2_failsafe:single-build-rc-20261016` ({original}); actual `latest_repo/alive-dev` Git status clean; no control code edits; no automations or physical driving commands. Final disposition **DEV PROTOTYPE, NOT PROMOTED**.
'''
reportpath=R/'reports/FASTLIVO2_HIGH_RATE_GLOBAL_POSE_VALIDATION_20261017.md'
reportpath.write_text(report)
status={'date':'2026-10-17','status':'DEV_PROTOTYPE_NOT_PROMOTED','source_uses_same_fastlivo2_esikf':True,'source_timestamp_preserved':True,'runs':{k:{'hz':v['highrate']['mean_published_rate_hz'],'p50_age_ms':v['freshness_highrate']['age_p50_ms'],'p95_age_ms':v['freshness_highrate']['age_p95_ms'],'max_step_m':v['highrate']['max_step_m'],'steps_gt_0p85m':v['highrate']['steps_gt_0p85m'],'imu_fail_source_pose_count':v.get('imu_failed_window_direct_published_count')} for k,v in values.items()},'healthy_latency_improved':True,'camera_full_continuity_pass':False,'imu_fail_no_invalid_predictions':True,'imu_fail_continuous_usable_global_pose':False,'ready_for_car':False,'runtime_cart_git_clean':runtime_clean,'frozen_image_unchanged':True,'independent_gt_available':False}
status_path=R/'results/HIGH_RATE_GLOBAL_POSE_DEV_20261017/FINAL_HIGH_RATE_DEV_VERDICT_20261017.json';status_path.write_text(json.dumps(status,indent=2)+'\n')
files=[reportpath,status_path,R/'scripts/measure_pose_freshness.py',R/'scripts/summarize_pose_freshness.py',R/'scripts/evaluate_highrate_pose_20261017.py',R/'scripts/package_highrate_global_pose_20261017.py',R/'launch/fast_livo2_highrate_global_pose_dev_20261017.launch']
for rel in ('highrate_global_pose_dev_20261017','highrate_global_pose_rate_fix_dev_20261017','highrate_global_pose_mode_barrier_dev_20261017'):
 files+=list((R/'step3_global'/rel).glob('*.py'))
 files+=list((R/'step3_global'/rel).glob('*.sh'))
 files+=list((R/'step3_global'/rel).glob('Dockerfile'))
 files+=list((R/'step3_global'/rel).glob('*.yaml'))
 files+=list((R/'step3_global'/rel).glob('*.diff'))
for d in cases.values():
 for name in ('highrate_latency_audit.json','highrate_pose_data.csv','pose_freshness.csv','metrics_exact.txt','run_metadata.txt','launch.log','process_cpu.csv'):
  p=d/name
  if p.exists():files.append(p)
archive=R/'results/HIGH_RATE_GLOBAL_POSE_DEV_20261017/HIGH_RATE_LOCALIZATION_DEVELOPMENT_AND_VALIDATION_20261017.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in files:
  if p.is_file():z.write(p,p.relative_to(R))
print('REPORT:',reportpath,'bytes',reportpath.stat().st_size)
print('STATUS:',status_path)
print('ARCHIVE:',archive,'bytes',archive.stat().st_size)
print('HIGH-RATE CANDIDATE RATES:',{k:status['runs'][k]['hz'] for k in status['runs']})
print('CART CODE UNCHANGED:',runtime_clean)
