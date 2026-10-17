#!/usr/bin/env python3
"""Archive measured latency experiments without promoting code or touching runtime."""
from pathlib import Path
import csv, json, statistics, subprocess, zipfile
from collections import defaultdict
import numpy as np
R=Path(__file__).resolve().parents[1]
A=R/'results/LOW_LATENCY_DEBUG_AB_20261017';B=R/'results/LOW_LATENCY_POSE_TRANSPORT_DEV_20261017'
normal=A/'LOW_LATENCY_CONTROL_HEALTHY_75S';dbg=A/'LOW_LATENCY_CANDIDATE_HEALTHY_75S';transport=B/'POSE_TRANSPORT_CANDIDATE_HEALTHY_55S'
T0=1788852570.782609

def measure(folder,limit,lo=14.0):
 # Restrict primary output to the first `limit` seconds since common bag start.
 age=[]
 with (folder/'pose_freshness.csv').open() as f:
  for row in csv.DictReader(f):
   if row['source']!='global_pose' or not row['age_ms']:continue
   t=float(row['header_stamp_ros_sec'])-T0
   if 0<=t<limit:age.append(float(row['age_ms']))
 assert len(age)>200,(folder,len(age))
 cpu=defaultdict(list)
 with (folder/'process_cpu.csv').open() as f:
  for row in csv.DictReader(f):
   try:
    w=float(row['wall_elapsed_s'])
    if lo<=w<=limit+7.0:cpu[row['process']].append(float(row['cpu_percent']))
   except(ValueError,KeyError):pass
 with (folder/'global_pose.csv').open() as f:poses=list(csv.DictReader(f))
 stamps=np.array([float(p['field.header.stamp'])*1e-9 for p in poses]);xyz=np.array([[float(p['field.pose.position.'+k]) for k in 'xyz'] for p in poses]);m=(stamps>=T0)&(stamps<T0+limit);stamps=stamps[m];xyz=xyz[m]
 assert len(stamps)>200
 step=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
 count=sum(1 for x in (folder/'gicp_map_pose.csv').open())-1
 return dict(pose_samples=int(len(stamps)),age_samples=len(age),p50_age_ms=round(float(np.percentile(age,50)),3),
     p95_age_ms=round(float(np.percentile(age,95)),3),p99_age_ms=round(float(np.percentile(age,99)),3),
     max_pose_gap_s=round(float(np.diff(stamps).max()),4),max_step_m=round(float(step.max()),4),
     mapper_cpu_pct_core=round(statistics.mean(cpu['fastlivo_mappin']),3),
     global_match_cpu_pct_core=round(statistics.mean(cpu['step3_global_acquirer']),3),
     sensor_adapter_cpu_pct_core=round(statistics.mean(cpu['fast_livo2_sensor_adapter.py']),3),
     gicp_pose_rows_all_run=count)

full={'baseline_75s':measure(normal,75),'skip_unused_debug_75s':measure(dbg,75),'baseline_first_55s':measure(normal,55),
      'fresh_pose_transport_55s':measure(transport,55)}
repo=R/'../latest_repo/alive-dev'
clean=not subprocess.check_output(['git','-C',str(repo),'status','--porcelain']).strip()
assert clean
assert (R/'config/calibration/new_cart_measured.yaml').read_text().count('  ready: false')==1
assert 'FRESH TRANSPORT]' in (transport/'launch.log').read_text(errors='replace')
full.update(assessment='No meaningful p95 age or CPU reduction in uncongested offline run; opt-in debug and transport experiments NOT promoted',
  measured_age_semantics='rosbag simulated-clock reception minus sensor header stamp inside same container, not remote planner/NUC latency',
  no_real_cart_test=True,real_nuc_not_tested=True,new_camera_not_tested=True,
  baseline_image_unchanged=True,cart_runtime_git_clean=clean,cart_calibration_locked=True,
  deployment_approved=False)
B.mkdir(exist_ok=True)
metric=B/'LATENCY_PROFILE_COMPARISON_20261017.json';metric.write_text(json.dumps(full,indent=2)+'\n')
x=full['baseline_75s'];y=full['skip_unused_debug_75s'];a=full['baseline_first_55s'];b=full['fresh_pose_transport_55s']
text=f'''# ALiVE FAST-LIVO2 — real-time localization / compute investigation (2026-10-17)

**Decision:** Preserve the frozen single-build baseline. Do **not** promote either opt-in modification based on the current data. Zero changes to the actual cart code/planning/control/braking. New bag + physical NUC/ROS bridge transport still untested.

## Why the old /pose was late

Read-only source inspection of `latest_repo/alive-dev/src/fast_livo2/src/floam_global/src/fast_livo2_floam_localizer.cpp` shows it publishes authoritative `/pose` only **after** performing its LiDAR scan-to-map/Ceres processing (the published header keeps the original scan time). Estimator already has `queue=1` plus `tcpNoDelay` for `/pose`, so it cannot repair an upstream late pose. This explains a plausible path for reported delay, but actual car round-trip milliseconds were **not measured**.

New Step-3 publishes global poses from FAST local odometry callbacks, independent of the ~1Hz scan matching callbacks (`AsyncSpinner(3)`). That can keep output responsive while GICP catches up, provided ESIKF/ROS queues have not accumulated and confidence remains sound.

## 1× replay measurements (diagnostic ROS /clock freshness, not car latency)

| Measure | 75s untouched baseline | 75s opt-in debug-work guard |
|---|---:|---:|
| Global pose age p50 | {x['p50_age_ms']:.1f} ms | {y['p50_age_ms']:.1f} ms |
| Global pose age p95 | {x['p95_age_ms']:.1f} ms | {y['p95_age_ms']:.1f} ms |
| FAST mapper CPU, one-core % | {x['mapper_cpu_pct_core']:.2f}% | {y['mapper_cpu_pct_core']:.2f}% |
| Global matcher CPU, one-core % | {x['global_match_cpu_pct_core']:.2f}% | {y['global_match_cpu_pct_core']:.2f}% |
| Max global pose output gap | {x['max_pose_gap_s']:.3f} s | {y['max_pose_gap_s']:.3f} s |
| Max published pose step | {x['max_step_m']:.3f} m | {y['max_step_m']:.3f} m |

Debug-work candidate (`step3_global/low_latency_debug_dev_20261017`) skips unused `/Laser_map`, `/cloud_registered`, `/rgb_img` conversion and caps/decimates RViz `/path` (opt-in only). Real ROS replay demonstrated no meaningful p95 age or CPU reduction, so **not promoted**. No changes to the ESIKF or source clocks.

| Measure | Baseline first 55s | 55s opt-in fresh-transport candidate |
|---|---:|---:|
| Global pose age p50 | {a['p50_age_ms']:.1f} ms | {b['p50_age_ms']:.1f} ms |
| Global pose age p95 | {a['p95_age_ms']:.1f} ms | {b['p95_age_ms']:.1f} ms |
| FAST mapper CPU, one-core % | {a['mapper_cpu_pct_core']:.2f}% | {b['mapper_cpu_pct_core']:.2f}% |
| Global matcher CPU, one-core % | {a['global_match_cpu_pct_core']:.2f}% | {b['global_match_cpu_pct_core']:.2f}% |
| Max global pose gap | {a['max_pose_gap_s']:.3f} s | {b['max_pose_gap_s']:.3f} s |
| Max published pose step | {a['max_step_m']:.3f} m | {b['max_step_m']:.3f} m |

Transport candidate (`step3_global/low_latency_pose_transport_dev_20261017`) sets `/fast_livo2/step3_global_pose` ROS publisher queue to 1, disables latch on dynamic pose, limits FAST odometry subscription queue to 10, and enables TCP_NODELAY. Actual ROS parameter and C++ runtime log confirmed enablement. **No improvement in an uncongested single-container run**; this patch only protects against backlogged ROS transports; benchmark on both physical NUCs before promotion. All 6 freshness probe unit tests passed; source compiled; replay saved and plotted.

## When the new camera/bags arrive

Do not optimize against reference RMSE first; compare throughput and age and keep diagnostic pose accuracy only as a no-regression check. Probe available:

```bash
# In a ROS1 Noetic environment attached to the right ROS_MASTER_URI.
# Run on the actual NUC/planner-side to measure the ROS network arrival too.
python3 scripts/measure_pose_freshness.py --pose-only --global-pose-topic /pose --csv /tmp/alive_pose_age.csv
# Stop with Ctrl-C and summarize:
python3 scripts/summarize_pose_freshness.py /tmp/alive_pose_age.csv --out /tmp/alive_pose_age_summary.json
```

For Step3 development use `--global-pose-topic /fast_livo2/step3_global_pose`; monitor FAST local odom and published global pose simultaneously. Both devices need chrony verified and consistent hardware stamps. The live config currently defaults camera to `rebase_live` (first packet ROS receipt as time offset), so apparent packet freshness may hide camera capture/network delay; verify physical clocks before interpreting ROS header ages.

**Design direction for actual responsiveness:** IMU-rate ESIKF prediction (50–100Hz) + asynchronously corrected low-rate globally verified map anchor, only with a bounded prediction horizon, transform calibration, covariance/staleness gating and failover handling. A faster pose topic must not re-stamp old geometry as current; must not claim localization when unobservable. Do not connect it to `/pose` until replay + actual NUC tests pass and exactly one publisher owns the map-frame pose.

**Initial on-device engineering targets, not achieved/safety-certified:** measure p50/p95/p99 age on the planner machine; aim p95 <100–150ms and detect/reject age >300ms; ensure no accumulating queues over 30 minutes including camera loss and IMU/LiDAR recoveries, stable CPU and memory, no delayed pose jumps. Test new high-res camera in FULL_LIVO and all three failures at 1×. Measure actual camera capture stamps, transmission, and FPS. Independent surveyed GT and physical cart calibration remain required for cart deployment.

**Status:** Measurement tooling ready. No actual NUC latency or newest-camera performance demonstrated. Frozen baseline and `latest_repo/alive-dev` are unchanged; the new-cart physical calibration file is still locked.
'''
report=R/'reports/LOCALIZATION_LOW_LATENCY_COMPUTE_AUDIT_20261017.md';report.write_text(text)
files=[report,metric]+[R/'scripts'/name for name in ('measure_pose_freshness.py','summarize_pose_freshness.py','test_pose_freshness_core_20261017.py','build_low_latency_debug_dev_20261017.py','build_low_latency_pose_transport_dev_20261017.py')]
files+=list((R/'step3_global/low_latency_debug_dev_20261017').glob('*'))
files+=list((R/'step3_global/low_latency_pose_transport_dev_20261017').glob('*'))
files+=[R/'launch/fast_livo2_step3_low_latency_transport_dev_20261017.launch']
for folder in (normal,dbg,transport):
 for fname in ('freshness_summary.json','pose_freshness.csv','process_cpu.csv','metrics_exact.txt','run_metadata.txt','launch.log','global_pose.csv'):
  p=folder/fname
  if p.is_file():files.append(p)
zipout=B/'LATENCY_COMPUTE_AUDIT_AND_DEV_CODE_20261017.zip'
with zipfile.ZipFile(zipout,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
 for p in files:
  if p.is_file():z.write(p,p.relative_to(R))
print('REPORT',report)
print('JSON',metric)
print('ZIP',zipout,'bytes',zipout.stat().st_size)
print('RESULT',json.dumps({k:v['p95_age_ms'] for k,v in full.items() if isinstance(v,dict) and 'p95_age_ms' in v}))
print('NO_RUNTIME_DIFF',clean)
