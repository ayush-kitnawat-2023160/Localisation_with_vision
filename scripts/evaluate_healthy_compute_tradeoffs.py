#!/usr/bin/env python3
"""Compare 1x healthy perf trials against SAME-bag loopclosed proxy, never GT.

Conservative admission gate: sizeable CPU benefit AND no material trajectory,
step, GICP or estimator health regression. Performance trials with distinct
CPU-affinity groups are indicative, not definitive benchmark comparisons.
"""
import csv
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
B=ROOT/'results/camera_lidar_imu_gps'
ref=list(csv.DictReader((ROOT/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv').open()))
rt=np.array([float(x['timestamp']) for x in ref])
xyz=np.array([[float(x[c]) for c in 'xyz'] for x in ref])

def load(name,resource=None):
    p=B/name
    z=list(csv.DictReader((p/'global_pose.csv').open()))
    t=np.array([float(a['field.header.stamp'])*1e-9 for a in z]); x=np.array([[float(a['field.pose.position.'+c]) for c in 'xyz'] for a in z])
    m=(t>=rt[0])&(t<rt[0]+90)
    r=np.stack([np.interp(t[m],rt,xyz[:,j]) for j in range(3)],axis=1)
    steps=np.linalg.norm(np.diff(x[m],axis=0),axis=1)
    g=list(csv.DictReader((p/'gicp_map_pose.csv').open())); q=np.array([float(a['field.header.stamp'])*1e-9 for a in g]);q=q[(q>=rt[0])&(q<rt[0]+90)]
    log=(p/'launch.log').read_text(errors='replace')
    out={'rmse_90s_m':round(float(np.sqrt(np.mean(np.sum((x[m]-r)**2,axis=1)))),3),
         'max_published_step_90s_m':round(float(np.max(steps)),3),
         'gicp_gap_max_90s_s':round(float(np.max(np.diff(q))),3),
         'fast_pose_gap_events':log.count('[STEP3 FAST-GUARD] FAST pose gap'),
         'imu_lidar_desync_warnings':log.count('IMU and LiDAR not synced'),
         'bag_duration_s':round(float(t[m][-1]-rt[0]),2)}
    perf=(p.parent / (resource or f'{p.name}.resource_samples.csv'))
    if perf.exists():
        samples=list(csv.DictReader(perf.open())); vals=np.array([float(a['cpu_percent'].strip('%')) for a in samples]);
        if len(vals)>4:vals=vals[2:-2]
        out['mean_cpu_percent']=round(float(np.mean(vals)),1)
        out['p95_cpu_percent']=round(float(np.percentile(vals,95)),1)
        out['cpu_sample_count']=len(vals)
    return out

cases={
  'control':('HEALTHY_VISUAL_FASTPATH_BENCH_20261015/HEALTHY_BASELINE_90S',None),
  'aggressive_feature_pruning':('HEALTHY_LOWCOMPUTE_DEV_20261014/HEALTHY_LOWCOMPUTE_90S','trial90_resource_samples.csv'),
  'four_iteration_solvers':('HEALTHY_LIGHT_SOLVER_DEV_20261014/HEALTHY_LIGHT_SOLVER_90S','trial90_resource_samples.csv'),
  'cache_and_cpu_affinity':('HEALTHY_CACHE_AFFINITY_DEV_20261014/HEALTHY_CACHE_AFFINITY_90S','trial90_resource_samples.csv'),
  'gated_visual_fastpath':('HEALTHY_VISUAL_FASTPATH_BENCH_20261015/HEALTHY_FASTPATH_90S',None),
  'lighter_pcd_gicp':('HEALTHY_PCD_LIGHT_DEV_20261015/HEALTHY_PCD_LIGHT_90S','trial90_resource_samples.csv'),
}
results={k:load(*v) for k,v in cases.items()}
base=results['control']
for k,a in results.items():
    if k=='control':continue
    a['qualifies_for_full_290s']=bool(a['rmse_90s_m']<=base['rmse_90s_m']+0.10
             and a['max_published_step_90s_m']<=base['max_published_step_90s_m']+0.10
             and a['gicp_gap_max_90s_s']<=max(1.7,base['gicp_gap_max_90s_s']+0.20)
             and a['fast_pose_gap_events']==0
             and a.get('mean_cpu_percent',1.e9)<=base['mean_cpu_percent']*0.90)
report={'comparison_reference':'SAME BAG loop-closed proxy; NOT independent ground truth',
 'bag_rate':'1x', 'duration_s':90,
 'note':'Run-to-run stochasticity and differing CPU affinity across early trials limit causal attribution. A single 90s test never proves full-run reliability.',
 'healthy_best_full_290s_preserved':'HEALTHY_GAP_HANDOFF_DEV_20261014/HEALTHY_GAP_HANDOFF_FULL_290S',
 'cases':results,
 'any_candidate_eligible_for_full':any(v.get('qualifies_for_full_290s',False) for v in results.values())}
out=B/'HEALTHY_COMPUTE_PROFILING_20261015.json';out.write_text(json.dumps(report,indent=2)+'\n');print('Saved',out)
for k,r in results.items():print(f'{k:28} RMSE={r["rmse_90s_m"]:.3f}m step={r["max_published_step_90s_m"]:.3f}m gap={r["gicp_gap_max_90s_s"]:.2f}s CPU={r.get("mean_cpu_percent","N/A")}% pass={r.get("qualifies_for_full_290s","reference")}')
