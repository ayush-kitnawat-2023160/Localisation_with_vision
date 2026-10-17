#!/usr/bin/env python3
"""Score the isolated loop cross-keyframe gate, including 40–60s gap and CPU.

Uses diagnostic SAME-BAG reference, not independent GT. Results are not a
deployment approval. Require both gap and smoothness to remain acceptable.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parents[1]
r=root/'results/camera_lidar_imu_gps'
out=r/'HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015'
reference=root/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv'
with reference.open() as f: rr=list(csv.DictReader(f))
rt=np.array([float(x['timestamp']) for x in rr])
rp=np.array([[float(x[c]) for c in 'xyz'] for x in rr])
origin=rt[0]

def rows(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))

def poses(path):
    data=rows(path)
    t=np.array([float(row['field.header.stamp'])*1e-9 for row in data])
    p=np.array([[float(row['field.pose.position.'+c]) for c in 'xyz'] for row in data])
    return t,p

def score(folder,duration):
    t,p=poses(folder/'global_pose.csv')
    valid=(t>=origin)&(t<origin+duration)
    t=t[valid];p=p[valid]
    if len(t)<duration*8 or t[-1]-origin<duration-5 or np.any(np.diff(t)<=0) or not np.isfinite(p).all():
        raise ValueError(f'incomplete/nonmonotonic poses: {folder}')
    gt=np.stack([np.interp(t,rt,rp[:,i]) for i in range(3)],axis=1)
    e=np.linalg.norm(p-gt,axis=1)
    dt=np.diff(t);dp=np.diff(p,axis=0);step=np.linalg.norm(dp,axis=1)
    good=(dt>=0.045)&(dt<=0.25)
    v=dp/dt[:,None]
    acc=np.linalg.norm(np.diff(v,axis=0)/((dt[1:]+dt[:-1])[:,None]/2),axis=1)
    good_a=good[:-1]&good[1:]
    g,_=poses(folder/'gicp_map_pose.csv')
    g=g[(g>=origin)&(g<origin+duration)]
    if len(g)<duration*.6 or good_a.sum()<duration*6:
        raise ValueError(f'incomplete accepted map matches: {folder}')
    gaps=np.diff(g)
    gl=np.argmax(gaps)
    loop=(t>=origin+40)&(t<origin+60)
    log=(folder/'launch.log').read_text(errors='replace')
    stats_path=folder.parent/(folder.name+'.resource_samples.csv')
    cpu=np.array([float(x['cpu_percent'].replace('%','')) for x in rows(stats_path) if x['cpu_percent']]) if stats_path.exists() else np.array([])
    cpu=cpu[2:-2] if len(cpu)>6 else cpu
    result={
        'proxy_rmse_3d_m':round(float(np.sqrt(np.mean(e**2))),4),
        'loop_40_60_proxy_rmse_m':round(float(np.sqrt(np.mean(e[loop]**2))),4),
        'max_pose_step_m':round(float(step.max()),4),
        'accel_p95_mps2':round(float(np.percentile(acc[good_a],95)),3),
        'accel_p99_mps2':round(float(np.percentile(acc[good_a],99)),3),
        'max_gicp_gap_s':round(float(gaps[gl]),3),
        'worst_gicp_gap_window_s':[round(float(g[gl]-origin),2),round(float(g[gl+1]-origin),2)],
        'gicp_fixes':len(g),
        'loop_bridge_accepted':log.count('[STEP3 LOOP-MOTION] ACCEPT'),
        'loop_bridge_checked':log.count('[STEP3 LOOP-MOTION] candidate='),
        'fast_pose_gap_warnings':log.count('[STEP3 FAST-GUARD] FAST pose gap'),
        'avg_cpu_percent':round(float(np.mean(cpu)),1) if len(cpu) else None,
        'cpu_samples':len(cpu),
        'duration_s':round(float(t[-1]-origin),2)}
    if duration==290:
        result['end_proxy_error_m']=round(float(e[-1]),3)
    return result

cli=argparse.ArgumentParser()
cli.add_argument('--duration',type=int,choices=[90,290],default=90)
cli.add_argument('--baseline-name',default=None)
cli.add_argument('--candidate-name',default=None)
a=cli.parse_args()
if a.duration==90:
    baseline_folder=out/'HEALTHY_BASELINE_90S'
    if not (baseline_folder/'metrics_exact.txt').exists():
        baseline_folder=r/'HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/HEALTHY_ORB_CACHE_90S'
else:
    baseline_folder=r/'HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/HEALTHY_ORB_CACHE_290S'
if a.baseline_name:
    baseline_folder=out/a.baseline_name
candidate_folder=out/(a.candidate_name or f'HEALTHY_BRIDGE_{a.duration}S')
b=score(baseline_folder,a.duration)
c=score(candidate_folder,a.duration)
best=score(r/'HEALTHY_GAP_HANDOFF_DEV_20261014/HEALTHY_GAP_HANDOFF_FULL_290S',290) if a.duration==290 else None
quality=(c['proxy_rmse_3d_m']<=b['proxy_rmse_3d_m']+.10 and
         c['loop_40_60_proxy_rmse_m']<=b['loop_40_60_proxy_rmse_m']+.10 and
         c['max_pose_step_m']<=b['max_pose_step_m']+.06 and
         c['accel_p95_mps2']<=b['accel_p95_mps2']*1.10 and
         c['max_gicp_gap_s']<=b['max_gicp_gap_s']+.35 and
         c['gicp_fixes']>=.95*b['gicp_fixes'] and
         c['fast_pose_gap_warnings']==0)
if best is not None:
    quality=(quality and c['proxy_rmse_3d_m']<=best['proxy_rmse_3d_m']+.10 and
             c['max_pose_step_m']<=best['max_pose_step_m']+.06 and
             c['max_gicp_gap_s']<=best['max_gicp_gap_s']+.35)
better_gap=(c['max_gicp_gap_s']<=b['max_gicp_gap_s']-.3)
report={'duration':a.duration,'reference_independent_ground_truth':False,
        'baseline_folder':str(baseline_folder),'candidate_folder':str(candidate_folder),
        'baseline':b,'historical_best':best,'candidate':c,'quality_gate':bool(quality),
        'improved_worst_gap':bool(better_gap),
        'candidate_for_next_review':bool(quality and better_gap),
        'note':'Single rosbag replays are scheduling sensitive; no sensor-failure acceptance implied.'}
path=out/(f'{candidate_folder.name.lower()}_score.json' if a.candidate_name else f'loop_bridge_{a.duration}s_score.json')
path.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
