#!/usr/bin/env python3
"""Strict 90s A/B gate: healthy runtime CPU AND map-alignment continuity.

Reference is a SAME-BAG trajectory proxy, not surveyed ground truth.
Do not treat a matcher-only speedup as whole-system performance.
"""
import csv
import json
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parents[1]
results=root/'results/camera_lidar_imu_gps'
historic_baseline=results/'HEALTHY_VISUAL_FASTPATH_BENCH_20261015/HEALTHY_BASELINE_90S'
paired_baseline=results/'HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/HEALTHY_BASELINE_90S'
baseline=paired_baseline if (paired_baseline/'metrics_exact.txt').exists() else historic_baseline
candidate=results/'HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/HEALTHY_NORMCACHE_90S'
reference=root/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv'
origin=None

def read_csv(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))

def read_trajectory(path):
    records=read_csv(path)
    t=np.array([float(row['field.header.stamp'])*1e-9 for row in records])
    p=np.array([[float(row['field.pose.position.'+c]) for c in 'xyz'] for row in records])
    return t,p

refrows=read_csv(reference)
ref_t=np.array([float(r['timestamp']) for r in refrows])
ref_p=np.array([[float(r[c]) for c in 'xyz'] for r in refrows])
origin=float(ref_t[0])

def score(path, res):
    t,p=read_trajectory(path/'global_pose.csv')
    mask=(t>=origin)&(t<origin+90)
    t=t[mask];p=p[mask]
    if len(t)<700 or (t[-1]-origin)<85 or np.any(np.diff(t)<=0) or not np.isfinite(p).all():
        raise ValueError(f'Missing/invalid full 90s healthy output in {path}')
    r=np.stack([np.interp(t,ref_t,ref_p[:,i]) for i in range(3)],axis=1)
    error=np.linalg.norm(p-r,axis=1)
    dt=np.diff(t);dp=np.diff(p,axis=0)
    step=np.linalg.norm(dp,axis=1)
    good=(dt>=0.045)&(dt<=0.25)
    vel=dp/dt[:,None]
    accel=np.linalg.norm(np.diff(vel,axis=0)/((dt[1:]+dt[:-1])[:,None]/2),axis=1)
    good_a=good[:-1]&good[1:]
    g_t,_=read_trajectory(path/'gicp_map_pose.csv')
    g_t=g_t[(g_t>=origin)&(g_t<origin+90)]
    if len(g_t)<10 or good.sum()<600 or good_a.sum()<550:
        raise ValueError('Missing GICP or continuous pose samples')
    resources=read_csv(res)
    cpu=np.array([float(row['cpu_percent'].replace('%','')) for row in resources])
    if len(cpu)<8: raise ValueError('Not enough CPU samples')
    # Drop startup/shutdown samples; include identical lifecycle window.
    cpu=cpu[2:-2]
    launch=(path/'launch.log').read_text(errors='replace')
    return {
        'reference_proxy_rmse3d_m':round(float(np.sqrt(np.mean(error**2))),3),
        'reference_proxy_rmse_40_60s_m':round(float(np.sqrt(np.mean(error[(t>=origin+40)&(t<origin+60)]**2))),3),
        'max_pose_step_m':round(float(np.max(step)),3),
        'pose_step_p95_m':round(float(np.percentile(step,95)),3),
        'accel_p95_mps2':round(float(np.percentile(accel[good_a],95)),3),
        'accel_p99_mps2':round(float(np.percentile(accel[good_a],99)),3),
        'max_gicp_gap_s':round(float(np.max(np.diff(g_t))),3),
        'gicp_fixes':len(g_t),
        'fast_pose_gap_events':launch.count('[STEP3 FAST-GUARD] FAST pose gap'),
        'mean_cpu_percent':round(float(np.mean(cpu)),1),
        'p95_cpu_percent':round(float(np.percentile(cpu,95)),1),
        'cpu_samples_used':len(cpu),
        'elapsed_s':round(float(t[-1]-origin),2),
    }

b=score(baseline,baseline.parent/'HEALTHY_BASELINE_90S.resource_samples.csv')
c=score(candidate,candidate.parent/'HEALTHY_NORMCACHE_90S.resource_samples.csv')
historic=score(historic_baseline,historic_baseline.parent/'HEALTHY_BASELINE_90S.resource_samples.csv')
quality=(c['reference_proxy_rmse3d_m']<=b['reference_proxy_rmse3d_m']+0.10
         and c['reference_proxy_rmse_40_60s_m']<=b['reference_proxy_rmse_40_60s_m']+0.10
         and c['max_pose_step_m']<=b['max_pose_step_m']+0.10
         and c['accel_p95_mps2']<=b['accel_p95_mps2']*1.10
         and c['max_gicp_gap_s']<=max(1.70,b['max_gicp_gap_s']+0.20)
         and c['gicp_fixes']>=0.9*b['gicp_fixes']
         and c['fast_pose_gap_events']==0)
perf=c['mean_cpu_percent']<=b['mean_cpu_percent']*0.90
report={
    'comparison':'same bag, 1x, first 90s; isolated matcher code only',
    'reference_independent_ground_truth':False,
    'baseline_image':'fast_livo2_failsafe:healthy-gap-handoff-dev-20261014',
    'candidate_image':'fast_livo2_failsafe:scan-descriptor-isolated-dev-20261015',
    'baseline_control':('same_session_replay' if baseline==paired_baseline else 'historical_replay'),
    'historical_baseline':historic,
    'baseline':b, 'candidate':c,
    'quality_gate_passed':bool(quality),
    'meaningful_cpu_saving_gate_passed':bool(perf),
    'eligible_for_290s_review':bool(quality and perf),
    'note':'CPU statistics from one paired run are noisy and require repetitions before deployment; 90s does not cover any injected outage.'
}
out=candidate.parent/'ab_score.json'
out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
