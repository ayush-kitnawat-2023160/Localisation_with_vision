#!/usr/bin/env python3
"""Compare heldout LiDAR failure using identical reference, truncated horizon.

Reject any candidate with a large pose jump, dropout or worse error. Scores
remain diagnostic same-bag PCD trajectory discrepancies, not ground truth.
"""
import csv, json, argparse
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser()
p.add_argument('--new',type=Path,required=True)
p.add_argument('--old',type=Path,required=True)
p.add_argument('--reference',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
p.add_argument('--horizon',type=float,default=160)
p.add_argument('--failure-start',type=float,default=90)
p.add_argument('--failure-end',type=float,default=130)
args=p.parse_args()

def records(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))

ref=records(args.reference)
rt=np.asarray([float(a['timestamp']) for a in ref])
rp=np.asarray([[float(a[k]) for k in 'xyz'] for a in ref])
origin=rt[0]

def score(folder):
    data=records(folder/'global_pose.csv')
    t=np.asarray([float(x['field.header.stamp'])*1.e-9 for x in data])
    xyz=np.asarray([[float(x['field.pose.position.'+c]) for c in 'xyz'] for x in data])
    mask=(t>=origin)&(t<=origin+args.horizon)
    t=t[mask];xyz=xyz[mask]
    if len(t)<args.horizon*7 or t[-1]<origin+args.horizon-5:
        raise ValueError(f'Incomplete {args.horizon}-second run: {folder} observed {(t[-1]-origin):.1f}s')
    if not np.isfinite(xyz).all() or np.any(np.diff(t)<=0):raise ValueError('invalid pose stream')
    err=np.linalg.norm(xyz-np.column_stack([np.interp(t,rt,rp[:,i]) for i in range(3)]),axis=1)
    dt=np.diff(t);step=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
    def w(a,b):
        m=(t>=origin+a)&(t<origin+b)
        return round(float(np.sqrt(np.mean(err[m]**2))),3) if m.any() else None
    logfile=(folder/'launch.log').read_text(errors='replace')
    return {'poses':len(t),'elapsed_s':round(float(t[-1]-origin),2),
            'full_proxy_rmse_m':round(float(np.sqrt(np.mean(err*err))),3),
            'pre_outage_60_to_90_rmse_m':w(60,args.failure_start),
            'outage_proxy_rmse_m':w(args.failure_start,args.failure_end),
            'recovery_proxy_rmse_m':w(args.failure_end,min(args.horizon,args.failure_end+20)),
            'max_published_pose_step_m':round(float(step.max()),3),
            'max_pose_stream_gap_s':round(float(dt.max()),3),
            'large_steps_0p85m':int((step>.85).sum()),
            'max_reference_difference_m':round(float(err.max()),3),
            'untrusted_log_events':logfile.count('[STEP3 TRUST] global pose UNTRUSTED'),
            'no_verified_depth_recovery_refusals':logfile.count('refusing false local bridge'),
            'queued_lidar_rejections':logfile.count('discarded in-flight GICP match after LiDAR loss')}

baseline=score(args.old); candidate=score(args.new)
quality=(candidate['max_published_pose_step_m']<.85 and
         candidate['max_pose_stream_gap_s']<.5 and
         candidate['outage_proxy_rmse_m']<=baseline['outage_proxy_rmse_m']+.3 and
         candidate['recovery_proxy_rmse_m']<=baseline['recovery_proxy_rmse_m']+.3 and
         candidate['full_proxy_rmse_m']<=baseline['full_proxy_rmse_m']+.3)
report={'development_only':True,'independent_ground_truth':False,
        'replay_horizon_s':args.horizon,'old':baseline,'new':candidate,
        'strict_quality_gate_passed':bool(quality),
        'deployment_approved':False,
        'note':'A trust Bool is not connected to the live planner/DBW; offline pass alone cannot approve cart deployment.'}
args.out.parent.mkdir(parents=True,exist_ok=True)
args.out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
