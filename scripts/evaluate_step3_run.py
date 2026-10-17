#!/usr/bin/env python3
import argparse,csv
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser()
p.add_argument('run_dir')
p.add_argument('--reference',required=True,help='Time-synchronized reference for THIS bag; not assumed ground truth')
p.add_argument('--reference-kind',choices=('diagnostic','validated_gt'),default='diagnostic',
               help='Diagnostic trajectory disagreement unless independently validated GT is supplied')
p.add_argument('--timeline-origin',choices=('reference','first_pose'),default='reference',
               help='Use first timestamp of same-bag reference (first LiDAR scan) as bag clock zero; first_pose is legacy behavior')
p.add_argument('--window',type=float,default=45.0)
p.add_argument('--sensor',default='none',choices=['none','camera','lidar','imu'])
p.add_argument('--drop-start',type=float,default=15.0)
p.add_argument('--drop-duration',type=float,default=20.0)
a=p.parse_args();run=Path(a.run_dir);ref=Path(a.reference)

def L(path):
    with open(path) as f:
        r=csv.DictReader(f)
        return np.array([[int(x['%time'])*1e-9,float(x['field.pose.position.x']),float(x['field.pose.position.y']),float(x['field.pose.position.z'])] for x in r],float)
def R(path):
    with open(path) as f:
        r=csv.DictReader(f)
        return np.array([[float(x['timestamp']),float(x['x']),float(x['y']),float(x['z'])] for x in r],float)

refa=R(ref);g=L(run/'global_pose.csv')
if len(g)<2 or len(refa)<2:
    raise ValueError('Need at least two global poses and reference samples')
if not np.isfinite(g).all() or not np.isfinite(refa).all():
    raise ValueError('Non-finite position or timestamp in pose/reference')
if np.any(np.diff(refa[:,0])<=0):
    raise ValueError('Reference timestamps must be strictly increasing')
t0=refa[0,0] if a.timeline_origin=='reference' else g[0,0]
g=g[(g[:,0]>=t0)&(g[:,0]<=t0+a.window)]
if len(g)<2:raise ValueError('No global pose samples inside the requested bag-relative time window')
if np.any(np.diff(g[:,0])<0):
    raise ValueError('Global pose timestamps are out of order')
# numpy.interp silently clamps to a *different recording's first/last pose*
# when time ranges do not overlap. That previously yielded fake RMSE figures.
tol=0.15
if refa[0,0]>g[0,0]+tol or refa[-1,0]<g[-1,0]-tol:
    raise ValueError(
        'Reference timestamp coverage INVALID for this run: '
        f'ref=[{refa[0,0]:.3f},{refa[-1,0]:.3f}] '
        f'run=[{g[0,0]:.3f},{g[-1,0]:.3f}]. '
        'Do not evaluate using a different bag or stale loop-closed trajectory.')
rr=np.column_stack([np.interp(g[:,0],refa[:,0],refa[:,i]) for i in (1,2,3)])
e=g[:,1:4]-rr;e3=np.linalg.norm(e,axis=1);e2=np.linalg.norm(e[:,:2],axis=1);s=np.linalg.norm(np.diff(g[:,1:4],axis=0),axis=1)
lines=[]
def add(x): lines.append(x);print(x)
def segment(name,lo,hi):
    m=(g[:,0]-t0>=lo)&(g[:,0]-t0<hi)
    if not np.any(m): return
    idx=np.where(m)[0]
    prefix='gt_' if a.reference_kind=='validated_gt' else 'reference_disagreement_'
    add(f'{prefix}{name}_rmse3d_m={np.sqrt(np.mean(e3[m]**2)):.6f}')
    add(f'{prefix}{name}_max3d_m={e3[m].max():.6f}')
    add(f'{prefix}{name}_end3d_m={e3[idx[-1]]:.6f}')

add(f'n={len(g)}');add(f'duration={g[-1,0]-g[0,0]:.6f}')
add(f'timeline_origin={a.timeline_origin}')
add(f'timeline_origin_timestamp={t0:.9f}')
add(f'reference_file={ref.resolve()}')
add(f'reference_kind={a.reference_kind}')
add('reference_independent_ground_truth='+str(a.reference_kind=='validated_gt').lower())
if a.reference_kind!='validated_gt':
    add('WARNING=Distances below are disagreement with an unvalidated trajectory, NOT localization accuracy')
add(f'reference_disagreement_2d_rmse_m={np.sqrt(np.mean(e2**2)):.6f}')
add(f'reference_disagreement_3d_rmse_m={np.sqrt(np.mean(e3**2)):.6f}')
add(f'reference_disagreement_mean3d_m={e3.mean():.6f}')
add(f'reference_disagreement_max3d_m={e3.max():.6f}')
add(f'reference_disagreement_end3d_m={e3[-1]:.6f}')
if a.reference_kind=='validated_gt':
    add(f'2d_rmse_m={np.sqrt(np.mean(e2**2)):.6f}')
    add(f'3d_rmse_m={np.sqrt(np.mean(e3**2)):.6f}')
add(f'step_max_m={s.max():.6f}');add(f'step_p95_m={np.percentile(s,95):.6f}')
if a.sensor!='none':
    fs=a.drop_start;fe=a.drop_start+a.drop_duration
    add(f'failure_sensor={a.sensor}');add(f'failure_start_s={fs:.3f}');add(f'failure_end_s={fe:.3f}')
    segment('pre_failure',0.0,fs)
    segment('failure',fs,fe)
    segment('post_failure_20s',fe,min(a.window,fe+20.0))
    segment('post_failure_rest',fe,a.window+1e-6)
else:
    # Useful long-run slices for healthy 290 s validation.
    for lo,hi in ((0,60),(60,120),(120,180),(180,240),(240,a.window+1e-6)):
        if hi>lo: segment(f'{int(lo)}-{int(min(hi,a.window))}s',lo,min(hi,a.window+1e-6))

q=L(run/'gicp_map_pose.csv')
if len(q):
    q=q[(q[:,0]>=t0)&(q[:,0]<=t0+a.window)]
    qr=np.column_stack([np.interp(q[:,0],refa[:,0],refa[:,i]) for i in (1,2,3)])
    qe=np.linalg.norm(q[:,1:4]-qr,axis=1)
    add(f'gicp_n={len(q)}')
    add(f'reference_disagreement_gicp_rmse_m={np.sqrt(np.mean(qe**2)):.6f}')
    add(f'reference_disagreement_gicp_mean_m={qe.mean():.6f}')
    add(f'reference_disagreement_gicp_max_m={qe.max():.6f}')
(run/'metrics_exact.txt').write_text('\n'.join(lines)+'\n')
