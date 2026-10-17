#!/usr/bin/env python3
"""Read-only diagnosis: withheld reference scoring only, never correction."""
import csv,json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1]
OUT=R/'results/LOCALIZATION_ONLY_COMPLETION_20261016'
OUT.mkdir(parents=True,exist_ok=True)
def poses(p):
 with open(p) as f:r=list(csv.DictReader(f))
 t=np.array([float(a['field.header.stamp'])*1e-9 for a in r])
 xyz=np.array([[float(a['field.pose.position.'+k]) for k in 'xyz'] for a in r])
 q=np.array([[float(a['field.pose.orientation.'+k]) for k in 'xyzw'] for a in r])
 return t,xyz,q
def ref(p):
 with open(p) as f:r=list(csv.DictReader(f))
 return np.array([float(a['timestamp']) for a in r]),np.array([[float(a[k]) for k in 'xyz'] for a in r])
cases=[
 ('main',R/'results/FINAL_SINGLE_IMAGE_RC_20261016/main/IMU_FAIL_SINGLE_IMAGE_1X',
 R/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv',(125,165)),
 ('sep18',R/'results/FINAL_SINGLE_IMAGE_RC_20261016/sept18/IMU_FAIL_SINGLE_IMAGE_1X',
 R/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv',(90,130)),
]
data={}
for name,d,rp,(a,b) in cases:
 rt,xyz=ref(rp);ot,op,oq=poses(d/'global_pose.csv');gt,gp,gq=poses(d/'gicp_map_pose.csv')
 t0=rt[0]
 out={}
 for label,t,p in [('published',ot,op),('gicp',gt,gp)]:
  good=(t>=rt[0])&(t<=rt[-1])
  t,p=t[good],p[good]
  v=np.column_stack([np.interp(t,rt,xyz[:,j]) for j in range(3)])
  err=np.linalg.norm(p-v,axis=1)
  masks={'all':np.ones(len(t),bool),'before':t-t0<a,
         'imu_failure':(t-t0>=a)&(t-t0<b),
         'recovery':(t-t0>=b)&(t-t0<b+20),
         'after':t-t0>=b+20}
  out[label]={k:{'n':int(m.sum()),
   'rmse3d_m':round(float(np.sqrt((err[m]**2).mean())),3) if m.sum() else None,
   'median_error_m':round(float(np.median(err[m])),3) if m.sum() else None}
   for k,m in masks.items()}
 out['gicp_sample_gaps_sec']={'median':round(float(np.median(np.diff(gt))),3),
  'p95':round(float(np.percentile(np.diff(gt),95)),3),
  'max':round(float(np.max(np.diff(gt))),3)}
 # Difference between original pose and latest / linearly interpolated GICP,
 # main insights are not a noncausal replay method.
 tmask=(ot>=rt[0]+a)&(ot<=rt[0]+b)
 out['imu_outage_global_at_gicp_times_error_m']=round(float(np.sqrt(np.mean(np.sum(
 (np.column_stack([np.interp(gt,ot,op[:,j]) for j in range(3)])-gp)**2,axis=1)))),3)
 data[name]=out
 print(name,json.dumps(out,indent=2),flush=True)
(OUT/'IMU_MAP_OBSERVATION_SOURCE_DIAGNOSIS.json').write_text(json.dumps(data,indent=2)+'\n')
