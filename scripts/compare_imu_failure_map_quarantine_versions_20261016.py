#!/usr/bin/env python3
"""Actual ROS pose CSVs only. References for quantitative evaluation, not fitting."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1]
OUT=R/'results/LOCALIZATION_ONLY_COMPLETION_20261016'
ROOT1=OUT/'imu_map_quarantine_dev'
ROOT2=OUT/'imu_map_quarantine_bounded_v2_dev'
CASES=[
 ('main',125,165,190,
 [R/'results/FINAL_SINGLE_IMAGE_RC_20261016/main/IMU_FAIL_SINGLE_IMAGE_1X/global_pose.csv',
  ROOT1/'main/IMU_MAP_GUARD_DEV_190S/global_pose.csv',
  ROOT2/'main/IMU_GUARD_BOUNDED_V2_190S/global_pose.csv'],
 R/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv'),
 ('sep18',90,130,160,
 [R/'results/FINAL_SINGLE_IMAGE_RC_20261016/sept18/IMU_FAIL_SINGLE_IMAGE_1X/global_pose.csv',
  ROOT1/'sept18/IMU_MAP_GUARD_DEV_160S/global_pose.csv',
  ROOT2/'sept18/IMU_GUARD_BOUNDED_V2_160S/global_pose.csv'],
 R/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv')
]
def pose_csv(p):
 assert p.is_file(),p
 with p.open() as f:r=list(csv.DictReader(f))
 return np.array([float(x['field.header.stamp'])*1e-9 for x in r]),np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in r])
def ref_csv(p):
 with p.open() as f:r=list(csv.DictReader(f))
 return np.array([float(x['timestamp']) for x in r]),np.array([[float(x[k]) for k in 'xyz'] for x in r])
summ={}
for name,a,b,end,inputs,ref in CASES:
 rt,refp=ref_csv(ref)
 scores=[]
 fig,ax=plt.subplots(2,1,figsize=(12,8),dpi=140,sharex=True)
 for idx,(label,path) in enumerate(zip(['frozen_baseline','map_motion_v1','bounded_map_motion_v2'],inputs)):
  t,p=pose_csv(path)
  m=(t>=rt[0])&(t<=min(rt[-1],rt[0]+end))
  t,p=t[m],p[m]
  gt=np.column_stack([np.interp(t,rt,refp[:,j]) for j in range(3)])
  error=np.linalg.norm(p-gt,axis=1)
  step=np.linalg.norm(np.diff(p,axis=0),axis=1)
  dt=np.diff(t)
  masks={'all':np.ones(len(t),bool),
         'pre':(t-rt[0]<a),
         'outage':(t-rt[0]>=a)&(t-rt[0]<b),
         'recovery20':(t-rt[0]>=b)&(t-rt[0]<b+20),
         'first_12s_failure':(t-rt[0]>=a)&(t-rt[0]<a+12)}
  d={'version':label,
     'pose_samples':len(t),
     'max_step_m':float(step.max()),
     'steps_above_0p85m':int((step>.85).sum()),
     'max_pose_gap_s':float(dt.max()),
     **{k+'_rmse_m':float(np.sqrt(np.mean(error[v]**2))) for k,v in masks.items() if np.any(v)}}
  scores.append(d)
  ax[0].plot(t-rt[0],error,lw=1,label=label)
  ax[1].plot(t[1:]-rt[0],step,lw=1,label=label)
 ax[0].set_ylabel('3D discrepancy from diagnostic reference (m)')
 ax[0].set_title(f'{name}: actual 1x IMU-failure ROS pose streams, same scoring timestamps')
 ax[1].set_ylabel('Consecutive pose step (m)');ax[1].set_xlabel('Bag local time (s)')
 ax[1].axhline(.85,color='#a33245',ls='--',label='0.85m jump requirement',lw=1)
 for x in ax:x.axvspan(a,b,alpha=.12,color='#b3b3c3');x.grid(alpha=.2);x.legend(fontsize=9)
 fig.tight_layout()
 fig.savefig(OUT/(name+'_ALL_THREE_IMU_ROS_COMPARISON.png'))
 plt.close(fig)
 summ[name]=scores
 print(name,flush=True)
 for x in scores:print(x,flush=True)
(OUT/'MAIN_AND_SEP18_THREE_IMU_VERSIONS.json').write_text(json.dumps(summ,indent=2)+'\n')
print('CONSOLIDATED 2-BAG IMU QUARANTINE AUDIT DONE',flush=True)
