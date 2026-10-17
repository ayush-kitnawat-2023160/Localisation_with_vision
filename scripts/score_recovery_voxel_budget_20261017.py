#!/usr/bin/env python3
"""Evaluate original and recovery-map CPU-budget 1x published ROS pose streams.

No reference is used by the estimator/experiment; here ONLY score.
"""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1]
OUT=R/'results/LOCALIZATION_ONLY_COMPLETION_20261017/recovery_voxel_budget'
CASES=[
 ('main',R/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv',
   R/'results/FINAL_SINGLE_IMAGE_RC_20261016/main/IMU_FAIL_SINGLE_IMAGE_1X/global_pose.csv',
   OUT/'main/IMU_RECOVERY_VOXEL_BUDGET_190S/global_pose.csv',125,165,190),
 ('sept18',R/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv',
   R/'results/FINAL_SINGLE_IMAGE_RC_20261016/sept18/IMU_FAIL_SINGLE_IMAGE_1X/global_pose.csv',
   OUT/'sept18/IMU_RECOVERY_VOXEL_BUDGET_160S/global_pose.csv',90,130,160)]
def read_pose(f):
 with open(f) as h:aa=list(csv.DictReader(h))
 t=np.array([float(x['field.header.stamp'])*1e-9 for x in aa])
 p=np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in aa])
 return t,p
def read_ref(f):
 with open(f) as h:aa=list(csv.DictReader(h))
 return np.array([float(x['timestamp']) for x in aa]),np.array([[float(x[k]) for k in 'xyz'] for x in aa])
def evaluate(t,p,r,ref,a,b,end):
 good=(t>=r[0])&(t<=min(r[-1],r[0]+end));t,p=t[good],p[good]
 gt=np.column_stack([np.interp(t,r,ref[:,j]) for j in range(3)])
 e=np.linalg.norm(p-gt,axis=1);steps=np.linalg.norm(np.diff(p,axis=0),axis=1)
 windows={'all':np.ones(len(t),bool),'outage':(t-r[0]>=a)&(t-r[0]<b),
 'recovery20':(t-r[0]>=b)&(t-r[0]<b+20),'after':t-r[0]>=b}
 obj={'poses':int(len(t)),'largest_pose_gap_s':round(float(np.diff(t).max()),4),
 'pose_gap_gt_0p3s':int((np.diff(t)>.3).sum()),
 'max_step_m':round(float(steps.max()),4),
 'step_gt_0p85m':int((steps>.85).sum())}
 for key,mask in windows.items():
  if mask.sum():obj[key+'_rmse3d_proxy_m']=round(float(np.sqrt(np.mean(e[mask]**2))),4)
 return obj,t,e,p
res={}
for name,r,before,after,a,b,end in CASES:
 assert after.exists(),after
 t,g=read_ref(r)
 obj={}
 fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True,dpi=130)
 for k,f,c in [('frozen',before,'#3369a4'),('recovery_voxel_budget_1400',after,'#dd7a28')]:
  stats,tt,err,p=evaluate(*read_pose(f),t,g,a,b,end)
  print(name,k,stats,flush=True)
  obj[k]=stats
  axes[0].plot(tt-t[0],err,color=c,label=k,linewidth=1)
  axes[1].plot(tt[1:]-t[0],np.linalg.norm(np.diff(p,axis=0),axis=1),color=c,label=k,linewidth=1)
 axes[0].set_title(f'{name} IMU sensor failure 1x ROS replay — unfit diagnostic reference')
 axes[0].set_ylabel('3D reference disagreement (m)')
 axes[1].set_ylabel('3D pose step (m)');axes[1].set_xlabel('Time since bag reference (s)')
 axes[1].axhline(.85,linestyle='--',color='red',linewidth=1)
 for ax in axes:ax.axvspan(a,b,alpha=.08,color='gray');ax.grid(alpha=.15);ax.legend()
 fig.tight_layout();fig.savefig(OUT/(name+'_BASELINE_VS_RECOVERY_BUDGET_1400.png'));plt.close(fig)
 res[name]=obj
(OUT/'REPLAY_1400_POINT_BUDGET_AUDIT.json').write_text(json.dumps(res,indent=2)+'\n')
print('SUMMARY AUDIT WRITTEN')
