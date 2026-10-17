#!/usr/bin/env python3
"""Score actual ROS 1x localization result CSVs, and only use reference for scoring.

Not used for estimator tuning or trajectory translation/rotation.
"""
from pathlib import Path
import numpy as np,csv,json,re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/LOCALIZATION_ONLY_COMPLETION_20261017'
CASES=[
 ('main',125,165,190,
  ROOT/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv',
  [
  ('original_frozen',ROOT/'results/FINAL_SINGLE_IMAGE_RC_20261016/main/IMU_FAIL_SINGLE_IMAGE_1X'),
  ('sensor_time_fifo',OUT/'sensor_time_fifo/main/STEP3_SENSOR_FIFO_IMU_190S'),
  ('odom_SE3_interpolated',OUT/'odom_se3_interpolation/main/STEP3_ODOM_SE3_IMU_190S')
  ]),
 ('sept18',90,130,160,
  ROOT/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv',
  [
  ('original_frozen',ROOT/'results/FINAL_SINGLE_IMAGE_RC_20261016/sept18/IMU_FAIL_SINGLE_IMAGE_1X'),
  ('same_frozen_repeat',OUT/'frozen_repro_repeat/sept18/FROZEN_SEP18_IMU_REPRO_160S'),
  ('sensor_time_fifo',OUT/'sensor_time_fifo/sept18/STEP3_SENSOR_FIFO_IMU_160S'),
  ('odom_SE3_interpolated',OUT/'odom_se3_interpolation/sept18/STEP3_ODOM_SE3_IMU_160S')
  ])]
def pose_csv(p):
 with open(p) as f:r=list(csv.DictReader(f))
 t=np.array([float(x['field.header.stamp'])*1e-9 for x in r])
 p=np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in r])
 return t,p
def reference_csv(p):
 with open(p) as f:r=list(csv.DictReader(f))
 return np.array([float(x['timestamp']) for x in r]),np.array([[float(x[k]) for k in 'xyz'] for x in r])
def run(name,win_a,win_b,end,ref,data,output):
 rt,rp=reference_csv(ref)
 fig,ax=plt.subplots(2,1,figsize=(13,8),sharex=True,dpi=120)
 stats={}
 for label,folder in data:
  t,p=pose_csv(folder/'global_pose.csv')
  m=(t>=rt[0])&(t<=min(rt[-1],rt[0]+end))
  t,p=t[m],p[m]
  target=np.column_stack([np.interp(t,rt,rp[:,j]) for j in range(3)])
  err=np.linalg.norm(p-target,axis=1)
  dt=np.diff(t);step=np.linalg.norm(np.diff(p,axis=0),axis=1)
  scores={'poses':len(t),'max_gap_s':round(float(dt.max()),4),
          'gaps_over_0p3':int((dt>.3).sum()),
          'max_step_m':round(float(step.max()),4),
          'steps_over_0p85':int((step>.85).sum())}
  for key,a,b in [('full',0,end),('outage',win_a,win_b),
                  ('recovery20',win_b,win_b+20),
                  ('late_recovery',win_b+5,win_b+20)]:
   select=(t>=rt[0]+a)&(t<rt[0]+b)
   scores[key+'_3d_rmse_m']=round(float(np.sqrt(np.mean(err[select]**2))),4) if select.any() else None
  scores['raw_gicp_matches_full_first_window']=-1
  log=(folder/'launch.log').read_text(errors='replace')
  stamps=[]
  for line in log.splitlines():
   if '[STEP3 TRACK] ACCEPT' not in line:continue
   m=re.search(r'\[\s*\d+\.\d+,\s*(\d+\.\d+)\s*\]',line)
   if m:
    q=float(m.group(1))-float(rt[0])
    if 0<=q<end:stamps.append(q)
  scores['raw_gicp_matches_full_first_window']=len(stamps)
  scores['raw_gicp_matches_during_outage']=sum(win_a<=x<win_b for x in stamps)
  scores['raw_gicp_matches_during_recovery20']=sum(win_b<=x<win_b+20 for x in stamps)
  stats[label]=scores
  ax[0].plot(t-rt[0],err,linewidth=.85,label=label)
  ax[1].plot(t[1:]-rt[0],step,linewidth=.85,label=label)
 ax[0].set_title(f'{name}: IMU dropout, true 1x replayed ROS poses (no GT fitting)')
 ax[0].set_ylabel('3D diagnostic reference disagreement, m')
 ax[1].set_xlabel('Reference-clock elapsed bag time, s')
 ax[1].set_ylabel('Consecutive global-pose displacement, m')
 ax[1].axhline(.85,linestyle='--',color='red',linewidth=1)
 for a in ax:
  a.axvspan(win_a,win_b,alpha=.09,color='gray')
  a.grid(alpha=.18);a.legend(fontsize=9)
 fig.tight_layout()
 fig.savefig(output/(name+'_TIME_ALIGNED_LOCALIZATION_1X_COMPARISON.png'))
 plt.close(fig)
 return stats
out=OUT/'timestamp_handoff_comparison';out.mkdir(parents=True,exist_ok=True)
scores={}
for name,a,b,end,ref,data in CASES:
 scores[name]=run(name,a,b,end,ref,data,out)
 print(name,json.dumps(scores[name],indent=2),flush=True)
(out/'TWO_BAG_SENSOR_TIMING_COMPARE.json').write_text(json.dumps(scores,indent=2)+'\n')
