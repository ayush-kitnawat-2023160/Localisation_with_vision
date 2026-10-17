#!/usr/bin/env python3
"""Quantitatively compare actual ROS1 1x LiDAR failover map tracks.
Corrected/same-bag references are used for SCORING ONLY; no pose changes.
"""
from pathlib import Path
import json, re, csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

R=Path(__file__).resolve().parents[1]
OUT=R/'results/LOCALIZATION_ONLY_COMPLETION_20261017/depth_source_clock_gate'
BASE=R/'results/FINAL_SINGLE_IMAGE_RC_20261016'
CASES=[
 ('sept18',90,130,165,R/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv'),
 ('main',125,165,190,R/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv')]
def pose(p):
 with p.open() as f: d=list(csv.DictReader(f))
 t=np.array([float(x['field.header.stamp'])*1e-9 for x in d])
 xyz=np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in d])
 return t,xyz
def ref(p):
 with p.open() as f: d=list(csv.DictReader(f))
 return np.array([float(x['timestamp']) for x in d]),np.array([[float(x[k]) for k in 'xyz'] for x in d])
def evaluate(p,rt,rxyz,a,b,end):
 t,xyz=pose(p);keep=(t>=rt[0])&(t<=min(rt[-1],rt[0]+end));t,xyz=t[keep],xyz[keep]
 assert len(t)>end*7 and np.diff(t).min()>0,(p,len(t))
 predict=np.column_stack([np.interp(t,rt,rxyz[:,j]) for j in range(3)])
 e=np.linalg.norm(xyz-predict,axis=1);step=np.linalg.norm(np.diff(xyz,axis=0),axis=1);dt=np.diff(t)
 ts=t-rt[0]
 mask={'full':np.ones(len(t),dtype=bool),'pre':ts<a,'outage':(ts>=a)&(ts<b),
       'recovery20':(ts>=b)&(ts<b+20)}
 data={key+'_3d_rmse_m':round(float(np.sqrt(np.mean(e[v]**2))),3) for key,v in mask.items()}
 data.update(pose_samples=len(t),max_step_m=round(float(step.max()),3),steps_gt_0p85=int((step>.85).sum()),
             max_gap_s=round(float(dt.max()),3),gaps_gt_0p3=int((dt>.3).sum()),max_error_m=round(float(e.max()),3))
 return t,xyz,e,step,data
summary={}
for bag,a,b,end,reference in CASES:
 rt,rxyz=ref(reference)
 baseline=BASE/bag/'LIDAR_FAIL_SINGLE_IMAGE_1X/global_pose.csv'
 newdir=OUT/bag/('LIDAR_DEPTH_SOURCE_CLOCK_'+bag+'_1X')
 newpose=newdir/'global_pose.csv'
 assert newpose.is_file() and newpose.stat().st_size>100,newpose
 old=evaluate(baseline,rt,rxyz,a,b,end)
 now=evaluate(newpose,rt,rxyz,a,b,end)
 logs=(newdir/'launch.log').read_text(errors='replace')
 oldlogs=(BASE/bag/'LIDAR_FAIL_SINGLE_IMAGE_1X/launch.log').read_text(errors='replace')
 evidence={'depth_source_clock_ticks':logs.count('[DEPTH SOURCE CLOCK] rate-selected'),
           'depth_map_fix_logs':logs.count('[STEP3 DEPTH MAP] verified'),
           'depth_map_fix_logs_baseline':oldlogs.count('[STEP3 DEPTH MAP] verified'),
           'failure_transition_seen':'FULL_LIVO -> VI_LIDAR_FAILED' in logs,
           'recovery_transition_seen':'VI_LIDAR_FAILED -> FULL_LIVO' in logs}
 oldm=old[4];newm=now[4]
 checks={'runtime_source_time_log_present':evidence['depth_source_clock_ticks']>0,
         'no_stale_map_frame':newm['max_gap_s']<=.3,
         'no_jump_over_0p85':newm['max_step_m']<=.85,
         'outage_error_not_worse_than_baseline':newm['outage_3d_rmse_m']<=oldm['outage_3d_rmse_m']+.15,
         'recovery_error_not_worse_than_baseline':newm['recovery20_3d_rmse_m']<=oldm['recovery20_3d_rmse_m']+.15,
         'full_error_not_worse_than_baseline':newm['full_3d_rmse_m']<=oldm['full_3d_rmse_m']+.15,
         'meets_0p5m_target':newm['full_3d_rmse_m']<=.5}
 summary[bag]={'baseline':oldm,'trial':newm,'checks':checks,'observations':evidence}
 print(bag,'OLD',oldm,'NEW',newm,flush=True)
 print('GATES',checks,'LOGS',evidence,flush=True)
 fig,ax=plt.subplots(1,3,figsize=(18,5),dpi=150)
 for label,vals,color in [('Frozen baseline',old,'#427bad'),('Sensor time depth GICP',now,'#e5792b')]:
  t,p,e,step,m=vals
  ax[0].plot(p[:,0],p[:,1],lw=1,color=color,label=label)
  ax[1].plot(t-rt[0],e,lw=.8,color=color,label=label)
  ax[2].plot(t[1:]-rt[0],step,lw=.8,color=color,label=label)
 ax[0].plot(rxyz[:,0],rxyz[:,1],ls='--',lw=.6,color='grey',label='Diagnostic reference')
 ax[0].axis('equal');ax[0].set_xlabel('Map x (m)');ax[0].set_ylabel('Map y (m)')
 ax[1].set_xlabel('Bag local time (s)');ax[1].set_ylabel('3D error (m)')
 ax[2].set_xlabel('Bag local time (s)');ax[2].set_ylabel('Published pose step (m)')
 ax[2].axhline(.85,ls='--',color='#b3283e')
 for plot in ax[1:]:plot.axvspan(a,b,color='grey',alpha=.12)
 for plot in ax:plot.grid(alpha=.2);plot.legend(fontsize=8)
 fig.suptitle(f'LiDAR loss {bag} — identical frozen localisation except depth source-time registration throttle')
 fig.tight_layout();fig.savefig(OUT/(bag+'_DEPTH_SOURCE_CLOCK_COMPARE.png'),bbox_inches='tight');plt.close(fig)
path=OUT/'TWO_BAG_DEPTH_SOURCE_CLOCK_VALIDATION.json'
path.write_text(json.dumps(summary,indent=2)+'\n')
print('FINAL TWO-BAG CLOCK GATE ACCEPTED?',all(all(x['checks'].values()) for x in summary.values()))
print('WROTE',path)
