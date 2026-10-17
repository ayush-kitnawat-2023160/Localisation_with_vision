#!/usr/bin/env python3
"""Validate real published 1× ROS poses from a frozen and trial Docker build.

All scoring is via the corrected diagnostic reference; reference never
participates in pose generation. Strictly require both recorded runs complete.
"""
from pathlib import Path
import csv,json, re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

R=Path(__file__).resolve().parents[1]
OUT=R/'results/LOCALIZATION_ONLY_COMPLETION_20261017/recovery_voxel_budget'
BASE=R/'results/FINAL_SINGLE_IMAGE_RC_20261016'
SPECS=[
 ('sept18',90,130,165,
  R/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv'),
 ('main',125,165,190,
  R/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv')]
def parse(p):
 with p.open() as f:rows=list(csv.DictReader(f))
 t=np.array([float(x['field.header.stamp'])*1e-9 for x in rows])
 xyz=np.array([[float(x['field.pose.position.'+q]) for q in 'xyz'] for x in rows])
 return t,xyz
def reference(p):
 with p.open() as f:rows=list(csv.DictReader(f))
 return np.array([float(x['timestamp']) for x in rows]),np.array([[float(x[k]) for k in 'xyz'] for x in rows])
def score(p,ref_t,ref_xyz,a,b,end):
 t,xyz=parse(p)
 keep=(t>=ref_t[0])&(t<=min(ref_t[-1],ref_t[0]+end))
 t,xyz=t[keep],xyz[keep]
 if len(t)<7*end or (t[-1]-ref_t[0])<end-6:
  raise RuntimeError(f'Pose stream too short: {p}: {len(t)} poses, {t[-1]-ref_t[0]} elapsed')
 assert np.all(np.isfinite(xyz)) and np.min(np.diff(t))>0,(p,'Invalid poses/clock')
 matched=np.column_stack([np.interp(t,ref_t,ref_xyz[:,i]) for i in range(3)])
 err=np.linalg.norm(xyz-matched,axis=1)
 dt=np.diff(t);step=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
 def rmse(mask):return round(float(np.sqrt(np.mean(err[mask]**2))),3) if np.any(mask) else None
 ts=t-ref_t[0]
 m={'pre':ts<a,'outage':(ts>=a)&(ts<b),'recovery20':(ts>=b)&(ts<b+20),'full':np.ones(len(t),dtype=bool)}
 d={k+'_3d_proxy_rmse_m':rmse(v) for k,v in m.items()}
 d.update(pose_count=len(t),max_step_m=round(float(step.max()),3),steps_gt_0p85=int(np.sum(step>.85)),
          max_pose_gap_s=round(float(dt.max()),3),pose_gaps_gt_0p3s=int(np.sum(dt>.3)),
          max_3d_proxy_error_m=round(float(err.max()),3))
 return t,xyz,err,dt,step,d

overall={}
for name,a,b,end,ref_file in SPECS:
 ref_t,ref_xyz=reference(ref_file)
 old=BASE/name/'IMU_FAIL_SINGLE_IMAGE_1X/global_pose.csv'
 trial=OUT/name/('IMU_RECOVERY_MAP_BUDGET_'+name+'_1X')
 result_file=trial/'global_pose.csv'
 assert result_file.is_file(),f'Incomplete suite: {result_file}'
 meta=(trial/'run_metadata.txt').read_text()
 assert str(ref_file) in meta
 original=score(old,ref_t,ref_xyz,a,b,end)
 new=score(result_file,ref_t,ref_xyz,a,b,end)
 log=(trial/'launch.log').read_text(errors='replace')
 build_samples=[]
 for before,after,ms in re.findall(r'\[FAILSAFE RECOVERY BUDGET\]\s+(\d+) -> (\d+) map seed points; build ([\d.]+) ms',log):
  build_samples.append((int(before),int(after),float(ms)))
 checks={'recovery_budget_log_observed':len(build_samples)>0,
         'failure_mode_transition_seen':'FULL_LIVO -> LV_IMU_FAILED' in log,
         'recovery_mode_seen':'-> FULL_LIVO' in log}
 original_data=original[-1];trial_data=new[-1]
 checks.update(
     no_pose_gap_above_0p3=trial_data['max_pose_gap_s']<=.3,
     no_pose_step_above_0p85=trial_data['max_step_m']<=.85,
     outage_not_worse=trial_data['outage_3d_proxy_rmse_m']<=original_data['outage_3d_proxy_rmse_m']+.15,
     recovery_not_worse=trial_data['recovery20_3d_proxy_rmse_m']<=original_data['recovery20_3d_proxy_rmse_m']+.15,
     full_not_worse=trial_data['full_3d_proxy_rmse_m']<=original_data['full_3d_proxy_rmse_m']+.15,
     below_target_0p5m=trial_data['full_3d_proxy_rmse_m']<=.5)
 summary={'baseline':original_data,'trial':trial_data,'diagnostics':{
  'recovery_map_budgets':build_samples,'recovery_map_budgets_count':len(build_samples),
  'median_recovery_map_build_ms':round(float(np.median([v[2] for v in build_samples])),2) if build_samples else None},
  'checks':checks}
 overall[name]=summary
 print(name,'BASE',original_data,flush=True)
 print(name,'TRIAL',trial_data,'LOG',summary['diagnostics'],'CHECKS',checks,flush=True)
 fig,ax=plt.subplots(1,3,figsize=(18,5.5),dpi=150)
 colors=['#527ba7','#e37723']
 for i,(label,data) in enumerate([('Frozen single-build',original),('Recovery voxel budget',new)]):
  t,xyz,error,dt,steps,stats=data
  ax[0].plot(xyz[:,0],xyz[:,1],label=label,color=colors[i],lw=1.1)
  ax[1].plot(t-ref_t[0],error,label=label,color=colors[i],lw=1)
  ax[2].plot(t[1:]-ref_t[0],steps,label=label,color=colors[i],lw=1)
 ax[0].plot(ref_xyz[:,0],ref_xyz[:,1],color='grey',ls='--',label='Reference proxy',lw=.7)
 ax[0].axis('equal');ax[0].set_title('Map XY published pose');ax[0].set_xlabel('Map X');ax[0].set_ylabel('Map Y')
 ax[1].set_title('3D discrepancy against diagnostic proxy');ax[1].set_ylabel('Metres')
 ax[2].set_title('Actual published step');ax[2].axhline(.85,ls='--',color='crimson',lw=1)
 ax[2].set_ylabel('Metres')
 for xy in ax[1:]:xy.set_xlabel('Bag relative time (s)');xy.axvspan(a,b,color='#c8c8c8',alpha=.25)
 for xy in ax:xy.grid(alpha=.2);xy.legend(fontsize=8)
 fig.suptitle('ROS 1x replay • '+name+' • actual FAST-LIVO2 map pose • reference not used for corrections')
 fig.tight_layout()
 fig.savefig(OUT/(name+'_RECOVERY_VOXEL_BUDGET_COMPARISON.png'),bbox_inches='tight')
 plt.close(fig)
(out:=OUT/'TWO_BAG_RECOVERY_VOXEL_BUDGET_METRICS.json').write_text(json.dumps(overall,indent=2)+'\n')
for name,x in overall.items():
 print('ACCEPTANCE',name,'PASS' if all(x['checks'].values()) else 'FAILED',flush=True)
print('OUTPUT',out,flush=True)
