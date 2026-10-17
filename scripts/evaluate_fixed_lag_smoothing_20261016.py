#!/usr/bin/env python3
"""Exact algorithmic replay of diagnostic ROS fixed-lag published-pose smoother.

No manual offsets, no reference-based filter fitting, no artistically redrawn
trajectory. Save every accepted PoseStamped (incl flags), comparative plots.
"""
import argparse,csv,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from robust_fixed_lag_pose_smoother_20261016 import FixedLagPoseSmoother

CASES={
 'main_healthy':('camera_lidar_imu_gps','HEALTHY',None,None),
 'main_camera':('camera_lidar_imu_gps','CAMERA_FAILED',125,165),
 'main_lidar':('camera_lidar_imu_gps','LIDAR_FAILED',125,165),
 'main_imu':('camera_lidar_imu_gps','IMU_FAILED',125,165),
 'holdout_healthy':('18_sep_2026_combined','HEALTHY',None,None),
 'holdout_camera':('18_sep_2026_combined','CAMERA_FAILED',90,130),
 'holdout_lidar':('18_sep_2026_combined','LIDAR_FAILED',90,130),
 'holdout_imu':('18_sep_2026_combined','IMU_FAILED',90,130),
}
REF={
 'camera_lidar_imu_gps':ROOT/'results/BEST_SO_FAR_20261015/camera_lidar_imu_gps/REFERENCE/loopclosed_reference_recovered_equivalent.csv',
 '18_sep_2026_combined':ROOT/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv',
}
OUT=ROOT/'results/FIXED_LAG_SMOOTHING_DEV_20261016'
def pose_csv(p):
 with p.open() as f:rows=list(csv.DictReader(f))
 t=np.array([float(x['field.header.stamp'])*1.e-9 for x in rows],dtype=float)
 xyz=np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in rows])
 quat=np.array([[float(x['field.pose.orientation.'+k]) for k in 'xyzw'] for x in rows])
 return t,xyz,quat
def ref_csv(p):
 with p.open() as f:r=list(csv.DictReader(f))
 return np.array([float(x['timestamp']) for x in r]),np.array([[float(x[k]) for k in 'xyz'] for x in r])
def roll(t,x,y,dir,outage,start,save,lag,window,degree=1,sigma=.30):
 sr=FixedLagPoseSmoother(lag=lag,window=window,degree=degree,sigma=sigma)
 res=[]
 for i in range(len(t)):
  v=sr.ingest(t[i],x[i],y[i])
  if v:res.append(v)
 if not res:return {}
 st=np.array([z.stamp for z in res]);sx=np.stack([z.xyz for z in res]);sq=np.stack([z.quat for z in res])
 flags=np.array([z.continuous for z in res]);reasons=[z.reason for z in res]
 # Compare raw path at EXACT output timestamps (avoids lag-induced wrong RMSE).
 # Compare reference at exact timestamps, no XY alignment / Z flattening.
 raw=np.column_stack([np.interp(st,t,x[:,j]) for j in range(3)])
 ref=np.column_stack([np.interp(st,dir[:,0],dir[:,j+1]) for j in range(3)])
 e0=np.linalg.norm(raw-ref,axis=1);e1=np.linalg.norm(sx-ref,axis=1)
 dt=np.diff(st);step=np.linalg.norm(np.diff(sx,axis=0),axis=1)
 rawstep=np.linalg.norm(np.diff(raw,axis=0),axis=1)
 def rmse(v,m):return round(float(np.sqrt(np.mean(v[m]**2))),3) if m.sum() else None
 interval=st-start
 masks={'full':np.ones(len(st),bool)}
 if outage is not None:
  masks.update(pre=(interval<outage[0]),failure=(interval>=outage[0])&(interval<outage[1]),
               recovery=(interval>=outage[1])&(interval<outage[1]+20))
 values={name:{'raw_rmse_m':rmse(e0,m),'smoothed_rmse_m':rmse(e1,m),
               'samples':int(m.sum()),'numerically_unsafe_fraction':round(float((~flags[m]).mean()),3)} for name,m in masks.items()}
 stats={'raw':{'max_step_m':round(float(rawstep.max()),3),
               'p95_step_m':round(float(np.percentile(rawstep,95)),3),
               'steps_above_0p85m':int((rawstep>.85).sum())},
        'smoothed':{'max_step_m':round(float(step.max()),3),
               'p95_step_m':round(float(np.percentile(step,95)),3),
               'steps_above_0p85m':int((step>.85).sum()),
               'max_output_gap_s':round(float(dt.max()),3),
               'numerically_unsafe_samples':int((~flags).sum()),
               'numerically_unsafe_fraction':round(float((~flags).mean()),3)},
        'positions':values,'n_original':len(t),'n_smoothed':len(st),
        'lag_s':lag,'fit_window_s':window}
 if save:
  save.mkdir(parents=True,exist_ok=True)
  with (save/'smoothed_pose.csv').open('w',newline='') as f:
   w=csv.writer(f);w.writerow(['timestamp','x','y','z','qx','qy','qz','qw','numerically_continuous_not_map_trust','reason','raw_map_disagreement_m'])
   for i,z in enumerate(res):
    w.writerow([f'{st[i]:.9f}',*list(sx[i]),*list(sq[i]),int(flags[i]),reasons[i],z.raw_disagreement_m])
  (save/'metrics.json').write_text(json.dumps(stats,indent=2)+'\n')
  plt.rcParams.update({'font.size':10})
  fig,axs=plt.subplots(1,3,figsize=(17,5))
  ax=axs[0];ax.plot(dir[:,1],dir[:,2],color='#aab9cc',label='Loop-closed reference (proxy)',lw=1.8,alpha=.65)
  ax.plot(x[:,0],x[:,1],color='#7b93be',alpha=.55,label='Raw published',lw=1.1)
  ax.plot(sx[:,0],sx[:,1],color='#e87727',lw=1.35,label='Robust fixed-lag published',alpha=.95)
  ax.set_title('XY map-frame path (actual filter output)');ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
  ax=axs[1];ax.plot(interval,e0,color='#7b93be',label='Raw proxy error',alpha=.7,lw=1);ax.plot(interval,e1,color='#e87727',label='Smoothed proxy error',lw=1.1)
  ax.set_ylabel('3D disagreement (m)');ax.set_xlabel('Seconds relative to bag reference');ax.set_title('Absolute-position accuracy (no fitting)');ax.grid(alpha=.2)
  ax=axs[2];ax.plot(interval[1:],rawstep,color='#7b93be',label='Raw step',lw=1,alpha=.7);ax.plot(interval[1:],step,color='#e87727',label='Smoothed step',lw=1.1)
  ax.axhline(.85,c='#c73744',ls='--',lw=1,label='0.85m threshold')
  ax.set_ylabel('Published step (m)');ax.set_xlabel('Seconds relative to bag reference');ax.set_title('Continuity; long gaps are NOT hidden');ax.grid(alpha=.2)
  for ax in axs[1:]:
   if outage is not None:ax.axvspan(outage[0],outage[1],color='#cebaab',alpha=.18)
   ax.legend(fontsize=8)
  fig.suptitle('Offline reproduction of LIVE diagnostic smoother — NOT autonomous navigation approved',fontweight='bold')
  fig.tight_layout();fig.savefig(save/'RAW_VS_SMOOTHED_XY_ERROR_STEPS.png',dpi=150);plt.close(fig)
 return stats
def main():
 p=argparse.ArgumentParser();p.add_argument('--lag',type=float,default=.18);p.add_argument('--window',type=float,default=.85);p.add_argument('--sigma',type=float,default=.30);p.add_argument('--degree',type=int,default=1);p.add_argument('--save',action='store_true');p.add_argument('--cases',nargs='+',choices=list(CASES),default=['main_healthy','main_lidar','holdout_healthy','holdout_lidar']);p.add_argument('--sweep',action='store_true');a=p.parse_args()
 runs={}
 for case in a.cases:
  bag,mode,fa,fb=CASES[case]
  t,x,q=pose_csv(ROOT/'results/BEST_SO_FAR_20261015'/bag/mode/'global_pose.csv')
  rt,rx=ref_csv(REF[bag])
  ref=np.column_stack([rt,rx])
  samples=([(.23,1.15,.30),(.23,1.60,.48),(.23,2.10,.65)] if a.sweep and bag=='camera_lidar_imu_gps' else [(a.lag,a.window,a.sigma)])
  for lag,window,sigma in samples:
   target=OUT/case if a.save and not a.sweep else None
   m=roll(t,x,q,ref,(fa,fb) if fa else None,rt[0],target,lag,window,a.degree,sigma)
   print(case,'lag',lag,'win',window,'sigma',sigma,'raw/smooth rmse',m['positions']['full']['raw_rmse_m'],m['positions']['full']['smoothed_rmse_m'],'maxstep',m['raw']['max_step_m'],m['smoothed']['max_step_m'],'unsafe',m['smoothed']['numerically_unsafe_fraction'],'pose gap',m['smoothed']['max_output_gap_s'],flush=True)
   runs[case]=m
 if a.save:
  OUT.mkdir(parents=True,exist_ok=True)
  (OUT/'summary.json').write_text(json.dumps({'filter_config':{'lag_s':a.lag,'window_s':a.window,'degree':a.degree,'sigma':a.sigma},'source':'20261015 curated complete runs, 1x; offline replay of intended ROS sidecar','reference_is_independent_ground_truth':False,'safe_for_autonomous_navigation':False,'cases':runs},indent=2)+'\n')
if __name__=='__main__':main()
