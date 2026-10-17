#!/usr/bin/env python3
"""Strict dev-bag 290s healthy continuity regression; proxy is NOT surveyed GT."""
import argparse,csv,json
from pathlib import Path
import numpy as np

def csvpose(path):
 with open(path,newline='') as f: rows=list(csv.DictReader(f))
 if not rows:raise ValueError('empty CSV '+str(path))
 t=np.array([int(r['%time'])*1e-9 if '%time' in r else float(r['timestamp']) for r in rows]);
 p=np.array([[float(r['field.pose.position.'+c] if '%time' in r else r[c]) for c in 'xyz'] for r in rows]);
 return t,p

def metrics(case,reference,bag_start):
 t,p=csvpose(case/'global_pose.csv');ref_t,ref_p=csvpose(reference)
 if t[-1]-bag_start<285:raise ValueError(f'{case}: incomplete 290sec run: {t[-1]-bag_start}')
 ref=np.stack([np.interp(t,ref_t,ref_p[:,i]) for i in range(3)],1)
 err=np.linalg.norm(p-ref,axis=1);step=np.linalg.norm(np.diff(p,axis=0),axis=1);dt=np.diff(t)
 fixes,xyz=csvpose(case/'gicp_map_pose.csv')
 inside=(t-bag_start>=42)&(t-bag_start<=60);idx=np.where(inside)[0]
 if len(idx)<120:raise ValueError('few poses in loop')
 loop_steps=step[idx[:-1]];loop_times=dt[idx[:-1]]
 return {'n':len(t),'duration_s':float(t[-1]-bag_start),
   'full_rmse_m':float(np.sqrt(np.mean(err**2))),
   'full_maxstep_m':float(np.max(step)),
   'full_max_posegap_s':float(np.max(dt)),
   'loop_rmse_m':float(np.sqrt(np.mean(err[inside]**2))),
   'loop_maxstep_m':float(np.max(loop_steps)),
   'loop_maxstep_interval_s':float(np.max(loop_times)),
   'loop_max_error_m':float(np.max(err[inside])),
   'gicp_count':len(fixes),'gicp_max_gap_s':float(np.max(np.diff(fixes))),
   'loop_gicp_count':int(np.sum((fixes-bag_start>=42)&(fixes-bag_start<=60)))}

def main():
 a=argparse.ArgumentParser();a.add_argument('--baseline',type=Path,required=True);a.add_argument('--candidate',type=Path,required=True);a.add_argument('--reference',type=Path,required=True);a.add_argument('--out',type=Path,required=True);q=a.parse_args()
 t,_=csvpose(q.reference);start=float(t[0]);m={'baseline':metrics(q.baseline,q.reference,start),'candidate':metrics(q.candidate,q.reference,start)}
 c=m['candidate']; b=m['baseline']
 checks={'loop_gicp_gap_le_1p7s':c['gicp_max_gap_s']<=1.7,
   'loop_pcd_matches_ge_15':c['loop_gicp_count']>=15,
   'loop_rmse_le_1p70m':c['loop_rmse_m']<=1.7,
   'loop_step_le_0p90m':c['loop_maxstep_m']<=0.90,
   'whole_rmse_le_1p65m':c['full_rmse_m']<=1.65,
   'whole_step_le_1p25m':c['full_maxstep_m']<=1.25,
   'whole_pose_gap_le_0p45s':c['full_max_posegap_s']<=0.45,
   'loop_accuracy_better':c['loop_rmse_m']<b['loop_rmse_m'],
   'loop_step_better':c['loop_maxstep_m']<b['loop_maxstep_m']}
 result={'dataset':'camera_lidar_imu_gps.bag','reference_is_ground_truth':False,'metrics':m,'checks':checks,'pass':all(checks.values())}
 q.out.parent.mkdir(parents=True,exist_ok=True);q.out.write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))
 if not result['pass']:raise SystemExit('DEVELOPMENT HEALTHY GATE FAILED')
if __name__=='__main__':main()
