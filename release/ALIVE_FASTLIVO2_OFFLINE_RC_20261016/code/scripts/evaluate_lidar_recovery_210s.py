#!/usr/bin/env python3
"""Reject camera+IMU/LiDAR-rejoin regressions before costly full 4-mode replay."""
import argparse,csv,json
from pathlib import Path
import numpy as np

def read(p):
 with open(p) as f: r=list(csv.DictReader(f))
 return np.array([int(x['%time'])*1e-9 for x in r]),np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in r])

def proxy_error(folder,reference):
 t,g=read(folder/'global_pose.csv');ref=np.array([[float(q[k]) for k in ('timestamp','x','y','z')] for q in csv.DictReader(open(reference))]); t0=ref[0,0];q=np.stack([np.interp(t,ref[:,0],ref[:,i]) for i in range(1,4)],axis=1);e=np.linalg.norm(g-q,axis=1)
 out={'pose_count':len(t),'last_s':float(t[-1]-t0),'maxstep_m':float(np.linalg.norm(np.diff(g,axis=0),axis=1).max())}
 for lo,hi in [(125,165),(165,185),(185,205)]:
  m=(t-t0>=lo)&(t-t0<hi)
  out[f'rmse_{lo}_{hi}_m']=float(np.sqrt(np.mean(e[m]**2))) if m.any() else None
 return out

def main():
 a=argparse.ArgumentParser();a.add_argument('--baseline',type=Path,required=True);a.add_argument('--trial',type=Path,required=True);a.add_argument('--reference',type=Path,required=True);a.add_argument('--out',type=Path,required=True);q=a.parse_args()
 base=proxy_error(q.baseline,q.reference);tr=proxy_error(q.trial,q.reference)
 checks={'replay_at_least_205sec':tr['last_s']>=205,
 'lidar_outage_no_large_regression':tr['rmse_125_165_m']<=base['rmse_125_165_m']+0.7,
 'first_20s_recovery_no_large_regression':tr['rmse_165_185_m']<=base['rmse_165_185_m']+0.8,
 'next_20s_recovery_no_large_regression':tr['rmse_185_205_m']<=base['rmse_185_205_m']+0.9,
 'max_pose_step_no_large_regression':tr['maxstep_m']<=base['maxstep_m']+0.1}
 result={'baseline':base,'trial':tr,'checks':checks,'pass':all(checks.values()),'ground_truth':False};q.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
 if not result['pass']:raise SystemExit('LiDAR recovery gate FAILED')
if __name__=='__main__':main()
