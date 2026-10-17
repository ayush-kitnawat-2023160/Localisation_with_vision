#!/usr/bin/env python3
"""Compare three original development-bag failure modes before HOLDOUT; not ground truth."""
import argparse,json
from pathlib import Path

def metrics(path):
 d={}
 for line in path.read_text().splitlines():
  if '=' not in line:continue
  k,v=line.split('=',1)
  try:d[k]=float(v)
  except ValueError:pass
 return d

def main():
 a=argparse.ArgumentParser();a.add_argument('--base',type=Path,required=True);a.add_argument('--candidate',type=Path,required=True);a.add_argument('--out',type=Path,required=True);q=a.parse_args()
 pairs={'CAMERA_FAIL':'CAMERA_FAIL_TURN_FASTCATCH_290S','LIDAR_FAIL':'LIDAR_FAIL_TURN_FASTCATCH_290S','IMU_FAIL':'IMU_FAIL_TURN_FASTCATCH_290S'}
 report={'dataset':'camera_lidar_imu_gps.bag','ground_truth':False,'cases':{},'pass':True}
 for name,folder in pairs.items():
  b=metrics(q.base/name/'metrics_exact.txt');c=metrics(q.candidate/folder/'metrics_exact.txt')
  keys=('reference_disagreement_3d_rmse_m','reference_disagreement_failure_rmse3d_m','step_max_m')
  vals={k:{'baseline':b[k],'candidate':c[k]} for k in keys}
  checks={'full_proxy_rmse_no_large_regression':c[keys[0]]<=b[keys[0]]+0.45,
       'failure_proxy_rmse_no_large_regression':c[keys[1]]<=b[keys[1]]+0.6,
       'published_jump_no_regression':c[keys[2]]<=b[keys[2]]+0.10}
  vals['checks']=checks;report['cases'][name]=vals
  if not all(checks.values()):report['pass']=False
 q.out.parent.mkdir(parents=True,exist_ok=True);q.out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
 if not report['pass']:raise SystemExit('Development failure-mode regression gate failed; do not validate heldout')
if __name__=='__main__':main()
