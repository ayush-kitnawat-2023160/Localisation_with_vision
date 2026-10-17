#!/usr/bin/env python3
"""Independent source-time/replay repeatability score; NO trajectory adjustment."""
from pathlib import Path
import csv,json,re
import numpy as np
R=Path(__file__).resolve().parents[1]
OUT=R/'results/LOCALIZATION_ONLY_COMPLETION_20261017/frozen_repro_repeat'
REF=R/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv'
REPLAYS={
 'original_20261016':R/'results/FINAL_SINGLE_IMAGE_RC_20261016/sept18/IMU_FAIL_SINGLE_IMAGE_1X',
 'same_image_replay_20261017':OUT/'sept18/FROZEN_SEP18_IMU_REPRO_160S',
 'different_dev_v2_20261016':R/'results/LOCALIZATION_ONLY_COMPLETION_20261016/imu_map_quarantine_bounded_v2_dev/sept18/IMU_GUARD_BOUNDED_V2_160S',
 'recovery_voxel_budget_1400':R/'results/LOCALIZATION_ONLY_COMPLETION_20261017/recovery_voxel_budget/sept18/IMU_RECOVERY_VOXEL_BUDGET_160S'
}
with REF.open() as f:rows=list(csv.DictReader(f))
ref_t=np.array([float(z['timestamp']) for z in rows]);ref=np.array([[float(z[k]) for k in 'xyz'] for z in rows])
output={}
for label,folder in REPLAYS.items():
 path=folder/'global_pose.csv'
 assert path.exists(),path
 with path.open() as f:rr=list(csv.DictReader(f))
 t=np.array([float(z['field.header.stamp'])*1e-9 for z in rr])
 p=np.array([[float(z['field.pose.position.'+k]) for k in 'xyz'] for z in rr])
 t0=ref_t[0]
 m=(t>=t0)&(t<=t0+160);t,p=t[m],p[m]
 gt=np.column_stack([np.interp(t,ref_t,ref[:,k]) for k in range(3)])
 err=np.linalg.norm(p-gt,axis=1)
 dt=np.diff(t);jump=np.linalg.norm(np.diff(p,axis=0),axis=1)
 d={'count':len(t),'max_pose_gap_s':round(float(dt.max()),4),
    'gap_over_0p3_count':int(np.sum(dt>.3)),
    'largest_jump_m':round(float(jump.max()),4),
    'jumps_over_0p85_count':int(np.sum(jump>.85))}
 for name,lo,hi in [('pre_failure',0,90),('imu_outage',90,130),('recovery20',130,150),('whole_160',0,160)]:
  flag=(t-t0>=lo)&(t-t0<hi)
  d[name+'_rmse3d_m']=round(float(np.sqrt(np.mean(err[flag]**2))),4)
 data=(folder/'launch.log').read_text(errors='replace')
 for pattern,label2 in [('FAST-GUARD.*FAST pose gap','fast_pose_gap_quarantines'),
                        ('FAST-GUARD.*implausible FAST local motion','implausible_local_motion_quarantines'),
                        ('Rebuilding local voxel map','recovery_map_rebuilds'),
                        ('LOCALIZATION RECOVERY BUDGET','budget_insert_logs'),
                        ('\\[STEP3 TRACK\\] ACCEPT','trusted_lidar_PCD_accept_logs')]:
  d[label2]=len(re.findall(pattern,data))
 # The Oct16 archived run is 218s long while experimental repeats are
 # exactly 160s. Do NOT compare the total 1Hz GICP accept count across
 # unequal durations: count ROS simulator/event-time matched fixes only.
 accepted=[]
 for line in data.splitlines():
  if '[STEP3 TRACK] ACCEPT' not in line:continue
  stamps=re.search(r'\[\s*\d+\.\d+,\s*(\d+\.\d+)\s*\]',line)
  if stamps and 0<=float(stamps.group(1))-float(ref_t[0])<160:
   accepted.append(float(stamps.group(1))-float(ref_t[0]))
 d['accepted_lidar_PCD_in_first_160s']=len(accepted)
 d['accepted_lidar_PCD_during_outage_90_130s']=sum(90<=x<130 for x in accepted)
 d['accepted_lidar_PCD_during_recovery_130_150s']=sum(130<=x<150 for x in accepted)
 output[label]=d
 print(label,json.dumps(d),flush=True)
(OUT/'FROZEN_REPLAY_REPEATABILITY_AUDIT.json').write_text(json.dumps(output,indent=2)+'\n')
