#!/usr/bin/env python3
"""Audit sensor-mode chatter and accuracy on the SAME 190s diagnostic proxy.

Read-only: the curated 8 best runs, maps, production images and sources are
never modified. This report must NOT be described as ground-truth accuracy.
"""
import csv
import json
import re
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'results/BEST_SO_FAR_20261015/camera_lidar_imu_gps/LIDAR_FAILED'
TRIALS={
 'curated_oct13_best':BASE,
 'slow_correction_ramp':ROOT/'step3_global/depth_ramp_rate_trial_20261016/LIDAR_FAIL_SLOW_RAMP_195S',
 'rgbd_vote_v1':ROOT/'step3_global/rgbd_mode_vote_trial_20261016/LIDAR_FAIL_RGBD_MODE_VOTE_195S',
 'rgbd_vote_hysteresis_v2':ROOT/'step3_global/rgbd_mode_vote_hysteresis_trial_20261016/LIDAR_FAIL_RGBD_VOTE_HYST_195S',
}
REFERENCE=ROOT/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv'

def score(folder,rt,refpos):
    with (folder/'global_pose.csv').open(newline='') as f:
        data=list(csv.DictReader(f))
    t=np.array([float(v['field.header.stamp'])*1e-9 for v in data])
    pos=np.array([[float(v['field.pose.position.'+k]) for k in 'xyz'] for v in data])
    m=(t>=rt[0])&(t<=rt[0]+190)
    t=t[m];pos=pos[m]
    if len(t)<1500 or t[-1]<rt[0]+189:
        raise ValueError(f'{folder}: not a 1x 190s complete trajectory')
    target=np.stack([np.interp(t,rt,refpos[:,i]) for i in range(3)],axis=1)
    err=np.linalg.norm(pos-target,axis=1)
    steps=np.linalg.norm(np.diff(pos,axis=0),axis=1)
    def rmse(a,b):
        mask=(t>=rt[0]+a)&(t<rt[0]+b)
        return round(float(np.sqrt(np.mean(err[mask]**2))),3)
    log=(folder/'launch.log').read_text(errors='replace')
    pattern=r'\[FAILSAFE\] VI_LIDAR_FAILED -> IMU_ONLY_BRIDGE'
    pattern2=r'\[FAILSAFE\] IMU_ONLY_BRIDGE -> VI_LIDAR_FAILED'
    switches=list(re.finditer(pattern,log))
    re_entries=list(re.finditer(pattern2,log))
    return {
        'full_0_to_190s_rmse_m':rmse(0,190),
        'pre_failure_60_to_90s_rmse_m':rmse(60,90),
        'outage_125_to_165s_rmse_m':rmse(125,165),
        'recovery_165_to_185s_rmse_m':rmse(165,185),
        'max_published_step_m':round(float(steps.max()),3),
        'steps_greater_0p85m':int((steps>.85).sum()),
        'max_timestamp_gap_s':round(float(np.diff(t).max()),3),
        'vi_to_imu_mode_switches':len(switches),
        'imu_to_vi_mode_switches':len(re_entries),
        'rgbd_vote_keeping_events':log.count('[FAILSAFE RGBD VOTE] keeping'),
        'rgbd_vote_stale_events':log.count('[FAILSAFE RGBD VOTE] sensor-time stale'),
        'depth_map_registration_events':log.count('PLACE-VERIFIED depth_saved_map_icp'),
        'depth_anchor_accepted_events':log.count('[STEP3 DEPTH VERIFIED]'),
        'complete_190s':True,
    }

def main():
    with REFERENCE.open(newline='') as f:
        data=list(csv.DictReader(f))
    rt=np.array([float(z['timestamp']) for z in data]);p=np.array([[float(z[c]) for c in 'xyz'] for z in data])
    scored={k:score(v,rt,p) for k,v in TRIALS.items()}
    result={
        'origin':'camera_lidar_imu_gps.bag, full 1x, same window [0,190]s; LiDAR withheld [125,165]s',
        'reference':str(REFERENCE.relative_to(ROOT)),
        'reference_is_independent_ground_truth':False,
        'any_experiment_promoted':False,
        'vehicle_safe_to_deploy':False,
        'outcome':'REJECT all three trials: none improves both continuity and outage/recovery error',
        'scored':scored,
    }
    out=ROOT/'step3_global/rgbd_mode_vote_hysteresis_trial_20261016/ALL_LIDAR_TRIALS_AUDIT_20261016.json'
    out.write_text(json.dumps(result,indent=2)+'\n')
    print('Saved:',out)
    print('Name                          | full | outage | recovery | max step | >.85m steps | VI->IMU | RGBD vote hits')
    for key,v in scored.items():
        print(f'{key:30} {v["full_0_to_190s_rmse_m"]:6.3f} {v["outage_125_to_165s_rmse_m"]:8.3f} '
              f'{v["recovery_165_to_185s_rmse_m"]:10.3f} {v["max_published_step_m"]:10.3f} '
              f'{v["steps_greater_0p85m"]:12d} {v["vi_to_imu_mode_switches"]:10d} '
              f'{v["rgbd_vote_keeping_events"]:12d}')

if __name__=='__main__':main()
