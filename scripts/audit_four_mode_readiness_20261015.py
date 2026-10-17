#!/usr/bin/env python3
"""Conservative post-run readiness audit; deliberately refuses false PASS.

Reads complete four-mode development and September holdout runs. Uses only
saved trajectories/logs/metadata and an UNVALIDATED same-bag proxy. Does not
infer localization ground truth or certify hardware latency/safety.
"""
import csv
import json
import math
from pathlib import Path
import argparse

cli=argparse.ArgumentParser()
cli.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[1])
cli.add_argument('--stage',choices=('development','holdout','all'),default='all')
a=cli.parse_args()
repo=a.repo
dev=repo/'results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015'
hold=repo/'results/18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015'
old=repo/'results/camera_lidar_imu_gps/LATEST_20261013_1X_290S'
cases={'healthy':'none','camera':'camera','lidar':'lidar','imu':'imu'}

def parse_metrics(path):
    out={}
    if not path.exists():return None
    for line in path.read_text(errors='replace').splitlines():
        if '=' not in line:continue
        k,v=line.split('=',1)
        try:out[k]=float(v)
        except ValueError:out[k]=v
    return out

def count_transitions(mode,log):
    expected={'camera':'LIO_CAMERA_FAILED','lidar':'VI_LIDAR_FAILED','imu':'LV_IMU_FAILED'}
    if mode=='healthy':return {'expected_failure_mode_detected':True}
    target=expected[mode]
    return {
        'expected_failure_mode_detected':target in log,
        'return_to_full_livo_detected':(' -> FULL_LIVO' in log or '-> FULL_LIVO' in log),
        'failure_target':target,
    }

def read_samples(csv_path):
    with csv_path.open(newline='') as f:
        rows=list(csv.DictReader(f))
    return rows

def check_dataset(stage,base,duration):
    summary={'duration_s':duration,'runs':{},'all_safety_quality_gates_passed':True}
    for mode,sensor in cases.items():
        stem={'healthy':'HEALTHY','camera':'CAMERA_FAIL','lidar':'LIDAR_FAIL','imu':'IMU_FAIL'}[mode]
        folder=base/(f'{stem}_BRIDGE_290S' if stage=='development' else
                     f'{stem}_HOLDOUT_220S')
        m=parse_metrics(folder/'metrics_exact.txt')
        if m is None:
            summary['runs'][mode]={'completed':False,'checks':{'run_exists':False}}
            summary['all_safety_quality_gates_passed']=False
            continue
        log=(folder/'launch.log').read_text(errors='replace')
        samples=read_samples(folder/'global_pose.csv')
        t=[float(x['field.header.stamp'])*1e-9 for x in samples]
        gaps=[y-x for x,y in zip(t,t[1:])]
        max_gap=max(gaps,default=float('inf'))
        xyz=[[float(x['field.pose.position.'+c]) for c in 'xyz'] for x in samples]
        steps=[math.dist(x,y) for x,y in zip(xyz,xyz[1:])]
        max_step=max(steps,default=float('inf'))
        fail_window=m.get('reference_disagreement_failure_rmse3d_m')
        recovery=m.get('reference_disagreement_post_failure_20s_rmse3d_m')
        proxy=m.get('reference_disagreement_3d_rmse_m')
        completed=(m.get('duration',0)>=duration-5 and len(samples)>=duration*8)
        transition=count_transitions(mode,log)
        # Confirm injectors actually changed modes; merely running the case
        # named *_FAIL is NOT evidence of dropout.
        transition_ok=all(bool(v) for k,v in transition.items() if k!='failure_target')
        baseline=parse_metrics(old/({'healthy':'HEALTHY','camera':'CAMERA_FAIL',
                                    'lidar':'LIDAR_FAIL','imu':'IMU_FAIL'}[mode])/'metrics_exact.txt') if stage=='development' else None
        checks={
          'full_duration_and_pose_stream':completed,
          'expected_mode_entered_and_exited':transition_ok,
          'no_pose_blackout_over_0p5s':max_gap<=.5,
          'no_abrupt_published_step_over_0p85m':max_step<=.85,
          'no_larger_than_0p5m_proxy_rmse':proxy is not None and proxy<=.5,
        }
        if baseline:
            checks['no_large_full_proxy_regression']=(proxy<=baseline['reference_disagreement_3d_rmse_m']+.30)
            checks['no_large_jump_regression']=(max_step<=baseline['step_max_m']+.15)
            if mode!='healthy' and fail_window is not None:
                checks['no_large_failure_proxy_regression']=fail_window<=baseline['reference_disagreement_failure_rmse3d_m']+.45
                checks['no_large_recovery_proxy_regression']=recovery<=baseline['reference_disagreement_post_failure_20s_rmse3d_m']+.45
        entry={'completed':completed,'source':str(folder),
               'proxy_rmse3d_m':proxy,'failure_proxy_rmse3d_m':fail_window,
               'first_20s_recovery_proxy_rmse3d_m':recovery,
               'max_pose_step_m':round(max_step,3),'max_pose_gap_s':round(max_gap,3),
               'steps_above_0p75m':sum(s>.75 for s in steps),
               'pose_count':len(samples),'transitions':transition,
               'checks':checks,'all_checks_passed':all(checks.values())}
        summary['runs'][mode]=entry
        if not entry['all_checks_passed']:summary['all_safety_quality_gates_passed']=False
    return summary

report={'reference_independent_ground_truth':False,
        '0p5m_accuracy_certifiable':False,
        'production_deployment_approved':False,
        'note':'No surveyed ground truth or real NUC closed-loop validation. Conservative safety limits are advisory thresholds, not a vehicle safety certificate.'}
if a.stage in ('development','all'):
    report['development']=check_dataset('development',dev,290)
if a.stage in ('holdout','all'):
    report['holdout']=check_dataset('holdout',hold,220)
dest=dev/'four_mode_readiness_audit.json'
dest.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
