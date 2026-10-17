#!/usr/bin/env python3
"""Timestamp-correct score of geometrically accepted *raw* RGB-D/PCD matches.

This does not change or fit the trajectory, estimator, map, or reference.
The loop-closed path is a proxy, NOT surveyed ground truth.
"""
import csv
import json
import re
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'step3_global/lidar_fail_depth_timing_diag_20261016/DEPTH_TIMING_180S'
REFERENCE=ROOT/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv'
pattern=(r'\[DEPTH TIMING DIAG\] depth_src=([\d.]+) depth_corrected=([\d.]+) '
         r'ros_now=([\d.]+) local_latest=([\d.]+) compensated=(\d+) '
         r'delay=([\d.-]+)s xyz=([\d.-]+),([\d.-]+),([\d.-]+) fitness=([\d.]+)')

def main():
    matches=re.findall(pattern,(RUN/'launch.log').read_text(errors='replace'))
    if not matches:raise SystemExit('No corrected depth-map measurements found in log')
    raw=np.array([[float(z) for z in item] for item in matches])
    with REFERENCE.open(newline='') as f:rows=list(csv.DictReader(f))
    ref_t=np.array([float(z['timestamp']) for z in rows])
    ref_x=np.array([[float(z[k]) for k in 'xyz'] for z in rows])
    measured_t=raw[:,1]
    xyz=raw[:,6:9]
    ref=np.stack([np.interp(measured_t,ref_t,ref_x[:,k]) for k in range(3)],axis=1)
    errors=xyz-ref

    def score(start,end):
        m=(measured_t-ref_t[0]>=start)&(measured_t-ref_t[0]<end)
        err=errors[m]
        if not len(err):return {'n':0}
        return dict(n=int(m.sum()),xy_rmse_m=round(float(np.sqrt((err[:,:2]**2).sum(axis=1).mean())),4),
                    xyz_rmse_m=round(float(np.sqrt((err**2).sum(axis=1).mean())),4),
                    xyz_median_error_m=round(float(np.median(np.linalg.norm(err,axis=1))),4),
                    mean_xyz_bias_m=np.round(err.mean(axis=0),4).tolist())

    scored={'125_to_135':score(125,135),'135_to_145':score(135,145),
            '145_to_155':score(145,155),'155_to_165':score(155,165),
            'entire_lidar_outage_125_to_165':score(125,165)}
    output=dict(result='diagnostic_measurement_quality_only_not_an_algorithm_improvement',
                reference_is_independent_ground_truth=False,
                source='actual accepted depth-to-saved-voxel-map GICP measurements',
                matched_by='corrected PoseStamped header stamp, not ROS log time',
                median_delay_seconds=round(float(np.median(raw[:,5])),4),
                p95_delay_seconds=round(float(np.percentile(raw[:,5],95)),4),
                maximum_delay_seconds=round(float(np.max(raw[:,5])),4),
                uncompensated_count=int((raw[:,4]==0).sum()),
                median_icp_fitness=round(float(np.median(raw[:,9])),4),scores=scored)
    path=RUN.parent/'RAW_DEPTH_VS_REFERENCE_TIMESTAMP_CORRECTED_20261016.json'
    path.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2));print('SAVED',path)

if __name__=='__main__': main()
