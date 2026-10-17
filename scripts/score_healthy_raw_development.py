#!/usr/bin/env python3
"""Score RAW local odometry on the original development bag, independent of map pose smoother.

The map-aligned loop-closed development trajectory is only a proxy, NOT surveyed
ground truth. Relative segment errors remove the arbitrary FAST local frame.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_pose(path):
    with open(path, newline='') as f:
        records = list(csv.DictReader(f))
    if not records:
        raise ValueError('empty poses: ' + str(path))
    return np.array([[float(r[key]) for key in ('timestamp', 'x', 'y', 'z')]
                     for r in records], dtype=float)


def rigid_align(x, y):
    xm, ym = x.mean(axis=0), y.mean(axis=0)
    U, s, Vt = np.linalg.svd((x-xm).T @ (y-ym))
    fix = np.diag((1.0, float(np.linalg.det(U @ Vt))))
    M = U @ fix @ Vt
    return M, ym - xm @ M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reference', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--reference-from-gicp', action='store_true',
                    help='compare raw poses against each run verified saved-PCD GICP matches (not GT)')
    ap.add_argument('--max-raw-error-at-25s-m', type=float,
                    help='optional strict development acceptance gate (meters)')
    ap.add_argument('--max-raw-error-at-60s-m', type=float,
                    help='optional strict development acceptance gate (meters)')
    ap.add_argument('--trajectory', required=True, action='append',
                    help='name:path to raw trajectory.csv; repeat')
    args = ap.parse_args()
    ref = read_pose(args.reference)
    assert 1788850000 < ref[0, 0] < 1788860000, 'must use original development reference'
    t0 = ref[0, 0]
    out = {}
    gate_failures = []
    for entry in args.trajectory:
        name, path = entry.split(':', 1)
        raw = read_pose(path)
        t = raw[:, 0]
        if args.reference_from_gicp:
            with open(Path(path).parent / 'gicp_map_pose.csv', newline='') as fp:
                matches=list(csv.DictReader(fp))
            gicp=np.array([[int(row['%time'])*1e-9] +
                          [float(row['field.pose.position.'+axis]) for axis in ('x','y')]
                           for row in matches])
            if len(gicp)<20:
                raise ValueError('insufficient LiDAR/PCD matches')
            reference_xy = gicp
            # Only score within the observed GICP timespan, never extrapolate
            # beyond verified saved-map observations.
            keep=(t>=gicp[0,0])&(t<=gicp[-1,0])
            raw=raw[keep]
            t=raw[:,0]
        else:
            reference_xy=ref[:,:3]
        y = np.stack([np.interp(t, reference_xy[:, 0], reference_xy[:, col])
                      for col in (1, 2)], axis=1)
        # One initial local-to-map transform, NEVER re-align each later window:
        # doing that would hide real accumulated trajectory drift.
        a = (t-t0 >= 4) & (t-t0 <= 10)
        if a.sum() < 8:
            raise ValueError(f'insufficient early odometry for {name}')
        M, b = rigid_align(raw[a, 1:3], y[a])
        xy = raw[:, 1:3] @ M+b
        e = np.linalg.norm(xy-y, axis=1)
        metrics = {'samples': len(raw), 'elapsed_last_s': float(t[-1]-t0),
                   'early_align_rotation_deg': float(np.degrees(np.arctan2(M[0, 1], M[0, 0])))}
        for sec in (10, 15, 20, 25, 30, 40, 50, 60, 75):
            if sec <= t[-1]-t0:
                i = np.argmin(abs(t-(t0+sec)))
                metrics[f'raw_xy_error_at_{sec}s_m'] = float(e[i])
        for lo,hi in ((10,20),(15,25),(20,30),(30,40),(40,50),(50,60)):
            if hi>t[-1]-t0:continue
            i = np.argmin(abs(t-(t0+lo)))
            j = np.argmin(abs(t-(t0+hi)))
            dp=xy[j]-xy[i]
            dr=y[j]-y[i]
            metrics[f'raw_relative_{lo}_{hi}s_displacement_error_m']=float(np.linalg.norm(dp-dr))
            metrics[f'raw_relative_{lo}_{hi}s_distance_bias_m']=float(np.linalg.norm(dp)-np.linalg.norm(dr))
        out[name] = metrics
        for sec, limit in ((25, args.max_raw_error_at_25s_m),
                           (60, args.max_raw_error_at_60s_m)):
            if limit is None:
                continue
            value = metrics.get(f'raw_xy_error_at_{sec}s_m')
            if value is None or value > limit:
                gate_failures.append(f'{name}: raw {sec}s error {value} exceeds {limit}m')
        print(name, 'RAW x/y disagreement at 25s=',
              round(metrics.get('raw_xy_error_at_25s_m', float('nan')), 3),
              'at 60s=',round(metrics.get('raw_xy_error_at_60s_m', float('nan')),3))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps({'dataset':'camera_lidar_imu_gps.bag',
                                         'reference_kind':'same-bag saved-PCD GICP' if args.reference_from_gicp else 'development loop-closed trajectory',
                                         'independent_ground_truth': False,
                                         'cases':out}, indent=2)+'\n')
    if gate_failures:
        raise SystemExit('HEALTHY RAW GATE FAILED — ' + '; '.join(gate_failures))


if __name__ == '__main__':
    main()
