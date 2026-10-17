#!/usr/bin/env python3
"""Report camera_lidar_imu_gps.bag replays against its OWN saved-map route.

All distances are disagreement with a map-aligned proxy, NOT independent GT.
The reference never comes from the held-out 18 September recording.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_trajectory(path):
    with open(path, newline='') as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        raise ValueError('Empty CSV: ' + str(path))
    if '%time' in rows[0]:
        t = np.array([int(row['%time']) * 1e-9 for row in rows])
        xyz = np.array([[float(row['field.pose.position.' + k]) for k in 'xyz']
                        for row in rows])
    else:
        t = np.array([float(row['timestamp']) for row in rows])
        xyz = np.array([[float(row[k]) for k in 'xyz'] for row in rows])
    return np.column_stack((t, xyz))


def read_pcd(path):
    # Direct little-endian PCD binary read, no dependency on ROS/open3d.
    with open(path, 'rb') as fp:
        headers = {}
        while True:
            line = fp.readline().decode('ascii').strip()
            if not line:
                raise ValueError('Incomplete PCD header')
            k, *v = line.split()
            headers[k] = v
            if k == 'DATA':
                break
        offset = fp.tell()
    if headers['DATA'] != ['binary']:
        raise ValueError('Expected binary PCD')
    fields = headers['FIELDS']
    fmts = []
    offsets = []
    stride = 0
    formats = {('F', 4): '<f4', ('F', 8): '<f8', ('U', 1): 'u1',
               ('I', 4): '<i4', ('U', 4): '<u4'}
    for kind, size, count in zip(headers['TYPE'], headers['SIZE'],
                                 headers.get('COUNT', ['1'] * len(fields))):
        size = int(size)
        if int(count) != 1:
            raise ValueError('Unexpected packed PCD fields')
        offsets.append(stride)
        fmts.append(formats[(kind, size)])
        stride += size
    dt = np.dtype({'names': fields, 'formats': fmts,
                   'offsets': offsets, 'itemsize': stride})
    data = np.memmap(path, dtype=dt, mode='r', offset=offset,
                     shape=(int(headers['POINTS'][0]),))
    xyz = np.stack([data[k] for k in 'xyz'], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)][::5]


def rmse(x):
    return float(np.sqrt(np.mean(x * x))) if len(x) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reference', required=True)
    ap.add_argument('--map-pcd', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--healthy', required=True)
    ap.add_argument('--camera', required=True)
    ap.add_argument('--lidar', required=True)
    ap.add_argument('--imu', required=True)
    ap.add_argument('--outage-start', type=float, default=125.)
    ap.add_argument('--outage-end', type=float, default=165.)
    args = ap.parse_args()
    ref = read_trajectory(Path(args.reference))
    t0 = float(ref[0, 0])
    if t0 < 1788850000 or t0 > 1788860000:
        raise ValueError('Development reference is not from 8 Sep 2026; refusing held-out or cross-bag reference')
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    mapping = {'Healthy': args.healthy, 'Camera failed': args.camera,
               'LiDAR failed': args.lidar, 'IMU failed': args.imu}
    stats, poses, errors = {}, {}, {}
    for mode, d in mapping.items():
        root = Path(d)
        g = read_trajectory(root / 'global_pose.csv')
        if len(g) < 1000 or g[-1, 0] < t0 + args.outage_end + 20:
            raise ValueError(f'Incomplete run {mode}: {len(g)} samples through {g[-1, 0]-t0:.1f}s')
        if ref[0, 0] > g[0, 0] + 0.25 or ref[-1, 0] < g[-1, 0] - .25:
            raise ValueError('Reference does not cover development replay: ' + str(root))
        expected = np.column_stack([np.interp(g[:, 0], ref[:, 0], ref[:, i])
                                    for i in (1, 2, 3)])
        e = np.linalg.norm(g[:, 1:] - expected, axis=1)
        ds = np.linalg.norm(np.diff(g[:, 1:], axis=0), axis=1)
        dt = np.diff(g[:, 0])
        mask = ((g[:, 0] >= t0 + args.outage_start) &
                (g[:, 0] < t0 + args.outage_end))
        pre = (g[:, 0] >= t0 + 60) & (g[:, 0] < t0 + args.outage_start)
        after = ((g[:, 0] >= t0 + args.outage_end) &
                 (g[:, 0] < t0 + args.outage_end + 20))
        log_path = root / 'launch.log'
        log = log_path.read_text(errors='replace') if log_path.exists() else ''
        stats[mode] = {
            'run': root.name,
            'samples': int(len(g)),
            'last_elapsed_s': round(float(g[-1, 0] - t0), 2),
            'full_reference_disagreement_rmse_m': rmse(e),
            'pre_failure_60_to_125_rmse_m': rmse(e[pre]),
            'failure_125_to_165_rmse_m': rmse(e[mask]),
            'first_20s_recovery_rmse_m': rmse(e[after]),
            'max_published_step_m': float(ds.max()),
            'p95_published_step_m': float(np.quantile(ds, .95)),
            'max_pose_time_gap_s': float(dt.max()),
            'published_steps_above_0p75m': int((ds > .75).sum()),
            'map_residual_measurements_published': log.count('[STEP3 ESIKF MAP MEAS]'),
            'map_residual_updates_applied': log.count('[STEP3 TRACK ESIKF] applied'),
            'accepted_map_track_total': log.count('[STEP3 TRACK] ACCEPT'),
            'ground_truth': False,
        }
        poses[mode], errors[mode] = g, e

    pcd = read_pcd(args.map_pcd)
    lo = ref[:, 1:3].min(axis=0) - 12
    hi = ref[:, 1:3].max(axis=0) + 12
    pcd = pcd[(pcd[:, 0] >= lo[0]) & (pcd[:, 0] <= hi[0]) &
              (pcd[:, 1] >= lo[1]) & (pcd[:, 1] <= hi[1])]
    fig, axs = plt.subplots(2, 2, figsize=(13, 11), dpi=145,
                            sharex=True, sharey=True, layout='constrained')
    for ax, (mode, pose) in zip(axs.flat, poses.items()):
        ax.scatter(pcd[:, 0], pcd[:, 1], s=.12, alpha=.25, color='gray', rasterized=True)
        ax.plot(ref[:, 1], ref[:, 2], '--', color='#d78a27', lw=1.6,
                label='Development loop-closed reference (not GT)')
        ax.plot(pose[:, 1], pose[:, 2], lw=1.6, color='#2062a5', label='Published map pose')
        if mode != 'Healthy':
            mask = ((pose[:, 0] >= t0 + args.outage_start) &
                    (pose[:, 0] < t0 + args.outage_end))
            ax.plot(pose[mask, 1], pose[mask, 2], lw=2.1, color='crimson',
                    label='125–165 s failure')
        ax.set_title(mode)
        ax.set_aspect('equal', 'box')
        ax.grid(alpha=.12)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.legend(fontsize=6.8, loc='best')
        ax.set_xlabel('PCD X (m)')
        ax.set_ylabel('PCD Y (m)')
    fig.suptitle('camera_lidar_imu_gps.bag — development four-mode test\n'
                 'LiDAR/PCD proxy comparison; NOT ground-truth accuracy', fontsize=13)
    fig.savefig(out / '01_four_mode_pcd_overlays.png')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(12, 5), dpi=155)
    for mode, pose in poses.items():
        ax.plot(pose[:, 0] - t0, errors[mode], lw=1.4, label=mode)
    ax.axvspan(args.outage_start, args.outage_end, color='gray', alpha=.12,
               label='125–165 s outage')
    ax.set_xlabel('Development bag elapsed time (s)')
    ax.set_ylabel('3D discrepancy from dev reference (m)')
    ax.set_title('Four-mode trajectory disagreement — not ground truth')
    ax.grid(alpha=.18)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / '02_four_mode_reference_disagreement.png')
    plt.close(fig)
    (out / 'comparison.json').write_text(json.dumps({
        'dataset': 'camera_lidar_imu_gps.bag',
        'reference': str(Path(args.reference).resolve()),
        'outage_start_s': args.outage_start,
        'outage_end_s': args.outage_end,
        'independent_ground_truth': False,
        'metrics': stats,
    }, indent=2) + '\n')
    with (out / 'comparison.csv').open('w', newline='') as fp:
        keys = ['mode'] + list(next(iter(stats.values())))
        w = csv.DictWriter(fp, fieldnames=keys)
        w.writeheader()
        for key, value in stats.items():
            w.writerow({'mode': key, **value})
    for name, row in stats.items():
        print(name, 'full', round(row['full_reference_disagreement_rmse_m'], 3),
              'outage', round(row['failure_125_to_165_rmse_m'], 3),
              'largest step', round(row['max_published_step_m'], 3), flush=True)
    print('SAVED', out)


if __name__ == '__main__':
    main()
