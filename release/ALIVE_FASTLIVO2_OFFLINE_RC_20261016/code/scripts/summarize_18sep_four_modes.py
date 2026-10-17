#!/usr/bin/env python3
"""Compare same-bag four-mode failsafe runs to a *non-GT* LiDAR/PCD proxy.

Keeps all original runs unchanged and records map-match continuity and
trajectory differences without mistaking a LiDAR-derived reference for truth.
"""
import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from build_18sep_heldout_map_reference import read_pcd_xyz, vox


NAMES = {
    'Healthy': 'REVAL_CORNER_FIXED_HEALTHY_170S_20261009',
    'Camera failed': 'REVAL_CORNER_FIXED_CAMERA_FAIL_170S_20261009',
    'LiDAR failed': 'REVAL_CORNER_FIXED_LIDAR_FAIL_170S_20261009',
    'IMU failed': 'REVAL_CORNER_FIXED_IMU_FAIL_170S_20261009',
}


def load_pose(path):
    with open(path) as f:
        return np.array([
            [int(r['%time']) / 1e9] +
            [float(r['field.pose.position.' + coord]) for coord in 'xyz']
            for r in csv.DictReader(f)
        ])


def stats(values):
    return float(np.sqrt(np.mean(values ** 2))) if len(values) else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', required=True)
    ap.add_argument('--reference', required=True)
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    root = Path(args.results)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(args.reference) as f:
        ref_rows = list(csv.DictReader(f))
    ref = np.array([[float(r[k]) for k in ('timestamp', 'x', 'y', 'z')]
                    for r in ref_rows])
    reliable = np.array([r['reference_quality'] in
                         ('high','medium','corner_multistart_map_verified',
                          'corner_transition_smoothing') for r in ref_rows])
    origin = ref[0, 0]  # Same-bag LiDAR initial stamp; ~0.067s after bag start.
    failures = (origin + 90, origin + 130)
    results = {}
    for mode, dirname in NAMES.items():
        directory = root / dirname
        path = directory / 'global_pose.csv'
        if not path.exists():
            raise FileNotFoundError(f'Missing fresh test {path}')
        pose = load_pose(path)
        pose = pose[(pose[:, 0] >= ref[0, 0]) & (pose[:, 0] <= ref[-1, 0])]
        if len(pose) < 50:
            raise RuntimeError(f'Incomplete run {mode}: {len(pose)} samples')
        xyz = np.column_stack([np.interp(pose[:, 0], ref[:, 0], ref[:, k])
                               for k in (1, 2, 3)])
        error = np.linalg.norm(pose[:, 1:4] - xyz, axis=1)
        mask = (pose[:, 0] >= failures[0]) & (pose[:, 0] < failures[1])
        pre = (pose[:, 0] >= origin + 60) & (pose[:, 0] < failures[0])
        post = (pose[:, 0] >= failures[1]) & (pose[:, 0] < origin + 150)
        qual = np.interp(pose[:, 0], ref[:, 0], reliable.astype(float)) > .99
        dstep = np.linalg.norm(np.diff(pose[:, 1:4], axis=0), axis=1)
        elapsed = pose[:, 0] - origin
        logpath = directory / 'launch.log'
        log = logpath.read_text(errors='replace') if logpath.exists() else ''
        match_t = [float(v) for v in re.findall(
            r'\[\d+\.\d+,\s*(\d+\.\d+)\]: \[STEP3 TRACK\] ACCEPT', log)]
        depth_t = [float(v) for v in re.findall(
            r'\[\d+\.\d+,\s*(\d+\.\d+)\]: \[FAILSAFE MAP\] PLACE-VERIFIED depth_saved_map_icp ICP accepted',
            log)]
        track_window = sorted(t for t in match_t if failures[0] <= t < failures[1])
        depth_window = sorted(t for t in depth_t if failures[0] <= t < failures[1])
        allmap = sorted(track_window + depth_window)
        boundaries = [failures[0]] + allmap + [failures[1]]
        longest_gap = max(np.diff(boundaries)) if len(boundaries) > 1 else 40.
        mm = mask & qual
        results[mode] = {
            'run': dirname,
            'n_poses': len(pose),
            'elapsed_last_s': round(float(elapsed[-1]), 2),
            'trajectory_proxy_full_rmse_m': round(stats(error), 3),
            'trajectory_proxy_outage_90_130_rmse_m': round(stats(error[mask]), 3),
            'trajectory_proxy_pre_outage_60_90_rmse_m': round(stats(error[pre]), 3),
            'trajectory_proxy_recovery_130_150_rmse_m': round(stats(error[post]), 3),
            'trajectory_proxy_outage_reliable_only_rmse_m': round(stats(error[mm]), 3),
            'trajectory_proxy_outage_max_disagreement_m': round(float(np.max(error[mask])), 3),
            'trajectory_proxy_outage_end_disagreement_m': round(float(error[np.where(mask)[0][-1]]), 3),
            'largest_published_step_m': round(float(dstep.max()), 3),
            'accepted_pcd_track_in_outage': len(track_window),
            'accepted_depth_map_in_outage': len(depth_window),
            'longest_gap_without_confirmed_map_match_s': round(float(longest_gap), 2),
            'is_independent_ground_truth': False,
        }
        results[mode]['_plot'] = {'t': elapsed, 'e': error, 'pose': pose}

    pcd = read_pcd_xyz(args.map).astype(np.float32)
    pcd = pcd[np.isfinite(pcd).all(axis=1)]
    pcd = pcd[(pcd[:, 0] > -10) & (pcd[:, 0] < 145) &
              (pcd[:, 1] > -190) & (pcd[:, 1] < 25)]
    pcd = vox(pcd, .78)[::2]
    for mode, entry in results.items():
        name = mode.lower().replace(' ', '_')
        pose = entry['_plot']['pose']
        fig, ax = plt.subplots(figsize=(9.1, 11.4), dpi=150)
        ax.scatter(pcd[:, 0], pcd[:, 1], s=.14, c='.7', alpha=.32,
                   rasterized=True, label='Previously loop-closed saved map')
        ax.plot(ref[:, 1], ref[:, 2], '--', color='#da8325', lw=2.2,
                label='18 Sep LiDAR/PCD reference proxy (not GT)')
        ax.plot(pose[:, 1], pose[:, 2], color='#2366a7', lw=1.7,
                label=f'{mode}: published global pose')
        drop = ((pose[:, 0] >= failures[0]) & (pose[:, 0] < failures[1]))
        if mode != 'Healthy':
            ax.plot(pose[drop, 1], pose[drop, 2], color='crimson', lw=2.5,
                    label='Injected 90–130 s failure')
        ax.set_aspect('equal', 'box')
        ax.set_xlim(-7, 136)
        ax.set_ylim(-185, 20)
        ax.legend(loc='lower left', fontsize=8)
        ax.set_xlabel('PCD X (m)')
        ax.set_ylabel('PCD Y (m)')
        ax.set_title(f'{mode} — corrected September 18 map reference\n'
                     'trajectory disagreement only, NOT ground-truth accuracy')
        fig.tight_layout()
        fig.savefig(out / f'{name}_pcd_overlay.png')
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 5), dpi=150)
    for mode, entry in results.items():
        payload = entry['_plot']
        ax.plot(payload['t'], payload['e'], lw=1.7, label=mode)
    ax.axvspan(90, 130, color='gray', alpha=.15, label='90–130 s outage')
    ax.set_xlabel('Time from first LiDAR scan (s)')
    ax.set_ylabel('3D disagreement vs LiDAR/PCD proxy (m)')
    ax.set_title('Four-mode September 18 comparison — NOT independent localization accuracy')
    ax.grid(alpha=.15)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / 'all_four_modes_disagreement_vs_time.png')
    plt.close(fig)

    cleaned = {mode: {k: v for k, v in s.items() if k != '_plot'}
               for mode, s in results.items()}
    (out / 'summary.json').write_text(json.dumps({
        'reference': str(Path(args.reference).resolve()),
        'evaluation_type': 'NON-GT: LiDAR-derived same-bag PCD reference proxy',
        'test_window': '90–130 seconds after recorded bag start',
        'runs': cleaned,
        'caveat': 'LiDAR+IMU relative odometry contributes to reference; '
                  'this comparison favors LiDAR-based pipelines and is not independent accuracy.'
    }, indent=2) + '\n')
    with (out / 'comparison.csv').open('w', newline='') as f:
        first = next(iter(cleaned.values()))
        w = csv.DictWriter(f, fieldnames=['mode'] + list(first))
        w.writeheader()
        for mode, s in cleaned.items():
            w.writerow({'mode': mode, **s})
    print(json.dumps(cleaned, indent=2))


if __name__ == '__main__':
    main()
