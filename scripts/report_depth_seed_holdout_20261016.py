#!/usr/bin/env python3
"""Compare the frozen best runs with *rejected* seed-correction experiments.

Diagnostic only: never alters the selected best runs or sets deployment-ready.
"""
from pathlib import Path
import csv
import json
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
TRIAL=ROOT/'step3_global/depth_bounded_seed_trial_20261016'
CASES={
 'main_035':'seed_cap_0p35_vs_curated_185s.json',
 'main_015':'seed_cap_0p15_vs_curated_185s.json',
 'holdout_035':'heldout_18sep_seed_0p35_vs_curated_210s.json',
}
RUNS={
 'main_035':'DEPTH_SEED_BOUNDED_0P35_190S',
 'main_015':'DEPTH_SEED_BOUNDED_0P15_190S',
 'holdout_035':'HELDOUT_18SEP_DEPTH_SEED_0P35_220S',
}

def motion_diagnostics(run):
    with (run/'global_pose.csv').open() as f:
        rows=list(csv.DictReader(f))
    stamp=np.array([float(x['field.header.stamp'])*1e-9 for x in rows])
    xyz=np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in rows])
    dt=np.diff(stamp)
    dist=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
    v=dist/np.maximum(dt,1.e-3)
    return {'p95_speed_mps':round(float(np.percentile(v,95)),3),
            'max_speed_mps':round(float(np.max(v)),3),
            'pose_steps_over_0p85m':int((dist>0.85).sum()),
            'speed_samples_over_6mps':int((v>6.).sum()),
            'status':'offline_only_not_connected_to_live_planner_or_dbw'}

def main():
    results={}
    for key,filename in CASES.items():
        data=json.loads((TRIAL/filename).read_text())
        assert not data['independent_ground_truth']
        assert not data['strict_quality_gate_passed']
        assert not data['deployment_approved']
        out=TRIAL/RUNS[key]
        assert (out/'plots/07_orange_trajectory_on_pcd_clean.png').is_file()
        assert 'play_rate=1' in (out/'run_metadata.txt').read_text()
        assert (out/'global_pose.csv').stat().st_size>100000
        results[key]={**data,'motion_diagnostics':motion_diagnostics(out),
                      'run_directory':str(out.relative_to(ROOT))}
    baseline=results['main_035']['old']
    improved=results['main_035']['new']
    too_little=results['main_015']['new']
    heldout_old=results['holdout_035']['old']
    heldout_new=results['holdout_035']['new']
    assert improved['outage_proxy_rmse_m']<baseline['outage_proxy_rmse_m']
    assert improved['recovery_proxy_rmse_m']>baseline['recovery_proxy_rmse_m']
    assert too_little['outage_proxy_rmse_m']>baseline['outage_proxy_rmse_m']
    assert heldout_new['outage_proxy_rmse_m']>heldout_old['outage_proxy_rmse_m']

    manifest={
       'date':'2026-10-16', 'status':'ALL_NEW_GICP_SEED_CAPS_REJECTED_FOR_PROMOTION',
       'same_bag_proxy_not_independent_ground_truth':True,
       'the_two_bags_are_not_an_apples_to_apples_control_for_prior_image_version':True,
       'training_or_fitting_done_on_sep18':False,
       'selected_best_runs_remain_unchanged':True,
       'deployable':False,
       'results':results,
    }
    target=ROOT/'reports/DEPTH_SEED_LIMIT_CROSS_BAG_VALIDATION_20261016.json'
    target.write_text(json.dumps(manifest,indent=2)+'\n')

    table=[
        '# RGB-D visual-seed correction cap: two-bag evaluation — 2026-10-16',
        '',
        '**Decision: do not promote any candidate, even the 0.35m cap.**',
        '',
        'All figures are differences against bag-specific loop-closed proxy trajectories,',
        '**not surveyed ground truth**. No map or visual keyframe database was fit to September 18.',
        '',
        '| Recording / version | 3D RMSE during LiDAR loss | Recovery 20s RMSE | Worst pose step |',
        '|---|---:|---:|---:|',
        f'| Main original curated | {baseline["outage_proxy_rmse_m"]:.3f}m | {baseline["recovery_proxy_rmse_m"]:.3f}m | {baseline["max_published_pose_step_m"]:.3f}m |',
        f'| Main: 0.35m max GICP shift | {improved["outage_proxy_rmse_m"]:.3f}m | {improved["recovery_proxy_rmse_m"]:.3f}m | {improved["max_published_pose_step_m"]:.3f}m |',
        f'| Main: 0.15m max GICP shift | {too_little["outage_proxy_rmse_m"]:.3f}m | {too_little["recovery_proxy_rmse_m"]:.3f}m | {too_little["max_published_pose_step_m"]:.3f}m |',
        f'| Sep18 curated best | {heldout_old["outage_proxy_rmse_m"]:.3f}m | {heldout_old["recovery_proxy_rmse_m"]:.3f}m | {heldout_old["max_published_pose_step_m"]:.3f}m |',
        f'| Sep18: unchanged 0.35m setting | {heldout_new["outage_proxy_rmse_m"]:.3f}m | {heldout_new["recovery_proxy_rmse_m"]:.3f}m | {heldout_new["max_published_pose_step_m"]:.3f}m |',
        '',
        '## Interpreting the result',
        '',
        '- The 0.35m variant improves the **main-bag outage** by 11.2% compared',
        '  with the frozen Oct13 best, consistent with bad ICP minima sliding',
        '  position away from a useful visual-place seed. However, it worsens',
        '  recovery and has many 0.85m+ published pose steps.',
        '- Tightening to 0.15m loses useful geometric correction: it regresses',
        '  both outage and recovery on the main bag. There is no safe monotonic',
        '  gain to obtain just by increasing/decreasing this parameter.',
        '- With the **unchanged** 0.35m setting, the Sep18 outage becomes',
        '  worse than the archived best despite some better recovery. This',
        '  rules out claiming cross-bag generalisation.',
        '- **Important confound:** the archived Sep18 curated baseline is the',
        '  older Oct7 development image, while the new run uses the Oct13',
        '  frozen base. Therefore the September difference is a model-version',
        '  **plus** parameter comparison, not a controlled isolated cap effect.',
        '- Spatial accuracy, geometry observability and heading consistency',
        '  must be fixed at the source using distinctive static landmarks',
        '  and independent registration constraints. The map and image-frame',
        '  transforms are numerically orthonormal; physical extrinsics remain',
        '  unverified to sub-0.5m field tolerance.',
        '',
        '## Deployment and safety',
        '',
        '- Retained curated best runs: `results/BEST_SO_FAR_20261015/` (all 8 intact).',
        '- No current candidate meets <=0.5m outage RMSE or jump/dropout gates.',
        '- None has a hardware-validated localizer-untrusted -> planner/DBW stop.',
        '- Offline velocity spikes and correction steps must not be hidden by',
        '  unverified interpolation or smoothing before the cart is tested.',
        "- Leave the live cart code untouched; don't promote or deploy.",
        '',
        '## Artifacts',
        '',
        f'- Full JSON audit: `{target.relative_to(ROOT)}`',
        f'- New trials and plots: `{TRIAL.relative_to(ROOT)}/`',
        '- Reproducibility: original bag input, exact 1x replay, fixed model,',
        '  original map and camera calibration, original Sep18 reference only',
        '  during offline scoring.',
    ]
    doc=ROOT/'reports/DEPTH_SEED_LIMIT_CROSS_BAG_VALIDATION_20261016.md'
    doc.write_text('\n'.join(table)+'\n')
    print('SAVED',doc)
    print('SAVED',target)
    for key in CASES:
        item=results[key]
        print(key,'outage',item['new']['outage_proxy_rmse_m'],
              'recovery',item['new']['recovery_proxy_rmse_m'],
              'step',item['new']['max_published_pose_step_m'],
              'max_speed_mps',item['motion_diagnostics']['max_speed_mps'])
    print('PROMOTION REFUSED')

if __name__=='__main__':main()
