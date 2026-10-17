#!/usr/bin/env python3
"""Fail-closed, reproducible evaluation of new LiDAR-outage registration trials.

All numbers use the same *non-independent* loopclosed reference and same
185-second window. Does not mutate production, maps, or curated results.
"""
from pathlib import Path
import csv
import json
import math
import re

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'results/BEST_SO_FAR_20261015/camera_lidar_imu_gps/LIDAR_FAILED'
EVIDENCE={
 'local_depth_tracker_2p2s':(
  'step3_global/depth_temporal_tracker_trial_20261016/depth_local_2p2_vs_curated_185s.json',
  'step3_global/depth_temporal_tracker_trial_20261016/DEPTH_LOCAL_2P2_190S'),
 'rgbd_saved_keyframe_pnp':(
  'step3_global/depth_feature_pnp_trial_20261016/feature_pnp_vs_curated_185s.json',
  'step3_global/depth_feature_pnp_trial_20261016/DEPTH_FEATURE_PNP_190S'),
 'two_map_xyz_blend':(
  'step3_global/depth_crossmap_trial_20261016/true_dual_map_vs_curated_185s.json',
  'step3_global/depth_crossmap_trial_20261016/DEPTH_DUAL_MAP_LOADED_190S'),
 'two_map_veto_only':(
  'step3_global/depth_crossmap_veto_trial_20261016/veto_only_vs_curated_185s.json',
  'step3_global/depth_crossmap_veto_trial_20261016/DEPTH_CROSSMAP_VETO_190S'),
}

def score_gate(baseline, trial):
    requirements={
     'complete_185s_1x_replay':trial['poses']>=1550 and trial['elapsed_s']>=184.9,
     'outage_accuracy_better_than_best':trial['outage_proxy_rmse_m']<baseline['outage_proxy_rmse_m'],
     'recovery_accuracy_better_than_best':trial['recovery_proxy_rmse_m']<baseline['recovery_proxy_rmse_m'],
     'continuity_better_than_best':trial['max_published_pose_step_m']<baseline['max_published_pose_step_m'],
     'outage_target_0p5m':trial['outage_proxy_rmse_m']<=0.5,
     'no_large_jump_0p85m':trial['max_published_pose_step_m']<=0.85,
     'no_extended_missing_pose_stream':trial['max_pose_stream_gap_s']<=0.30,
    }
    return requirements

def main():
    results={}
    common_old=None
    for label,(scorefile,folder) in EVIDENCE.items():
        p=ROOT/scorefile;directory=ROOT/folder
        d=json.loads(p.read_text())
        assert d['independent_ground_truth'] is False
        assert d['development_only'] is True
        assert not d['strict_quality_gate_passed'], 'Unexpected comparator pass; check manually'
        assert (directory/'global_pose.csv').stat().st_size >100_000
        assert (directory/'plots/07_orange_trajectory_on_pcd_clean.png').is_file()
        assert (directory/'metrics_exact.txt').is_file()
        baseline=d['old'];new=d['new']
        if common_old is None:common_old=baseline
        else:
            for field in ('outage_proxy_rmse_m','recovery_proxy_rmse_m','max_published_pose_step_m'):
                assert abs(common_old[field]-baseline[field])<0.001
        steps=score_gate(baseline,new)
        log=(directory/'launch.log').read_text(errors='replace')
        assert 'FULL_LIVO -> VI_LIDAR_FAILED' in log
        cross={'accepted':log.count('[DEPTH CROSSMAP] accepted'),
               'rejected':log.count('[DEPTH CROSSMAP] reject')}
        if label.startswith('two_map_'):
            assert cross['accepted']>=5
            assert '/prior_map.pcd' in log and '/visual_map.pcd' in log
        results[label]={
          'source_run':folder,
          'metrics':new,
          'geometry_crosscheck_counts':cross,
          'requirements':steps,
          'eligible_for_promotion':False,  # other-bag, full-run and hardware validation still absent
        }
        assert not all(steps.values()), 'Candidate unexpectedly passes: test full-run and holdout'
    manifest={
      'date':'2026-10-16',
      'ground_truth':'loopclosed same-bag proxy; NOT independent GT',
      'reference_used_only_after_replay':True,
      'bag':'camera_lidar_imu_gps.bag',
      'rate':'1x',
      'window_s':[0,185],
      'lidar_failed_s':[125,165],
      'baseline_curated':str(BASE.relative_to(ROOT)),
      'baseline_metrics':common_old,
      'results':results,
      'all_trials_rejected':True,
      'production_unchanged':True,
      'deployment_approved':False,
      'stop_condition':'Only promote after full 290s + independent Sep18 holdout, sub-0.5m failure accuracy, smoothness gate and live planner/DBW fail-closed test.',
    }
    audit=ROOT/'reports/LIDAR_FAILURE_REGISTRATION_TRIALS_AUDIT_20261016.json'
    audit.write_text(json.dumps(manifest,indent=2)+'\n')
    lines=[
      '# LiDAR-failure localisation experiments — 16 October 2026','',
      '**Outcome: all trial implementations REJECTED. Existing best retained. No deployment.**','',
      'All four trials used the same main rosbag at **1×**, with the same 40s LiDAR outage',
      '(125–165s). The comparison window is [0,185]s for every variant.',
      'Every RMSE is disagreement with the same-bag loop-closed trajectory **proxy**.',
      'It is **not independent ground truth** and cannot establish field accuracy.','',
      '| Trial | Outage RMSE | Recovery RMSE | Max pose step | Decision |',
      '| --- | ---: | ---: | ---: | --- |',
      f'| Frozen curated Oct 13 best | **{common_old["outage_proxy_rmse_m"]:.3f} m** | **{common_old["recovery_proxy_rmse_m"]:.3f} m** | {common_old["max_published_pose_step_m"]:.3f} m | Still NOT deployment-safe |',
    ]
    aliases={
       'local_depth_tracker_2p2s':'Enable local tracker for 2.2s',
       'rgbd_saved_keyframe_pnp':'ORB + RGB-D keyframe PnP',
       'two_map_xyz_blend':'Visual voxel + LiDAR PCD XYZ blend',
       'two_map_veto_only':'Visual voxel + LiDAR PCD veto only',
    }
    for key,item in results.items():
        v=item['metrics'];lines.append(f'| {aliases[key]} | {v["outage_proxy_rmse_m"]:.3f} m | {v["recovery_proxy_rmse_m"]:.3f} m | {v["max_published_pose_step_m"]:.3f} m | **REJECT** |')
    lines += [
       '', '## Concrete findings', '',
       '1. The old map-localizer config had `depth_tracker_max_age_sec: 0.0`,',
       '   disabling its bounded local RGB-D saved-map tracker. Enabling a',
       '   2.2-second window yielded some valid-looking local matches but',
       '   suppressed independent global reacquisition and drifted badly.',
       '2. Enabling opt-in depth-backed ORB/PnP produced hundreds of geometric',
       '   image inliers but worse pose accuracy and recovery under this CPU load.',
       '3. In the frozen Step-3 launch, both `map_path` and `visual_map_path`',
       '   pointed to `/visual_map.pcd`. In the depth-failover-only frontend,',
       '   the geometric `map_` was not loaded at all. A genuinely independent',
       '   crossmap test required BOTH an isolated launch correction AND opt-in',
       '   PCD loading in the frontend. These are fixed ONLY in dev artifacts.',
       '4. Independent canonical saved-LiDAR-PCD GICP confirmed 29 visual-map',
       '   fixes and vetoed 5 geometry disagreements in the dual-map blend',
       '   trial, but injecting 75% of its XYZ made positional error worse.',
       f'5. The final veto-only crosscheck retained {results["two_map_veto_only"]["geometry_crosscheck_counts"]["accepted"]} matches, rejected {results["two_map_veto_only"]["geometry_crosscheck_counts"]["rejected"]},',
       '   and improved post-recovery pose/step continuity, but had **worse',
       '   outage RMSE**. It must not be promoted.',
       '6. The earlier timestamped independent depth registration audit',
       '   showed 34 accepted raw saved-map fixes with **2.014m** 3D RMSE',
       '   during outage; median transport delay only **0.084s**. Repeated',
       '   road/map geometry, inaccurate absolute matches, and insufficient',
       '   map-geometry observability are now the limiting factors—not',
       '   primarily a missing publication smoother or processing delay.','',
       '## Files and reproducibility','',
       '- JSON audit: `reports/LIDAR_FAILURE_REGISTRATION_TRIALS_AUDIT_20261016.json`',
       '- Baseline: `results/BEST_SO_FAR_20261015/camera_lidar_imu_gps/LIDAR_FAILED/`',
       '- Local trial: `step3_global/depth_temporal_tracker_trial_20261016/`',
       '- Feature-PnP trial: `step3_global/depth_feature_pnp_trial_20261016/`',
       '- Dual-map blend: `step3_global/depth_crossmap_trial_20261016/`',
       '- Veto-only crossmap: `step3_global/depth_crossmap_veto_trial_20261016/`',
       '- Isolated launch: `launch/fast_livo2_step3_global_dual_map_dev_20261016.launch`',
       '- Isolated image: `fast_livo2_failsafe:depth-crossmap-veto-only-dev-20261016`',
       '- Builders: `scripts/build_isolated_depth_crossmap_20261016.py` and',
       '  `scripts/build_depth_crossmap_veto_only_20261016.py`',
       '- Saved PCD maps and camera/IMU original calibration were not fitted',
       '  to the reference; no production launch/config/source was changed.',
       '', '## Next research/engineering milestone', '',
       'Do not accept a map match from just GICP fitness, overlap, or PnP',
       'inlier count; they are insufficient on repeated-road geometry.',
       'Develop a confidence-calibrated, well-conditioned relative-to-absolute',
       'RGB-D/IMU registration using distinctive, STATIC multi-frame landmarks',
       'and independent 3D correspondence verification, plus explicit',
       'observability diagnostics of weak along-road direction. Evaluate on',
       'a separate mapping session, not the held-out Sep18 bag for fitting.',
       'Before any cart run, connect a sustained untrusted localisation state',
       'to a bench-tested planner/DBW fail-closed stop. The current offline',
       'success criteria are not a tested safety control.','',
       '**None of this establishes a sub-0.5m result, stable four-mode sensor',
       'failure operation, or NUC real-time capability.**',
    ]
    (ROOT/'reports/LIDAR_FAILURE_REGISTRATION_TRIALS_20261016.md').write_text('\n'.join(lines)+'\n')
    for key,item in results.items():
        v=item['metrics'];print(f'{key:33s} outage={v["outage_proxy_rmse_m"]:.3f}m recovery={v["recovery_proxy_rmse_m"]:.3f}m step={v["max_published_pose_step_m"]:.3f}m REJECTED')
    print('Audit files saved, all four trial folders verified, no changes to frozen eight-best artifacts.')

if __name__=='__main__':main()
