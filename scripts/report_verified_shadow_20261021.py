#!/usr/bin/env python3
"""Aggregate ACTUAL ROS1 shadow monitoring artifacts, never invented values."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/deployment_audit_20261021'
RUNS={
  'normal_36s':'shadow_remapped_xfeat_from2_36s_20261021',
  'lidar_blackout_40s':'shadow_final_isolated_fault_from2_40s_20261021',
  'difficult_45s':'shadow_remapped_local3_xfeat_45s_20261021',
}

def load(p):return json.loads(p.read_text())

def summary(run):
    folder=OUT/run
    if not (folder/'status.txt').is_file():raise ValueError('Missing status for '+run)
    status=(folder/'status.txt').read_text()
    if not all(marker in status for marker in ('PLAY_EXIT=0','FINISHED')):
        raise RuntimeError('Incomplete run '+run+':'+status)
    report=load(folder/'monitor.json')
    gate=load(folder/'runtime_postconditions.json')
    if not gate.get('passed'):raise RuntimeError('Runtime gate failed '+run)
    manifest=dict(line.split('=',1) for line in
                  (folder/'experiment_manifest.env').read_text().splitlines() if '=' in line)
    return {
      'bag':manifest['BAG']+'.bag',
      'duration_requested_s':int(manifest['DURATION']),
      'raw_lidar_fault_injected':manifest.get('INJECT_OFFLINE_LIDAR_FAULT')=='true',
      'fault_start_s':manifest.get('FAULT_START') or None,
      'fault_end_s':manifest.get('FAULT_END') or None,
      'xfeat_worker_enabled':manifest.get('VISUAL')=='true',
      'FAST_local_odom_messages':report['received'].get('fast_odom',0),
      'KLT_local_candidate_messages':report['received'].get('klt_odom',0),
      'KLT_continuation_messages':report['received'].get('klt_continuation',0),
      'XFeat_global_candidate_messages':report['received'].get('xfeat_global',0),
      'LiDAR_GICP_map_pose_messages':report['received'].get('step3_map',0),
      'fused_shadow_pose_messages':report['received'].get('fused_shadow',0),
      'Step3_final_status':report['step3_status_sequence'][-1],
      'health_transitions':[item['mode'] for item in report['health_transitions']],
      'forbidden_topic_counts':{k:v for k,v in gate['counts'].items() if k.startswith(('forbidden','unexpected'))},
      'runtime_gate_passed':gate['passed'],
    }


def main():
    results={label:summary(run) for label,run in RUNS.items()}
    fault=OUT/RUNS['lidar_blackout_40s']
    drift=load(fault/'posthoc_lio_reference_metrics.json')
    audit=load(OUT/'HARD_CODE_BAG_LEAK_AUDIT_20261021.json')
    document={
      'version':'2026-10-21_12h03_IST_sanitized_non_authoritative',
      'readiness':'OFFLINE_INTEGRATED_SHADOW_ONLY',
      'motion_authorized':False,
      'automatic_odom_switching_authorized':False,
      'map_reset_authorized':False,
      'real_hardware_braking_connected':False,
      'map_surveyed_ground_truth':False,
      'physical_extrinsics_verified':False,
      'raw_lidar_only_reference_is_ground_truth':False,
      'step3_feedback_into_FAST_state_disabled':True,
      'legacy_navigation_topics_quarantined':True,
      'hardcode_audit_assertions':audit['critical_checks'],
      'bag_specific_precomputed_odometry_used_for_estimation':False,
      'gps_search_prior_used':False,
      'forced_target_pose_used':False,
      'real_Orin_NUC_compute_profile_obtained':False,
      'runs':results,
      'lidar_blackout_relative_disagreement_with_separate_uninterrupted_lio_odometry':{
        k:drift[k] for k in (
        'paired','duration_s','relative_position_RMSE_m','relative_position_endpoint_m',
        'orientation_divergence_endpoint_deg','RPE_1s_translation_median_m',
        'RPE_5s_translation_median_m')},
      'required_before_live_shadow':[
        'Physically measured LiDAR-base, LiDAR-IMU, camera-body and camera-IMU extrinsics and time offset',
        'Driver point-time units and original sensor-clock timing verified on actual hardware',
        'Physical stop/motion inhibit system evaluated independently of ROS',
        'Concurrent Orin+NUC profiling and ROS transport latency measurement',
      ],
      'required_before_vehicle_motion':['Real hardware stop acknowledgments',
          'Independent surveyed/loop-closed map truth and blind false-positive testing',
          'Persisted supervised map/odom epoch handoff accepted by planner/controller/TF consumers',
          'Controller validation for dropout, pose jumps, stale maps and process crashes'],
      'warning':'Validating message flow and pose continuity is NOT proof of absolute position accuracy or safe autonomous navigation.'
    }
    (OUT/'REVALIDATED_GPSFREE_DEPLOYMENT_20261021.json').write_text(json.dumps(document,indent=2)+'\n')
    lines=['# Revalidated GPS-free shadow stack (October 21, 2026)','',
           '**OFFLINE SHADOW READY — NOT CLEARED FOR VEHICLE MOTION.**','',
           '## Latest ROS1 integration replays after FAST internal-feedback disable and namespace isolation','',
           '| Recording | FAST local odom | KLT local | XFeat global | LiDAR map poses | KLT continuation | Final global state | Forbidden topics |',
           '|---|---:|---:|---:|---:|---:|---|---:|']
    for label,r in results.items():
        forbidden=sum(r['forbidden_topic_counts'].values())
        lines.append(f"| {r['bag']} ({r['duration_requested_s']}s{' LiDAR blackout' if r['raw_lidar_fault_injected'] else ''}) | {r['FAST_local_odom_messages']} | {r['KLT_local_candidate_messages']} | {r['XFeat_global_candidate_messages']} | {r['LiDAR_GICP_map_pose_messages']} | {r['KLT_continuation_messages']} | {r['Step3_final_status']} | **{forbidden}** |")
    lines+=['','## KLT continuation: offline LiDAR [20,29]s fault, separate reference',
            '',
            f"- {drift['paired']} candidates over {drift['duration_s']:.2f}s; relative position RMSE {drift['relative_position_RMSE_m']:.3f}m; endpoint disagreement {drift['relative_position_endpoint_m']:.3f}m.",
            f"- Relative 1s translation RPE median {drift['RPE_1s_translation_median_m']:.3f}m; 5s RPE median {drift['RPE_5s_translation_median_m']:.3f}m.",
            '- The independent LiDAR-only reference is **not** surveyed ground truth and was only read AFTER the candidate trajectory was recorded. No LiDAR reference was injected into the RGB-D odometry estimator.',
            '', '## No forced bag output or hidden global-feedback path', '',
            '- No GPS soft prior, legacy ORB+GPS visual matcher, fake initial pose, pretrained local odometry injection, or yaw-trajectory fit in the new real-sensor replay.',
            '- Step3 publishes map candidates, but the FAST mapper correction flag and Step3 prior, tracker and bias feedback flags are **all disabled**. Verified with `roslaunch --dump-params` after the included YAML is loaded.',
            '- All internal map/odometry sources are namespaced; legacy `/aft_mapped_to_init`, `/tf`, `/path`, `/pose`, `/cmd_vel`, `/initialpose` receive **zero publications** from this ROS shadow launch. Runtime validator fails closed if that changes.',
            f"- Hardcode static assertions: **{sum(audit['critical_checks'].values())}/{len(audit['critical_checks'])}** pass. These do not prove absence of *unrecognized* hardcodes or causal calibration bias.",
            '- GICP/inlier/consistency thresholds still reflect development on known campus data; blind held-out multi-session false-positive testing and surveying are required before vehicle motion.',
            '', '## Files and instructions', '',
            '- `launch/deployment_gpsfree_shadow_20261021.launch`',
            '- `scripts/run_deployment_gpsfree_shadow_rosbag_20261021.sh` (requires explicit BAG and RUN)',
            '- `scripts/start_deployment_gpsfree_live_shadow_20261021.sh` (dry-run by default, rejects archival calibration)',
            '- `scripts/validate_deployment_replay_20261021.py` (online output and forbidden-topic evidence)',
            '- `scripts/audit_deployment_hardcodes_20261021.py` (source-based static evidence)',
            '- `docs/GPSFREE_SHADOW_STACK_20261021.md` (full architecture and operator guide)',
            '', '## Outstanding blockers for actual cart', '',
            'Physical transforms/clock alignment and live hardware profiling, independent brake and motion inhibit, surveyed map truth, robust global branch recovery and true supervised epoch-aware planner/controller/TF handoff. Current global-local confidence loss always stops/requests review, never authorizes switching.',
            '', '*Zero unexpected publications means no forbidden ROS topic received a message in this isolated experiment; it does not attest to other processes already running on the physical cart.*']
    (OUT/'REVALIDATED_GPSFREE_DEPLOYMENT_20261021.md').write_text('\n'.join(lines)+'\n')
    print('CREATED FINAL VERIFIED REPORT AND JSON, BAG COUNTS:')
    for label,x in results.items():
        print(label,x['bag'],x['FAST_local_odom_messages'],x['KLT_local_candidate_messages'],x['Step3_final_status'],x['runtime_gate_passed'])


if __name__=='__main__':main()
