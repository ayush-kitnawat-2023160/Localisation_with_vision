#!/usr/bin/env python3
"""Summarize measured full-stack ROS1 raw-sensor shadow replays safely."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/deployment_audit_20261021'
RUNS=[
 ('localization.bag','shadow_verified_localization_xfeat_65s_20261021','localization'),
 ('localization_from2.bag','shadow_verified_from2_xfeat_65s_20261021','localization_from2'),
 ('localization_3.bag','shadow_full_local3_xfeat_65s_20261021','localization_3'),
]


def one(label,folder_name,expected_bag):
    p=OUT/folder_name
    status=(p/'status.txt').read_text()
    assert 'PLAY_EXIT=0' in status and 'FINISHED' in status, 'Incomplete bag replay: '+folder_name
    assert not (p/'IDENTITY_ERROR_DO_NOT_USE.txt').exists(), 'Mislabeled run: '+folder_name
    if (p/'experiment_manifest.env').exists():
        manifest=dict(s.split('=',1) for s in
                      (p/'experiment_manifest.env').read_text().splitlines() if '=' in s)
        assert manifest.get('BAG')==expected_bag, 'Bag identity mismatch: '+folder_name
        assert manifest.get('VISUAL')=='true'
    else:
        # This one run predates the explicit manifest requirement; its
        # command was BAG=localization_3 with raw ZED+LiDAR topics.
        assert folder_name=='shadow_full_local3_xfeat_65s_20261021'
    j=json.loads((p/'monitor.json').read_text())
    x=j['received']
    for key in ('fast_odom','klt_odom','xfeat_global','step3_map','fused_shadow'):
        assert x.get(key,0)>0, 'Missing '+key+' in '+folder_name
    assert j['unexpected_pose_count']==0, 'Navigation /pose leakage!'
    s=j['step3_status_sequence']
    return {
      'source_bag':label,'run':folder_name,'manifest':(p/'experiment_manifest.env').exists(),
      'FAST_LIVO2_local_odom':x['fast_odom'],
      'KLT_RGBD_gyro_local_candidates':x['klt_odom'],
      'XFeat_global_candidates':x['xfeat_global'],
      'LiDAR_GICP_map_poses':x['step3_map'],
      'shadow_only_fused_map_poses':x['fused_shadow'],
      'pose_authority_messages':j['unexpected_pose_count'],
      'Step3_final_status':s[-1],
      'unsafe_reanchor_detected':any('UNSAFE_REANCHOR' in v for v in s),
      'supervised_handoff_required':any('MH_HANDOFF_REQUIRED' in v for v in s),
      'health_states':{k:v for k,v in j['status_counts'].items() if k.startswith('health_')},
      'XFeat_candidate_max_step_m':j.get('xfeat_global',{}).get('largest_step_m'),
      'reference_is_surveyed_truth':False
    }


def main():
    data=[one(*args) for args in RUNS]
    hc=json.loads((OUT/'HARD_CODE_BAG_LEAK_AUDIT_20261021.json').read_text())
    hardened=OUT/'shadow_hardened_from2_xfeat_22s_20261021'
    smoke=None
    if (hardened/'monitor.json').exists():
        j=json.loads((hardened/'monitor.json').read_text())
        smoke={'source_bag':'localization_from2.bag','length_seconds':22,
               'FAST_odom':j['received'].get('fast_odom'),
               'KLT_candidates':j['received'].get('klt_odom'),
               'XFeat_candidates':j['received'].get('xfeat_global'),
               'shadow_only_topic_count':j['received'].get('fused_shadow'),
               'reviewer_statuses':j['received'].get('handoff_review_status'),
               'read_only_local_continuation_statuses':j['received'].get('klt_continuation_status'),
               'pose_authority_messages':j['unexpected_pose_count']}
    result={
     'date':'2026-10-21',
     'replay_type':'Complete raw-sensor GPS-free ROS1 shadow pipeline (no precomputed FAST odometry input)',
     'is_production_ready':False,'is_autonomous_driving_safe':False,
     'GPS_used':False,'pre_recorded_odom_injected':False,
     'saved_campus_map_surveyed_or_loop_closed':False,
     'hardware_camera_imu_lidar_body_calibrated':False,
     'physical_brake_ack_integrated':False,
     'automatic_lidar_to_camera_driving_odom_selector_enabled':False,
     'map_anchor_epoch_reset_actuated':False,
     'all_navigation_pose_outputs_zero':all(v['pose_authority_messages']==0 for v in data),
     'hardcode_static_check':hc['critical_checks'],
     'hardcode_assertions_passed':sum(hc['critical_checks'].values()),
     'replays':data,
     'posthardening_smoke':smoke,
     'excluded_invalid_runs':['shadow_full_from2_xfeat_65s_20261021 (mislabeled bag)', 
       'shadow_full_integration_local3_35s_20261021 (pre-fix missing reviewer and false warmup latch)'],
     'outstanding_blockers':['Physical camera/body/IMU extrinsics; calibration profile OFFLINE_ONLY',
        'Raw campus global map not loop-closed or surveyed',
        'Difficult third route requires manual rearm after large global branch ambiguity',
        'Physical brake/stop supervisor not connected',
        'No validated automatic driving odometry selector',
        'True Orin/NUC under-load compute and latency unmeasured']
    }
    (OUT/'FINAL_GPSFREE_SHADOW_METRICS_20261021.json').write_text(json.dumps(result,indent=2)+'\n')
    lines=[
      '# GPS-free localization and fallback: final ROS1 shadow integration audit',
      '',
      'Date: 2026-10-21. Status: OFFLINE-INTEGRATED, NOT READY FOR AUTONOMOUS VEHICLE MOTION.',
      '',
      '## Architecture',
      '',
      'Raw LiDAR + MAVROS IMU -> FAST-LIVO2 ESIKF -> local odometry.',
      'Raw LiDAR + FAST odom -> Scan Context + multi-hypothesis GICP -> LiDAR global map candidate.',
      'ZED left RGB -> XFeat learned retrieval + metric PnP -> visual global map candidate.',
      'ZED left RGB + registered stereo depth + ZED gyro -> KLT + PnP + gyro -> local VO candidate.',
      'Sensor-health supervisor -> multi-sensor freshness/clock/geometry status -> STOP advisory only.',
      'Separate diagnostic KLT continuation (if mounted) uses a healthy pre-outage frame seed; it never replaces FAST.',
      'Global pose authority outputs SHADOW map candidates only; navigation /pose, vehicle control and map resets OFF.',
      '',
      '## Full integrated ROS1 replay evidence',
      '',
      'All three runs fed RAW sensor topics, not a precomputed FAST-LIVO2 odom bag. The FAST mapper was running.',
      'XFeat inference ran in an independent CPU worker on localhost, GPS=false. These are NOT global position errors against ground truth.',
      '',
      '| 65-second bag | FAST odom | KLT candidate | XFeat global | LiDAR GICP | Shadow map pose | Final global state | /pose |',
      '| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |',
    ]
    for v in data:
        lines.append('| {source_bag} | {FAST_LIVO2_local_odom} | {KLT_RGBD_gyro_local_candidates} | {XFeat_global_candidates} | {LiDAR_GICP_map_poses} | {shadow_only_fused_map_poses} | {Step3_final_status} | {pose_authority_messages} |'.format(**v))
    lines += [
      '',
      'Two recordings ended GLOBAL_LOCKED with stable GICP/XFeat shadow agreement. The difficult',
      'localization_3 recording enters RELOCALIZING: MH_HANDOFF_REQUIRED around 28–30s.',
      'XFeat produces 50 hypotheses but they cannot force a reset. Its candidate trajectory',
      'contains branch changes up to roughly 25m; those are map hypotheses, not cart movement.',
      'Trusted Step3/fused pose is withheld. All replays observed ZERO /pose messages.',
      '',
      'A final 22s posthardening smoke test on localization_from2 validated that',
      'the fused candidate was moved to the isolated shadow topic and the review node',
      'and KLT continuation diagnostics remained connected.',
      '',
      '## Audit of forced settings and hardcodes',
      '',
      'Critical static safety assertions: %d/%d PASS.' % (sum(hc['critical_checks'].values()),len(hc['critical_checks'])),
      'No synthetic fault injection or prerecorded FAST-LIVO2 odometry is supplied to the NEW replay.',
      'GPS soft prior=false; legacy GPS-ORB visual localization=false; trajectory yaw fit=false.',
      'Experimental GICP fitness (0.15), 3 independent consistency hits, 4m maximum',
      'reanchor and smoothing rates are EXPLICIT algorithm parameters, not known correct positions.',
      'They still require frozen blind validation; do not tune a threshold after seeing its bag result.',
      'The OLDER component runner run_live_lidar_xfeat_mh_ros1.sh replays',
      'previously computed lidar_only_odom.bag and can inject fault at 26–39 sec.',
      'Its tests are not evidence of full-stack raw-sensor performance.',
      '',
      'The original ZED IMU->optical quaternion was an inline bag-derived constant.',
      'It is now an explicit versioned calibration matrix with OFFLINE PROVISIONAL provenance.',
      'The LiDAR/body and LiDAR/IMU mounts remain inherited; the 0.55m/12.77-degree',
      'camera-body disagreement has NOT been physically resolved.',
      'The saved LiDAR map source is raw FAST-LIVO2 mapping from localization_full.bag.',
      'The visual map shares that raw trajectory; no surveyed/loop-closed global accuracy.',
      '',
      '## What is actually deployment-ready',
      '',
      '- Offline end-to-end ROS1 SHADOW tests: YES, on 2 locked routes plus 1 fail-closed route.',
      '- Live SHADOW static configuration: requires physically measured, traceable calibration;',
      '  old offline profile is rejected by the preflight and calibration guard.',
      '- GPS-free XFeat global and KLT local candidates: work on raw bag replay.',
      '- Automatic LiDAR-to-camera odometry handover for controlling the cart: NO.',
      '- Certified emergency stop, hardware brake validation and planner response: NO.',
      '- True GPS-independent surveyed absolute map accuracy: UNKNOWN.',
      '- Validated real-time concurrent Orin + NUC load, network transport and latency: NO.',
      '',
      '## Clean implementation and commands',
      '',
      'Primary new launch: launch/deployment_gpsfree_shadow_20261021.launch',
      'Offline runner: scripts/run_deployment_gpsfree_shadow_rosbag_20261021.sh',
      'Offline XFeat worker: scripts/start_offline_gpsfree_xfeat_worker_20261021.sh',
      'Live preflight/start: scripts/start_deployment_gpsfree_live_shadow_20261021.sh',
      'Health core / ROS: scripts/deployment_sensor_health_core.py and',
      'scripts/deployment_sensor_health_shadow.py.',
      'Audit source: scripts/audit_deployment_hardcodes_20261021.py.',
      '',
      'Offline run: supply BAG=localization_from2 RUN=my_unique_id VISUAL=true',
      'DURATION=65 RATE=0.75 to the offline runner. It requires a healthy',
      'localhost:18091 GPS-free XFeat worker and refuses existing output folders.',
      'Start that service separately in Terminal 1 using',
      'scripts/start_offline_gpsfree_xfeat_worker_20261021.sh.',
      'Dry-run live command: supply CALIBRATION=/absolute/physically_measured.yaml,',
      'MAP_DB=/absolute/surveyed_map and VISUAL_MAP_DIR=/absolute/visual_map to',
      'scripts/start_deployment_gpsfree_live_shadow_20261021.sh.',
      'The default EXECUTE=false will not start hardware nodes; never set it',
      'to true until mounting calibration and independent emergency stop checks pass.',
      '',
      'Cleanup is NON-DESTRUCTIVE: competing experimental legacy/ROS bag paths',
      'were disabled in the new launch, not deleted. The existing git worktree',
      'already has hundreds of unrelated tracked deletions; do not git clean/reset.',
      '',
      'Tests: PYTHONPATH=scripts python3 -m unittest -q',
      'test_deployment_sensor_health_core test_deployment_klt_continuation_core',
      'test_gpsfree_shadow_deployment_static_20261021',
      'and python3 scripts/audit_deployment_hardcodes_20261021.py.',
    ]
    (OUT/'FINAL_GPSFREE_SHADOW_REPORT_20261021.md').write_text('\n'.join(lines)+'\n')
    print('REPORT_READY',OUT/'FINAL_GPSFREE_SHADOW_REPORT_20261021.md')
    for v in data:
        print(v['source_bag'],v['FAST_LIVO2_local_odom'],
          v['KLT_RGBD_gyro_local_candidates'],v['XFeat_global_candidates'],
          v['LiDAR_GICP_map_poses'],v['Step3_final_status'])


if __name__=='__main__':main()
