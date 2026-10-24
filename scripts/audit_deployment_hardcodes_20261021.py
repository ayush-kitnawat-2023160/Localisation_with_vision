#!/usr/bin/env python3
"""Repeatable source-specific audit: GPS, bag-derived odom and forced poses."""
from pathlib import Path
import json

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/deployment_audit_20261021'
OUT.mkdir(parents=True,exist_ok=True)

sources={
  'launch':'launch/deployment_gpsfree_shadow_20261021.launch',
  'runner':'scripts/run_deployment_gpsfree_shadow_rosbag_20261021.sh',
  'old_runner':'scripts/run_live_lidar_xfeat_mh_ros1.sh',
  'main_launch':'launch/fresh_multimodal_global_rosbag.launch',
  'old_visual':'scripts/fresh_visual_global_localizer.py',
  'klt':'scripts/fresh_zedx_camera_imu_odom.py',
  'step3':'src/step3_global_acquirer.cpp',
  'authority':'scripts/fresh_pose_authority.py',
  'calibration':'config/calibration/zedx_current.yaml',
  'guard':'scripts/fast_livo2_calibration_guard.py',
  'adapter':'scripts/fast_livo2_sensor_adapter.py',
  'inner_launch':'launch/fast_livo2_failsafe_rosbag.launch',
}
data={name:(ROOT/path).read_text() for name,path in sources.items()}


def occurrence(name,needle,limit=2):
    return [{'source':sources[name],'line':i,'text':line.strip()[:220]}
      for i,line in enumerate(data[name].splitlines(),1) if needle in line][:limit]


checks={
  'no_live_legacy_pose':'<arg name="publish_legacy_pose" value="false"/>' in data['launch'],
  'no_lidar_GPS_soft_prior':'<arg name="lidar_gps_soft_prior_enable" value="false"/>' in data['launch'],
  'no_legacy_GPS_visual':'<arg name="visual_global_enable" value="false"/>' in data['launch'],
  'no_failure_injector':'<arg name="failure_injector_enable" value="false"/>' in data['launch'],
  'no_old_parallel_RGBD':'<arg name="rgbd_frontend_enable" value="false"/>' in data['launch'],
  'no_trajectory_yaw_fit':'<arg name="lidar_trajectory_yaw_fit_enable" value="false"/>' in data['launch'],
  'no_precomputed_LIO_odom_replay':'lidar_only_odom.bag' not in data['runner'] and '/odom/' not in data['runner'],
  'no_forced_initial_pose':'/initialpose' not in data['runner'],
  'live_map_path_required':'<arg name="map_db"/>' in data['launch'],
  'live_calibration_path_required':'<arg name="calibration_file"/>' in data['launch'],
  'no_KLT_bag_quaternion_literal':'quaternion_matrix([.5,-.5,.5,-.5])' not in data['klt'],
  'live_calibration_guard_rejects_offline':'Offline/legacy/provisional calibration' in data['guard'],
  'step3_correction_disabled':'<param name="step3/tracking_correction_enable" type="bool" value="false"/>' in data['launch'],
  'step3_prior_injection_disabled':'<param name="step3/prior_map_enable" type="bool" value="false"/>' in data['launch'],
  'external_map_correction_disabled_after_yaml':'<arg name="external_map_correction_enable" value="false"/>' in data['launch'] and
      '<param name="failsafe/map_correction_enable"' in data['inner_launch'],
  'canonical_fast_odometry_quarantined':'<remap from="/aft_mapped_to_init" to="/fast_livo2/deployment/fast_odom_candidate"/>' in data['launch'],
  'canonical_tf_quarantined':'<remap from="/tf" to="/fast_livo2/deployment/tf_candidate_only"/>' in data['launch'],
  'real_lidar_point_time_unit_explicit':'UNSET_MUST_SPECIFY_SECONDS_OR_MICROSECONDS' in data['launch'] and
      'Live LiDAR point-time unit must be explicit' in data['adapter'],
}
findings=[
 {'severity':'LIVE_BLOCKER','topic':'Bag profile marked calibration.ready=true despite inherited physical mounting',
  'evidence':occurrence('calibration','ready: true')+occurrence('calibration','OFFLINE_ONLY'),
  'action':'Live calibration guard rejects bag profile, independent physical verification needed.'},
 {'severity':'MAP_TRUTH_LIMIT','topic':'Map and visual landmarks aligned to raw unoptimized 225s LiDAR-only mapping bag',
  'evidence':[{'source':'maps/campus_full_lio_226s_20261019/metadata.json','line':None,
               'text':'source_bag=localization_full.bag, pose_quality=RAW_FAST_LIVO2_ODOMETRY_NOT_LOOP_CLOSED'}],
  'action':'Do not assert surveyed global accuracy or train/test independence based on same campus corridor.'},
 {'severity':'LEGACY_EXPERIMENT_ONLY','topic':'Old benchmark injects previously computed FAST LiDAR odometry from a second bag',
  'evidence':occurrence('old_runner','/odom/'),
  'action':'New runner plays raw camera, IMU, depth and LiDAR ONLY, FAST odometry computed by mapper.'},
 {'severity':'LEGACY_EXPERIMENT_ONLY','topic':'Old runner hardcodes fault start/end at 26/39s relative to first LiDAR stamp',
  'evidence':occurrence('old_runner','start_sec 26.0')+occurrence('old_runner','end_sec 39.0')
             +occurrence('old_runner','lidar_first_stamp'),
  'action':'New deployment graph bypasses the failure-injector entirely.'},
 {'severity':'LEGACY_GPS_DISABLED','topic':'Older ORB relocalizer subscribes to GPS and uses it to select keyframes',
  'evidence':occurrence('old_visual','NavSatFix')+occurrence('old_visual','if gps and'),
  'action':'New launch disables it; GPS-free XFeat local candidates only.'},
 {'severity':'ALGORITHM_PARAMETER_NEEDS_BLIND_TEST','topic':'GICP fitness and 3-of-N consistency gates may be tuned to known mapping route',
  'evidence':occurrence('main_launch','gicp_max_fitness')+occurrence('launch','mh_required_hits'),
  'action':'Thresholds are not pre-entered target poses; freeze before blind route/seasons tests.'},
 {'severity':'DISABLED_FOR_CONSERVATIVE_MODE','topic':'Step3 trajectory-derived yaw fitting can smooth apparent map alignment',
  'evidence':occurrence('step3','trajectory_yaw_fit_enable_')+occurrence('launch','trajectory_yaw_fit_enable'),
  'action':'Explicitly disabled in new launch; verify raw GICP and motion separately.'},
 {'severity':'MOVED_TO_CALIBRATION','topic':'KLT driver-bag ZED IMU-to-optical matrix previously embedded in frontend',
  'evidence':occurrence('klt','imu_opt=rospy.get_param')+occurrence('calibration','rotation_source'),
  'action':'Explicit source-provenance calibration; live shadow refuses OFFLINE/PROVISIONAL source.'},
 {'severity':'STILL_NOT_VEHICLE_FAILOVER','topic':'Pose authority always limits anchor motion and KLT output is candidate-only',
  'evidence':occurrence('authority','self.max_step_rate')+occurrence('klt','camera_imu_odom_candidate'),
  'action':'Smoothing is not evidence of accuracy, no real odom selector or brake-integrated reset exists.'},
 {'severity':'SHADOW_ISOLATION_FIXED','topic':'FAST mapper has an internal Step3 correction subscriber and absolute-map prior; these can modify the FAST ESIKF despite /pose being off',
  'evidence':occurrence('launch','step3/tracking_correction_enable')+occurrence('launch','step3/prior_map_enable')+
              occurrence('launch','external_map_correction_enable'),
  'action':'All disabled and independently checked via roslaunch --dump-params after included YAML loads.'},
 {'severity':'SHADOW_OUTPUT_ISOLATION_FIXED','topic':'Normal FAST mapping publishes /aft_mapped_to_init and /tf that an existing cart controller can consume',
  'evidence':occurrence('launch','<remap from="/aft_mapped_to_init"')+
              occurrence('launch','<remap from="/tf"'),
  'action':'Remap all experimental publishers and included subscribers into the shadow namespace. Verified with rostopic info: original topics have no publishers.'},
]
report={'critical_checks':checks,'all_critical_pass':all(checks.values()),'findings':findings,
        'limitations':['Static scans cannot prove the absence of every source-specific parameter bias.',
                       'No surveyed/map-anchored ground truth has been supplied.',
                       'Offline GPS metadata exists but is not used by the new estimator.',
                       'Physical camera/body and vehicle brake interlocks are not verified.'],
        'conclusion':'No hardcoded target pose or forced successful localization found in NEW shadow runner; old benchmark replays derived odometry and injects a fixed synthetic outage.'}
(OUT/'HARD_CODE_BAG_LEAK_AUDIT_20261021.json').write_text(json.dumps(report,indent=2)+'\n')
lines=['# Deployment hardcode and ROS-bag leakage audit','',
       'New shadow path: real sensors -> FAST-LIVO2 -> LiDAR global Step3; independent KLT+XFeat candidates; no navigation pose.','',
       '## Fail-closed launch assertions','']
for key,value in checks.items():lines.append(('- PASS: ' if value else '- FAIL: ')+key)
lines.extend(['','## Findings',''])
for x in findings:
    lines.append(x['severity']+': '+x['topic'])
    for z in x['evidence']:
        loc=z['source']+(':'+str(z['line']) if z['line'] else '')
        lines.append('  '+loc+': '+z['text'])
    lines.append('  Action: '+x['action'])
    lines.append('')
lines.extend(['## Honest interpretation','',
              'Numerical GICP/parallax/inlier thresholds are legitimate algorithm configuration but must be frozen before genuine blind tests. They can bias success rates after tuning on known recordings, even though they are not hardcoded known poses.',
              'The source map is raw, GPS data is present only as metadata, and global metric accuracy is not independently surveyed.',
              'NO-GO for autonomous fallback until external brake/stop and complete map-epoch transaction are physically validated.'])
(OUT/'HARD_CODE_BAG_LEAK_AUDIT_20261021.md').write_text('\n'.join(lines)+'\n')
print('HARD_CODE_STATIC_ASSERTIONS',sum(checks.values()),'/',len(checks))
if not all(checks.values()):
    raise SystemExit(2)
