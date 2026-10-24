#!/usr/bin/env python3
"""Offline evidence gates for the XFeat/Step3 integration; NOT a vehicle release."""
import json
from pathlib import Path
R=Path(__file__).resolve().parents[1]
O=R/'outputs/learned_visual_20261020'
all=json.load(open(O/'ROS1_INTEGRATION_METRICS.json'))
base=all['MH + visual seed, 1Hz global reacquisition (unsafe publish, before guard)']
fixed=all['MH + visual seed, 1Hz reacquisition (handoff guard)']
fault=all['MH + visual seed, LiDAR+odom outage (fail-closed retest)']
stables=[all['MH + visual seed, stable from2'],all['MH + visual seed, stable localization']]
bench=json.load(open(O/'LEARNED_TEMPORAL_COMPARISON.json'))
conditions={
 'XFeat_offline_third_bag_at_least_25_of_32':bench['xfeat_mnn_learned_retrieval']['localization_3']['PnP_passed']>=25,
 'stable_heldouts_remained_lidar_globally_locked':all(r['lidar_final_status']=='GLOBAL_LOCKED' for r in stables),
 'both_stable_heldouts_no_unexpected_pose_publication':all(r['forbidden_pose_messages']==0 for r in stables),
 'GPS_disabled_in_all_measured_replays':all(r['gps_flag']=='false' for r in all.values()),
 'visual_guided_GICP_located_alt_branch':fixed['visual_seed_near_late_294'] or fixed['visual_gicp_pass']>=10,
 'long_gap_recovery_does_NOT_resume_map_pose':fixed['lidar_final_status']=='RELOCALIZING: MH_HANDOFF_REQUIRED' and fixed['lidar_last_published_bag_sec']<31.0,
 'long_gap_recovery_has_no_large_published_step':fixed['max_published_map_pose_step_m']<1.0,
 'previous_11m_publication_regression_reproduced':base['max_published_map_pose_step_m']>10.0,
 'new_guard_prevents_previous_large_published_step':base['max_published_map_pose_step_m']>10.0 and fixed['max_published_map_pose_step_m']<1.0,
 'LiDAR_and_odom_dropout_caused_quarantine':fault['fast_odom_gap_quarantine'],
 'LiDAR_dropout_does_not_restart_old_pose_publishing':fault['lidar_poses_after_39s_resume']==0,
 'camera_candidates_survive_LiDAR_dropout':fault['visual_candidate_messages_during_26_to_39s_dropout']>0,
 'no_unexpected_legacy_pose_messages_during_any_run':all(r['forbidden_pose_messages']==0 for r in all.values()),
}
out={'offline_diagnostic_checks':conditions,'passes':sum(conditions.values()),'total':len(conditions),'all_pass':all(conditions.values()),
'vehicle_deployment_approved':False,'deployment_blockers':['Physical LiDAR–IMU/base extrinsic unverified','Map is unsurveyed and not loop optimized','True multi-hypothesis temporal arbitration and controlled handoff after long outage not implemented','ROS1 bridge/camera/LiDAR estimation not profiled together on Orin/NUC','Real autonomous sensor-failure and safety-stop evaluation not completed']}
(O/'ACCEPTANCE_GATES.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
if not out['all_pass']:raise SystemExit(1)
