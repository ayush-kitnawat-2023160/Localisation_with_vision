#!/usr/bin/env python3
"""Independent offline visual votes for two saved LiDAR map-pose hypotheses.
No ROS pose publication, no GPS, and NO surveyed ground truth is implied.
"""
import json,math
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parents[1]
O=R/'outputs/visual_global_20261020'
cal=yaml.safe_load(open(R/'config/calibration/zedx_current.yaml'))['extrin_calib']
def T(a,b):
 m=np.eye(4);m[:3,:3]=np.array(a).reshape(3,3);m[:3,3]=b;return m
T_cam_body=T(cal['Rcl'],cal['Pcl'])@np.linalg.inv(T(cal['extrinsic_R'],cal['extrinsic_T']))
held=json.load(open(O/'dense_1hz_top24/localization_3_visual_matches.json'))
stamps=json.load(open(R/'outputs/looooo_integrity_20261019/localization_3.json'))['sensor_timestamps'];cam_minus_lidar=stamps['camera']['first_stamp']-stamps['lidar']['first_stamp']
base=R/'outputs/localization3_alternate_cold_start_20261019';branches={}
for key,d in [('continuous_from_22s','start_22s_42s_gps_off'),('alternative_cold_at_54s','start_54s_10s_gps_off')]:
 branches[key]=np.array(json.load(open(base/d/'events.json'))['map_pose_trace'],float)
items=[]
for r in held:
 if not r.get('pnp_verified') or r['query_sec']<60:continue
 w=r['winner'];T_map_cam=np.eye(4);T_map_cam[:3,:3]=Rotation.from_quat(w['rotation_xyzw']).as_matrix();T_map_cam[:3,3]=w['mappos_m'];T_map_body=T_map_cam@T_cam_body
 time_in_lidar=float(r['query_sec']+cam_minus_lidar)
 result={'query_camera_time_sec':float(r['query_sec']),'query_lidar_time_sec':time_in_lidar,'visual_map_body_xyz_m':T_map_body[:3,3].tolist(),'visual_PnP_inliers':int(w['inliers']),'visual_reprojection_px':float(w['reproj_median_px']),'visual_map_keyframe':int(w['keyframe_id']),'candidate_branches':{}}
 for name,p in branches.items():
  if not p[0,0]<=time_in_lidar<=p[-1,0]:continue
  xyz=np.array([np.interp(time_in_lidar,p[:,0],p[:,k]) for k in (1,2,3)])
  result['candidate_branches'][name]={'map_body_xyz_m':xyz.tolist(),'xy_disagreement_m':float(np.linalg.norm(T_map_body[:2,3]-xyz[:2]))}
 items.append(result)
common=[r for r in items if len(r['candidate_branches'])==2]
chosen=None
if len(common)>=2:
 errors={name:float(np.median([r['candidate_branches'][name]['xy_disagreement_m'] for r in common])) for name in branches}
 s=sorted(errors,key=errors.get)
 if errors[s[0]]<2.0 and errors[s[1]]>8.0:chosen=s[0]
else:errors={}
report={'purpose':'Offline visual multi-frame check of competing LiDAR global-map branches on localization_3.bag','gps_used':False,'camera_imu_extrinsic_is_unverified':True,'visual_map_not_surveyed':True,'rows':items,'contemporaneous_visual_observations_with_both_branches':len(common),'median_xy_disagreement_on_common_timestamps_m':errors,'preferred_lidar_branch_under_test_thresholds':chosen,'acceptance_policy':'Require >=2 temporally confirmed 1Hz visual observations matching a LiDAR hypothesis within 2m and competing branch farther than 8m. This is diagnostic only and does not grant global pose authority.','limitations':'Both map branches and 3D landmarks use the same raw unsurveyed mapping odometry. Correlated map geometry and extrinsic errors are not measured. Camera image-only PnP uses distinct observations, but cannot prove absolute geo accuracy.'}
(O/'visual_lidar_branch_arbitration.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
for i in common:print('t',round(i['query_camera_time_sec'],1),'inliers',i['visual_PnP_inliers'],'errors',{k:round(v['xy_disagreement_m'],3) for k,v in i['candidate_branches'].items()})
