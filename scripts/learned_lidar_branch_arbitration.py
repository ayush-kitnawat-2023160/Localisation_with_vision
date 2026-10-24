#!/usr/bin/env python3
"""Offline-only cross-sensor hypothesis vote with learned-camera body poses.
Reports support for one of two independently replayed LiDAR branches.
Never promotes an observation to /pose or asserts surveyed accuracy.
"""
import csv,json,math
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parents[1];O=R/'outputs/learned_visual_20261020';b='localization_3';root=R/'outputs/localization3_alternate_cold_start_20261019'
cal=yaml.safe_load(open(R/'config/calibration/zedx_current.yaml'))['extrin_calib']
def T(rot,trans):
 m=np.eye(4);m[:3,:3]=np.asarray(rot).reshape(3,3);m[:3,3]=trans;return m
T_cam_body=T(cal['Rcl'],cal['Pcl'])@np.linalg.inv(T(cal['extrinsic_R'],cal['extrinsic_T']))
stamps=json.load(open(R/'outputs/looooo_integrity_20261019/localization_3.json'))['sensor_timestamps'];offset=stamps['camera']['first_stamp']-stamps['lidar']['first_stamp']
branches={name:np.asarray(json.load(open(root/d/'events.json'))['map_pose_trace'],float) for name,d in [('continuous22','start_22s_42s_gps_off'),('cold54','start_54s_10s_gps_off')]}
report={}
for method,folder in [('xfeat_mnn','heldout_xfeat_mnn'),('xfeat_lighterglue','heldout_xfeat_lighterglue'),('superpoint_lightglue','heldout_superpoint_lightglue')]:
 rows=json.load(open(O/folder/(b+'_matches.json')));outs=[]
 for item in rows:
  if not item.get('matched') or item['t_sec']<58:continue
  x=item['best'];Tc=T(Rotation.from_quat(x['rotation_xyzw']).as_matrix(),x['map_xyz_m']);Tb=Tc@T_cam_body
  t=item['t_sec']+offset;z={'camera_t_sec':item['t_sec'],'body_xyz':Tb[:3,3].tolist(),'PnP_inliers':x['inliers'],'keyframe':x['map_keyframe'],'branch_xy_disagreement_m':{}}
  for name,trace in branches.items():
   if trace[0,0]<=t<=trace[-1,0]:
    xyz=np.array([np.interp(t,trace[:,0],trace[:,j]) for j in (1,2,3)])
    z['branch_xy_disagreement_m'][name]=float(np.linalg.norm(Tb[:2,3]-xyz[:2]))
  outs.append(z)
 agree=[r for r in outs if 'continuous22' in r['branch_xy_disagreement_m'] and 'cold54' in r['branch_xy_disagreement_m']];agree.sort(key=lambda x:x['camera_t_sec'])
 supported=[x for x in agree if x['branch_xy_disagreement_m']['cold54']<=2 and x['branch_xy_disagreement_m']['continuous22']>=8]
 result={'late_verified_visual_poses':len(outs),'contemporaneous_two_lidar_hypothesis_comparisons':len(agree),'supporting_cold54_branch':len(supported),'claim':'Camera matches agree with late LiDAR cold-start hypothesis under shared raw map and inherited extrinsics, not surveyed ground truth','samples':agree}
 report[method]=result
(O/'LEARNED_LIDAR_ARBITRATION.json').write_text(json.dumps(report,indent=2)+'\n')
for n,v in report.items():
 print(n,'late',v['late_verified_visual_poses'],'common',v['contemporaneous_two_lidar_hypothesis_comparisons'],'support cold54',v['supporting_cold54_branch'])
 for z in v['samples']:print(' ',round(z['camera_t_sec'],1),'inliers',z['PnP_inliers'], {k:round(a,2) for k,a in z['branch_xy_disagreement_m'].items()})
