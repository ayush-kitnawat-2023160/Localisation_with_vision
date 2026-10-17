#!/usr/bin/env python3
"""Read-only integrity checks for the Oct16 clean single-build release."""
from pathlib import Path
import csv,hashlib,json,os
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
O=ROOT/'release/ALIVE_FASTLIVO2_OFFLINE_RC_20261016/manifest'
IMAGE=(ROOT/'results/FINAL_SINGLE_IMAGE_RC_20261016/frozen_docker_image_id.txt').read_text().strip()
def info(path):
 p=ROOT/path
 assert p.is_file() and p.stat().st_size>100
 return {'path':str(path),'bytes':p.stat().st_size}
C=ROOT/'config/calibration/recorded_cart_bag.yaml'
F=ROOT/'config/failsafe_rosbag.yaml'
M=ROOT/'config/map_failsafe_recorded_cart.yaml'
cal=yaml.safe_load(C.read_text());rcl=np.array(cal['extrin_calib']['Rcl']).reshape(3,3)
rbl=np.array(cal['extrin_calib']['extrinsic_R']).reshape(3,3)
def proper(r):
 return {'determinant':round(float(np.linalg.det(r)),8),
         'orthonormal_error':round(float(np.linalg.norm(r@r.T-np.eye(3))),10)}
geometry={'recorded_cart_lidar_to_camera':proper(rcl),
          'recorded_cart_lidar_to_body':proper(rbl)}
for k,v in geometry.items():
 assert abs(v['determinant']-1)<1e-4 and v['orthonormal_error']<1e-5,(k,v)
main=ROOT/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv'
sep=ROOT/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv'
ref_stats={}
for p in (main,sep):
 with p.open() as f:r=list(csv.DictReader(f))
 t=np.array([float(x['timestamp']) for x in r])
 xyz=np.array([[float(x[k]) for k in 'xyz'] for x in r])
 assert np.isfinite(xyz).all() and len(t)>200 and np.min(np.diff(t))>0
 ref_stats[p.name]={'samples':len(r),'span_s':round(float(t[-1]-t[0]),3),
     'monotone_timestamps':True,'surveyed_independent_ground_truth':False}
image_dir=ROOT/'step3_global/healthy_validation_assets_20261014/visual_map_images'
assert image_dir.is_dir()
total_images=sum(1 for p in image_dir.rglob('*') if p.is_file() and p.suffix.lower() in ('.jpg','.jpeg','.png'))
assert total_images>30
assets=[
'step3_global/map_db_sweeps_fast/cache/global_map.pcd',
'step3_global/healthy_validation_assets_20261014/final_camera_rgb_lidar_voxel_15cm.pcd',
'step3_global/healthy_validation_assets_20261014/orb_cache_dev_20261015/visual_orb_retrieval_exact.npz']
rep={'validated_date':'2026-10-16','frozen_image_id':IMAGE,
     'source_calibration_file':str(C.relative_to(ROOT)),
     'sensor_rig_rotation_sanity':geometry,
     'config_files_present':[str(p.relative_to(ROOT)) for p in (C,F,M)],
     'reference_sanity':ref_stats,
     'saved_visual_keyframe_image_count':total_images,
     'external_assets':[info(p) for p in assets],
     'original_rosbags_not_bundled':True,
     'absolute_extrinsics_field_verified':False,
     'clock_synced_on_actual_cart':False,
     'latency_profiled_on_actual_cart':False,
     'stop_to_DBW_verified':False,
     'surveyed_GT_available':False,
     'readiness':'PASS STATIC SOURCE/DATA PRESENCE CHECKS ONLY; NOT VEHICLE SAFE'}
O.mkdir(parents=True,exist_ok=True)
p=O/'ASSET_AND_CALIBRATION_INTEGRITY.json'
p.write_text(json.dumps(rep,indent=2)+'\n')
print('Read-only asset audit PASSED',p)
print('Saved-keyframe images',total_images,'rotations',geometry)
