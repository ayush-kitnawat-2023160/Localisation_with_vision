#!/usr/bin/env python3
"""Static preflight: fail closed on missing/incompatible GPS-free artifacts.

An offline PASS means only 'files and configuration structurally consistent'.
It is NOT a test of accuracy, physical calibration, sensor control or safety.
Live PASS additionally requires a declared physical calibration provenance;
it cannot independently certify hardware measurement or brake interlocks.
"""
import argparse
import json
import math
import hashlib
from pathlib import Path
import sys
import urllib.request
import yaml
import numpy as np


def rotation(arr):
    r=np.asarray(arr,float).reshape(3,3)
    return np.isfinite(r).all() and abs(np.linalg.det(r)-1)<.001 and np.linalg.norm(r.T@r-np.eye(3))<.001


def sha256_file(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as handle:
        while True:
            chunk=handle.read(1<<20)
            if not chunk:break
            h.update(chunk)
    return h.hexdigest()


def run(args):
    errors=[];warnings=[];facts={}
    def error(s):errors.append(s)
    def warning(s):warnings.append(s)
    calib=Path(args.calibration)
    lidar=Path(args.lidar_map)
    visual=Path(args.visual_map)
    learned=Path(args.learned_map) if args.learned_map else None
    for path,what in [(calib,'calibration'),(lidar/'pose.json','lidar pose database'),
             (lidar/'cache/scan_context.bin','Scan Context cache'),
             (lidar/'metadata.json','LiDAR map metadata'),
             (visual/'metadata.json','Visual map metadata')]:
        if not path.exists():error('MISSING_'+what.replace(' ','_').upper()+' '+str(path))
    if calib.exists():
        data=yaml.safe_load(calib.read_text())
        c=data.get('calibration',{})
        facts['calibration_id']=c.get('measurement_id')
        facts['calibration_ready']=bool(c.get('ready',False))
        if not facts['calibration_ready']:error('CALIBRATION_LOCKED_READY_FALSE')
        if not c.get('measurement_id') or 'REPLACE_ME' in str(c.get('measurement_id')):
            error('CALIBRATION_MISSING_MEASUREMENT_ID')
        for key in ('lidar_to_base_rotation','lidar_to_imu_rotation'):
            try:
                if not rotation(c[key]):error('INVALID_'+key)
            except (KeyError,ValueError,TypeError):error('MISSING_'+key)
        x=data.get('camera_imu_fallback',{})
        facts['camera_imu_rotation_source']=x.get('rotation_source','MISSING')
        try:
            if not rotation(x['imu_to_optical_rotation']):error('INVALID_CAMERA_IMU_ROTATION')
        except (KeyError,ValueError,TypeError):error('MISSING_CAMERA_IMU_ROTATION')
        if args.mode=='live':
            if any(x in str(c.get('measurement_id','')).upper() for x in
                    ('OFFLINE','BAG','LEGACY','PROVISIONAL','UNMEASURED')):
                error('LIVE_PROFILE_BAG_OR_INHERITED_MOUNT_REJECTED')
            if any(x in str(facts['camera_imu_rotation_source']).upper() for x in
                    ('OFFLINE','BAG','LEGACY','PROVISIONAL','UNMEASURED','MISSING')):
                error('LIVE_CAMERA_IMU_EXTRINSIC_PROVENANCE_NOT_VERIFIED')
            elif not str(facts['camera_imu_rotation_source']).upper().startswith('PHYSICALLY_MEASURED_'):
                error('LIVE_CAMERA_IMU_REQUIRES_PHYSICALLY_MEASURED_RUN_ID')
        else:
            if 'OFFLINE' in str(facts['calibration_id']).upper():
                warning('OFFLINE_REPLAY_CALIBRATION_ONLY; do not drive a vehicle with this file')
    if (lidar/'metadata.json').exists():
        ld=json.loads((lidar/'metadata.json').read_text())
        facts['lidar_map_source_bag']=ld.get('source_bag')
        facts['lidar_pose_quality']=ld.get('pose_quality')
        if ld.get('pose_quality')!='SURVEYED_LOOP_CLOSED':
            warning('LiDAR map is NOT surveyed/loop-closed; measured global accuracy unknown')
    else:ld={}
    if (visual/'metadata.json').exists():
        vd=json.loads((visual/'metadata.json').read_text())
        facts['visual_map_source_bag']=vd.get('source_bag')
        facts['visual_map_trajectory_loop_closed']=vd.get('trajectory_is_loop_closed')
        if vd.get('gps_used'):error('VISUAL_MAP_USED_GPS')
        if vd.get('source_bag')!=ld.get('source_bag'):
            error('LIDAR_AND_VISUAL_MAPS_FROM_DIFFERENT_MAPPING_SESSIONS')
        if vd.get('trajectory_is_loop_closed') is not True:
            warning('Visual map is built on RAW/UNOPTIMIZED LiDAR trajectory')
        if vd.get('physical_imu_lidar_extrinsics_validated') is not True:
            warning('Map construction uses unverified physical IMU/LiDAR extrinsics')
    if args.bag:
        name=Path(args.bag).name
        if name==ld.get('source_bag'):
            warning('DATA_LEAKAGE: test bag equals mapping source bag; NOT held-out validation')
        facts['test_bag']=name
    if args.require_visual_worker:
        if learned is None:
            error('LEARNED_WORKER_MAP_PATH_MUST_BE_EXPLICIT')
        elif not (learned/'visual/keyframes.csv').is_file():
            error('LEARNED_WORKER_MAP_KEYFRAMES_MISSING '+str(learned))
        try:
            url=args.worker_url.rstrip('/')+'/health'
            # Worker is listening on localhost only.
            if not (url.startswith('http://127.0.0.1:') or url.startswith('http://localhost:')):
                error('VISUAL_WORKER_NON_LOCALHOST_URL')
            else:
                with urllib.request.urlopen(url,timeout=1.5) as response:
                    health=json.load(response)
                if not health.get('ready',False) or health.get('gps_used',True):
                    error('VISUAL_WORKER_NOT_READY_OR_GPS_USED')
                facts['worker_backend']=health.get('backend')
                facts['worker_map_sha256']=health.get('map_sha256')
                if health.get('retrieval_mode')!='learned_float_flann':
                    error('VISUAL_WORKER_RETRIEVAL_MODE_NOT_EXPECTED')
                if health.get('backend')!='xfeat_mnn':
                    error('VISUAL_WORKER_BACKEND_NOT_EXPECTED')
                if (learned is not None and (learned/'visual/keyframes.csv').is_file()
                        and health.get('map_sha256')!=sha256_file(learned/'visual/keyframes.csv')):
                    error('VISUAL_WORKER_LEARNED_MAP_HASH_MISMATCH')
                if (visual/'visual/keyframes.csv').is_file() and \
                        health.get('orb_map_sha256')!=sha256_file(visual/'visual/keyframes.csv'):
                    error('VISUAL_WORKER_ORB_SOURCE_MAP_HASH_MISMATCH')
                if calib.exists() and health.get('calibration_sha256')!=sha256_file(calib):
                    error('VISUAL_WORKER_CALIBRATION_PROFILE_HASH_MISMATCH')
                if calib.exists():
                    k=yaml.safe_load(calib.read_text()).get('laserMapping',{})
                    if health.get('expected_image_width')!=int(k.get('cam_width',0)) or \
                            health.get('expected_image_height')!=int(k.get('cam_height',0)):
                        error('VISUAL_WORKER_IMAGE_DIMENSIONS_MISMATCH')
        except Exception as e:error('VISUAL_WORKER_NOT_REACHABLE '+str(e)[:160])
    facts.update({'mode':args.mode,'gps_soft_prior_enabled':False,
                  'autonomous_pose_enabled':False,
                  'physical_brake_interlock_tested':False})
    result={'pass_static_preflight':not errors,
            'eligible_for_navigation':False,
            'mode':args.mode,
            'errors':errors,'warnings':warnings,'facts':facts,
            'note':'No numerical success rate or hardware readiness can be inferred from static files.'}
    print(json.dumps(result,indent=2))
    if args.output:
        path=Path(args.output)
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,indent=2)+'\n')
    return 0 if not errors else 2


def main():
    root=Path(__file__).resolve().parents[1]
    ap=argparse.ArgumentParser()
    ap.add_argument('--mode',choices=['offline','live'],default='offline')
    ap.add_argument('--calibration',default=str(root/'config/calibration/zedx_current.yaml'))
    ap.add_argument('--lidar-map',default=str(root/'maps/campus_full_lio_226s_20261019'))
    ap.add_argument('--visual-map',default=str(root/'maps/campus_full_visual_rgbd_226s_20261020'))
    ap.add_argument('--bag',default='')
    ap.add_argument('--require-visual-worker',action='store_true')
    ap.add_argument('--worker-url',default='http://127.0.0.1:18091')
    ap.add_argument('--learned-map',default='',help='EXPLICIT learned map root to bind worker fingerprint')
    ap.add_argument('--output',default='')
    sys.exit(run(ap.parse_args()))


if __name__=='__main__':main()
