#!/usr/bin/env python3
"""Offline ZED X-to-body transform consistency audit, NOT physical calibration.

The recorded /tf_static camera<->IMU chain is a driver convention, whereas
the existing camera<->LiDAR transform comes from supplied reprojection
correspondences and the LiDAR<->base mounting is still inherited/unverified.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag
import yaml
from tf.transformations import quaternion_matrix


def t_ros(st):
    p=st.transform.translation
    q=st.transform.rotation
    t=quaternion_matrix([q.x,q.y,q.z,q.w])
    t[:3,3]=[p.x,p.y,p.z]
    return t


def t_cfg(r,p):
    t=np.eye(4)
    t[:3,:3]=np.array(r).reshape(3,3)
    t[:3,3]=p
    return t


def find_path(edges,source,target):
    q=[(source,np.eye(4),[source])]
    seen=set()
    while q:
        name,current,path=q.pop(0)
        if name==target:return current,path
        if name in seen:continue
        seen.add(name)
        for next_name,step in edges.get(name,[]):
            if next_name not in seen:q.append((next_name,current@step,path+[next_name]))
    return None,[]


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--bag',required=True)
    ap.add_argument('--calibration',required=True)
    ap.add_argument('--out',required=True)
    a=ap.parse_args()
    config=yaml.safe_load(open(a.calibration))
    edges={};tf_records=[]
    camera=None
    with rosbag.Bag(a.bag) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/tf_static','/zedx/zed_node/left/camera_info']):
            if topic=='/tf_static':
                for st in msg.transforms:
                    fr=st.header.frame_id.lstrip('/')
                    to=st.child_frame_id.lstrip('/')
                    if (fr,to) in tf_records:continue
                    tf_records.append((fr,to))
                    tf=t_ros(st)
                    edges.setdefault(fr,[]).append((to,tf))
                    edges.setdefault(to,[]).append((fr,np.linalg.inv(tf)))
            elif camera is None:
                camera=msg
            if camera is not None and len(tf_records)>=10:break
    recorded_base_camera,route=find_path(edges,'base_link','zedx_left_camera_optical_frame')
    imu_camera,imu_route=find_path(edges,'zedx_imu_link','zedx_left_camera_optical_frame')
    if recorded_base_camera is None or imu_camera is None:
        raise RuntimeError('Incomplete ZED X optical/IMU/base TF chain')
    c=config['calibration'];e=config['extrin_calib']
    lidar_base=t_cfg(c['lidar_to_base_rotation'],c['lidar_to_base_translation'])
    cam_lidar=t_cfg(e['Rcl'],e['Pcl'])
    # lidar_base is conventionally base<-LiDAR; camera<-LiDAR from Rcl/Pcl.
    base_cam_inherited=lidar_base@np.linalg.inv(cam_lidar)
    R=recorded_base_camera[:3,:3].T@base_cam_inherited[:3,:3]
    angular=math.degrees(math.acos(float(np.clip((np.trace(R)-1)/2,-1,1))))
    translation=float(np.linalg.norm(recorded_base_camera[:3,3]-base_cam_inherited[:3,3]))
    K=config['laserMapping']
    diff={name:{'bag_px':float(camera.K[index]),'yaml_px':float(K['cam_'+name]),
                'difference_px':float(camera.K[index]-K['cam_'+name])}
          for name,index in [('fx',0),('fy',4),('cx',2),('cy',5)]}
    report={'bag':Path(a.bag).name,'camera_info':{'resolution':[camera.width,camera.height],
        'frame_id':camera.header.frame_id,'intrinsics':diff},
        'recorded_imu_to_camera_driver_tf':imu_camera.tolist(),
        'recorded_imu_camera_chain':imu_route,
        'recorded_base_camera_driver_tf':recorded_base_camera.tolist(),
        'recorded_base_camera_chain':route,
        'inherited_map_body_camera_from_lidar_configuration':base_cam_inherited.tolist(),
        'translation_disagreement_m':translation,'rotation_disagreement_deg':angular,
        'physical_body_camera_calibration_verified':False,
        'independent_imu_noise_and_time_offset_calibrated':False,
        'meaning':'Different software transform chains; cannot establish which is physically correct without surveying/cart measurement',
        'physical_calibration_steps':[
          'Record static camera, IMU, LiDAR, and chassis mount locations with measured 6DoF extrinsics (not driver identity TF)',
          'Use Kalibr with an Aprilgrid plus synchronized inertial excitation to estimate ZED left optical to ZED IMU and time offset',
          'Collect stationary IMU log for Allan deviation and calibrate gyro/accel noise densities and bias random walks before deploying camera-IMU odometry',
          'Use a surveyed marker target or independent RTK/total station to check camera/base against LiDAR global map',
          'Validate translation/rotation sign convention via PnP, TF composition, checkerboard reprojections, and recorded motion',
          'Preserve calibration.ready=false for cart deployment until all checks pass']}
    out=Path(a.out)
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+'\n')
    print('RECORDING_TF_AUDIT translation_disagreement_m',round(translation,4),
          'rotation_disagreement_deg',round(angular,3),'physical_truth=UNVERIFIED')


if __name__=='__main__':main()
