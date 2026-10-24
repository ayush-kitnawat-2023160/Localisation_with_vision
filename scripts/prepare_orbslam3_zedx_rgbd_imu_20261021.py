#!/usr/bin/env python3
"""Create a PROVISIONAL RGB-D+IMU ORB-SLAM3 settings file from a ZED X bag.

Only for offline comparison; ZED driver TF and borrowed IMU noise are not
physically verified. No motion control, global localization or pose authority.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
from tf.transformations import quaternion_matrix


def matrix(stamped):
    p=stamped.transform.translation
    q=stamped.transform.rotation
    T=quaternion_matrix((q.x,q.y,q.z,q.w))
    T[:3,3]=[p.x,p.y,p.z]
    return T


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--bag',required=True)
    ap.add_argument('--out',required=True)
    a=ap.parse_args()
    edges={}
    camera=None
    times=[]
    counts={}
    with rosbag.Bag(a.bag) as b:
        counts={k:v.message_count for k,v in b.get_type_and_topic_info().topics.items()
                if k.startswith('/zedx/zed_node/') and
                any(n in k for n in ('image_rect','depth_registered','imu/data','camera_info'))}
        for topic,msg,_ in b.read_messages(topics=['/tf_static',
            '/zedx/zed_node/left/camera_info','/zedx/zed_node/imu/data']):
            if topic=='/tf_static':
                for st in msg.transforms:
                    edges[(st.header.frame_id,st.child_frame_id)]=matrix(st)
            elif topic.endswith('camera_info') and camera is None:
                camera=msg
            else:
                if len(times)<400:times.append(msg.header.stamp.to_sec())
            if camera is not None and len(times)>=400 and (
                    'zedx_left_camera_frame','zedx_imu_link') in edges and (
                    'zedx_left_camera_frame','zedx_left_camera_optical_frame') in edges:
                break
    if camera is None:raise RuntimeError('Missing ZED X camera info')
    T_li=edges['zedx_left_camera_frame','zedx_imu_link']
    T_lo=edges['zedx_left_camera_frame','zedx_left_camera_optical_frame']
    T_imu_cam=np.linalg.inv(T_li)@T_lo
    freq=round(float(1./np.median(np.diff(times))),2)
    grid=', '.join(f'{x:.9g}' for x in T_imu_cam.astype(float).reshape(-1))
    fx,fy,cx,cy=(float(camera.K[j]) for j in (0,4,2,5))
    k=list(camera.D)+[0]*5
    yaml=f'''%YAML:1.0
File.version: "1.0"
Camera.type: "PinHole"
Camera1.fx: {fx:.9f}
Camera1.fy: {fy:.9f}
Camera1.cx: {cx:.9f}
Camera1.cy: {cy:.9f}
Camera1.k1: {k[0]:.9f}
Camera1.k2: {k[1]:.9f}
Camera1.p1: {k[2]:.9f}
Camera1.p2: {k[3]:.9f}
Camera1.k3: {k[4]:.9f}
Camera.width: {camera.width}
Camera.height: {camera.height}
Camera.fps: 13
Camera.RGB: 0
Stereo.ThDepth: 40.0
Stereo.b: 0.12
RGBD.DepthMapFactor: 1.0
IMU.T_b_c1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [{grid}]
# Placeholder IMU densities borrowed from previously configured cart.
# NOT MEASURED using Allan-variance on this ZED X.
IMU.NoiseGyro: 0.00035
IMU.NoiseAcc: 0.003
IMU.GyroWalk: 0.00003
IMU.AccWalk: 0.003
IMU.Frequency: {freq:.1f}
IMU.InsertKFsWhenLost: 0
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
System.thFarPoints: 0.0
'''
    target=Path(a.out)
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(yaml)
    provenance={
      'bag':Path(a.bag).name,'source':'ZED driver TF and left camera_info',
      'camera_intrinsics':[fx,fy,cx,cy],
      'camera_resolution':[camera.width,camera.height],
      'imu_frequency_hz_from_headers':freq,
      'T_imu_left_optical_driver_tf':T_imu_cam.tolist(),
      'camera_imu_extrinsics_physically_verified':False,
      'imu_noise_allan_verified':False,
      'body_camera_mount_physically_verified':False,
      'camera_input':'rectified left RGB','depth_input':'registered float32 metric depth',
      'sensor_mode':'ORB-SLAM3 System::IMU_RGBD',
      'camera_rgb':False,'depth_scale':1.0,'bag_topics':counts}
    target.with_suffix('.provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print('ORB_CONFIG_WRITTEN',str(target),'IMU_HZ',freq,'CAMERA',camera.width,camera.height)


if __name__=='__main__':main()
