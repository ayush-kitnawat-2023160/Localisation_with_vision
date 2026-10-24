#!/usr/bin/env python3
"""Offline fair-ish SE3 relative position drift against held-out LiDAR odom.

ORB-SLAM3 pose is optical camera; KLT fallback publishes a body candidate.
Alignment uses only the FIRST pose after 20s, never the later reference.
The LiDAR-only odometry and camera/body extrinsics are not surveyed truth.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rosbag
import yaml
from tf.transformations import quaternion_matrix,quaternion_slerp


def pose(p,q):
    T=quaternion_matrix(q)
    T[:3,3]=p
    return T


def transform(r,t):
    T=np.eye(4)
    T[:3,:3]=np.asarray(r,dtype=float).reshape(3,3)
    T[:3,3]=t
    return T


def interp(t,ts,xyz,quats):
    j=int(np.searchsorted(ts,t))
    if j<1 or j>=len(ts):return None
    if ts[j]-ts[j-1]>.3:return None
    w=(t-ts[j-1])/(ts[j]-ts[j-1])
    return pose(xyz[j-1]*(1-w)+xyz[j]*w,quaternion_slerp(quats[j-1],quats[j],w))


def evaluate(label,events,origin,ts,xyz,quats,body_camera,start=20.,end=55.):
    valid=[]
    for x in events:
        t=x['stamp']
        if not origin+start<=t<=origin+end:continue
        reference=interp(t,ts,xyz,quats)
        if reference is None:continue
        if label.startswith('ORB-SLAM3'):
            reference=reference@body_camera
        guess=pose(x['xyz'],x['quat'])
        if not np.isfinite(guess).all() or not np.isfinite(reference).all():continue
        valid.append((t,guess,reference))
    if len(valid)<3:return {'status':'INSUFFICIENT_ESTIMATES','paired_count':len(valid)}
    first=valid[0]
    alignment=first[2]@np.linalg.inv(first[1])
    errs=[]
    for t,p,r in valid:
        predicted=alignment@p
        e=np.linalg.norm(predicted[:3,3]-r[:3,3])
        errs.append((t-origin,float(e)))
    times=np.array([t for t,_ in errs])
    errors=np.array([e for _,e in errs])
    return {
      'paired_count':len(valid),'first_bag_sec':float(times[0]),
      'last_bag_sec':float(times[-1]),
      'max_timestamp_gap_sec':float(np.max(np.diff(times))),
      'outage_relative_rmse_m':float(np.sqrt(np.mean(errors**2))),
      'outage_endpoint_difference_m':float(errors[-1]) if end-times[-1]<.5 else None,
      'outage_p95_difference_m':float(np.percentile(errors,95)),
      'outage_max_difference_m':float(errors.max()),
      'ground_truth':False,
      'interpretation':'Absolute alignment only at first paired pose in window; LiDAR-only odom proxy not surveyed truth',
    }


def main():
    a=argparse.ArgumentParser()
    a.add_argument('--orb-dir',required=True)
    a.add_argument('--klt-dir',required=True)
    a.add_argument('--ref-bag',required=True)
    a.add_argument('--calibration',required=True)
    a.add_argument('--output',required=True)
    a.add_argument('--orb-mode',choices=['rgbd','rgbd_imu'],default='rgbd_imu')
    args=a.parse_args()
    origin=None;ref=[]
    with rosbag.Bag(args.ref_bag) as bag:
        origin=bag.get_start_time()
        for _,m,_ in bag.read_messages(topics=['/aft_mapped_to_init']):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            ref.append((m.header.stamp.to_sec(),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
    if not ref:raise RuntimeError('reference LiDAR-only odometry missing')
    ts=np.array([x[0] for x in ref])
    xyz=np.array([x[1] for x in ref])
    quat=np.array([x[2] for x in ref])
    cfg=yaml.safe_load(open(args.calibration))['extrin_calib']
    T_body_cam=transform(cfg['extrinsic_R'],cfg['extrinsic_T'])@np.linalg.inv(
               transform(cfg['Rcl'],cfg['Pcl']))
    orb_dir=Path(args.orb_dir)
    orb=json.load(open(orb_dir/'pose_events.json'))
    klt=json.load(open(Path(args.klt_dir)/'pose_events.json'))
    orbposes=[dict(stamp=z['stamp'],xyz=z['xyz'],quat=z['quat']) for z in orb['poses']]
    kltposes=[dict(stamp=z['stamp'],xyz=z['xyz'],quat=z['quat']) for z in klt['odometry']]
    frame_rows=list(csv.DictReader(open(orb_dir/'frame_compute.csv')))
    frame_ms=np.array([float(r['tracking_ms']) for r in frame_rows])
    status=(orb_dir/'status.txt').read_text()
    profile=json.load(open(orb_dir/'process_cpu_ram.json'))
    s=dict(line.split('=',1) for line in status.splitlines() if '=' in line)
    c=profile['samples']
    during=[v for v in c if float(s['PLAY_START_EPOCH'])<=v['epoch_s']<=float(s['PLAY_END_EPOCH'])]
    if len(during)>=2:
        cpu_play=(during[-1]['cpu_seconds']-during[0]['cpu_seconds'])/(
                      during[-1]['epoch_s']-during[0]['epoch_s'])
    else:cpu_play=None
    tracking={k:sum(r['tracking_state']==str(k) for r in frame_rows)
              for k in sorted(set(int(r['tracking_state']) for r in frame_rows))}
    results={
      'bag':Path(args.ref_bag).name.replace('_lidar_only_odom.bag','.bag'),
      'orb_mode':args.orb_mode,
      'input_mode':'ZED left rectified RGB + registered float32 depth + ZED IMU',
      'benchmark_playback_rate':1.0,
      'orb_state2_pose_count':len(orbposes),
      'orb_frames_processed':len(frame_rows),
      'orb_state_occurrences':tracking,
      'orb_max_bounded_pair_queue_drop':max((int(r['paired_queue_drop']) for r in frame_rows),default=0),
      'orb_frame_processing_median_ms':float(np.median(frame_ms)),
      'orb_frame_processing_p95_ms':float(np.percentile(frame_ms,95)),
      'orb_process_cpu_mean_cores_including_idle':profile['mean_cpu_cores'],
      'orb_process_cpu_cores_during_bag_playback':cpu_play,
      'orb_process_peak_rss_mib':profile['peak_rss_mib'],
      'orb_process_exit_code':profile['exit_code'],
      'orb_input_rgb_count':orb['counts'].get('image_in'),
      'orb_input_imu_count':orb['counts'].get('imu_in'),
      'orb_unexpected_pose_publications':orb['counts'].get('unexpected_pose',0),
      'orb_standalone_agreement':evaluate('ORB-SLAM3 '+args.orb_mode,orbposes,origin,ts,xyz,quat,T_body_cam),
      'klt_standalone_agreement':evaluate('KLT PnP RGBD+gyro',kltposes,origin,ts,xyz,quat,T_body_cam),
      'klt_profile_comparison':json.load(open(Path(args.klt_dir)/'rgbd_process_resources.json'))['summary'],
      'important_caveats':[
         'ORB-SLAM3 may perform map reoptimizations and relocalizations, so an absolute candidate trajectory is not guaranteed odometry-continuous',
         'Reference LiDAR-only odometry is a proxy and has drift; it is not surveyed ground truth',
         'Camera-to-cart extrinsic is inherited from unverified LiDAR mounting',
         'IMU noise calibration derived from preliminary values, not Allan variance',
         'ROS replay CPU excludes ZED SDK stereo depth-generation GPU cost',
         'Local map tracking state=2 does not guarantee reliable covariance or safe cart fallback',
         'ORB is not connected to /pose, FAST-LIVO2 internal estimator, TF or vehicle motion controls']}
    Path(args.output).write_text(json.dumps(results,indent=2)+'\n')
    print('ORB_KLT_COMPARE',json.dumps({k:v for k,v in results.items()
      if k.endswith('_agreement') or k in ('orb_state2_pose_count','orb_frames_processed',
          'orb_frame_processing_median_ms','orb_frame_processing_p95_ms',
          'orb_process_cpu_cores_during_bag_playback','orb_process_peak_rss_mib',
          'orb_process_exit_code')},indent=2))


if __name__=='__main__':main()
