#!/usr/bin/env python3
"""Same-frame ZED X RGB-D comparison: existing KLT+gyro vs keyframe+ESKF.

Run `--mode baseline` and `--mode keyframe_eskf` SEPARATELY for fair wall CPU
and RAM, or `--mode paired` to debug tracking on identical accepted frames.
The LiDAR-only reference is opened ONLY after both/one camera estimates finish.
Offline candidate only: no /pose, /tf, FAST-LIVO2 updates, or vehicle actions.
"""
import argparse
import csv
import json
import os
import time
from collections import OrderedDict,Counter,deque
from pathlib import Path

import cv2
import numpy as np
import rosbag
import yaml
from tf.transformations import quaternion_matrix,quaternion_from_matrix,quaternion_slerp

from zedx_rgbd_imu_odom_core import ZedRGBDIMUOdometry,pose
from zedx_rgbd_keyframe_eskf_core import ZedKeyframeESKFOdometry


def as_bgr(m):
    a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)
    if m.encoding=='bgra8':return cv2.cvtColor(a[:,:m.width*4].reshape(m.height,m.width,4),cv2.COLOR_BGRA2BGR)
    if m.encoding=='bgr8':return a[:,:m.width*3].reshape(m.height,m.width,3).copy()
    if m.encoding=='rgb8':return cv2.cvtColor(a[:,:m.width*3].reshape(m.height,m.width,3),cv2.COLOR_RGB2BGR)
    raise ValueError('Unknown image encoding '+m.encoding)


def interp_ref(t,ts,xyz,qs):
    if t<ts[0] or t>ts[-1]:return None
    j=int(np.searchsorted(ts,t))
    j=max(1,min(j,len(ts)-1))
    if ts[j]-ts[j-1]>.30:return None
    u=(t-ts[j-1])/(ts[j]-ts[j-1])
    T=quaternion_matrix(quaternion_slerp(qs[j-1],qs[j],u))
    T[:3,3]=xyz[j-1]*(1-u)+xyz[j]*u
    return T


def reference_metrics(rows,ts,xyz,qs,body_camera,origin,lo,hi):
    eligible=[]
    for v in rows:
        if not v['accepted'] or not origin+lo<=v['stamp']<=origin+hi:continue
        if not np.isfinite(v['T']).all():continue
        ref=interp_ref(v['stamp'],ts,xyz,qs)
        if ref is not None:eligible.append((v['stamp'],v['T'],ref))
    if len(eligible)<5:
        return {'status':'INSUFFICIENT_VALID_CAMERA_ESTIMATES','paired':len(eligible)}
    start=eligible[0]
    aligned=start[2]@body_camera@np.linalg.inv(start[1])
    tim=[];pred=[];reference=[]
    for t,T,R in eligible:
        pred.append(aligned@T@np.linalg.inv(body_camera));reference.append(R);tim.append(t-origin)
    pred=np.stack(pred);ref=np.stack(reference)
    tt=np.array(tim)
    err=np.linalg.norm(pred[:,:3,3]-ref[:,:3,3],axis=1)
    result={'status':'RELATIVE_COMPARISON_ONLY_NOT_GROUND_TRUTH',
      'paired':len(tt),'first_bag_s':float(tt[0]),'last_bag_s':float(tt[-1]),
      'max_pose_timestamp_gap_s':float(max(np.diff(tt))) if len(tt)>1 else None,
      'position_rmse_m':float(np.sqrt(np.mean(err**2))),
      'endpoint_position_disagreement_m':float(err[-1]) if hi-tt[-1]<.5 else None,
      'position_p95_m':float(np.percentile(err,95)),
      'position_max_m':float(err.max()),
      'body_mount_physically_verified':False,'reference_ground_truth':False,
      'position_error_samples_m':err.tolist(),
      'time_samples_s':tt.tolist()}
    for window in [1.,5.]:
        values=[]
        for i,t in enumerate(tt):
            if t+window>hi:continue
            j=int(np.searchsorted(tt,t+window))
            if j>=len(tt) or abs(tt[j]-(t+window))>.15:continue
            dA=np.linalg.inv(pred[i])@pred[j]
            dB=np.linalg.inv(ref[i])@ref[j]
            values.append(float(np.linalg.norm(dA[:3,3]-dB[:3,3])))
        result[f'rpe_{window:g}s_median_translation_m']=float(np.median(values)) if values else None
        result[f'rpe_{window:g}s_p95_translation_m']=float(np.percentile(values,95)) if values else None
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--bag',required=True)
    p.add_argument('--reference',required=True)
    p.add_argument('--calibration',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--mode',choices=['baseline','keyframe_eskf','paired'],required=True)
    p.add_argument('--frame-period',type=float,default=.065)
    p.add_argument('--scale',type=float,default=.65)
    p.add_argument('--outage-start',type=float,default=20.)
    p.add_argument('--outage-end',type=float,default=55.)
    p.add_argument('--keyframe-translation-m',type=float,default=.45)
    p.add_argument('--keyframe-max-age-sec',type=float,default=2.)
    p.add_argument('--duration',type=float,default=70.)
    args=p.parse_args()
    out=Path(args.output)
    if out.exists():raise RuntimeError('Refusing overwrite '+str(out))
    out.mkdir(parents=True)
    calib=yaml.safe_load(open(args.calibration))
    c=calib['laserMapping']
    K=np.array([[c['cam_fx'],0,c['cam_cx']],[0,c['cam_fy'],c['cam_cy']],[0,0,1.]],float)
    K[:2,:]*=args.scale
    # IMU->optical: recorded driver ZED tf_static, not physically surveyed.
    rleftopt=quaternion_matrix([.5,-.5,.5,-.5])[:3,:3]
    rleftimu=quaternion_matrix([-.0031,.0009,.0027,1.])[:3,:3]
    R_opt_imu=rleftopt.T@rleftimu
    wantbaseline=args.mode in ['baseline','paired']
    wantnew=args.mode in ['keyframe_eskf','paired']
    baseline=ZedRGBDIMUOdometry(K,R_opt_imu=R_opt_imu,imu_fraction=.35,
       track_points=650,min_inliers=20,min_inlier_fraction=.42) if wantbaseline else None
    novel=ZedKeyframeESKFOdometry(K,R_opt_imu=R_opt_imu,
       keyframe_translation_m=args.keyframe_translation_m,
       keyframe_max_interval_s=args.keyframe_max_age_sec) if wantnew else None
    rows={'baseline':[],'keyframe_eskf':[]}
    cpu_times={'baseline_image_sec':0.,'novel_image_sec':0.,'novel_imu_sec':0.,
               'baseline_imu_sec':0.,'image_convert_sec':0.}
    streams=Counter()
    rgb_cache=OrderedDict()
    depth_cache=OrderedDict()
    pending=deque()
    imu_stash=deque()
    max_imu_stamp=-1e30
    origin=None;last_img=-1e30;first_image=None;processed=0
    import psutil
    process=psutil.Process(os.getpid())
    peak_rss_mib=0.
    t_wall=time.monotonic()
    cpu_start=process.cpu_times()
    def run_one(t,rgb,depth):
        nonlocal last_img,processed,peak_rss_mib,first_image
        if t-last_img<args.frame_period-1e-7:return
        last_img=t
        processed+=1
        if first_image is None:first_image=t
        t0=time.perf_counter()
        color=as_bgr(rgb)
        z=np.frombuffer(depth.data,np.float32).reshape(depth.height,depth.step//4)[:,:depth.width].copy()
        if args.scale!=1.:
            color=cv2.resize(color,None,fx=args.scale,fy=args.scale,interpolation=cv2.INTER_AREA)
            z=cv2.resize(z,(color.shape[1],color.shape[0]),interpolation=cv2.INTER_NEAREST)
        cpu_times['image_convert_sec']+=time.perf_counter()-t0
        if baseline is not None:
            t0=time.perf_counter()
            b=baseline.process(t,color,z)
            ms=1000*(time.perf_counter()-t0)
            cpu_times['baseline_image_sec']+=ms/1000
            valid=b.state=='VISION_IMU_TRACKING'
            rows['baseline'].append({'stamp':t,'T':b.transform.copy(),'accepted':valid,
                'state':b.state,'tracks':b.tracks,'inliers':b.inliers,
                'parallax_deg':b.parallax_deg,'reproj_px':b.reproj_px,
                'compute_image_ms':ms,'imu_segment_ms':0.,'keyframes':0,
                'pos_sigma_m':None,'innovation_nis':None})
        if novel is not None:
            t0=time.perf_counter()
            x=novel.process(t,color,z)
            ms=1000*(time.perf_counter()-t0)
            cpu_times['novel_image_sec']+=ms/1000
            valid=x.state=='KEYFRAME_ESKF_TRACKING'
            rows['keyframe_eskf'].append({'stamp':t,'T':x.transform.copy(),
              'accepted':valid,'state':x.state,'tracks':x.tracks,'inliers':x.inliers,
              'parallax_deg':x.parallax_deg,'reproj_px':x.reproj_px,
              'compute_image_ms':ms,'imu_segment_ms':0.,'keyframes':x.keyframes,
              'pos_sigma_m':x.uncertainty_pos_sigma_m,'innovation_nis':x.innovation_nis,
              'velocity_mps':x.body_speed_mps})
        if processed%100==0:
            peak_rss_mib=max(peak_rss_mib,process.memory_info().rss/1048576)
            print('FRAME',processed,'bagsec',round(t-origin,2),
              'base',rows['baseline'][-1]['state'] if baseline else None,
              'new',rows['keyframe_eskf'][-1]['state'] if novel else None,
              'new_accepted',novel.accepted if novel else None,
              'wall',round(time.monotonic()-t_wall,1),flush=True)
    def pump(force=False):
        while pending and (force or max_imu_stamp>=pending[0][0]+.002):
            t,rgb,depth=pending.popleft()
            if novel is not None:
                while imu_stash and imu_stash[0][0]<=t:
                    zt,omega,accel=imu_stash.popleft()
                    t0=time.perf_counter()
                    novel.add_imu(zt,omega,accel)
                    cpu_times['novel_imu_sec']+=time.perf_counter()-t0
            run_one(t,rgb,depth)
    with rosbag.Bag(args.bag) as bag:
        origin=bag.get_start_time()
        for topic,m,_ in bag.read_messages(topics=['/zedx/zed_node/left/image_rect_color',
                 '/zedx/zed_node/depth/depth_registered','/zedx/zed_node/imu/data']):
            t=m.header.stamp.to_sec()
            if t-origin>args.duration:break
            if topic.endswith('/imu/data'):
                streams['imu_received']+=1
                g=m.angular_velocity;a=m.linear_acceleration
                omega=np.array([g.x,g.y,g.z]);accel=np.array([a.x,a.y,a.z])
                if baseline is not None:
                    tic=time.perf_counter();baseline.add_imu(t,omega)
                    cpu_times['baseline_imu_sec']+=time.perf_counter()-tic
                if novel is not None:imu_stash.append((t,omega,accel))
                max_imu_stamp=max(max_imu_stamp,t)
                pump()
            elif topic.endswith('/depth/depth_registered'):
                streams['depth_received']+=1
                key=round(t,5)
                rgb=rgb_cache.pop(key,None)
                if rgb is not None:
                    pending.append((t,rgb,m));streams['pairs_received']+=1
                else:depth_cache[key]=m
                pump()
            else:
                streams['rgb_received']+=1
                key=round(t,5)
                dep=depth_cache.pop(key,None)
                if dep is not None:
                    pending.append((t,m,dep));streams['pairs_received']+=1
                else:rgb_cache[key]=m
                pump()
            while len(depth_cache)>12:depth_cache.popitem(last=False)
            while len(rgb_cache)>12:rgb_cache.popitem(last=False)
    pump(force=True)
    peak_rss_mib=max(peak_rss_mib,process.memory_info().rss/1048576)
    estimate_wall=time.monotonic()-t_wall
    cpu_current=process.cpu_times()
    estimate_cpu=(cpu_current.user+cpu_current.system)-(cpu_start.user+cpu_start.system)
    # Do not expose LiDAR-only odometry to either visual estimator.
    ts=[];xyz=[];q=[]
    with rosbag.Bag(args.reference) as lidarbag:
        for _,m,_ in lidarbag.read_messages(topics=['/aft_mapped_to_init']):
            P=m.pose.pose.position;Q=m.pose.pose.orientation
            ts.append(m.header.stamp.to_sec());xyz.append([P.x,P.y,P.z]);q.append([Q.x,Q.y,Q.z,Q.w])
    ts=np.array(ts);xyz=np.array(xyz);q=np.array(q)
    extra=calib['extrin_calib']
    T_body_lidar=pose(np.array(extra['extrinsic_R']).reshape(3,3),extra['extrinsic_T'])
    T_cam_lidar=pose(np.array(extra['Rcl']).reshape(3,3),extra['Pcl'])
    T_body_cam=T_body_lidar@np.linalg.inv(T_cam_lidar)
    result={
       'mode':args.mode,'bag':Path(args.bag).name,
       'reference':Path(args.reference).name,'outage_window':[args.outage_start,args.outage_end],
       'processed_paired_frames':processed,'source_counts':dict(streams),
       'estimation_wall_sec_excluding_lidar_reference':estimate_wall,
       'process_cpu_sec_excluding_reference':estimate_cpu,
       'average_process_cpu_cores':estimate_cpu/estimate_wall if estimate_wall else None,
       'peak_rss_mib':peak_rss_mib,'cpu_times_by_component_sec':cpu_times,
       'sensor_note':'ZED depth precomputed; GPU stereo depth engine is not included',
       'calibration_status':'CAMERA-BODY EXTRINSICS UNVERIFIED; IMU NOISE PROVISIONAL',
       'reference_is_surveyed_ground_truth':False,'uses_lidar_for_estimation':False,
       'uses_gps_for_estimation':False,'pose_authority':False,
       'methods':{}}
    for name,rr in rows.items():
        if not rr:continue
        accept=[r for r in rr if r['accepted']]
        times=np.array([r['compute_image_ms'] for r in rr])
        steps=[float(np.linalg.norm(rr[i+1]['T'][:3,3]-rr[i]['T'][:3,3]))
               for i in range(len(rr)-1)]
        dt=np.diff([r['stamp'] for r in rr])
        speed=np.array(steps)/np.maximum(dt,1e-6)
        states=Counter(r['state'] for r in rr)
        ms={
         'frame_count':len(rr),'accepted_count':len(accept),
         'accepted_fraction':len(accept)/len(rr),'status_counts':dict(states),
         'vision_frame_process_ms_median':float(np.median(times)),
         'vision_frame_process_ms_p95':float(np.percentile(times,95)),
         'vision_frame_process_ms_max':float(max(times)),
         'max_candidate_speed_including_untrusted_mps':float(max(speed)),
         'keyframe_count':int(rr[-1]['keyframes']),
         'relative_lidar_proxy':reference_metrics(rr,ts,xyz,q,T_body_cam,origin,
               args.outage_start,args.outage_end)}
        if name=='keyframe_eskf':
            f=novel.filter
            nis=[x['innovation_nis'] for x in rr if x['accepted'] and np.isfinite(x['innovation_nis'])]
            psigma=[x['pos_sigma_m'] for x in rr if x['accepted']]
            ms.update({'imu_updates':f.n_imuprop,'IMU_initial_accel_std_mps2':f.initial_accel_std,
              'IMU_initial_mean_gyro_rps':f.initial_gyro_mean,
              'gravity_bootstrap_sec':f.gravity_bootstrapped_at-origin if f.gravity_bootstrapped_at else None,
              'final_estimated_gyro_bias_radps':f.bg.tolist(),
              'final_estimated_accel_bias_mps2':f.ba.tolist(),
              'innovation_NIS_median':float(np.median(nis)) if nis else None,
              'innovation_NIS_p95':float(np.percentile(nis,95)) if nis else None,
              'reported_pos_sigma_median':float(np.median(psigma)) if psigma else None,
              'reported_pos_sigma_max':float(np.max(psigma)) if psigma else None,
              'covariance_psd_checked_final':bool(f.covariance_ok()),
              'final_position_m':f.p.tolist(),'last_valid_state':novel.health})
        result['methods'][name]=ms
        path=out/(name+'_trajectory.csv')
        with path.open('w',newline='') as fh:
            fields=['stamp','sec','state','accepted','x','y','z','qx','qy','qz','qw',
                    'tracks','inliers','reproj_px','parallax_deg','compute_image_ms',
                    'keyframes','pos_sigma_m','innovation_nis']
            writer=csv.DictWriter(fh,fieldnames=fields);writer.writeheader()
            for row in rr:
                T=row['T'];qv=quaternion_from_matrix(T)
                writer.writerow({'stamp':row['stamp'],'sec':row['stamp']-origin,
                     'state':row['state'],'accepted':int(row['accepted']),
                     'x':T[0,3],'y':T[1,3],'z':T[2,3],
                     'qx':qv[0],'qy':qv[1],'qz':qv[2],'qw':qv[3],
                     'tracks':row['tracks'],'inliers':row['inliers'],
                     'reproj_px':row['reproj_px'],'parallax_deg':row['parallax_deg'],
                     'compute_image_ms':row['compute_image_ms'],
                     'keyframes':row['keyframes'],'pos_sigma_m':row['pos_sigma_m'],
                     'innovation_nis':row['innovation_nis']})
        # Exclude raw time series from compact metrics, retain in dedicated JSON.
        ref=result['methods'][name]['relative_lidar_proxy']
        if ref.get('time_samples_s'):
            (out/(name+'_proxy_errors.json')).write_text(json.dumps({'seconds':ref.pop('time_samples_s'),
                 'errors_m':ref.pop('position_error_samples_m')},indent=2)+'\n')
    (out/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'status.txt').write_text('COMPLETE_OFFLINE_NO_POSE_AUTHORITY\n')
    print('COMPLETED_BENCHMARK',json.dumps({'mode':args.mode,
      'wall_sec':estimate_wall,'cpu_cores':result['average_process_cpu_cores'],
      'peak_mib':peak_rss_mib,'methods':result['methods']},indent=2),flush=True)


if __name__=='__main__':main()
