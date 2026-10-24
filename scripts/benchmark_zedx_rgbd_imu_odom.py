#!/usr/bin/env python3
"""Benchmark local RGB-D + ZED gyro VO without using LiDAR/GPS for estimation.

LiDAR-only odometry bag is opened ONLY AFTER all VO has been estimated, for an
independent-but-not-ground-truth trajectory *comparison*. Position metrics are
relative to alignment at the simulated LiDAR dropout onset.
"""
import argparse,csv,json,time
from collections import OrderedDict
from pathlib import Path
import numpy as np,cv2,rosbag,yaml
from tf.transformations import quaternion_matrix,quaternion_from_matrix,quaternion_slerp
from zedx_rgbd_imu_odom_core import ZedRGBDIMUOdometry

def make_pose(msg):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    T=np.eye(4);T[:3,:3]=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3];T[:3,3]=[p.x,p.y,p.z]
    return T

def interp_pose(t,stamps,xyz,quats):
    j=int(np.searchsorted(stamps,t));j=max(1,min(j,len(stamps)-1))
    if t<stamps[0] or t>stamps[-1] or min(abs(stamps[j]-t),abs(stamps[j-1]-t))>.25:return None
    a,b=stamps[j-1:j+1];u=(t-a)/(b-a)
    R=quaternion_matrix(quaternion_slerp(quats[j-1],quats[j],u))[:3,:3]
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=xyz[j-1]*(1-u)+xyz[j]*u
    return T

def bgr(m):
    a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)
    if m.encoding=='bgra8':return cv2.cvtColor(a[:,:m.width*4].reshape(m.height,m.width,4),cv2.COLOR_BGRA2BGR)
    if m.encoding=='bgr8':return a[:,:m.width*3].reshape(m.height,m.width,3)
    if m.encoding=='rgb8':return cv2.cvtColor(a[:,:m.width*3].reshape(m.height,m.width,3),cv2.COLOR_RGB2BGR)
    raise ValueError('Unknown color encoding '+m.encoding)


def main():
    p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--reference-odom',required=True)
    p.add_argument('--calibration',required=True);p.add_argument('--out',required=True)
    p.add_argument('--frame-period',type=float,default=.13);p.add_argument('--scale',type=float,default=.65)
    p.add_argument('--outage-start',type=float,default=20.0);p.add_argument('--outage-end',type=float,default=55.0)
    p.add_argument('--imu-mode',choices=['gyro_fused','visual_only'],default='gyro_fused')
    args=p.parse_args();o=Path(args.out);o.mkdir(parents=True,exist_ok=True)
    if (o/'status.txt').exists():raise RuntimeError('Refusing to overwrite existing completed run')
    calib=yaml.safe_load(open(args.calibration))['laserMapping'];fx,fy,cx,cy=[float(calib[k]) for k in ('cam_fx','cam_fy','cam_cx','cam_cy')]
    K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],float);smallK=K.copy();smallK[:2,:]*=args.scale
    # Static ZEDX TF measured in the recording: left_cam_frame -> optical is
    # quaternion (0.5,-0.5,0.5,-0.5); left_cam_frame -> zedx_imu_link is
    # (-.0031,.0009,.0027,~1). This is explicitly NOT the unverified old cart
    # LiDAR/IMU transform. Re-check this mount before using on the robot.
    R_left_opt=quaternion_matrix([.5,-.5,.5,-.5])[:3,:3]
    R_left_imu=quaternion_matrix([-.0031,.0009,.0027,1.])[:3,:3]
    R_opt_imu=R_left_opt.T@R_left_imu
    odo=ZedRGBDIMUOdometry(smallK,R_opt_imu=R_opt_imu,
                           imu_fraction=.35 if args.imu_mode=='gyro_fused' else 0.,
                           track_points=650,min_inliers=20,min_inlier_fraction=.42)
    last=-1e30;first=None;depth_cache=OrderedDict();rgb_cache=OrderedDict();rows=[]
    counters={'imu_messages':0,'depth_messages':0,'rgb_messages':0,'paired_images':0,'processed':0,'unmatched_images':0}
    tstart=time.monotonic()
    def run(im,dep,t):
        nonlocal last,first
        if first is None:first=t
        if t-last<args.frame_period:return
        last=t
        color=bgr(im)
        depth=np.frombuffer(dep.data,np.float32).reshape(dep.height,dep.step//4)[:,:dep.width].copy()
        if args.scale!=1.0:
            color=cv2.resize(color,None,fx=args.scale,fy=args.scale,interpolation=cv2.INTER_AREA)
            depth=cv2.resize(depth,(color.shape[1],color.shape[0]),interpolation=cv2.INTER_NEAREST)
        wall0=time.perf_counter()
        step=odo.process(t,color,depth)
        process_ms=(time.perf_counter()-wall0)*1000.0
        T=step.transform;rr=quaternion_from_matrix(T)
        rows.append(dict(stamp=t,bag_sec=t-first,status=step.state,
                         x=T[0,3],y=T[1,3],z=T[2,3],qx=rr[0],qy=rr[1],qz=rr[2],qw=rr[3],
                         klt_tracks=step.tracks,depth_tracks=step.depth_tracks,inliers=step.inliers,
                         reproj_px=step.reproj_px,gyro_coverage=step.gyro_coverage,
                         position_step_m=step.position_step_m,duration_s=step.duration_s,processing_ms=process_ms))
        counters['processed']+=1
        if len(rows)%45==0:
            print('VISUAL_IMU',Path(args.bag).stem,len(rows),'bag',round(t-first,1),
                  'state',step.state,'inliers',step.inliers,'elapsed_wall',round(time.monotonic()-tstart,1),flush=True)
    # Parse bag in recorded receipt order. Do not read any reference odometry
    # until AFTER visual+IMU trajectory computation is fully complete.
    with rosbag.Bag(args.bag) as bag:
        for topic,m,_ in bag.read_messages(topics=['/zedx/zed_node/left/image_rect_color','/zedx/zed_node/depth/depth_registered','/zedx/zed_node/imu/data']):
            t=m.header.stamp.to_sec();key=round(t,5)
            if topic.endswith('/imu/data'):
                counters['imu_messages']+=1
                w=m.angular_velocity
                if args.imu_mode=='gyro_fused':odo.add_imu(t,[w.x,w.y,w.z])
            elif topic.endswith('/depth/depth_registered'):
                counters['depth_messages']+=1
                color=rgb_cache.pop(key,None)
                if color is not None:
                    counters['paired_images']+=1;run(color,m,t)
                else:depth_cache[key]=m
            else:
                counters['rgb_messages']+=1
                depth=depth_cache.pop(key,None)
                if depth is not None:
                    counters['paired_images']+=1;run(m,depth,t)
                else:rgb_cache[key]=m
            while len(depth_cache)>12:depth_cache.popitem(last=False)
            while len(rgb_cache)>12:
                rgb_cache.popitem(last=False);counters['unmatched_images']+=1
    with (o/'trajectory.csv').open('w',newline='') as f:
        wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
    # Load LiDAR-only reference *after* estimation. It is NOT surveyed truth.
    ts=[];xyz=[];q=[]
    with rosbag.Bag(args.reference_odom) as ref:
        for _,m,_ in ref.read_messages(topics=['/aft_mapped_to_init']):
            ts.append(m.header.stamp.to_sec());p=m.pose.pose.position;Q=m.pose.pose.orientation
            xyz.append([p.x,p.y,p.z]);q.append([Q.x,Q.y,Q.z,Q.w])
    ts=np.array(ts);xyz=np.array(xyz);q=np.array(q)
    times=np.array([x['stamp'] for x in rows]);Tvo=[];Tref=[];mask=[]
    for x in rows:
        Tr=interp_pose(x['stamp'],ts,xyz,q)
        if Tr is None:continue
        P=np.eye(4);P[:3,:3]=quaternion_matrix([x[k] for k in ('qx','qy','qz','qw')])[:3,:3];P[:3,3]=[x[k] for k in ('x','y','z')]
        Tvo.append(P);Tref.append(Tr);mask.append(x['bag_sec'])
    mask=np.array(mask);Tvo=np.stack(Tvo);Tref=np.stack(Tref)
    k=int(np.argmin(abs(mask-args.outage_start)))
    # Camera-relative VO coordinates must NEVER be compared directly to the
    # LiDAR body's odom frame. The optical axes are rotated ~90 degrees and
    # a naive single-point SE(3) alignment gives spurious ~40m deviations.
    # Body<-LiDAR and camera<-LiDAR are inherited calibration assumptions;
    # their uncertainty is explicitly reported and requires measurement.
    extr=yaml.safe_load(open(args.calibration))['extrin_calib']
    T_body_lidar=np.eye(4);T_body_lidar[:3,:3]=np.asarray(extr['extrinsic_R']).reshape(3,3)
    T_body_lidar[:3,3]=extr['extrinsic_T']
    T_cam_lidar=np.eye(4);T_cam_lidar[:3,:3]=np.asarray(extr['Rcl']).reshape(3,3)
    T_cam_lidar[:3,3]=extr['Pcl']
    T_body_cam=T_body_lidar@np.linalg.inv(T_cam_lidar)
    T_align=Tref[k]@T_body_cam@np.linalg.inv(Tvo[k])
    aligned=T_align@Tvo@np.linalg.inv(T_body_cam)
    error=np.linalg.norm(aligned[:,:3,3]-Tref[:,:3,3],axis=1)
    idx=(mask>=args.outage_start)&(mask<=args.outage_end)
    outage_errors=error[idx]
    # RPE 1s/5s (relative transformed 3D displacement, not survey-accuracy).
    steps={}
    for window in (1.,5.,10.):
        v=[]
        for i,t in enumerate(mask):
            if not args.outage_start<=t<=args.outage_end-window:continue
            j=int(np.argmin(abs(mask-(t+window))))
            if abs(mask[j]-t-window)>.2:continue
            dvo=np.linalg.inv(aligned[i])@aligned[j]
            dref=np.linalg.inv(Tref[i])@Tref[j]
            v.append(np.linalg.norm(dvo[:3,3]-dref[:3,3]))
        steps[str(window)+'s_median_translation_rpe_m']=float(np.median(v)) if v else None
    mode=[x['status'] for x in rows];valid=np.array([s.startswith('VISION') for s in mode]);
    metrics=dict(bag=Path(args.bag).name,reference=Path(args.reference_odom).name,
       reference_ground_truth=False,reference_is_independent_survey=False,
       uses_lidar_for_estimation=False,uses_gps_for_estimation=False,
       uses_camera_depth_for_metric_scale=True,uses_imu_gyro_for_relative_rotation=args.imu_mode=='gyro_fused',
       zed_imu_camera_rotation_source='ZED tf_static from bag',
       starts_from_camera_relative_identity=True,simulated_outage_window_sec=[args.outage_start,args.outage_end],
       source_data=counters,frames_processed=len(rows),vision_good_count=int(valid.sum()),
       vision_good_fraction=float(valid.mean()),
       imu_fused_good_fraction=float(np.mean([s=='VISION_IMU_TRACKING' for s in mode])),
       accepted_vo_frames=odo.accepted,rejected_vo_frames=odo.rejected,
       frame_latency_wall_median_ms=None,
       largest_local_step_m=float(max(x['position_step_m'] for x in rows)),
       outage_samples=int(idx.sum()),outage_end_error_to_lio_proxy_m=float(outage_errors[-1]) if len(outage_errors) else None,
       outage_position_error_vs_lio_proxy_rmse_m=float(np.sqrt(np.mean(outage_errors**2))) if len(outage_errors) else None,
       outage_position_error_vs_lio_proxy_p95_m=float(np.percentile(outage_errors,95)) if len(outage_errors) else None,
       processing_wall_sec=time.monotonic()-tstart,
       relative_errors=steps,estimator='KLT RGBD PnP + ZED X gyro short-increment fusion; no inertial bias/covariance optimizer',
       caveats=['LiDAR reference is itself an unsurveyed and drifting estimator',
          'Camera<->IMU extrinsics from TF must be independently checked on physical cart',
          'Map-body comparison uses inherited body<->camera extrinsics that need physical verification',
          'Camera/IMU drift and delayed image updates require careful time alignment',
          'Standalone candidate-only odometry; no FAST-LIVO2 state or map->odom authority changes',
          'Rejected visual frames hold old translation, NOT a reliable motion estimate'])
    (o/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,6))
    ax.plot(Tref[:,0,3],Tref[:,1,3],label='LiDAR-IMU local odom (proxy, not ground truth)',lw=1.6)
    ax.plot(aligned[:,0,3],aligned[:,1,3],label='RGB-D + ZED IMU candidate (aligned at loss onset)',lw=1.4)
    ax.scatter([Tref[k,0,3]],[Tref[k,1,3]],s=65,label='Simulated LiDAR outage onset',zorder=4)
    ax.axis('equal');ax.grid(alpha=.3);ax.legend();ax.set(xlabel='Local X (m)',ylabel='Local Y (m)',title='Independent RGB-D/IMU motion vs LiDAR-only odom proxy: '+Path(args.bag).stem)
    fig.tight_layout();fig.savefig(o/'xy_trajectory.png',dpi=155);plt.close(fig)
    print('COMPLETED',json.dumps({k:metrics[k] for k in ('frames_processed','vision_good_fraction','imu_fused_good_fraction','outage_end_error_to_lio_proxy_m','outage_position_error_vs_lio_proxy_rmse_m','processing_wall_sec')},indent=2),flush=True)
    (o/'status.txt').write_text('COMPLETE\n')
if __name__=='__main__':main()
