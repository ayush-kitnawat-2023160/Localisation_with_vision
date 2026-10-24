#!/usr/bin/env python3
"""Offline, NO-LiDAR estimation benchmark: PnP+gyro vs depth-to-depth Kabsch.

Both methods use EXACTLY the same recorded ZED RGB, registered depth and gyro,
with identical frame selection. The LiDAR-only odometry bag is opened ONLY
AFTER both camera trajectories are estimated. The Kabsch method is a genuine
alternative: RANSAC matches 3D in two adjacent depth frames instead of PnP's
old 3D / new 2D correspondence model.

Also measures gyro de-rotated two-view bearing parallax. Parallax is a health
signal, not an independent scale-correct VO solution. Camera/body extrinsics
are inherited and NOT physically surveyed; metrics are proxy disagreement.
"""
import argparse
import csv
import json
import math
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
import rosbag
import yaml
from tf.transformations import quaternion_from_matrix,quaternion_matrix,quaternion_slerp

from zedx_rgbd_imu_odom_core import ZedRGBDIMUOdometry,pose


def as_bgr(m):
    raw=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)
    if m.encoding=='bgra8':return cv2.cvtColor(raw[:,:m.width*4].reshape(m.height,m.width,4),cv2.COLOR_BGRA2BGR)
    if m.encoding=='rgb8':return cv2.cvtColor(raw[:,:m.width*3].reshape(m.height,m.width,3),cv2.COLOR_RGB2BGR)
    if m.encoding=='bgr8':return raw[:,:m.width*3].reshape(m.height,m.width,3).copy()
    raise ValueError('Unknown RGB encoding '+m.encoding)


def xyz_of(uv,z,K):
    return np.stack(((uv[:,0]-K[0,2])*z/K[0,0],
                     (uv[:,1]-K[1,2])*z/K[1,1],z),axis=1)


def rigid_svd(old,new):
    if len(old)<3:return None
    a=old.mean(axis=0);b=new.mean(axis=0)
    u,s,vt=np.linalg.svd((old-a).T@(new-b),full_matrices=False)
    D=np.diag([1.,1.,np.linalg.det(vt.T@u.T)])
    R=vt.T@D@u.T
    return pose(R,b-R@a)


def three_d_ransac(old,new,z,*,iterations=72):
    if len(old)<16:return None,np.zeros(len(old),bool),float('inf')
    # ZED depth gets worse with distance: adaptive 3D residual threshold.
    thresholds=np.clip(.07+.014*np.maximum(z,1.),.085,.38)
    seed=int((len(old)*6361+np.rint(np.sum(old[:4,:])*100))%(2**32-1))
    rng=np.random.default_rng(seed)
    best=None;best_ids=np.zeros(len(old),bool);best_score=-1
    for _ in range(iterations):
        ids=rng.choice(len(old),3,replace=False)
        T=rigid_svd(old[ids],new[ids])
        if T is None:continue
        residual=np.linalg.norm((T[:3,:3]@old.T).T+T[:3,3]-new,axis=1)
        inlier=residual<thresholds
        score=int(inlier.sum())
        if score>best_score:
            best,best_ids,best_score=T,inlier,score
    if best_score<14:return None,best_ids,float('inf')
    T=rigid_svd(old[best_ids],new[best_ids])
    residual=np.linalg.norm((T[:3,:3]@old[best_ids].T).T+T[:3,3]-new[best_ids],axis=1)
    return T,best_ids,float(np.median(residual))


class Depth3DRigid:
    def __init__(self,K,R_opt_imu,*,parallax_gate_deg=0.):
        self.helper=ZedRGBDIMUOdometry(K,R_opt_imu=R_opt_imu,track_points=650)
        self.K=K
        self.prev=None
        self.stamp=None
        self.T=np.eye(4)
        self.good=0;self.rejected=0
        self.parallax_gate_deg=float(parallax_gate_deg)

    def add_imu(self,t,w):self.helper.add_imu(t,w)

    def process(self,t,bgr,depth):
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        result={'stamp':t,'state':'INITIALIZING','inliers':0,'parallax_deg':None,
                'near_depth_fraction':None,'depth_correspondences':0,
                'median_3d_residual_m':None,'raw_flow_px_median':None}
        if self.prev is None:
            self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        old_gray,old_depth=self.prev;dt=t-self.stamp
        if dt<=0 or dt>.27:
            result['state']='UNTRUSTED_GAP'
            self.rejected+=1;self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        old_uv=self.helper._candidate_points(old_gray,old_depth)
        if len(old_uv)<20:
            result['state']='UNTRUSTED_FEW_OLD_FEATURES'
            self.rejected+=1;self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        f,fs,_=cv2.calcOpticalFlowPyrLK(old_gray,gray,old_uv.reshape(-1,1,2),None,
              winSize=(23,23),maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,25,.018))
        if f is None:
            result['state']='UNTRUSTED_FLOW';self.rejected+=1
            self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        b,bs,_=cv2.calcOpticalFlowPyrLK(gray,old_gray,f,None,
              winSize=(23,23),maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,25,.018))
        if b is None:
            result['state']='UNTRUSTED_BACK_FLOW';self.rejected+=1
            self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        u0=old_uv;u1=f.reshape(-1,2);back=b.reshape(-1,2)
        mask=(fs[:,0]>0)&(bs[:,0]>0)&(np.linalg.norm(back-u0,axis=1)<1.5)
        mask&=(u1[:,0]>2)&(u1[:,0]<gray.shape[1]-3)&(u1[:,1]>2)&(u1[:,1]<gray.shape[0]-3)
        u0=u0[mask];u1=u1[mask]
        ix0=np.rint(u0[:,0]).astype(int);iy0=np.rint(u0[:,1]).astype(int)
        ix1=np.rint(u1[:,0]).astype(int);iy1=np.rint(u1[:,1]).astype(int)
        z0=old_depth[iy0,ix0];z1=depth[iy1,ix1]
        with np.errstate(invalid='ignore'):
            have=np.isfinite(z0)&np.isfinite(z1)&(z0>.65)&(z1>.65)&(z0<24.)&(z1<24.)
        u0=u0[have];u1=u1[have];z0=z0[have];z1=z1[have]
        result['depth_correspondences']=len(u0)
        result['near_depth_fraction']=float(np.mean(z0<8.)) if len(z0) else 0.
        if len(u0)<20:
            result['state']='UNTRUSTED_DEPTH_CORRESPONDENCES'
            self.rejected+=1;self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        old=xyz_of(u0,z0,self.K);new=xyz_of(u1,z1,self.K)
        result['raw_flow_px_median']=float(np.median(np.linalg.norm(u1-u0,axis=1)))
        T,ids,err=three_d_ransac(old,new,z0)
        result['inliers']=int(ids.sum());result['median_3d_residual_m']=err
        if T is None or ids.sum()<20 or ids.sum()/len(u0)<.42:
            result['state']='UNTRUSTED_3D_RANSAC'
            self.rejected+=1;self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        q=u1[ids]
        if np.ptp(q[:,0])<80 or np.ptp(q[:,1])<55:
            result['state']='UNTRUSTED_SMALL_FEATURE_SPREAD'
            self.rejected+=1;self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        increment=np.linalg.inv(T)
        vec_vis,_=cv2.Rodrigues(increment[:3,:3]);vec_vis=vec_vis.reshape(3)
        gyro,coverage=self.helper.gyro_increment(self.stamp,t)
        if gyro is not None:
            if np.linalg.norm(vec_vis-gyro)>max(.12,1.75*dt):
                result['state']='UNTRUSTED_GYRO_CONFLICT'
                self.rejected+=1;self.prev=(gray,depth.copy());self.stamp=t
                return self.T.copy(),result
            Rnewold,_=cv2.Rodrigues(gyro)
            # Rotation-compensated optical parallax between old/new rays.
            b0=np.column_stack(((u0[:,0]-self.K[0,2])/self.K[0,0],
                                  (u0[:,1]-self.K[1,2])/self.K[1,1],np.ones(len(u0))))
            b1=np.column_stack(((u1[:,0]-self.K[0,2])/self.K[0,0],
                                  (u1[:,1]-self.K[1,2])/self.K[1,1],np.ones(len(u1))))
            b0=b0/np.linalg.norm(b0,axis=1,keepdims=True)
            b1=b1/np.linalg.norm(b1,axis=1,keepdims=True)
            predicted=(Rnewold.T@b0.T).T
            angles=np.degrees(np.arccos(np.clip(np.sum(predicted*b1,axis=1),-1,1)))
            result['parallax_deg']=float(np.median(angles[ids]))
            if self.parallax_gate_deg>0 and result['parallax_deg']<self.parallax_gate_deg:
                result['state']='UNTRUSTED_LOW_TRANSLATION_PARALLAX'
                self.rejected+=1;self.prev=(gray,depth.copy());self.stamp=t
                return self.T.copy(),result
            fused=.65*vec_vis+.35*gyro
            increment[:3,:3],_=cv2.Rodrigues(fused)
            result['state']='TRACKING_DEPTH3D_GYRO'
        else:
            result['state']='TRACKING_DEPTH3D_VISUAL_ONLY'
        if np.linalg.norm(increment[:3,3])/dt>8. or np.linalg.norm(vec_vis)/dt>2.4:
            result['state']='UNTRUSTED_IMPLAUSIBLE_SPEED'
            self.rejected+=1;self.prev=(gray,depth.copy());self.stamp=t
            return self.T.copy(),result
        self.T=self.T@increment;self.good+=1
        self.prev=(gray,depth.copy());self.stamp=t
        return self.T.copy(),result


def interp(t,ts,xyz,quat):
    if t<ts[0] or t>ts[-1]:return None
    j=int(np.searchsorted(ts,t));j=max(1,min(j,len(ts)-1))
    if min(abs(t-ts[j-1]),abs(t-ts[j]))>.25:return None
    t0,t1=ts[j-1],ts[j];u=(t-t0)/(t1-t0)
    q=quaternion_slerp(quat[j-1],quat[j],u)
    return pose(quaternion_matrix(q)[:3,:3],xyz[j-1]*(1-u)+xyz[j]*u)


def compare(est,ts,xyz,quat,body_cam,first_time,outage):
    Tref=[];Tvo=[];seconds=[]
    for r in est:
        P=interp(r['stamp'],ts,xyz,quat)
        if P is None:continue
        Tref.append(P);Tvo.append(np.array(r['T'],float));seconds.append(r['stamp']-first_time)
    if len(Tref)<10:return {'error':'NOT_ENOUGH_MATCHES'}
    ref=np.stack(Tref);vo=np.stack(Tvo);seconds=np.array(seconds)
    k=int(np.argmin(abs(seconds-outage[0])))
    # T_body_cam is inherited: no independent physical verification.
    T_map_camera=ref[k]@body_cam@np.linalg.inv(vo[k])
    aligned=T_map_camera@vo@np.linalg.inv(body_cam)
    err=np.linalg.norm(aligned[:,:3,3]-ref[:,:3,3],axis=1)
    mask=(seconds>=outage[0])&(seconds<=outage[1])
    e=err[mask]
    if not len(e):return {'error':'NO_SAMPLES_IN_REQUESTED_OUTAGE_WINDOW',
                            'not_surveyed_ground_truth':True}
    return {'outage_sample_count':int(mask.sum()),'outage_rmse_m':float(np.sqrt(np.mean(e*e))),
            'outage_endpoint_disagreement_m':float(e[-1]),
            'outage_p95_disagreement_m':float(np.percentile(e,95)),
            'outage_peak_disagreement_m':float(e.max()),
            'not_surveyed_ground_truth':True}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--bag',required=True)
    p.add_argument('--reference',required=True)
    p.add_argument('--calibration',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--period',type=float,default=.13)
    p.add_argument('--scale',type=float,default=.65)
    p.add_argument('--duration',type=float,default=65.)
    p.add_argument('--parallax-gate-deg',type=float,default=0.)
    p.add_argument('--outage-start',type=float,default=20.)
    p.add_argument('--outage-end',type=float,default=55.)
    args=p.parse_args()
    out=Path(args.output)
    if out.exists():raise RuntimeError('Refusing overwrite '+str(out))
    out.mkdir(parents=True)
    config=yaml.safe_load(open(args.calibration))
    cam=config['laserMapping']
    K=np.array([[cam['cam_fx'],0,cam['cam_cx']],
                [0,cam['cam_fy'],cam['cam_cy']],[0,0,1]],float)
    K[:2,:]*=args.scale
    R_left_opt=quaternion_matrix([.5,-.5,.5,-.5])[:3,:3]
    R_left_imu=quaternion_matrix([-.0031,.0009,.0027,1.])[:3,:3]
    R_opt_imu=R_left_opt.T@R_left_imu
    base=ZedRGBDIMUOdometry(K,R_opt_imu=R_opt_imu,imu_fraction=.35,
             track_points=650,min_inliers=20,min_inlier_fraction=.42)
    alternative=Depth3DRigid(K,R_opt_imu,parallax_gate_deg=args.parallax_gate_deg)
    pending_rgb=OrderedDict();pending_depth=OrderedDict()
    counts={'rgb':0,'depth':0,'imu':0,'paired':0,'skipped':0}
    first=None;last=-1e9
    baseline=[];variant=[]
    wall=time.monotonic()
    def process(rgb,dep,t):
        nonlocal last
        if t-last<args.period:
            counts['skipped']+=1;return
        last=t
        color=as_bgr(rgb)
        z=np.frombuffer(dep.data,np.float32).reshape(dep.height,dep.step//4)[:,:dep.width].copy()
        if args.scale!=1.0:
            color=cv2.resize(color,None,fx=args.scale,fy=args.scale,interpolation=cv2.INTER_AREA)
            z=cv2.resize(z,(color.shape[1],color.shape[0]),interpolation=cv2.INTER_NEAREST)
        t0=time.perf_counter()
        rb=base.process(t,color,z)
        mb=(time.perf_counter()-t0)*1000
        t1=time.perf_counter()
        Tv,rv=alternative.process(t,color,z)
        mv=(time.perf_counter()-t1)*1000
        baseline.append({'stamp':t,'T':rb.transform.copy(),'state':rb.state,
                         'runtime_ms':mb,'tracks':rb.tracks,'inliers':rb.inliers,
                         'gyro_coverage':rb.gyro_coverage,'reproj_px':rb.reproj_px})
        variant.append({**rv,'T':Tv.copy(),'runtime_ms':mv})
        if len(baseline)%45==0:print('FRAMES',len(baseline),'sim_s',round(t-first,2),
            'baseline',rb.state,'alternative',rv['state'],flush=True)
    with rosbag.Bag(args.bag) as b:
        for topic,m,_ in b.read_messages(topics=['/zedx/zed_node/left/image_rect_color',
                 '/zedx/zed_node/depth/depth_registered','/zedx/zed_node/imu/data']):
            t=m.header.stamp.to_sec()
            if first is None:first=t
            if t-first>args.duration:break
            key=round(t,5)
            if topic.endswith('/imu/data'):
                counts['imu']+=1
                w=m.angular_velocity;vec=[w.x,w.y,w.z]
                base.add_imu(t,vec);alternative.add_imu(t,vec)
            elif topic.endswith('/depth/depth_registered'):
                counts['depth']+=1
                other=pending_rgb.pop(key,None)
                if other is None:pending_depth[key]=m
                else:counts['paired']+=1;process(other,m,t)
            else:
                counts['rgb']+=1
                other=pending_depth.pop(key,None)
                if other is None:pending_rgb[key]=m
                else:counts['paired']+=1;process(m,other,t)
            while len(pending_rgb)>12:pending_rgb.popitem(last=False)
            while len(pending_depth)>12:pending_depth.popitem(last=False)
    elapsed=time.monotonic()-wall
    # Reference bag deliberately read only AFTER camera-only estimates exist.
    ts=[];xyz=[];quat=[]
    with rosbag.Bag(args.reference) as b:
        for _,m,_ in b.read_messages(topics=['/aft_mapped_to_init']):
            P=m.pose.pose.position;Q=m.pose.pose.orientation
            ts.append(m.header.stamp.to_sec());xyz.append([P.x,P.y,P.z]);quat.append([Q.x,Q.y,Q.z,Q.w])
    ts=np.array(ts);xyz=np.array(xyz);quat=np.array(quat)
    ext=config['extrin_calib']
    T_body_lidar=pose(np.array(ext['extrinsic_R']).reshape(3,3),ext['extrinsic_T'])
    T_camera_lidar=pose(np.array(ext['Rcl']).reshape(3,3),ext['Pcl'])
    T_body_camera=T_body_lidar@np.linalg.inv(T_camera_lidar)
    def details(rows):
        processing=np.array([r['runtime_ms'] for r in rows])
        states={s:sum(r['state']==s for r in rows) for s in sorted({r['state'] for r in rows})}
        valid=[r for r in rows if r['state'].startswith(('VISION_','TRACKING_'))]
        outcome={'processed_frames':len(rows),'accepted_frames':len(valid),
           'accepted_fraction':len(valid)/len(rows),'statuses':states,
           'processing_ms_median':float(np.median(processing)),
           'processing_ms_p95':float(np.percentile(processing,95)),
           'processing_ms_mean':float(processing.mean()),
           'processing_ms_max':float(processing.max())}
        outcome.update(compare(rows,ts,xyz,quat,T_body_camera,first,
                              (args.outage_start,args.outage_end)))
        return outcome
    ref={
        'bag':Path(args.bag).name,'reference':Path(args.reference).name,
        'frame_period':args.period,'image_scale':args.scale,
        'camera_estimation_did_not_use_lidar_or_gps':True,
        'reference_is_not_surveyed_ground_truth':True,
        'physical_extrinsics_unverified':True,'wall_processing_sec_both':elapsed,
        'sources':counts,'baseline_pnp_gyro':details(baseline),
        'alternative_depth3d_ransac_gyro':details(variant),
        'parallax_gate_deg':args.parallax_gate_deg}
    pv=[r['parallax_deg'] for r in variant if r['parallax_deg'] is not None]
    if pv:
        v=np.array(pv)
        ref['parallax_deg_stats']={'count':len(v),'p10':float(np.percentile(v,10)),
            'median':float(np.median(v)),'p90':float(np.percentile(v,90)),
            'fraction_below_0p1deg':float(np.mean(v<.1)),
            'fraction_below_0p2deg':float(np.mean(v<.2)),
            'fraction_below_0p5deg':float(np.mean(v<.5)),
            'note':'Gyro-rotation-compensated bearing disparity, only on frames with enough depth and RANSAC inliers. Not independent global scale or ground truth.'}
    (out/'metrics.json').write_text(json.dumps(ref,indent=2)+'\n')
    with (out/'frames.csv').open('w',newline='') as f:
        fields=['stamp','sim_sec','pnp_state','pnp_ms','pnp_tracks','pnp_inliers',
                'rigid_state','rigid_ms','rigid_depth_tracks','rigid_inliers',
                'rigid_parallax_deg','rigid_median_3d_residual_m','raw_flow_px_median']
        wr=csv.DictWriter(f,fieldnames=fields);wr.writeheader()
        for r,s in zip(baseline,variant):
            wr.writerow({'stamp':r['stamp'],'sim_sec':r['stamp']-first,
               'pnp_state':r['state'],'pnp_ms':r['runtime_ms'],
               'pnp_tracks':r['tracks'],'pnp_inliers':r['inliers'],
               'rigid_state':s['state'],'rigid_ms':s['runtime_ms'],
               'rigid_depth_tracks':s['depth_correspondences'],
               'rigid_inliers':s['inliers'],'rigid_parallax_deg':s['parallax_deg'],
               'rigid_median_3d_residual_m':s['median_3d_residual_m'],
               'raw_flow_px_median':s['raw_flow_px_median']})
    print('BENCHMARK_COMPLETE',json.dumps({k:v for k,v in ref.items()
        if k in ('parallax_deg_stats','baseline_pnp_gyro','alternative_depth3d_ransac_gyro')},indent=2),flush=True)


if __name__=='__main__':main()
