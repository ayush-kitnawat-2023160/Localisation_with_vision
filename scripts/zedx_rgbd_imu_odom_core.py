#!/usr/bin/env python3
"""GPS-/LiDAR-independent metric RGB-D + short-interval gyroscope odometry.

Outputs a CANDIDATE local camera trajectory, not a global pose or a full VIO
filter. Translation comes from stereo depth + tracked image features; short
incremental rotation incorporates ZED IMU gyro (calibrated IMU->optical rotation).
No accelerometer bias/gravity estimation, no IMU preintegration covariance, no
bundle adjustment, no loop closure, no real-world approval. An invalid visual
measurement does NOT propagate fictitious translation using stale velocity.
"""
from __future__ import annotations
from dataclasses import dataclass
from collections import deque
import math
import cv2
import numpy as np


def pose(R=None, t=None):
    P=np.eye(4,dtype=np.float64)
    if R is not None:P[:3,:3]=np.asarray(R,dtype=np.float64).reshape(3,3)
    if t is not None:P[:3,3]=np.asarray(t,dtype=np.float64).reshape(3)
    return P


def project_rigid(xyz_src, uv_dst, K, reproj=2.5, iterations=100):
    """Solve T_dst_src, with 3D in old camera / 2D in new camera."""
    if len(xyz_src)<12:return None, np.array([],dtype=int), float('inf')
    ok,rvec,tvec,inliers=cv2.solvePnPRansac(
        np.ascontiguousarray(xyz_src,dtype=np.float64),
        np.ascontiguousarray(uv_dst,dtype=np.float64),K,None,
        iterationsCount=iterations,reprojectionError=reproj,confidence=.995,
        flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None:return None,np.array([],dtype=int),float('inf')
    ids=inliers.reshape(-1)
    if len(ids)<12:return None,ids,float('inf')
    ok,rvec,tvec=cv2.solvePnP(
        np.ascontiguousarray(xyz_src[ids],dtype=np.float64),
        np.ascontiguousarray(uv_dst[ids],dtype=np.float64),K,None,
        rvec,tvec,True,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:return None,ids,float('inf')
    R,_=cv2.Rodrigues(rvec)
    pred,_=cv2.projectPoints(xyz_src[ids],rvec,tvec,K,None)
    errors=np.linalg.norm(pred.reshape(-1,2)-uv_dst[ids],axis=1)
    return pose(R,tvec[:,0]),ids,float(np.median(errors))


def rotation_compensated_parallax_deg(uv_old,uv_new,K,R_old_to_new):
    """Median 2-view bearing parallax after removing camera rotation.

    Pure *diagnostic*, not metric VO or evidence that the cart has stopped.
    Slow forward motion / distant road features can produce tiny parallax even
    when the cart is moving; do not hard-reject otherwise valid visual odometry.
    """
    old=np.asarray(uv_old,dtype=np.float64).reshape(-1,2)
    new=np.asarray(uv_new,dtype=np.float64).reshape(-1,2)
    if len(old)==0 or old.shape!=new.shape:return float('nan')
    K=np.asarray(K,dtype=np.float64).reshape(3,3)
    R=np.asarray(R_old_to_new,dtype=np.float64).reshape(3,3)
    if not np.all(np.isfinite(old)) or not np.all(np.isfinite(new)) or not np.all(np.isfinite(R)):
        return float('nan')
    def rays(uv):
        b=np.column_stack(((uv[:,0]-K[0,2])/K[0,0],
                           (uv[:,1]-K[1,2])/K[1,1],np.ones(len(uv))))
        return b/np.linalg.norm(b,axis=1,keepdims=True)
    b0,b1=rays(old),rays(new)
    predicted=(R@b0.T).T
    return float(np.median(np.degrees(np.arccos(np.clip(np.sum(predicted*b1,axis=1),-1,1)))))


@dataclass
class FrameResult:
    stamp:float
    state:str
    transform:np.ndarray
    tracks:int=0
    depth_tracks:int=0
    inliers:int=0
    reproj_px:float=float('inf')
    gyro_coverage:float=0.0
    gyro_angle_rad:float=0.0
    position_step_m:float=0.0
    duration_s:float=0.0
    method:str='KLT_PNP_RGBD_GYRO'
    parallax_deg:float=float('nan')


class ZedRGBDIMUOdometry:
    def __init__(self,K,*,R_opt_imu=None,max_frame_gap=0.27,
                 min_depth=.65,max_depth=24.,max_linear_mps=8.,
                 min_inliers=20,min_inlier_fraction=.42,
                 max_reproj_px=2.5,imu_fraction=.35,gyro_gap=.045,
                 track_points=550,fast_motion=False):
        self.K=np.asarray(K,dtype=np.float64)
        if not np.all(np.isfinite(self.K)) or self.K[0,0]<=0 or self.K[1,1]<=0:raise ValueError('Invalid camera intrinsics')
        self.R_opt_imu=np.eye(3) if R_opt_imu is None else np.asarray(R_opt_imu,dtype=np.float64).reshape(3,3)
        if np.linalg.norm(self.R_opt_imu.T@self.R_opt_imu-np.eye(3))>.03:raise ValueError('Invalid IMU camera rotation')
        self.max_frame_gap=max_frame_gap;self.min_depth=min_depth;self.max_depth=max_depth
        self.max_linear_mps=max_linear_mps;self.min_inliers=min_inliers
        self.min_inlier_fraction=min_inlier_fraction;self.max_reproj_px=max_reproj_px
        self.imu_fraction=imu_fraction;self.gyro_gap=gyro_gap
        self.track_points=track_points
        self.prev_gray=None;self.prev_depth=None;self.prev_stamp=None
        self.T_odom_camera=np.eye(4)
        self.gyro=deque(maxlen=300)
        self.frames=0;self.accepted=0;self.rejected=0
        self.last_accepted_stamp=None
        self.fast_motion=fast_motion

    def add_imu(self,stamp,omega_imu):
        o=np.asarray(omega_imu,dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(o)) or np.linalg.norm(o)>10.:return
        if self.gyro and stamp<=self.gyro[-1][0]:return
        self.gyro.append((float(stamp),o))

    def gyro_increment(self,t0,t1):
        samples=[(t,o) for t,o in self.gyro if t0-self.gyro_gap<=t<=t1+self.gyro_gap]
        if len(samples)<2:return None,0.
        # Piecewise linear integration between endpoints, interpolating gyro
        # across the camera exposure time (not callback arrival time).
        times=np.array([s[0] for s in samples]);vec=np.array([s[1] for s in samples]);
        if times[0]>t0+self.gyro_gap or times[-1]<t1-self.gyro_gap:return None,0.
        ts=np.r_[t0,times[(times>t0)&(times<t1)],t1]
        omega=np.column_stack([np.interp(ts,times,vec[:,i]) for i in range(3)])
        integral=np.sum(0.5*(omega[1:]+omega[:-1])*np.diff(ts)[:,None],axis=0)
        camera_delta=self.R_opt_imu@integral
        coverage=max(0.,1.-max(0.,times[0]-t0,max(0.,t1-times[-1]))/max(.01,t1-t0))
        return camera_delta,float(coverage)

    def _candidate_points(self,gray,depth):
        # Depth-valid, spatially spread corners reduce dominance of sky/road.
        pts=cv2.goodFeaturesToTrack(gray,maxCorners=self.track_points,
               qualityLevel=.012,minDistance=6,blockSize=5,useHarrisDetector=False)
        if pts is None:return np.empty((0,2),np.float32)
        xy=pts.reshape(-1,2)
        ix=np.rint(xy[:,0]).astype(int);iy=np.rint(xy[:,1]).astype(int)
        good=(ix>=3)&(ix<depth.shape[1]-3)&(iy>=3)&(iy<depth.shape[0]-3)
        z=depth[np.clip(iy,0,depth.shape[0]-1),np.clip(ix,0,depth.shape[1]-1)]
        with np.errstate(invalid='ignore'):
            good &= np.isfinite(z)&(z>=self.min_depth)&(z<=self.max_depth)
        return np.ascontiguousarray(xy[good],dtype=np.float32)

    def process(self,stamp,bgr,depth):
        stamp=float(stamp)
        if (not math.isfinite(stamp) or bgr is None or depth is None or
            bgr.ndim!=3 or depth.ndim!=2 or bgr.shape[:2]!=depth.shape[:2]):
            raise ValueError('Invalid registered RGB-D image')
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        depth=np.asarray(depth,dtype=np.float32)
        self.frames+=1
        if self.prev_stamp is None:
            self.prev_gray=gray;self.prev_depth=depth;self.prev_stamp=stamp
            return FrameResult(stamp,'INITIALIZING',self.T_odom_camera.copy())
        dt=stamp-self.prev_stamp
        if dt<=0 or dt>self.max_frame_gap:
            self.prev_gray=gray;self.prev_depth=depth;self.prev_stamp=stamp
            self.rejected+=1
            return FrameResult(stamp,'UNTRUSTED_IMAGE_GAP',self.T_odom_camera.copy(),duration_s=dt)
        points=self._candidate_points(self.prev_gray,self.prev_depth)
        gyro,coverage=self.gyro_increment(self.prev_stamp,stamp)
        result=FrameResult(stamp,'UNTRUSTED_VISUAL',self.T_odom_camera.copy(),
                           tracks=len(points),duration_s=dt,
                           gyro_coverage=coverage)
        if len(points)<self.min_inliers:
            self._advance(gray,depth,stamp);self.rejected+=1;return result
        pnew,status,_=cv2.calcOpticalFlowPyrLK(
            self.prev_gray,gray,points.reshape(-1,1,2),None,
            winSize=(23,23),maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,25,.018))
        if pnew is None:
            self._advance(gray,depth,stamp);self.rejected+=1;return result
        pback,status2,_=cv2.calcOpticalFlowPyrLK(
            gray,self.prev_gray,pnew, None,
            winSize=(23,23),maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,25,.018))
        if pback is None:
            self._advance(gray,depth,stamp);self.rejected+=1;return result
        new=pnew.reshape(-1,2);back=pback.reshape(-1,2)
        valid=(status[:,0]!=0)&(status2[:,0]!=0)&(np.linalg.norm(back-points,axis=1)<1.5)
        valid &= (new[:,0]>=1)&(new[:,0]<gray.shape[1]-2)&(new[:,1]>=1)&(new[:,1]<gray.shape[0]-2)
        old=points[valid];uv=new[valid];result.depth_tracks=len(old)
        if len(old)<self.min_inliers:
            self._advance(gray,depth,stamp);self.rejected+=1;return result
        ix=np.clip(np.rint(old[:,0]).astype(int),0,depth.shape[1]-1)
        iy=np.clip(np.rint(old[:,1]).astype(int),0,depth.shape[0]-1)
        z=self.prev_depth[iy,ix].astype(np.float64)
        coords=np.column_stack(((old[:,0]-self.K[0,2])*z/self.K[0,0],
                                (old[:,1]-self.K[1,2])*z/self.K[1,1],z))
        reg,ids,err=project_rigid(coords,uv,self.K,self.max_reproj_px)
        result.inliers=len(ids);result.reproj_px=err
        if (reg is None or len(ids)<self.min_inliers or
            len(ids)/max(1,len(old))<self.min_inlier_fraction or err>self.max_reproj_px):
            self._advance(gray,depth,stamp);self.rejected+=1;return result
        # Optical flow degeneracy: require broad image coverage of inliers.
        q=uv[ids]
        if np.ptp(q[:,0])<80 or np.ptp(q[:,1])<55:
            self._advance(gray,depth,stamp);self.rejected+=1
            result.state='UNTRUSTED_POOR_FEATURE_DISTRIBUTION'
            return result
        # PnP gives T_current_previous. A camera odom step is its inverse.
        increment=np.linalg.inv(reg)
        trans=float(np.linalg.norm(increment[:3,3]))
        rvis,_=cv2.Rodrigues(increment[:3,:3]);rv=rvis.reshape(3)
        if trans/dt>self.max_linear_mps or np.linalg.norm(rv)/dt>2.4:
            self._advance(gray,depth,stamp);self.rejected+=1
            result.state='UNTRUSTED_IMPLAUSIBLE_MOTION';return result
        if gyro is not None:
            # Gyro orientation cannot establish metric scale; only the RGB-D
            # depth generates translation. Reject angular contradictions.
            disagreement=float(np.linalg.norm(rv-gyro))
            if disagreement>max(.12,1.75*dt):
                result.state='UNTRUSTED_CAMERA_IMU_CONFLICT'
                self._advance(gray,depth,stamp);self.rejected+=1;return result
            fused=(1.-self.imu_fraction)*rv+self.imu_fraction*gyro
            increment[:3,:3],_=cv2.Rodrigues(fused)
            result.gyro_angle_rad=float(np.linalg.norm(gyro))
            result.state='VISION_IMU_TRACKING'
        else:
            # Cannot label an RGB-only estimate as genuine VIO.
            result.state='VISION_ONLY_IMU_STALE'
        # Gyro-derotated translational parallax is extra observability
        # telemetry ONLY; never changes the accepted pose/state here.
        # PnP reg maps OLD optical rays into NEW optical coordinates.
        if gyro is not None:
            R_old_to_new,_=cv2.Rodrigues(-gyro)
        else:
            R_old_to_new=reg[:3,:3]
        result.parallax_deg=rotation_compensated_parallax_deg(
            old[ids],uv[ids],self.K,R_old_to_new)
        if not np.all(np.isfinite(increment)):
            self._advance(gray,depth,stamp);self.rejected+=1;return result
        self.T_odom_camera=self.T_odom_camera@increment
        self.accepted+=1;self.last_accepted_stamp=stamp
        result.transform=self.T_odom_camera.copy();result.position_step_m=trans
        self._advance(gray,depth,stamp)
        return result

    def _advance(self,gray,depth,stamp):
        self.prev_gray=gray;self.prev_depth=depth;self.prev_stamp=stamp
        while self.gyro and self.gyro[0][0]<stamp-1.2:self.gyro.popleft()
