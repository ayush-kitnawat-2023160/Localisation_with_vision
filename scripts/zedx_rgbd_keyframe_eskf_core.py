#!/usr/bin/env python3
"""Experimental ZED RGB-D KLT persistent-keyframe + clone ESKF (OFFLINE).

21-dimensional error-state: [position, velocity, rotation, gyro bias,
accelerometer bias, keyframe position, keyframe rotation]. IMU propagates the
15-state nominal inertial model; each keyframe-to-camera PnP is used as a
*relative pose measurement* against the retained 6DoF cloned keyframe,
including cross-covariances. No gyro blending inside PnP (avoid information
double-counting), no LiDAR/GPS use. Filter covariance remains ESTIMATED only:
noise values, times, physical camera/IMU extrinsics are provisional.

Translation propagation on missing visual updates is NOT safe local odometry.
Untrusted states are explicitly marked, and the module never publishes TF,
vehicle commands or global /pose.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
import cv2

from zedx_rgbd_imu_odom_core import pose,project_rigid,rotation_compensated_parallax_deg


def skew(v):
    x,y,z=np.asarray(v,float).reshape(3)
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]],float)


def exp_so3(v):
    R,_=cv2.Rodrigues(np.asarray(v,float).reshape(3))
    return R


def log_so3(R):
    v,_=cv2.Rodrigues(np.asarray(R,float))
    return v[:,0]


def stabilize_rotation(R):
    U,_,V=np.linalg.svd(R)
    out=U@V
    if np.linalg.det(out)<0:U[:,-1]*=-1;out=U@V
    return out


@dataclass
class KFFrameResult:
    stamp:float
    state:str
    transform:np.ndarray
    tracks:int=0
    inliers:int=0
    reproj_px:float=float('inf')
    parallax_deg:float=float('nan')
    keyframes:int=0
    innovation_nis:float=float('nan')
    uncertainty_pos_sigma_m:float=float('inf')
    body_speed_mps:float=0.
    propagation_gap_s:float=0.
    imu_gyro_used:int=0
    imu_accel_used:int=0


class CloneESKF:
    """15 nominal / 21 error-state inertial EKF with a pose clone.

    Conventions: R maps optical-camera/IMU-aligned coordinates to initial
    camera-world coordinates, with right-multiplicative small-angle errors.
    Input IMU acceleration is *specific force* in optical coordinates.
    Gravity direction is bootstrapped from a verified-looking stationary
    initial IMU segment; if unavailable, initializer remains UNTRUSTED.
    """
    def __init__(self, R_opt_imu, *, sigma_gyro=.018,
                 sigma_accel=.18,sigma_bias_g=.0007,
                 sigma_bias_a=.008):
        self.R_opt_imu=np.asarray(R_opt_imu,float).reshape(3,3)
        self.p=np.zeros(3)
        self.v=np.zeros(3)
        self.R=np.eye(3)
        self.bg=np.zeros(3)
        self.ba=np.zeros(3)
        self.pk=np.zeros(3)
        self.Rk=np.eye(3)
        self.P=np.zeros((21,21))
        np.fill_diagonal(self.P[:15,:15],
            [.025]*3 + [.12]*3 + [.02]*3 + [4e-6]*3 + [.0025]*3)
        self.P[15:,15:]=np.eye(6)*1e-3
        self.gravity=np.zeros(3)
        self.initialized=False
        self.last_t=None
        self.last_measurement=None
        self.last_dt=0.
        self.samples=[]
        self.sigma_gyro=sigma_gyro
        self.sigma_accel=sigma_accel
        self.sigma_bias_g=sigma_bias_g
        self.sigma_bias_a=sigma_bias_a
        self.n_imuprop=0
        self.failed_gap=False
        self.last_imu=None
        self.initial_accel_std=float('nan')
        self.initial_gyro_mean=float('nan')
        self.gravity_bootstrapped_at=None

    def add_imu(self,t,omega,accel):
        omega=np.asarray(omega,float).reshape(3)
        accel=np.asarray(accel,float).reshape(3)
        if not np.isfinite(t) or not np.isfinite(omega).all() or not np.isfinite(accel).all():return False
        if np.linalg.norm(omega)>10 or np.linalg.norm(accel)>45:return False
        if self.last_imu is not None and t<=self.last_imu[0]:return False
        om=self.R_opt_imu@omega
        ac=self.R_opt_imu@accel
        self.last_imu=(float(t),om,ac)
        if not self.initialized:
            self.samples.append((float(t),om.copy(),ac.copy()))
            # stationary initialization uses 0.6s, at least 75 readings.
            if len(self.samples)>=75 and self.samples[-1][0]-self.samples[0][0]>=.55:
                O=np.stack([x[1] for x in self.samples])
                A=np.stack([x[2] for x in self.samples])
                self.initial_accel_std=float(np.linalg.norm(A.std(axis=0)))
                self.initial_gyro_mean=float(np.linalg.norm(O.mean(axis=0)))
                # Accelerometer points up while stationary (specific force).
                if (self.initial_accel_std<=.28 and
                    self.initial_gyro_mean<=.10 and
                    8.8<=np.linalg.norm(A.mean(axis=0))<=10.8):
                    self.gravity=-9.81*A.mean(axis=0)/np.linalg.norm(A.mean(axis=0))
                    self.bg=np.mean(O,axis=0)
                    self.initialized=True
                    self.last_t=float(t)
                    self.gravity_bootstrapped_at=float(t)
                    self.samples=[]
                elif self.samples[-1][0]-self.samples[0][0]>=3.:
                    self.samples=self.samples[-200:]
            return False
        # Estimator processes image timestamp in sorted sensor order.
        if self.last_t is None:self.last_t=float(t);return True
        dt=t-self.last_t
        if dt<1e-8:return True
        if dt>.09:
            self.failed_gap=True
            self.last_t=float(t)
            return False
        self.propagate(dt,om,ac)
        self.last_t=float(t)
        self.n_imuprop+=1
        return True

    def propagate(self,dt,w_meas,a_meas):
        w=w_meas-self.bg
        a=a_meas-self.ba
        R0=self.R
        a_world=R0@a+self.gravity
        self.p=self.p+self.v*dt+0.5*a_world*dt*dt
        self.v=self.v+a_world*dt
        self.R=stabilize_rotation(R0@exp_so3(w*dt))
        self.last_measurement=(w_meas.copy(),a_meas.copy())
        self.last_dt=dt
        # Right-invariant error linearization, discrete first order.
        F=np.eye(21)
        F[0:3,3:6]=np.eye(3)*dt
        F[0:3,6:9]=-0.5*R0@skew(a)*dt*dt
        F[0:3,12:15]=-0.5*R0*dt*dt
        F[3:6,6:9]=-R0@skew(a)*dt
        F[3:6,12:15]=-R0*dt
        F[6:9,6:9]=np.eye(3)-skew(w)*dt
        F[6:9,9:12]=-np.eye(3)*dt
        Q=np.zeros((21,21))
        # Continuous-time white noises discretized for each IMU interval.
        na=self.sigma_accel**2
        ng=self.sigma_gyro**2
        Q[3:6,3:6]=np.eye(3)*(na*dt)
        Q[0:3,0:3]=np.eye(3)*(na*dt**3/3.)
        Q[0:3,3:6]=np.eye(3)*(na*dt*dt/2.)
        Q[3:6,0:3]=Q[0:3,3:6]
        Q[6:9,6:9]=np.eye(3)*(ng*dt)
        Q[9:12,9:12]=np.eye(3)*(self.sigma_bias_g**2*dt)
        Q[12:15,12:15]=np.eye(3)*(self.sigma_bias_a**2*dt)
        self.P=F@self.P@F.T+Q
        self.P=(self.P+self.P.T)*.5

    def clone_current_pose(self):
        self.pk=self.p.copy()
        self.Rk=self.R.copy()
        old=self.P[:15,:15].copy()
        H=np.zeros((6,15))
        H[0:3,0:3]=np.eye(3)
        H[3:6,6:9]=np.eye(3)
        self.P[:15,15:21]=old@H.T
        self.P[15:21,:15]=H@old
        self.P[15:21,15:21]=H@old@H.T
        self.P=(self.P+self.P.T)*.5

    def relative_prediction(self):
        R_relative=self.R.T@self.Rk
        t_relative=self.R.T@(self.pk-self.p)
        return R_relative,t_relative

    def visual_update(self,reg,inliers,reproj_px,parallax_deg):
        # PnP returns T_current_key; use its 6D *relative* innovation.
        if not self.initialized:return False,float('nan')
        R_pred,t_pred=self.relative_prediction()
        residual=np.r_[reg[:3,3]-t_pred,
                       log_so3(R_pred.T@reg[:3,:3])]
        H=np.zeros((6,21))
        Rt=self.R.T
        H[0:3,0:3]=-Rt
        H[0:3,6:9]=skew(t_pred)
        H[0:3,15:18]=Rt
        # Prediction h(x)=Log(R_c^T R_key): this is dh/d(error), not
        # d(residual)/d(error). Residual uses R_pred^T R_measured, so
        # dh/d(theta_current)=-R_pred^T; dh/d(theta_key)=+I.
        H[3:6,6:9]=-R_pred.T
        H[3:6,18:21]=np.eye(3)
        # Model keyframe-to-frame depth and landmark uncertainty. This is
        # deliberately conservative and uncalibrated, not a final covariance.
        quality=max(.25,min(1.,inliers/100.))
        uncertainty=2. if not np.isfinite(parallax_deg) or parallax_deg<.08 else 1.
        sigma_t=np.clip((.12+.025*min(float(reproj_px),3.))/math.sqrt(quality)*uncertainty,.12,.65)
        sigma_r=np.clip((.032+.005*min(float(reproj_px),3.))/math.sqrt(quality)*uncertainty,.03,.18)
        cov=np.diag([sigma_t*sigma_t]*3+[sigma_r*sigma_r]*3)
        S=H@self.P@H.T+cov
        try:
            solved=np.linalg.solve(S,residual)
            nis=float(residual@solved)
            # Chi-square 6d 99.9%:22.46. Conservative outlier rejection;
            # if rejected, do NOT label candidate healthy or reset covariance.
            if not np.isfinite(nis) or nis>22.46:
                return False,nis
            K=np.linalg.solve(S,(self.P@H.T).T).T
            delta=K@residual
            if np.linalg.norm(delta[0:3])>2 or np.linalg.norm(delta[6:9])>.65:
                return False,nis
            self.p+=delta[0:3]
            self.v+=delta[3:6]
            self.R=stabilize_rotation(self.R@exp_so3(delta[6:9]))
            self.bg+=delta[9:12]
            self.ba+=delta[12:15]
            self.pk+=delta[15:18]
            self.Rk=stabilize_rotation(self.Rk@exp_so3(delta[18:21]))
            I=np.eye(21)
            A=I-K@H
            self.P=A@self.P@A.T+K@cov@K.T
            self.P=(self.P+self.P.T)*.5
            # Orientation right-error reset Jacobian; Joseph update above.
            G=np.eye(21)
            G[6:9,6:9]-=.5*skew(delta[6:9])
            G[18:21,18:21]-=.5*skew(delta[18:21])
            self.P=G@self.P@G.T
            self.P=(self.P+self.P.T)*.5
            return True,nis
        except np.linalg.LinAlgError:
            return False,float('nan')

    def covariance_ok(self):
        return (np.isfinite(self.P).all() and
                np.min(np.linalg.eigvalsh(self.P))>-1e-8 and
                np.max(np.diag(self.P))<1000.)


class ZedKeyframeESKFOdometry:
    def __init__(self,K,R_opt_imu,*,scale=1.,track_points=650,
                 keyframe_translation_m=.45,keyframe_rotation_deg=10.,
                 keyframe_max_interval_s=2.0,max_frame_gap=.30,
                 min_inliers=25,max_linear_mps=8.0):
        self.K=np.asarray(K,dtype=float).reshape(3,3)
        self.filter=CloneESKF(R_opt_imu)
        self.track_points=track_points
        self.max_frame_gap=max_frame_gap
        self.min_inliers=min_inliers
        self.max_linear_mps=max_linear_mps
        self.keyframe_translation_m=keyframe_translation_m
        self.keyframe_rotation_rad=math.radians(keyframe_rotation_deg)
        self.keyframe_max_interval_s=keyframe_max_interval_s
        self.key_gray=None
        self.key_depth=None
        self.key_uv=None
        self.key_xyz=None
        self.key_stamp=None
        self.key_initial_tracks=0
        self.prev_gray=None
        self.prev_uv=None
        self.prev_stamp=None
        self.last_pose=None
        self.frames=0
        self.accepted=0
        self.rejected=0
        self.keyframe_count=0
        self.visual_nis=[]
        self.health='BOOTSTRAP'
        self.last_visual_stamp=None
        self.last_reseed_stamp=-1e30
        self.reacquire_hits=0
        self.external_rearm_required=False

    def add_imu(self,stamp,omega_imu,accel_imu):
        return self.filter.add_imu(stamp,omega_imu,accel_imu)

    def to_transform(self):return pose(self.filter.R,self.filter.p)

    def new_keyframe(self,gray,depth,stamp):
        pts=cv2.goodFeaturesToTrack(gray,maxCorners=self.track_points,
            qualityLevel=.012,minDistance=6,blockSize=5,useHarrisDetector=False)
        if pts is None:return False
        xy=pts.reshape(-1,2)
        valid=(xy[:,0]>=4)&(xy[:,1]>=4)&(xy[:,0]<depth.shape[1]-4)&(xy[:,1]<depth.shape[0]-4)
        uv=xy[valid].copy()
        ix=np.rint(uv[:,0]).astype(int);iy=np.rint(uv[:,1]).astype(int)
        z=depth[iy,ix]
        with np.errstate(invalid='ignore'):
            stable=np.isfinite(z)&(z>=.65)&(z<=24.)
        uv=uv[stable];z=z[stable]
        if len(uv)<self.min_inliers*2:return False
        xyz=np.column_stack(((uv[:,0]-self.K[0,2])*z/self.K[0,0],
                              (uv[:,1]-self.K[1,2])*z/self.K[1,1],z))
        self.key_gray=gray.copy()
        self.key_depth=depth.copy()
        self.key_uv=uv.astype(np.float32)
        self.key_xyz=xyz.astype(float)
        self.key_initial_tracks=len(uv)
        self.key_stamp=float(stamp)
        self.prev_gray=gray.copy()
        self.prev_uv=uv.astype(np.float32)
        self.prev_stamp=float(stamp)
        self.filter.clone_current_pose()
        self.keyframe_count+=1
        return True

    def _result(self,t,state,inliers=0,tracks=0,reproj=float('inf'),
                para=float('nan'),nis=float('nan')):
        f=self.filter
        sig=float(np.sqrt(max(np.max(np.linalg.eigvalsh(f.P[:3,:3])),0)))
        return KFFrameResult(float(t),state,self.to_transform(),
            tracks=tracks,inliers=inliers,reproj_px=reproj,parallax_deg=para,
            keyframes=self.keyframe_count,innovation_nis=nis,
            uncertainty_pos_sigma_m=sig,body_speed_mps=float(np.linalg.norm(f.v)),
            imu_gyro_used=f.n_imuprop,imu_accel_used=f.n_imuprop)

    def _reseed_on_visual_loss(self,gray,depth,stamp,reason,*,tracks=0):
        # Re-detect depth-valid points after an occlusion; do not silently
        # treat the inertial extrapolation as an authoritative new origin.
        # A long camera outage requires an EXTERNAL stationary re-anchor.
        self.rejected+=1
        self.reacquire_hits=0
        if self.last_visual_stamp is not None and stamp-self.last_visual_stamp>1.2:
            self.external_rearm_required=True
        if stamp-self.last_reseed_stamp>=.23:
            self.last_reseed_stamp=stamp
            if self.new_keyframe(gray,depth,stamp):
                self.health='UNTRUSTED_VISUAL_RESEED'
                return self._result(stamp,self.health,tracks=tracks)
        # Expired old tracks cannot be reused with a stale source image.
        self.prev_stamp=stamp
        self.prev_gray=gray
        self.prev_uv=np.empty((0,2),np.float32)
        self.health=reason
        return self._result(stamp,self.health,tracks=tracks)

    def process(self,stamp,bgr,depth):
        stamp=float(stamp)
        self.frames+=1
        if bgr is None or depth is None or bgr.shape[:2]!=depth.shape or not np.isfinite(stamp):
            raise ValueError('Invalid RGBD data')
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        depth=np.asarray(depth,np.float32)
        f=self.filter
        if not f.initialized:
            self.health='UNTRUSTED_INERTIAL_BOOTSTRAP'
            return self._result(stamp,self.health)
        if f.last_t is None or stamp-f.last_t>.05 or stamp<f.last_t-.05 or f.failed_gap:
            self.health='UNTRUSTED_IMU_TIME_GAP'
            self.rejected+=1
            return self._result(stamp,self.health)
        if self.key_gray is None:
            if not self.new_keyframe(gray,depth,stamp):
                self.health='UNTRUSTED_INITIAL_KF_FEATURES'
                return self._result(stamp,self.health)
            return self._result(stamp,'INITIALIZING_KEYFRAME')
        dt=stamp-self.prev_stamp
        if dt<=0 or dt>self.max_frame_gap:
            return self._reseed_on_visual_loss(
                gray,depth,stamp,'UNTRUSTED_IMAGE_GAP_RESTART_REQUIRED')
        if self.prev_uv is None or len(self.prev_uv)<self.min_inliers:
            return self._reseed_on_visual_loss(
                gray,depth,stamp,'UNTRUSTED_NOT_ENOUGH_PREVIOUS_TRACKS',
                tracks=0 if self.prev_uv is None else len(self.prev_uv))
        pnew,st,_=cv2.calcOpticalFlowPyrLK(self.prev_gray,gray,
            self.prev_uv.reshape(-1,1,2),None,winSize=(23,23),maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,25,.018))
        if pnew is None:
            return self._reseed_on_visual_loss(gray,depth,stamp,'UNTRUSTED_NO_OPTICAL_FLOW')
        pback,bs,_=cv2.calcOpticalFlowPyrLK(gray,self.prev_gray,pnew,None,
            winSize=(23,23),maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,25,.018))
        if pback is None:
            return self._reseed_on_visual_loss(gray,depth,stamp,'UNTRUSTED_REVERSE_OPTICAL_FLOW')
        new=pnew.reshape(-1,2)
        back=pback.reshape(-1,2)
        valid=(st[:,0]>0)&(bs[:,0]>0)&(np.linalg.norm(self.prev_uv-back,axis=1)<1.5)
        valid&=(new[:,0]>=1)&(new[:,1]>=1)&(new[:,0]<gray.shape[1]-2)&(new[:,1]<gray.shape[0]-2)
        keypts=self.key_uv[valid]
        objpts=self.key_xyz[valid]
        nowpts=new[valid]
        # Continue multi-frame IDs, and their original keyframe 3D anchors.
        self.key_uv=keypts
        self.key_xyz=objpts
        self.prev_uv=np.asarray(nowpts,np.float32)
        self.prev_gray=gray
        self.prev_stamp=stamp
        if len(keypts)<self.min_inliers:
            return self._reseed_on_visual_loss(
                gray,depth,stamp,'UNTRUSTED_TOO_FEW_TRACKS',tracks=len(keypts))
        reg,ids,err=project_rigid(objpts,nowpts,self.K,2.5)
        if (reg is None or len(ids)<self.min_inliers or len(ids)/len(keypts)<.42 or
            not np.isfinite(err) or err>2.5):
            self.health='UNTRUSTED_KEYFRAME_PNP'
            self.rejected+=1
            return self._result(stamp,self.health,tracks=len(keypts),inliers=len(ids),reproj=err)
        uv=nowpts[ids]
        if np.ptp(uv[:,0])<80 or np.ptp(uv[:,1])<55:
            self.health='UNTRUSTED_FEATURE_SPREAD'
            self.rejected+=1
            return self._result(stamp,self.health,tracks=len(keypts),inliers=len(ids),reproj=err)
        R_key_cur=reg[:3,:3]
        Rpred,t_pred=f.relative_prediction()
        t_cam_k=reg[:3,3]
        # PnP returns cur<-key; reject grossly infeasible geometric motion.
        t_relative=float(np.linalg.norm(t_cam_k))
        if t_relative>self.max_linear_mps*max(stamp-self.key_stamp,.1)*1.4+1.:
            self.health='UNTRUSTED_GEOMETRIC_SPEED'
            self.rejected+=1
            return self._result(stamp,self.health,tracks=len(keypts),inliers=len(ids),reproj=err)
        # Use measured relative rotation to de-rotate feature bearing parallax;
        # this visual-only diagnostic is NOT fused separately as gyro data.
        para=rotation_compensated_parallax_deg(keypts[ids],uv,self.K,R_key_cur)
        accepted,nis=f.visual_update(reg,len(ids),err,para)
        if not accepted:
            self.health='UNTRUSTED_ESKF_INNOVATION'
            self.rejected+=1
            return self._result(stamp,self.health,tracks=len(keypts),inliers=len(ids),reproj=err,para=para,nis=nis)
        self.visual_nis.append(nis)
        self.accepted+=1
        self.last_visual_stamp=stamp
        self.reacquire_hits+=1
        self.health=('KEYFRAME_ESKF_TRACKING' if not self.external_rearm_required
                     and self.reacquire_hits>=3 else
                     'UNTRUSTED_REACQUIRING_OR_EXTERNAL_REARM_REQUIRED')
        angle=float(np.linalg.norm(log_so3(R_key_cur)))
        # Compare surviving tracks against the number of depth-valid tracks
        # in THIS keyframe, not against maxCorners. Otherwise valid regions
        # with only ~250/650 depth-valid corners spuriously create a fresh
        # keyframe on every video frame, starving long feature tracks.
        tracked_fraction=len(keypts)/max(1,self.key_initial_tracks)
        if (stamp-self.key_stamp>=self.keyframe_max_interval_s or
            t_relative>=self.keyframe_translation_m or
            angle>=self.keyframe_rotation_rad or
            (tracked_fraction<.45 and len(keypts)<.8*self.key_initial_tracks)):
            self.new_keyframe(gray,depth,stamp)
        if not f.covariance_ok():
            self.health='UNTRUSTED_COVARIANCE_INVALID'
            self.rejected+=1
        return self._result(stamp,self.health,tracks=len(keypts),inliers=len(ids),reproj=err,para=para,nis=nis)
