#!/usr/bin/env python3
"""Pure GPS-free local candidate bridge, NEVER a driving-odom selector.

Cross-aligns already-estimated KLT body trajectory with FAST-LIVO2 odometry
ONLY while LiDAR and both estimators are healthy, using causal paired stamps.
On independently OBSERVED LiDAR sensor timeout it can output a DIAGNOSTIC
candidate in the former FAST local frame. No /pose, map, TF, epoch, control.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


def validate(t,T):
    M=np.asarray(T,float)
    if not math.isfinite(t) or t<=0 or M.shape!=(4,4) or not np.isfinite(M).all():return False
    R=M[:3,:3]
    return (np.linalg.norm(M[3]-np.array([0,0,0,1]))<1e-7 and
            abs(np.linalg.det(R)-1.)<.01 and
            np.linalg.norm(R@R.T-np.eye(3))<.01)


def yaw(T):return math.atan2(T[1,0],T[0,0])


def angular_error(a,b):return abs(math.atan2(math.sin(a-b),math.cos(a-b)))


@dataclass
class LocalOutput:
    stamp:float
    transform:np.ndarray
    outage_age:float
    anchor_time:float
    max_pair_residual_m:float


class CandidateContinuation:
    def __init__(self, *, pair_max_dt=.12,seed_age_max=.85,
                 pair_min_count=5,pair_duration_min=.35,max_align_translation=.65,
                 max_align_yaw_deg=16.,max_camera_gap=.60,max_outage_sec=15.,
                 candidate_max_speed=8.,max_lidar_age=.75):
        self.pair_max_dt=pair_max_dt
        self.seed_age_max=seed_age_max
        self.pair_min_count=pair_min_count
        self.pair_duration_min=pair_duration_min
        self.max_align_translation=max_align_translation
        self.max_align_yaw=math.radians(max_align_yaw_deg)
        self.max_camera_gap=max_camera_gap
        self.max_outage_sec=max_outage_sec
        self.candidate_max_speed=candidate_max_speed
        self.max_lidar_age=max_lidar_age
        self.fast=deque(maxlen=12)
        self.visual=deque(maxlen=12)
        self.pairs=deque(maxlen=20)
        self.last_pair_signature=None
        self.last_lidar=None
        self.lidar_observed=False
        self.first_lidar_timeout=None
        self.camera_ok=False
        self.last_camera_status=None
        self.health_observed=False
        self.health_good=False
        self.external_failure_observed=False
        self.anchor=None
        self.anchor_stamp=None
        self.seed_max_residual=float('inf')
        self.active=False
        self.terminal=False
        self.outage_ever=False
        self.last_output=None
        self.last_clock=None
        self.state='WARMUP_NO_SEED'
        self.candidate_count=0

    def observe_lidar(self,t):
        if math.isfinite(t) and t>0 and (self.last_lidar is None or t>self.last_lidar):
            self.last_lidar=float(t)
            self.lidar_observed=True

    def camera_status(self,healthy,stamp):
        if math.isfinite(stamp) and stamp>0:
            self.camera_ok=bool(healthy)
            self.last_camera_status=float(stamp)
            if not healthy and self.active:self._terminate('CAMERA_HEALTH_LOST_REARM_REQUIRED')

    def monitor_health(self,primary_ok):
        self.health_observed=True
        self.health_good=bool(primary_ok)

    def add_fast(self,t,T):
        if self.active or self.terminal or not validate(t,T):return False
        if self.fast and t<=self.fast[-1][0]:return False
        self.fast.append((float(t),np.asarray(T,float).copy()))
        self._pair()
        return True

    def add_visual(self,t,T):
        if self.terminal or not validate(t,T):return None
        if self.visual and t<=self.visual[-1][0]:return None
        self.visual.append((float(t),np.asarray(T,float).copy()))
        if not self.active:
            self._pair()
            return None
        if not self.camera_ok or self.last_camera_status is None or t-self.last_camera_status>self.max_camera_gap:
            self._terminate('CAMERA_UNTRUSTED_REARM_REQUIRED')
            return None
        # Never infer LiDAR return from an image stamp: callback scheduling
        # can put a camera exposure ~100-300ms behind current ROS /clock.
        # tick(now), called before this update by the node, checks the actual
        # latest LiDAR measurement against the same ROS clock domain.
        if t-self.first_lidar_timeout>self.max_outage_sec:
            self._terminate('OUTAGE_DURATION_EXCEEDED_REARM_REQUIRED')
            return None
        if self.last_output and (t-self.last_output.stamp>self.max_camera_gap or t<=self.last_output.stamp):
            self._terminate('CAMERA_OUTPUT_GAP_REARM_REQUIRED')
            return None
        output=self.anchor@np.asarray(T,float)
        if self.last_output:
            step=float(np.linalg.norm(output[:3,3]-self.last_output.transform[:3,3]))
            dt=t-self.last_output.stamp
            if step>self.candidate_max_speed*dt+.15:
                self._terminate('IMPLAUSIBLE_CANDIDATE_MOTION_REARM_REQUIRED')
                return None
        result=LocalOutput(float(t),output,t-self.first_lidar_timeout,
                           self.anchor_stamp,self.seed_max_residual)
        self.last_output=result
        self.candidate_count+=1
        self.state='CAMERA_CONTINUATION_CANDIDATE_ONLY'
        return result

    def _pair(self):
        if not self.fast or not self.visual or not self.camera_ok or not self.health_good:
            return
        if self.last_lidar is None:return
        ft,TF=self.fast[-1]
        if ft-self.last_lidar>self.max_lidar_age:return
        closest=min(self.visual,key=lambda x:abs(x[0]-ft))
        vt,TV=closest
        if abs(vt-ft)>self.pair_max_dt:return
        signature=(ft,vt)
        if self.last_pair_signature==signature:return
        self.last_pair_signature=signature
        anchor=TF@np.linalg.inv(TV)
        self.pairs.append((min(ft,vt),anchor))
        self._maybe_seed()

    def _maybe_seed(self):
        if len(self.pairs)<self.pair_min_count:return
        window=list(self.pairs)[-self.pair_min_count:]
        if window[-1][0]-window[0][0]<self.pair_duration_min:return
        latest=window[-1][1]
        translation=[float(np.linalg.norm(p[:3,3]-latest[:3,3])) for _,p in window]
        rotation=[angular_error(yaw(p),yaw(latest)) for _,p in window]
        max_res=max(translation)
        if max_res>self.max_align_translation or max(rotation)>self.max_align_yaw:
            self.state='WARMUP_PAIR_MOTION_INCONSISTENT'
            return
        self.anchor=latest.copy()
        self.anchor_stamp=window[-1][0]
        self.seed_max_residual=max_res
        self.state='WARMUP_TIME_ALIGNED_SEED_CANDIDATE'

    def tick(self,now):
        if not math.isfinite(now) or now<=0:
            self._terminate('CLOCK_INVALID_REARM_REQUIRED');return self.state
        if self.last_clock is not None and now<self.last_clock-.15:
            self._terminate('CLOCK_ROLLBACK_REARM_REQUIRED');return self.state
        self.last_clock=now
        if self.terminal:return self.state
        if self.last_lidar is None:return self.state
        age=now-self.last_lidar
        if age<-.15:
            self._terminate('LIDAR_FUTURE_STAMP_REARM_REQUIRED');return self.state
        if self.active:
            if age<=self.max_lidar_age:self._terminate('LIDAR_RETURNED_MANUAL_REJOIN_REQUIRED')
            elif self.last_output and now-self.last_output.stamp>self.max_camera_gap+.15:
                self._terminate('CAMERA_TIMEOUT_REARM_REQUIRED')
            elif self.first_lidar_timeout is not None and now-self.first_lidar_timeout>self.max_outage_sec:
                self._terminate('OUTAGE_DURATION_EXCEEDED_REARM_REQUIRED')
            return self.state
        if age<=self.max_lidar_age:return self.state
        if self.outage_ever:
            self._terminate('SECOND_LIDAR_LOSS_REARM_REQUIRED');return self.state
        self.outage_ever=True
        self.external_failure_observed=True
        self.first_lidar_timeout=now
        if not self.health_observed or not self.camera_ok:
            self._terminate('NO_VERIFIED_VISUAL_STATUS_ON_LIDAR_LOSS');return self.state
        if self.anchor is None or now-self.anchor_stamp>self.seed_age_max:
            self._terminate('NO_FRESH_TIME_ALIGNED_PRE_OUTAGE_SEED');return self.state
        if not self.visual or now-self.visual[-1][0]>self.max_camera_gap:
            self._terminate('CAMERA_TOO_OLD_TO_CONTINUE');return self.state
        self.active=True
        self.state='OUTAGE_SEEDED_CANDIDATE_ONLY'
        return self.state

    def _terminate(self,reason):
        self.terminal=True
        self.active=False
        self.state=reason
