#!/usr/bin/env python3
"""Pure fail-closed sensor-health rules for GPS-free cart SHADOW deployment.

No estimate fusion, motor enable, ROS pose, TF, or reset operation. Receipt
timestamps and measurement timestamps are independent. A status string can
never prove physical braking or authorize automatic localization re-arming.
"""
from dataclasses import dataclass,field
from typing import Dict,Optional
import math

MAX_AGE={
 'lidar':.75,'fast_imu':.35,'fast_odom':.75,
 'rgb':.80,'depth':.80,'zed_imu':.35,'visual_odom':.90,
 'lidar_map_pose':3.0,'visual_map_pose':3.0,
}


@dataclass
class Observation:
    stamp:float
    received:float
    n:int=1


@dataclass
class Snapshot:
    mode:str
    local_source:str
    lidar_global:bool
    visual_global:bool
    visual_odom:bool
    map_pose_trusted:bool
    continuity_latched:bool
    missing:tuple
    reasons:tuple
    cross_sensor_visual_agreement:bool=False
    authority_enabled:bool=False
    autonomous_motion_authorized:bool=False
    may_auto_reset_map_anchor:bool=False


@dataclass
class Health:
    max_age:dict=field(default_factory=lambda:dict(MAX_AGE))
    camera_depth_max_skew:float=.075
    max_future_stamp:float=.20
    max_receipt_age_multiplier:float=2.
    _data:Dict[str,Observation]=field(default_factory=dict)
    _status:str='SEARCHING'
    _status_seen:bool=False
    _visual_status:str=''
    _system_fault:Optional[str]=None
    _last_clock:Optional[float]=None
    _last_lidar_was_healthy:bool=False
    _lost_trust_latched:bool=False
    _last_local_source:str='NONE'
    _cross_visual_stamp:Optional[float]=None
    _cross_visual_received:Optional[float]=None

    def add(self,name,stamp,recv,clock):
        """Accept only strictly ordered, finite, non-future source stamps.

        For test cases and real ROS, recv is monotonic clock seconds and
        clock is ROS wall or bag /clock. No guessing from topic existence.
        """
        if name not in self.max_age:return False
        if not all(math.isfinite(x) for x in (stamp,recv,clock)) or stamp<=0 or clock<=0:
            return False
        if self._last_clock is not None and clock<self._last_clock-.20:
            self._system_fault='ROS_CLOCK_ROLLBACK_RESTART_REQUIRED'
            self._lost_trust_latched=True
        self._last_clock=clock
        if stamp>clock+self.max_future_stamp:return False
        old=self._data.get(name)
        if old and (stamp<=old.stamp or recv<old.received):return False
        self._data[name]=Observation(stamp,recv,(old.n+1) if old else 1)
        return True

    def set_step3_status(self,s):
        if not isinstance(s,str) or len(s)>512:return
        self._status=s
        self._status_seen=True
        if s not in ('GLOBAL_LOCKED','GLOBAL_REACQUIRED_SMOOTH'):
            if self._last_lidar_was_healthy:
                self._lost_trust_latched=True

    def set_visual_odom_status(self,text):
        if isinstance(text,str):
            self._visual_status=text[:512]
            if ('REARM_REQUIRED' in text or
                'UNTRUSTED_CAMERA_TIMEOUT' in text) and \
                self._last_local_source=='KLT_RGBD_ODOM_CANDIDATE_ONLY':
                self._lost_trust_latched=True

    def mark_external_discontinuity(self,reason):
        self._system_fault=reason
        self._lost_trust_latched=True

    def confirm_visual_lidar_agreement(self,stamp,received):
        # This only records a NEW cross-sensor observation; mere arrival of
        # XFeat candidate cannot establish global map trust.
        if (not all(math.isfinite(x) for x in (stamp,received)) or stamp<=0):
            return False
        if self._cross_visual_stamp is not None and stamp<=self._cross_visual_stamp:
            return False
        self._cross_visual_stamp=stamp
        self._cross_visual_received=received
        return True

    def invalidate_visual_lidar_agreement(self):
        self._cross_visual_stamp=None
        self._cross_visual_received=None

    def healthy(self,name,now,recv_now):
        x=self._data.get(name)
        if x is None or name not in self.max_age:return False
        age=now-x.stamp
        delivery=recv_now-x.received
        return (math.isfinite(age) and -.12<=age<=self.max_age[name] and
                math.isfinite(delivery) and 0<=delivery<=
                    self.max_age[name]*self.max_receipt_age_multiplier)

    def evaluate(self,now,recv_now):
        if not all(math.isfinite(x) and x>0 for x in (now,recv_now)):
            return Snapshot('STOP_REQUIRED','NONE',False,False,False,False,
                True,(),('CLOCK_UNAVAILABLE',))
        if self._last_clock is not None and now<self._last_clock-.20:
            self._system_fault='ROS_CLOCK_ROLLBACK_RESTART_REQUIRED'
            self._lost_trust_latched=True
        present={name:self.healthy(name,now,recv_now) for name in self.max_age}
        reasons=[]
        missing=[name for name,isvalid in present.items() if not isvalid]
        # At least one image pair with near-identical measurement timestamps.
        rgb=self._data.get('rgb')
        dep=self._data.get('depth')
        rgb_depth_synced=(present['rgb'] and present['depth'] and rgb and dep and
            abs(rgb.stamp-dep.stamp)<=self.camera_depth_max_skew)
        if not rgb_depth_synced:reasons.append('RGB_DEPTH_OUT_OF_SYNC_OR_STALE')
        imu_lidar=present['fast_imu']
        fast_healthy=present['lidar'] and imu_lidar and present['fast_odom']
        camera_status=self._visual_status.startswith('state=TRACKING_VISION_GYRO')
        camera_good=(rgb_depth_synced and present['zed_imu'] and
           present['visual_odom'] and camera_status)
        lidar_good=(fast_healthy and present['lidar_map_pose'] and
            self._status_seen and
            self._status in ('GLOBAL_LOCKED','GLOBAL_REACQUIRED_SMOOTH'))
        # Visual-only relocalization is a hypothesis; no LiDAR verification
        # without an independently trusted current scan/map relationship.
        visual_candidate=bool(camera_good and present['visual_map_pose'])
        cross_ok=bool(visual_candidate and lidar_good and
            self._cross_visual_stamp is not None and
            -.12<=now-self._cross_visual_stamp<=2.5 and
            self._cross_visual_received is not None and
            0<=recv_now-self._cross_visual_received<=3.0)
        if visual_candidate and not cross_ok:
            reasons.append('VISUAL_GLOBAL_CANDIDATE_NOT_LIDAR_CROSSCHECKED')
        if self._last_lidar_was_healthy and not lidar_good:
            self._lost_trust_latched=True
            reasons.append('GLOBAL_LIDAR_TRUST_LOSS_REARM_REQUIRED')
        self._last_lidar_was_healthy=bool(lidar_good)
        if fast_healthy:source='FAST_LIVO2_ODOM'
        elif camera_good:source='KLT_RGBD_ODOM_CANDIDATE_ONLY'
        else:source='NONE'
        if self._last_local_source=='FAST_LIVO2_ODOM' and source!='FAST_LIVO2_ODOM':
            self._lost_trust_latched=True
            reasons.append('LOCAL_ODOM_SOURCE_CHANGE_REARM_REQUIRED')
        # The KLT fallback is a *candidate*, never an armed source. During
        # startup it typically appears before FAST-LIVO2 local odometry.
        # Promoting this observational label to "FAST" is NOT a real mux
        # event and must not permanently latch a false-positive handoff.
        # Actual loss of previously healthy FAST odometry IS still latched.
        self._last_local_source=source
        if not present['fast_imu'] and not present['zed_imu']:
            reasons.append('ALL_IMUS_UNAVAILABLE')
        if not fast_healthy:reasons.append('FAST_LIVO2_NOT_HEALTHY')
        if not camera_good:reasons.append('CAMERA_ODOMETRY_NOT_HEALTHY')
        if not lidar_good:reasons.append('GLOBAL_LIDAR_NOT_TRUSTED')
        if not visual_candidate:reasons.append('VISUAL_GLOBAL_CANDIDATE_NOT_VERIFIED')
        if self._system_fault:reasons.append(self._system_fault)
        if self._lost_trust_latched:reasons.append('SUPERVISED_STOP_AND_REARM_REQUIRED')
        if self._system_fault or self._lost_trust_latched:
            mode='STOP_REQUIRED_REARM_LATCHED'
        elif lidar_good and camera_good and cross_ok:
            mode='SHADOW_ALL_MODALITIES_CROSSCHECKED'
        elif lidar_good:
            mode='SHADOW_PRIMARY_LIDAR_GLOBAL'
        elif camera_good:
            mode='SHADOW_CAMERA_FALLBACK_CANDIDATE_ONLY'
        else:
            mode='STOP_REQUIRED_NO_TRUSTED_LOCALIZATION'
        return Snapshot(mode,source,bool(lidar_good),visual_candidate,
            bool(camera_good),bool(lidar_good and not self._lost_trust_latched),
            bool(self._lost_trust_latched),tuple(missing),tuple(sorted(set(reasons))),
            cross_sensor_visual_agreement=cross_ok)
