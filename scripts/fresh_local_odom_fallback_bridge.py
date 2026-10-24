#!/usr/bin/env python3
"""ROS1 experimental FAST-local-odom continuation *candidate* from RGB-D/IMU.

This does NOT write /aft_mapped_to_init, /pose, TF, map->odom, or control.
Offline-only. Requires an explicit simulated LiDAR-failure state, not inference
from continued FAST odometry (which may itself just be stale prediction).
Anchor is established from time-overlapping healthy FAST-LIVO2 odom and
vision-gyro local odom. On failure it publishes only a separate, fixed-frame
candidate. Returning LiDAR cannot auto-resume pose authority.
"""
import collections
import math
import threading
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Bool
from tf.transformations import quaternion_matrix,quaternion_from_matrix


def mat(m):
    q=m.pose.pose.orientation
    t=m.pose.pose.position
    T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[t.x,t.y,t.z]
    return T


class LocalOdomFailoverBridge:
    def __init__(self):
        self.lock=threading.RLock()
        self.offline_only=rospy.get_param('~offline_only',True)
        if not self.offline_only:raise RuntimeError('This local-odom bridge is OFFLINE ONLY')
        self.fast=collections.deque(maxlen=100)
        self.visual=collections.deque(maxlen=100)
        self.mode_received=False
        self.outage_active=False
        self.outage_ever=False
        self.terminal=False
        self.vision_status='INIT'
        self.last_fast_stamp=None
        self.anchor=None
        self.anchor_stamp=None
        self.last_pub=None
        self.published=0
        self.max_align_sec=float(rospy.get_param('~max_align_sec',.12))
        self.max_seed_age=float(rospy.get_param('~max_seed_age_sec',.6))
        self.max_output_gap=float(rospy.get_param('~max_output_gap_sec',.65))
        self.max_speed_mps=float(rospy.get_param('~max_candidate_speed_mps',9.))
        self.odom_pub=rospy.Publisher('/fast_livo2/fallback/seeded_local_odom_candidate',Odometry,queue_size=10)
        self.status_pub=rospy.Publisher('/fast_livo2/fallback/seeded_local_odom_status',String,queue_size=10,latch=True)
        rospy.Subscriber(rospy.get_param('~fast_odom_topic','/fallback_test/healthy_fast_odom'),Odometry,self.on_fast,queue_size=50)
        rospy.Subscriber('/fast_livo2/fallback/camera_imu_odom_candidate',Odometry,self.on_visual,queue_size=50)
        rospy.Subscriber('/fast_livo2/fallback/camera_imu_status',String,self.on_vision_status,queue_size=20)
        rospy.Subscriber(rospy.get_param('~lidar_failure_verified_topic','/fallback_test/lidar_failed_simulated'),Bool,self.on_lidar_failure,queue_size=10)
        rospy.Timer(rospy.Duration(.20),self.watchdog)
        self.report('WARMUP_NOT_TRUSTED')
        rospy.logwarn('[LOCAL_ODOM_BRIDGE] OFFLINE ONLY; no /pose or /aft_mapped_to_init; simulated failover only')

    def report(self,state):
        self.status_pub.publish(String(data='state=%s anchor_stamp=%s candidate_count=%d authority=FALSE offline_only=TRUE'%
                                   (state,str(self.anchor_stamp),self.published)))

    def on_vision_status(self,m):
        with self.lock:self.vision_status=m.data.split()[0]

    def _best_pair(self):
        pairs=[]
        for ts,Tf in reversed(self.fast):
            for tv,Tv in reversed(self.visual):
                dt=abs(ts-tv)
                if dt<=self.max_align_sec:
                    pairs.append((ts,tv,dt,Tf,Tv))
        if not pairs:return None
        # The most recent common measurement pair, not the smallest residual
        # from the entire run. Never fit to post-outage LIO predictions.
        return max(pairs,key=lambda z:(min(z[0],z[1]),-z[2]))

    def on_fast(self,m):
        stamp=m.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp<=0:return
        with self.lock:
            if self.terminal or self.outage_active:return
            if self.fast and stamp<=self.fast[-1][0]:return
            self.fast.append((stamp,mat(m)))
            self.last_fast_stamp=stamp

    def watchdog(self,_):
        # No image->pose callbacks arrive during camera loss. Explicitly
        # invalidate the candidate instead of leaving its last status latched
        # as if it were fresh. Never restart the same local-odom session.
        with self.lock:
            if self.terminal or not self.outage_active or self.last_pub is None:
                return
            now=rospy.Time.now().to_sec()
            if now>=1 and now-self.last_pub[0]>max(.95,self.max_output_gap+.25):
                self.terminal=True
                self.report('CAMERA_IMU_ODOM_STALE_REARM_REQUIRED')

    def on_lidar_failure(self,m):
        with self.lock:
            self.mode_received=True
            if self.terminal:return
            requested=bool(m.data)
            if requested and not self.outage_active:
                if self.outage_ever:
                    self.terminal=True;self.report('REARM_REQUIRED_REPEATED_OUTAGE');return
                self.outage_ever=True
                match=self._best_pair()
                if match is None:
                    self.terminal=True;self.report('NO_TIME_ALIGNED_HEALTHY_SEED');return
                ts,tv,err,Tf,Tv=match
                now=rospy.Time.now().to_sec()
                if not 0<=now-ts<=self.max_seed_age or not 0<=now-tv<=self.max_seed_age:
                    self.terminal=True;self.report('STALE_PRE_OUTAGE_SEED');return
                self.anchor=Tf@np.linalg.inv(Tv)
                self.anchor_stamp=min(ts,tv)
                self.outage_active=True
                self.last_pub=None
                self.report('OUTAGE_SEEDED_CANDIDATE_ONLY')
            elif not requested and self.outage_active:
                self.outage_active=False
                self.terminal=True
                self.report('LIDAR_RETURNED_REJOIN_REQUIRES_EXTERNAL_REVIEW')

    def on_visual(self,m):
        stamp=m.header.stamp.to_sec()
        with self.lock:
            if not math.isfinite(stamp) or stamp<=0:return
            T=mat(m)
            if not np.all(np.isfinite(T)):return
            if self.visual and stamp<=self.visual[-1][0]:return
            self.visual.append((stamp,T))
            if self.terminal or not self.outage_active or self.anchor is None:return
            if not self.vision_status.startswith('state=TRACKING_VISION_GYRO'):
                self.report('VISION_GYRO_UNTRUSTED_NO_OUTPUT')
                return
            if self.last_pub is not None and (stamp-self.last_pub[0]>self.max_output_gap or stamp<=self.last_pub[0]):
                self.terminal=True
                self.report('CANDIDATE_TRAJECTORY_GAP_REARM_REQUIRED')
                return
            now=rospy.Time.now().to_sec()
            if not (-.12<=now-stamp<=.65):
                self.report('STALE_VISION_CANDIDATE')
                return
            output=self.anchor@T
            if self.last_pub is not None:
                dt=stamp-self.last_pub[0]
                jump=np.linalg.norm(output[:3,3]-self.last_pub[1][:3,3])
                if jump>self.max_speed_mps*dt+.15:
                    self.terminal=True;self.report('IMPLAUSIBLE_CANDIDATE_STEP_REARM_REQUIRED');return
            result=Odometry();result.header.stamp=m.header.stamp
            result.header.frame_id='fast_livo2_odom_continuation_CANDIDATE_ONLY'
            result.child_frame_id='camera_imu_fallback_body_NOT_AUTHORITY'
            result.pose.pose.position.x,result.pose.pose.position.y,result.pose.pose.position.z=map(float,output[:3,3])
            result.pose.pose.orientation.x,result.pose.pose.orientation.y,result.pose.pose.orientation.z,result.pose.pose.orientation.w=map(float,quaternion_from_matrix(output))
            result.pose.covariance=list(m.pose.covariance)
            result.twist.covariance=list(m.twist.covariance)
            self.odom_pub.publish(result)
            self.last_pub=(stamp,output)
            self.published+=1
            if self.published%60==0:self.report('CANDIDATE_ODOM_DURING_VERIFIED_LIDAR_FAILURE')


if __name__=='__main__':
    rospy.init_node('fresh_local_odom_fallback_bridge')
    LocalOdomFailoverBridge()
    rospy.spin()
