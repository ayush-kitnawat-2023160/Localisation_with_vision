#!/usr/bin/env python3
"""Read-only LiDAR-loss KLT continuation in the OLD FAST local frame.

Requires independently observed source LiDAR timeout AND pre-failure overlap
with good FAST odom, healthy KLT+gyro and a global LiDAR health observation.
Never publishes '/aft_mapped_to_init', '/pose', TF, /initialpose or commands.
Not wheel-safe; old map anchor + new camera poses remain candidates only.
"""
import json
import math
import os
import sys
import threading
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf.transformations import quaternion_matrix,quaternion_from_matrix

sys.path.insert(0,os.path.dirname(__file__))
from deployment_klt_continuation_core import CandidateContinuation


def transform(m):
    q=m.pose.pose.orientation;p=m.pose.pose.position
    T=quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3]=[p.x,p.y,p.z]
    return T


class ShadowBridge:
    def __init__(self):
        if not bool(rospy.get_param('~shadow_only',True)):
            raise ValueError('This is NOT an authoritative odometry selector')
        self.lock=threading.RLock()
        self.core=CandidateContinuation(
            max_outage_sec=float(rospy.get_param('~max_outage_sec',15.)),
            seed_age_max=float(rospy.get_param('~max_seed_age_sec',.90)),
            pair_max_dt=float(rospy.get_param('~max_pair_skew_sec',.12)))
        self.last_state=''
        self.odom=rospy.Publisher('/fast_livo2/deployment/klt_continuation_candidate',Odometry,queue_size=12)
        self.status=rospy.Publisher('/fast_livo2/deployment/klt_continuation_status',String,queue_size=6,latch=True)
        self.subs=[
            rospy.Subscriber('/fast_livo2/input/lidar',PointCloud2,self.on_lidar,queue_size=2),
            rospy.Subscriber('/aft_mapped_to_init',Odometry,self.on_fast,queue_size=50),
            rospy.Subscriber('/fast_livo2/fallback/camera_imu_odom_candidate',Odometry,
                             self.on_camera,queue_size=50),
            rospy.Subscriber('/fast_livo2/fallback/camera_imu_status',String,self.on_camera_status,queue_size=10),
            rospy.Subscriber('/fast_livo2/deployment_health_status',String,self.on_health,queue_size=10),
        ]
        rospy.Timer(rospy.Duration(.10),self.watchdog)
        rospy.logwarn('[KLT CONTINUATION] CANDIDATE-ONLY, NO /POSE, NO TF, NO ACTUATION')

    def _state(self):
        text='state=%s count=%d rearm_latched=%s authority=FALSE' %(
            self.core.state,self.core.candidate_count,str(self.core.terminal).upper())
        if text!=self.last_state:
            self.status.publish(String(data=text))
            self.last_state=text

    def on_lidar(self,m):
        with self.lock:self.core.observe_lidar(m.header.stamp.to_sec())

    def on_camera_status(self,m):
        # Published after each candidate evaluation. Do not treat topic
        # presence alone as visual tracking success.
        now=rospy.Time.now().to_sec()
        with self.lock:
            self.core.camera_status(m.data.startswith('state=TRACKING_VISION_GYRO'),now)

    def on_health(self,m):
        try:
            v=json.loads(m.data)
            healthy=bool(v.get('lidar_global_pose_observed',False) and
                         v.get('visual_local_odom_observed',False) and
                         v.get('local_source')=='FAST_LIVO2_ODOM' and
                         not v.get('continuity_rearm_latched',True))
        except (TypeError,ValueError):healthy=False
        with self.lock:self.core.monitor_health(healthy)

    def on_fast(self,m):
        if m.header.frame_id not in ('camera_init','fast_livo2_odom','odom',''):
            # Frame ID may depend on the FAST fork; don't conflate map or
            # arbitrary external producer with the local FAST odom frame.
            rospy.logwarn_throttle(10,'Unexpected FAST odometry frame %s',m.header.frame_id)
            return
        with self.lock:self.core.add_fast(m.header.stamp.to_sec(),transform(m))

    def on_camera(self,m):
        if m.header.frame_id!='zedx_camera_imu_initial_body' or \
                m.child_frame_id!='zedx_body_candidate_not_base_link':return
        now=rospy.Time.now().to_sec()
        with self.lock:
            if now<=0:return
            self.core.tick(now)
            r=self.core.add_visual(m.header.stamp.to_sec(),transform(m))
            if r is None:return
            # Inflate source covariance; not an estimator-certified bound.
            out=Odometry()
            out.header.stamp=m.header.stamp
            out.header.frame_id='fast_livo2_local_frame_DIAGNOSTIC_ONLY'
            out.child_frame_id='zedx_body_candidate_NO_TF_NO_AUTHORITY'
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,r.transform[:3,3])
            q=quaternion_from_matrix(r.transform)
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
            out.pose.covariance=list(m.pose.covariance)
            out.twist.covariance=[10000. if i%7==0 else 0. for i in range(36)]
            age=max(0.,r.outage_age)
            for i in (0,7,14):
                out.pose.covariance[i]=max(float(out.pose.covariance[i]),.25+.15*age)
            for i in (21,28,35):
                out.pose.covariance[i]=max(float(out.pose.covariance[i]),.1+.05*age)
            self.odom.publish(out)

    def watchdog(self,_):
        now=rospy.Time.now().to_sec()
        if now<=0:return
        with self.lock:
            self.core.tick(now)
            self._state()


if __name__=='__main__':
    rospy.init_node('deployment_klt_continuation_shadow')
    ShadowBridge()
    rospy.spin()
