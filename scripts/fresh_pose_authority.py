#!/usr/bin/env python3
"""Single map->odom authority. FAST-LIVO2 is the ONLY continuous ESIKF.

LiDAR reference: independently verified Step3 ScanContext+GICP map pose.
LiDAR outage: geometrically/temporally verified RGB-D visual map pose.
The output continues at local odometry cadence; anchor follows bounded steps.
No /pose output unless explicitly enabled after coordinate-frame validation.
"""
from collections import deque
import math
import threading

import numpy as np
import rospy
from tf.transformations import quaternion_from_matrix, quaternion_matrix, quaternion_slerp
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def pose_matrix(p):
    q = p.orientation
    T=quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3]=[p.position.x,p.position.y,p.position.z]
    return T


def yaw(T):
    return math.atan2(T[1,0],T[0,0])


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


class PoseAuthority:
    def __init__(self):
        self.lock=threading.RLock()
        self.history=deque(maxlen=250)
        self.anchor=None
        self.target=None
        self.mode="FULL_LIVO"
        self.source="UNLOCALIZED"
        self.last_map_fix=rospy.Time(0)
        self.last_pub=rospy.Time(0)
        self.max_fix_age=float(rospy.get_param("~max_fix_age_sec",.75))
        self.max_visual_innovation=float(rospy.get_param("~max_visual_innovation_m",4.0))
        self.max_visual_yaw=float(rospy.get_param("~max_visual_yaw_deg",20.0))
        self.max_lidar_innovation=float(rospy.get_param("~max_lidar_innovation_m",4.5))
        self.max_step_rate=float(rospy.get_param("~max_anchor_translation_rate_mps",1.5))
        self.max_yaw_rate=float(rospy.get_param("~max_anchor_yaw_rate_deg_s",12.0))
        self.allow_pose=bool(rospy.get_param("~publish_legacy_pose",False))
        # Opt-in for the LiDAR MH experiment. A dead-reckoned pose from an
        # old map anchor is NOT a trusted global pose; publishing it under
        # a fresh timestamp after a long outage creates unsafe discontinuity.
        self.require_mh_trusted_status=bool(rospy.get_param("~require_mh_trusted_status",False))
        self.mh_status="SEARCHING"
        self.mh_was_trusted=False
        self.mh_rearm_required=False
        self.mh_max_fix_age=float(rospy.get_param("~mh_max_fix_age_sec",2.0))
        self.mh_max_output_resume_gap=float(rospy.get_param("~mh_max_output_resume_gap_sec",2.5))
        self.pub=rospy.Publisher(
            rospy.get_param('~fused_pose_topic','/fast_livo2/fused_map_pose'),
            PoseStamped,queue_size=10)
        self.pose=rospy.Publisher("/pose",PoseStamped,queue_size=10) if self.allow_pose else None
        self.status=rospy.Publisher(
            rospy.get_param('~fused_status_topic','/fast_livo2/fused_global_status'),
            String,queue_size=10)
        rospy.Subscriber("/aft_mapped_to_init",Odometry,self.odom_cb,queue_size=50)
        rospy.Subscriber("/fast_livo2/step3_gicp_map_pose",PoseStamped,self.lidar_cb,queue_size=5)
        rospy.Subscriber("/fast_livo2/visual_global_candidate",PoseWithCovarianceStamped,self.visual_cb,queue_size=3)
        rospy.Subscriber("/fast_livo2/failsafe_mode",String,self.mode_cb,queue_size=5)
        if self.require_mh_trusted_status:
            rospy.Subscriber("/fast_livo2/step3_status",String,self.step3_status_cb,queue_size=10)

    def step3_status_cb(self,m):
        with self.lock:
            self.mh_status=m.data
            trust=m.data in ("GLOBAL_LOCKED","GLOBAL_REACQUIRED_SMOOTH")
            if trust:
                self.mh_was_trusted=True
            elif self.mh_was_trusted:
                # Only a supervised stop-and-reset transaction (not available
                # here) may re-arm pose output after losing global trust.
                self.mh_rearm_required=True

    def mode_cb(self,m):
        with self.lock:self.mode=m.data

    def find_odom(self,stamp):
        if not self.history:return None
        nearby=min(self.history,key=lambda e:abs((e[0]-stamp).to_sec()))
        return nearby[1] if abs((nearby[0]-stamp).to_sec())<.14 else None

    def fix(self,stamp,map_body,source):
        with self.lock:
            if self.require_mh_trusted_status:
                if self.mh_rearm_required or self.mh_status not in ("GLOBAL_LOCKED","GLOBAL_REACQUIRED_SMOOTH"):
                    return
            now=rospy.Time.now()
            if stamp.is_zero() or abs((now-stamp).to_sec())>self.max_fix_age:return
            if source=="lidar" and self.mode!="FULL_LIVO":return
            if source=="visual" and self.mode not in ("VI_LIDAR_FAILED","IMU_ONLY_BRIDGE"):return
            odom=self.find_odom(stamp)
            if odom is None:return
            new=map_body@np.linalg.inv(odom)
            if self.anchor is not None:
                predicted=self.anchor@odom
                delta=float(np.linalg.norm(predicted[:3,3]-map_body[:3,3]))
                max_delta=self.max_visual_innovation if source=="visual" else self.max_lidar_innovation
                if delta>max_delta:return
                dyaw=abs(wrap(yaw(predicted)-yaw(map_body)))
                if source=="visual" and math.degrees(dyaw)>self.max_visual_yaw:return
            if self.anchor is None:
                self.anchor=new.copy()
            self.target=new.copy()
            self.last_map_fix=stamp
            self.source=source

    def lidar_cb(self,m):
        self.fix(m.header.stamp,pose_matrix(m.pose),"lidar")

    def visual_cb(self,m):
        self.fix(m.header.stamp,pose_matrix(m.pose.pose),"visual")

    def odom_cb(self,m):
        stamp=m.header.stamp
        odom=pose_matrix(m.pose.pose)
        with self.lock:
            self.history.append((stamp,odom))
            if self.anchor is None:return
            if self.require_mh_trusted_status:
                if self.mh_rearm_required or self.mh_status not in ("GLOBAL_LOCKED","GLOBAL_REACQUIRED_SMOOTH"):
                    self.status.publish(String(data="MH_GLOBAL_UNTRUSTED: pose output withheld; re-arm requires supervised reset"))
                    return
                fix_age=(stamp-self.last_map_fix).to_sec()
                if fix_age<-.1 or fix_age>self.mh_max_fix_age:
                    self.mh_rearm_required=True
                    self.status.publish(String(data="MH_STALE_GLOBAL_FIX: pose output withheld; re-arm required"))
                    return
                if not self.last_pub.is_zero() and (stamp-self.last_pub).to_sec()>self.mh_max_output_resume_gap:
                    self.mh_rearm_required=True
                    self.status.publish(String(data="MH_POSE_RESUME_GAP: pose output withheld; re-arm required"))
                    return
            dt=max(.001,min(.25,(stamp-self.last_pub).to_sec())) if not self.last_pub.is_zero() else .05
            if self.target is not None:
                delta=self.target[:3,3]-self.anchor[:3,3]
                size=np.linalg.norm(delta)
                budget=self.max_step_rate*dt
                if size>budget:self.anchor[:3,3]+=delta*(budget/size)
                else:self.anchor[:3,3]=self.target[:3,3]
                q0=quaternion_from_matrix(self.anchor)
                q1=quaternion_from_matrix(self.target)
                dy=abs(wrap(yaw(self.target)-yaw(self.anchor)))
                factor=min(1.,math.radians(self.max_yaw_rate)*dt/max(1e-8,dy))
                self.anchor[:3,:3]=quaternion_matrix(quaternion_slerp(q0,q1,factor))[:3,:3]
            global_pose=self.anchor@odom
            q=quaternion_from_matrix(global_pose)
            p=PoseStamped()
            p.header.stamp=stamp;p.header.frame_id="map"
            p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,global_pose[:3,3])
            p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,q)
            self.pub.publish(p)
            if self.pose is not None:self.pose.publish(p)
            age=(stamp-self.last_map_fix).to_sec()
            source=self.source if age<2.0 else "DEAD_RECKONING_NO_RECENT_GLOBAL_FIX"
            self.status.publish(String(data=f"{source} mode={self.mode} map_fix_age={age:.2f}s"))
            self.last_pub=stamp


if __name__=="__main__":
    rospy.init_node("fresh_pose_authority")
    PoseAuthority()
    rospy.spin()
