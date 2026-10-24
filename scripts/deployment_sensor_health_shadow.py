#!/usr/bin/env python3
"""ROS1 read-only health dashboard for a GPS-free shadow localization stack.

Crucially this node is NOT a wheel-speed/brake health certification and never
publishes authoritative /pose, /tf, mode switch, /initialpose or actuators.
It can be used with /use_sim_time or true hardware ROS time. Status messages
are advisory; the autonomous controller must fail closed independently.
"""
import json
import math
import os
import sys
import threading
import time
from collections import deque
import numpy as np

import rospy
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Image,Imu
from std_msgs.msg import String

sys.path.insert(0,os.path.dirname(__file__))
from deployment_sensor_health_core import Health


class ShadowMonitor:
    def __init__(self):
        if not bool(rospy.get_param('~shadow_only',True)):
            raise RuntimeError('Health supervisor is only implemented for SHADOW mode')
        self.lock=threading.RLock()
        self.h=Health()
        self.last_state=None
        self.last_clock=None
        self.msgs=0
        self.lidar_fix_history=deque(maxlen=8)
        self.visual_fix_history=deque(maxlen=8)
        self.visual_global_disagreements=0
        self.pub=rospy.Publisher('/fast_livo2/deployment_health_status',String,queue_size=6,latch=True)
        self.diagnostics=rospy.Publisher('/fast_livo2/deployment_health_diagnostics',DiagnosticArray,queue_size=3)
        self.warning=rospy.Publisher('/fast_livo2/deployment_stop_advisory',String,queue_size=3,latch=True)
        topics={
          'lidar':(rospy.get_param('~lidar_topic','/fast_livo2/input/lidar'),PointCloud2),
          'fast_imu':(rospy.get_param('~fast_imu_topic','/fast_livo2/input/imu'),Imu),
          'fast_odom':(rospy.get_param('~fast_odom_topic','/aft_mapped_to_init'),Odometry),
          'rgb':(rospy.get_param('~rgb_topic','/zedx/zed_node/left/image_rect_color'),Image),
          'depth':(rospy.get_param('~depth_topic','/zedx/zed_node/depth/depth_registered'),Image),
          'zed_imu':(rospy.get_param('~zed_imu_topic','/zedx/zed_node/imu/data'),Imu),
          'visual_odom':(rospy.get_param('~visual_odom_topic','/fast_livo2/fallback/camera_imu_odom_candidate'),Odometry),
          'lidar_map_pose':(rospy.get_param('~lidar_map_pose_topic','/fast_livo2/step3_gicp_map_pose'),PoseStamped),
          'visual_map_pose':(rospy.get_param('~visual_map_pose_topic','/fast_livo2/visual_global_candidate'),PoseWithCovarianceStamped),
        }
        self.subs=[]
        for name,(topic,typ) in topics.items():
            self.subs.append(rospy.Subscriber(topic,typ,self.on_measurement,callback_args=name,
                                               queue_size=2,tcp_nodelay=True))
        self.subs.append(rospy.Subscriber('/fast_livo2/step3_status',String,self.on_step3,queue_size=5))
        self.subs.append(rospy.Subscriber('/fast_livo2/fallback/camera_imu_status',
                                        String,self.on_visual_status,queue_size=5))
        rospy.Timer(rospy.Duration(.2),self.report)
        rospy.logwarn('DEPLOYMENT SENSOR MONITOR: read-only SHADOW, no physical braking or odometry authority')

    def on_measurement(self,msg,name):
        stamp=msg.header.stamp.to_sec()
        now=rospy.Time.now().to_sec()
        if now<=0:return
        # Reject empty LiDAR, missing image payloads, malformed covariance,
        # invalid quaternion and non-finite IMU before treating as fresh.
        if name=='lidar' and (msg.width*msg.height<30 or msg.point_step<=0 or len(msg.data)<msg.point_step*30):
            return
        if name in ('rgb','depth') and (msg.width<32 or msg.height<32 or msg.step<=0 or
                         len(msg.data)<msg.step*msg.height):
            return
        if name=='depth' and msg.encoding not in ('32FC1','16UC1'):
            return
        if name=='depth':
            # Transport-valid does not mean geometry-valid. Check a sparse,
            # spatially spread view of actual metric depths to reject black,
            # all-NaN and all-Inf depth streams at bounded CPU cost.
            if msg.encoding=='32FC1':
                if msg.step%4 or msg.step<msg.width*4:return
                d=np.frombuffer(msg.data,dtype=np.float32)
                if not len(d):return
                sample=d[::max(1,len(d)//1000)]
                with np.errstate(invalid='ignore'):
                    quality=np.isfinite(sample)&(sample>.45)&(sample<25.)
            else:
                if msg.step%2 or msg.step<msg.width*2:return
                d=np.frombuffer(msg.data,dtype=np.uint16)
                if not len(d):return
                sample=d[::max(1,len(d)//1000)]
                quality=(sample>450)&(sample<25000)
            if float(np.mean(quality))<.10:return
        if name in ('fast_imu','zed_imu'):
            w=msg.angular_velocity;a=msg.linear_acceleration
            vals=(w.x,w.y,w.z,a.x,a.y,a.z)
            if not all(math.isfinite(x) for x in vals):return
            if sum(x*x for x in vals[:3])>100 or sum(x*x for x in vals[3:])>2500:return
        if name in ('fast_odom','visual_odom','lidar_map_pose','visual_map_pose'):
            m=msg.pose.pose if hasattr(msg.pose,'pose') else msg.pose
            p=m.position;q=m.orientation
            if (not all(math.isfinite(z) for z in
                (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or
                not .9<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.1):return
        if name=='visual_odom':
            if msg.header.frame_id!='zedx_camera_imu_initial_body' or \
               msg.child_frame_id!='zedx_body_candidate_not_base_link':return
        if name in ('lidar_map_pose','visual_map_pose') and msg.header.frame_id!='map':return
        with self.lock:
            self.h.add(name,stamp,time.monotonic(),now)
            self.msgs+=1
            if name in ('lidar_map_pose','visual_map_pose'):
                sample=(stamp,np.array([p.x,p.y,p.z],dtype=float),
                        np.array([q.x,q.y,q.z,q.w],dtype=float))
                if name=='lidar_map_pose':self.lidar_fix_history.append(sample)
                else:self.visual_fix_history.append(sample)
                self._crosscheck_recent_pose_pair(name)

    def _crosscheck_recent_pose_pair(self,trigger):
        if not self.lidar_fix_history or not self.visual_fix_history:return
        t1,pos_lidar,q_lidar=self.lidar_fix_history[-1]
        t2,pos_vis,q_vis=self.visual_fix_history[-1]
        if abs(t1-t2)>1.3:return
        # Both are T_map_body candidates by their publisher contracts.
        # A 2.5m/25deg alignment at one instant is only a shadow consistency
        # check, NOT surveyed map truth or permission to hand over authority.
        xy=float(np.linalg.norm(pos_lidar[:2]-pos_vis[:2]))
        angle=2*math.degrees(math.acos(min(1.,abs(float(q_lidar@q_vis)))))
        if xy<=2.5 and angle<=25.:
            self.h.confirm_visual_lidar_agreement(max(t1,t2),time.monotonic())
        else:
            self.visual_global_disagreements+=1
            self.h.invalidate_visual_lidar_agreement()

    def on_step3(self,m):
        with self.lock:self.h.set_step3_status(m.data)

    def on_visual_status(self,m):
        with self.lock:self.h.set_visual_odom_status(m.data)

    def report(self,_):
        now=rospy.Time.now().to_sec()
        if now<=0:return
        with self.lock:
            snap=self.h.evaluate(now,time.monotonic())
            values={
              'mode':snap.mode,
              'local_source':snap.local_source,
              'lidar_global_pose_observed':snap.lidar_global,
              'visual_global_candidate_observed':snap.visual_global,
              'visual_lidar_geometric_agreement_observed':snap.cross_sensor_visual_agreement,
              'visual_global_disagreement_samples':self.visual_global_disagreements,
              'visual_local_odom_observed':snap.visual_odom,
              'map_pose_trusted_for_navigation':False,
              'continuity_rearm_latched':snap.continuity_latched,
              'missing_or_stale':list(snap.missing),
              'reasons':list(snap.reasons),
              'physical_calibration_verified':False,
              'physical_brake_interlock_verified':False,
              'vehicle_autonomy_authorized':False,
              'pose_authority_enabled':False,
              'automatic_handoff_enabled':False,
              'evidence':'Sensor and candidate receipt/stamps only; no hardware-safety verification',
            }
        out=json.dumps(values,sort_keys=True)
        self.pub.publish(String(data=out))
        diag=DiagnosticArray()
        diag.header.stamp=rospy.Time.now()
        stat=DiagnosticStatus()
        stat.name='gps_free_deployment_shadow'
        stat.hardware_id='FAST-LIVO2+ZED_X_SENSOR_CANDIDATES_ONLY'
        stat.level=DiagnosticStatus.WARN if snap.mode.startswith('SHADOW_') else DiagnosticStatus.ERROR
        stat.message=snap.mode
        stat.values=[KeyValue(key=str(k),value=json.dumps(v)) for k,v in values.items()]
        diag.status=[stat]
        self.diagnostics.publish(diag)
        self.warning.publish(String(data='STOP_ADVISORY_ONLY_NO_BRAKE_ACTUATION '+snap.mode))
        if snap.mode!=self.last_state:
            rospy.logwarn('[DEPLOYMENT SHADOW] %s src=%s reasons=%s',snap.mode,
                          snap.local_source,','.join(snap.reasons))
            self.last_state=snap.mode


if __name__=='__main__':
    rospy.init_node('deployment_sensor_health_shadow')
    ShadowMonitor()
    rospy.spin()
