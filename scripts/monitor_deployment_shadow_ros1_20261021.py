#!/usr/bin/env python3
"""Non-authoritative ROS1 runtime probe for shadow global+local stack."""
import argparse
import collections
import csv
import json
import math
import signal
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image,Imu,PointCloud2


class Probe:
    def __init__(self,out):
        self.out=out
        self.counter=collections.Counter()
        self.last_states=collections.Counter()
        self.step3=[]
        self.health=[]
        self.points=collections.defaultdict(list)
        self.full_poses=collections.defaultdict(list)
        self.start=time.monotonic()
        rospy.init_node('monitor_deployment_shadow',anonymous=True,disable_signals=True)
        self.subs=[
          rospy.Subscriber('/fast_livo2/deployment/fast_odom_candidate',Odometry,self.on_odom,'fast_odom',queue_size=40),
          rospy.Subscriber('/fast_livo2/deployment/klt_local_odom_candidate',Odometry,self.on_odom,'klt_odom',queue_size=40),
          rospy.Subscriber('/fast_livo2/deployment/klt_continuation_candidate',Odometry,self.on_odom,'klt_continuation',queue_size=40),
          rospy.Subscriber('/fast_livo2/deployment/step3_gicp_map_pose_candidate',PoseStamped,self.on_pose,'step3_map',queue_size=30),
          rospy.Subscriber('/fast_livo2/shadow_fused_map_pose',PoseStamped,self.on_pose,'fused_shadow',queue_size=30),
          rospy.Subscriber('/fast_livo2/deployment/visual_global_candidate',PoseWithCovarianceStamped,self.on_pose,'xfeat_global',queue_size=30),
          rospy.Subscriber('/fast_livo2/deployment_health_status',String,self.on_health,queue_size=20),
          rospy.Subscriber('/fast_livo2/deployment/step3_status',String,self.on_step3,queue_size=20),
          rospy.Subscriber('/fast_livo2/deployment/klt_continuation_status',String,self.on_local_status,queue_size=20),
          rospy.Subscriber('/fast_livo2/mh_handoff_review_status',String,self.on_review,queue_size=15),
          rospy.Subscriber('/fast_livo2/deployment/offline_lidar_fault_status',String,self.on_fault,queue_size=15),
          # AnyMsg intentionally sees even a publisher with the wrong type.
          # Isolated ROS master must have ZERO forbidden command/pose messages.
          rospy.Subscriber('/pose',rospy.AnyMsg,self.on_forbidden,'forbidden_pose',queue_size=40),
          rospy.Subscriber('/cmd_vel',rospy.AnyMsg,self.on_forbidden,'forbidden_cmd_vel',queue_size=40),
          rospy.Subscriber('/initialpose',rospy.AnyMsg,self.on_forbidden,'forbidden_initialpose',queue_size=20),
          rospy.Subscriber('/fast_livo2/fused_map_pose',rospy.AnyMsg,self.on_forbidden,'unexpected_legacy_fused',queue_size=40),
          rospy.Subscriber('/aft_mapped_to_init',rospy.AnyMsg,self.on_forbidden,'forbidden_canonical_fast',queue_size=40),
          rospy.Subscriber('/path',rospy.AnyMsg,self.on_forbidden,'forbidden_canonical_path',queue_size=40),
          rospy.Subscriber('/tf',rospy.AnyMsg,self.on_forbidden,'forbidden_canonical_tf',queue_size=40),
          rospy.Subscriber('/fast_livo2/step3_global_lock',rospy.AnyMsg,self.on_forbidden,'forbidden_step3_lock',queue_size=20),
          rospy.Subscriber('/fast_livo2/map_correction',rospy.AnyMsg,self.on_forbidden,'forbidden_map_correction',queue_size=20),
        ]

    def on_forbidden(self,msg,key):
        self.counter[key]+=1

    def on_odom(self,msg,key):
        p=msg.pose.pose.position
        self.record(key,msg.header.stamp.to_sec(),p.x,p.y,p.z)
        q=msg.pose.pose.orientation
        self.keep_pose(key,msg.header.stamp.to_sec(),p,q)

    def on_pose(self,msg,key):
        v=msg.pose.pose if hasattr(msg.pose,'pose') else msg.pose
        p=v.position
        self.record(key,msg.header.stamp.to_sec(),p.x,p.y,p.z)
        self.keep_pose(key,msg.header.stamp.to_sec(),p,v.orientation)

    def keep_pose(self,key,t,p,q):
        vals=(t,p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if len(self.full_poses[key])<25000 and all(math.isfinite(float(x)) for x in vals):
            self.full_poses[key].append(tuple(float(x) for x in vals))

    def record(self,key,t,x,y,z):
        self.counter[key]+=1
        if len(self.points[key])<25000 and all(math.isfinite(a) for a in (t,x,y,z)):
            self.points[key].append((float(t),float(x),float(y),float(z)))

    def on_step3(self,msg):
        self.counter['step3_status']+=1
        self.last_states['step3_'+msg.data[:90]]+=1
        if len(self.step3)<2000:self.step3.append(msg.data)

    def on_local_status(self,msg):
        self.counter['klt_continuation_status']+=1
        self.last_states['continuation_'+msg.data.split(' ',1)[0]]+=1

    def on_review(self,msg):
        self.counter['handoff_review_status']+=1
        self.last_states['handoff_review_'+msg.data[:90]]+=1

    def on_fault(self,msg):
        self.counter['offline_fault_events']+=1
        self.last_states['offline_fault_'+msg.data[:90]]+=1

    def on_health(self,msg):
        self.counter['health_status']+=1
        try:
            status=json.loads(msg.data)
            self.last_states['health_'+status.get('mode','MISSING')]+=1
            if len(self.health)<4000:
                self.health.append({'ros_sec':rospy.Time.now().to_sec(),
                                    'mode':status.get('mode'),
                                    'missing':status.get('missing_or_stale'),
                                    'reasons':status.get('reasons')})
        except json.JSONDecodeError:self.counter['health_invalid_json']+=1

    def finish(self):
        result={'received':dict(self.counter),'status_counts':dict(self.last_states),
                'step3_status_sequence':self.step3,
                'health_last':self.health[-10:],
                'health_transitions':[a for i,a in enumerate(self.health)
                    if i==0 or a['mode']!=self.health[i-1]['mode']],
                'runtime_wall_sec':time.monotonic()-self.start,
                'unexpected_pose_count':self.counter['forbidden_pose'],
                'unexpected_actuator_command_count':self.counter['forbidden_cmd_vel'],
                'unexpected_initialpose_count':self.counter['forbidden_initialpose'],
                'unexpected_legacy_fused_count':self.counter['unexpected_legacy_fused'],
                'unexpected_canonical_fast_count':self.counter['forbidden_canonical_fast'],
                'unexpected_canonical_tf_count':self.counter['forbidden_canonical_tf'],
                'pose_published_to_navigation':False}
        for key,series in self.points.items():
            if not series:continue
            import numpy as np
            A=np.asarray(series)
            dt=np.diff(A[:,0]);ds=np.linalg.norm(np.diff(A[:,1:],axis=0),axis=1)
            result[key]={'count':len(series),'stamp_start':float(A[0,0]),
              'stamp_end':float(A[-1,0]),
              'largest_step_m':float(ds.max()) if len(ds) else None,
              'largest_gap_s':float(dt.max()) if len(dt) else None}
        self.out.parent.mkdir(parents=True,exist_ok=True)
        for key,items in self.full_poses.items():
            if not items:continue
            with (self.out.parent/(key+'_trajectory.csv')).open('w',newline='') as handle:
                writer=csv.writer(handle)
                writer.writerow(('stamp','x','y','z','qx','qy','qz','qw'))
                writer.writerows(items)
        self.out.write_text(json.dumps(result,indent=2)+'\n')
        print('SHADOW_MONITOR_DONE counts',dict(self.counter),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--output',required=True)
    args=p.parse_args()
    mon=Probe(Path(args.output))
    def stop(*_):
        mon.finish()
        rospy.signal_shutdown('recorded offline evidence')
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    rospy.spin()
