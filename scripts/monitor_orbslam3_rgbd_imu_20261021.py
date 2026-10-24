#!/usr/bin/env python3
"""Read-only ORB-SLAM3 RGB-D inertial output monitor (ROS1)."""
import argparse
import json
import signal
import time
from collections import Counter

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image,Imu
from std_msgs.msg import Int32


ap=argparse.ArgumentParser()
ap.add_argument('--output',required=True)
ap.add_argument('--mode',choices=['rgbd_imu','rgbd'],default='rgbd_imu')
a=ap.parse_args()
rospy.init_node('monitor_orbslam3_rgbd_imu',anonymous=True,disable_signals=True)
events=[]
counts=Counter()
start=time.monotonic()


def on_pose(m):
    p=m.pose.position
    q=m.pose.orientation
    events.append({'stamp':m.header.stamp.to_sec(),'xyz':[p.x,p.y,p.z],
                   'quat':[q.x,q.y,q.z,q.w],'wall_elapsed':time.monotonic()-start})
    counts['pose']+=1
    if counts['pose']%25==0:
        print('ORB_POSES',counts['pose'],round(m.header.stamp.to_sec(),3),flush=True)


def state(m):
    counts['state_'+str(m.data)]+=1


def count(k):
    def inner(_):counts[k]+=1
    return inner


rospy.Subscriber('/orbslam3_benchmark/rgbd_imu_pose' if a.mode=='rgbd_imu' else '/orbslam3_benchmark/rgbd_pose',PoseStamped,on_pose,queue_size=30)
rospy.Subscriber('/orbslam3_benchmark/tracking_state',Int32,state,queue_size=35)
rospy.Subscriber('/zedx/zed_node/left/image_rect_color',Image,count('image_in'),queue_size=2)
rospy.Subscriber('/zedx/zed_node/depth/depth_registered',Image,count('depth_in'),queue_size=2)
rospy.Subscriber('/zedx/zed_node/imu/data',Imu,count('imu_in'),queue_size=150)
rospy.Subscriber('/pose',rospy.AnyMsg,count('unexpected_pose'),queue_size=10)


def stop(*_):
    res={'counts':dict(counts),'poses':events,'runtime_wall_sec':time.monotonic()-start}
    with open(a.output,'w') as f:json.dump(res,f,indent=2)
    print('ORB_MONITOR_FINISHED',len(events),dict(counts),flush=True)
    rospy.signal_shutdown('stop')


signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)
while not rospy.is_shutdown():time.sleep(.2)
