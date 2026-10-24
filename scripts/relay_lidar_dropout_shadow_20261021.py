#!/usr/bin/env python3
"""Optional OFFLINE replay fault injector: raw LiDAR messages only.

Cannot publish /pose, /cmd_vel, odom or a synthetic 'sensor failed' Boolean.
Every surviving point cloud keeps its original header/bytes unchanged. A
separate run manifest must document the externally chosen outage window.
"""
import argparse
import math
import threading

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class Relay:
    def __init__(self,source,target,start,end):
        if not rospy.get_param('/use_sim_time',False):
            raise RuntimeError('LiDAR blackout relay is FORBIDDEN with live vehicle ROS time')
        if not 0<=start<end or end-start>120:
            raise ValueError('Invalid offline fault start/end')
        self.start=start;self.end=end
        self.first=None
        self.accepted=0;self.dropped=0
        self.lock=threading.Lock()
        self.pub=rospy.Publisher(target,PointCloud2,queue_size=12)
        self.status=rospy.Publisher('/fast_livo2/deployment/offline_lidar_fault_status',String,queue_size=6,latch=True)
        rospy.Subscriber(source,PointCloud2,self.cb,queue_size=12,buff_size=16*1024*1024)
        rospy.logwarn('[REPLAY FAULT ONLY] masking LiDAR relative source time %.2f..%.2fs',start,end)

    def cb(self,m):
        t=m.header.stamp.to_sec()
        if not math.isfinite(t) or t<=0:return
        with self.lock:
            if self.first is None:self.first=t
            relative=t-self.first
            if self.start<=relative<=self.end:
                self.dropped+=1
                if self.dropped==1:
                    self.status.publish(String(data='OFFLINE_FAULT_ACTIVE source_scan_seconds=%.3f'%relative))
                return
            self.accepted+=1
            if self.dropped>0 and self.dropped%100==0:
                self.status.publish(String(data='OFFLINE_FAULT_ENDED accepted=%d dropped=%d'%(self.accepted,self.dropped)))
        self.pub.publish(m)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--source',default='/shadow_offline/source_lidar')
    p.add_argument('--target',default='/lidar103/velodyne_points')
    p.add_argument('--start',type=float,required=True)
    p.add_argument('--end',type=float,required=True)
    args=p.parse_args()
    rospy.init_node('relay_lidar_dropout_shadow')
    Relay(args.source,args.target,args.start,args.end)
    rospy.spin()
