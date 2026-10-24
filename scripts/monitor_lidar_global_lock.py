#!/usr/bin/env python3
"""Log Step3 global reacquisition using both ROS sensor time and monotonic wall time."""
import argparse
import json
from pathlib import Path
import time
import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Monitor:
    def __init__(self,args):
        self.path=Path(args.out)
        self.started=time.monotonic()
        self.first_lidar=None
        self.first_odom=None
        self.first_lock=None
        self.start_bag=float(args.bag_start)
        self.events=[]
        self.counts={}
        self.last_status=None
        rospy.Subscriber('/lidar103/velodyne_points',PointCloud2,self.lidar,queue_size=2)
        rospy.Subscriber('/aft_mapped_to_init',Odometry,self.odom,queue_size=10)
        rospy.Subscriber('/fast_livo2/step3_global_lock',PoseStamped,self.lock,queue_size=2)
        rospy.Subscriber('/fast_livo2/step3_status',String,self.status,queue_size=10)
        rospy.Subscriber('/fast_livo2/step3_gicp_map_pose',PoseStamped,self.tracking,queue_size=4)
        rospy.on_shutdown(self.save)

    def event(self,name,sensor_time=None,extra=None):
        wall=time.monotonic()
        self.counts[name]=self.counts.get(name,0)+1
        if len(self.events)<10000:
            self.events.append({'kind':name,'wall_s_since_monitor_start':wall-self.started,
                                'sensor_s_since_bag_start':sensor_time-self.start_bag if sensor_time is not None else None,
                                'details':extra})
        return wall

    def lidar(self,m):
        if self.first_lidar is None:
            self.first_lidar=self.event('first_lidar',m.header.stamp.to_sec())
        self.counts['lidar_scans']=self.counts.get('lidar_scans',0)+1

    def odom(self,m):
        if self.first_odom is None:
            self.first_odom=self.event('first_odom',m.header.stamp.to_sec())
        self.counts['odom_poses']=self.counts.get('odom_poses',0)+1

    def lock(self,m):
        if self.first_lock is None:
            self.first_lock=self.event('first_global_lock',m.header.stamp.to_sec(),
                                       [m.pose.position.x,m.pose.position.y,m.pose.position.z])
            self.save()

    def status(self,m):
        if m.data!=self.last_status:
            self.event('status',rospy.Time.now().to_sec(),m.data)
            self.last_status=m.data

    def tracking(self,m):
        self.event('accepted_map_pose',m.header.stamp.to_sec(),
                   [m.pose.position.x,m.pose.position.y,m.pose.position.z])

    def save(self):
        result={
            'first_lock_observed':self.first_lock is not None,
            'first_lock_wall_s_after_first_lidar':None if self.first_lock is None or self.first_lidar is None else self.first_lock-self.first_lidar,
            'first_lock_wall_s_after_first_odom':None if self.first_lock is None or self.first_odom is None else self.first_lock-self.first_odom,
            'first_lock_sensor_s_after_bag_start':next((e['sensor_s_since_bag_start'] for e in self.events if e['kind']=='first_global_lock'),None),
            'counts':self.counts,'last_status':self.last_status,'events':self.events,
            'notes':'Wall latency includes rosbag playback, acquisition cadence, data callback, and GICP; not a single-query solver benchmark. LIO odometry is from independent saved replay, not a reference trajectory.'}
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temp=self.path.with_name(self.path.name+'.tmp')
        temp.write_text(json.dumps(result,indent=2)+'\n')
        temp.replace(self.path)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--out',required=True)
    p.add_argument('--bag-start',type=float,required=True)
    args=p.parse_args()
    rospy.init_node('step3_validation_monitor',anonymous=True)
    monitor=Monitor(args)
    rospy.spin()
