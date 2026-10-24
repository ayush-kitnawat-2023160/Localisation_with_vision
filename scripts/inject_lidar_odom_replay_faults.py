#!/usr/bin/env python3
"""Offline-only LiDAR and saved-odometry feed dropout relay for ROS1.

This is a topic-level injection, NOT a realistic recomputation of the SLAM
estimator during sensor dropout. Do not present it as validated onboard VIO.
"""
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
rospy.init_node('lidar_odom_topic_failure_relay')
start=float(rospy.get_param('~lidar_first_stamp'))
blackout_start=float(rospy.get_param('~start_sec',26.0))
blackout_end=float(rospy.get_param('~end_sec',39.0))
drop_odom=bool(rospy.get_param('~drop_odom_too',True))
lidar_pub=rospy.Publisher('/visual_test/lidar_faulted',PointCloud2,queue_size=1)
odom_pub=rospy.Publisher('/visual_test/odom_faulted',Odometry,queue_size=10)
status=rospy.Publisher('/visual_test/lidar_odom_fault_status',String,queue_size=10)
counters={'lidar_forward':0,'lidar_drop':0,'odom_forward':0,'odom_drop':0}
reported=False

def blocked(stamp):
 t=stamp.to_sec()-start
 return blackout_start<=t<blackout_end

def callback(m,kind,pub):
 global reported
 if blocked(m.header.stamp) and (kind=='lidar' or drop_odom):
  counters[kind+'_drop']+=1
  if not reported:
   reported=True
   status.publish(String(data='LIDAR_ODOM_BLACKOUT_STARTED t=%.2f'% (m.header.stamp.to_sec()-start)))
  return
 if reported and m.header.stamp.to_sec()-start>=blackout_end:
  reported=False
  status.publish(String(data='LIDAR_ODOM_BLACKOUT_ENDED t=%.2f'% (m.header.stamp.to_sec()-start)))
 counters[kind+'_forward']+=1;pub.publish(m)

rospy.Subscriber('/lidar103/velodyne_points',PointCloud2,lambda m:callback(m,'lidar',lidar_pub),queue_size=1,buff_size=10*1024*1024)
rospy.Subscriber('/aft_mapped_to_init',Odometry,lambda m:callback(m,'odom',odom_pub),queue_size=10)
rospy.on_shutdown(lambda: rospy.loginfo('TOPIC_FAULT_RELAY %s',counters))
rospy.spin()
