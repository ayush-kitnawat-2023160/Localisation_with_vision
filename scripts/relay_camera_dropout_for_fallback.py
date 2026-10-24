#!/usr/bin/env python3
"""Offline ROS1 test fixture to drop only RGB and registered depth 30-35s."""
import rospy
from sensor_msgs.msg import Image
rospy.init_node('offline_camera_dropout_relay')
start=float(rospy.get_param('~first_lidar_stamp'))
t0=float(rospy.get_param('~drop_start_sec',30.))
t1=float(rospy.get_param('~drop_end_sec',35.))
color=rospy.Publisher('/fallback_test/rgb_relay',Image,queue_size=3)
depth=rospy.Publisher('/fallback_test/depth_relay',Image,queue_size=3)
dropped={'rgb':0,'depth':0};seen={'rgb':0,'depth':0}

def forward(msg,kind,pub):
    t=msg.header.stamp.to_sec()-start
    if t0<=t<t1:dropped[kind]+=1;return
    seen[kind]+=1;pub.publish(msg)
rospy.Subscriber('/zedx/zed_node/left/image_rect_color',Image,lambda m:forward(m,'rgb',color),queue_size=3,buff_size=6*1024*1024)
rospy.Subscriber('/zedx/zed_node/depth/depth_registered',Image,lambda m:forward(m,'depth',depth),queue_size=3,buff_size=6*1024*1024)
rospy.on_shutdown(lambda:rospy.logwarn('OFFLINE_CAMERA_DROPOUT: forwarded=%s dropped=%s',seen,dropped))
rospy.spin()
