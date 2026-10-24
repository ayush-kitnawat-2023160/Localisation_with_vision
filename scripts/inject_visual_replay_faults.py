#!/usr/bin/env python3
"""ROS1 test-only relay: camera blackout, stale timestamp, NO LiDAR/GPS input."""
import copy
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
rospy.init_node('visual_fault_relay')
first=float(rospy.get_param('~first_camera_stamp'))
blackout_start=float(rospy.get_param('~blackout_start_s',55));blackout_end=float(rospy.get_param('~blackout_end_s',59))
inj_time=float(rospy.get_param('~stale_inject_at_s',60.5));injected=False;dropped=0;forwarded=0
pub=rospy.Publisher('/visual_test/image_faulted',Image,queue_size=2)
status=rospy.Publisher('/visual_test/fault_status',String,queue_size=3)
def cb(m):
 global dropped,forwarded,injected
 t=m.header.stamp.to_sec()-first
 if blackout_start<=t<blackout_end:
  dropped+=1
  if dropped==1:status.publish(String(data='CAMERA_BLACKOUT_STARTED'))
  return
 if not injected and t>=inj_time:
  x=copy.deepcopy(m);x.header.stamp=m.header.stamp-rospy.Duration(4.0)
  pub.publish(x);status.publish(String(data='INJECTED_STALE_IMAGE_4S'));injected=True
 pub.publish(m);forwarded+=1
rospy.Subscriber('/zedx/zed_node/left/image_rect_color',Image,cb,queue_size=1,buff_size=8*1024*1024)
rospy.spin()
