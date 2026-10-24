#!/usr/bin/env python3
"""TEST ONLY: replay two previously recorded alternative LiDAR pose hypotheses
as a Step3 formatted String to exercise the observer. Not a live MH bank.
"""
import argparse,json
import numpy as np,rospy
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--old',required=True);p.add_argument('--late',required=True);p.add_argument('--lidar-first-stamp',required=True,type=float);args=p.parse_args()
rospy.init_node('lidar_candidate_fixture');pub=rospy.Publisher('/fast_livo2/step3_mh_status',String,queue_size=1)
left=np.array(json.load(open(args.old))['map_pose_trace']);right=np.array(json.load(open(args.late))['map_pose_trace'])
rate=rospy.Rate(3)
while not rospy.is_shutdown():
 t=rospy.Time.now().to_sec()-args.lidar_first_stamp
 if t<60 or t>=63.9:rate.sleep();continue
 if not (left[0,0]<=t<=left[-1,0] and right[0,0]<=t<=right[-1,0]):rate.sleep();continue
 la=np.array([np.interp(t,left[:,0],left[:,j]) for j in (1,2)]);ra=np.array([np.interp(t,right[:,0],right[:,j]) for j in (1,2)])
 message='stamp=%.9f hypotheses=2 [id=25 hits=5 fit=0.06 xy=%.4f,%.4f] [id=293 hits=5 fit=0.06 xy=%.4f,%.4f]'%(rospy.Time.now().to_sec(),la[0],la[1],ra[0],ra[1]);pub.publish(String(data=message));rate.sleep()
