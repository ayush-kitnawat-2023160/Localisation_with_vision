#!/usr/bin/env python3
"""ROS1 offline contract check: MH trust loss permanently silences map poses.

Requires an isolated roscore and fresh_pose_authority.py with
~require_mh_trusted_status=true. Never starts a robot stack.
"""
import json,threading,time,sys
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

rospy.init_node('test_mh_pose_authority_guard',anonymous=True)
statuses=[];published=[];unexpected=[]
rospy.Subscriber('/fast_livo2/fused_map_pose',PoseStamped,lambda m:published.append(m.header.stamp.to_sec()),queue_size=20)
rospy.Subscriber('/pose',rospy.AnyMsg,lambda m:unexpected.append(1),queue_size=20)
rospy.Subscriber('/fast_livo2/fused_global_status',String,lambda m:statuses.append(m.data),queue_size=20)
status_pub=rospy.Publisher('/fast_livo2/step3_status',String,queue_size=8,latch=True)
odo_pub=rospy.Publisher('/aft_mapped_to_init',Odometry,queue_size=8)
fix_pub=rospy.Publisher('/fast_livo2/step3_gicp_map_pose',PoseStamped,queue_size=8)
time.sleep(.65)
def odo(stamp):
 m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.pose.pose.orientation.w=1.0;m.twist.twist.linear.x=0.0;odo_pub.publish(m)
def fix(stamp):
 m=PoseStamped();m.header.stamp=stamp;m.header.frame_id='map';m.pose.orientation.w=1.0;fix_pub.publish(m)
def check(condition,message):
 if not condition:raise AssertionError(message+' publications='+repr(published)+' status='+repr(statuses[-4:]))
try:
 status_pub.publish(String(data='GLOBAL_LOCKED'));time.sleep(.35)
 t=rospy.Time.now();odo(t);time.sleep(.23);fix(t);time.sleep(.25)
 odo(rospy.Time.now());time.sleep(.28)
 check(len(published)>=1,'initial trusted global pose not published')
 initial=len(published)
 status_pub.publish(String(data='RELOCALIZING: MH_HANDOFF_REQUIRED'));time.sleep(.28)
 odo(rospy.Time.now());time.sleep(.24)
 check(len(published)==initial,'untrusted odom leaked fused_map_pose')
 status_pub.publish(String(data='GLOBAL_LOCKED'));time.sleep(.25)
 t=rospy.Time.now();odo(t);time.sleep(.14);fix(t);time.sleep(.15);odo(rospy.Time.now());time.sleep(.30)
 check(len(published)==initial,'map pose auto-resumed after trust loss without reset')
 check(not unexpected,'production /pose published despite default-disabled authority')
 check(any('MH_GLOBAL_UNTRUSTED' in s for s in statuses),'no fail-closed diagnostic emitted')
 print('MH_POSE_AUTHORITY_FAIL_CLOSED_PASS',json.dumps({'trusted_initial_publications':initial,'later_untrusted_publications':len(published)-initial,'forbidden_pose':len(unexpected),'status_events':statuses[-5:]}))
except Exception as exc:
 print('MH_POSE_AUTHORITY_FAIL_CLOSED_FAILURE',repr(exc));sys.exit(1)
