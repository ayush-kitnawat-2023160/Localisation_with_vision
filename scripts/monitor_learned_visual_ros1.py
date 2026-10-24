#!/usr/bin/env python3
"""Record learned ROS1 bridge candidate/status stream; no actuation."""
import argparse,json,time
from pathlib import Path
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
rospy.init_node('learned_visual_observer',anonymous=True)
events=[]
def status(m):
 events.append({'wall':time.time(),'type':'status','message':m.data});print('VISUAL_STATUS',m.data,flush=True)
def pose(m):
 p=m.pose.pose.position;events.append({'wall':time.time(),'type':'pose_candidate','stamp_s':m.header.stamp.to_sec(),'xyz':[p.x,p.y,p.z]});print('VISUAL_CANDIDATE',m.header.stamp.to_sec(),p.x,p.y,p.z,flush=True)
def arbitration(m):
 events.append({'wall':time.time(),'type':'branch_decision','message':m.data});print('BRANCH_DECISION',m.data,flush=True)
def map_pose(m):
 p=m.pose.position
 events.append({'wall':time.time(),'type':'lidar_map_pose','stamp_s':m.header.stamp.to_sec(),'xyz':[p.x,p.y,p.z]})
def forbidden_pose(m):
 # AnyMsg works regardless of the /pose message type. The test must never
 # accidentally treat an authority output as harmless diagnostics.
 events.append({'wall':time.time(),'type':'forbidden_pose_published'})
 print('SAFETY_FAILURE_UNEXPECTED_POSE_PUBLICATION',flush=True)
def recovery_candidate(m):
 p=m.pose.pose.position
 events.append({'wall':time.time(),'type':'lidar_recovery_candidate',
                'stamp_s':m.header.stamp.to_sec(),'xyz':[p.x,p.y,p.z]})
def handoff_review(m):
 events.append({'wall':time.time(),'type':'handoff_review_status','message':m.data})
 print('HANDOFF_REVIEW',m.data,flush=True)
rospy.Subscriber('/fast_livo2/multimodal_branch_decision',String,arbitration,queue_size=30)
rospy.Subscriber('/fast_livo2/visual_match_status',String,status,queue_size=30)
rospy.Subscriber('/fast_livo2/visual_global_candidate',PoseWithCovarianceStamped,pose,queue_size=15)
rospy.Subscriber('/fast_livo2/step3_global_pose',PoseStamped,map_pose,queue_size=50)
rospy.Subscriber('/fast_livo2/step3_mh_recovery_candidate',PoseWithCovarianceStamped,recovery_candidate,queue_size=30)
rospy.Subscriber('/fast_livo2/mh_handoff_review_status',String,handoff_review,queue_size=10)
rospy.Subscriber('/pose',rospy.AnyMsg,forbidden_pose,queue_size=5)
def save():
 Path(args.output).write_text(json.dumps({'events':events,'candidate_count':sum(x['type']=='pose_candidate' for x in events)},indent=2)+'\n')
rospy.on_shutdown(save)
rospy.spin()
