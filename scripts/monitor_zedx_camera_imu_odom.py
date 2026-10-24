#!/usr/bin/env python3
"""Read-only ROS1 monitor for independent candidate VO output and safety."""
import json,argparse,collections
from pathlib import Path
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
rospy.init_node('monitor_camera_imu_candidate',anonymous=True)
records=[];states=[];forbidden=[]

def odom(m):
    q=m.pose.pose.orientation;t=m.pose.pose.position
    records.append(dict(stamp=m.header.stamp.to_sec(),frame=m.header.frame_id,child=m.child_frame_id,
                        xyz=[t.x,t.y,t.z],quat=[q.x,q.y,q.z,q.w],
                        variance_x=m.pose.covariance[0]))
    if len(records)%60==0:print('CANDIDATE_SAMPLES',len(records),'stamp',round(records[-1]['stamp'],2),flush=True)

def status(m):
    states.append(dict(t=rospy.Time.now().to_sec(),text=m.data))
    if len(states)%80==0:print('CANDIDATE_HEALTH',m.data,flush=True)

rospy.Subscriber('/fast_livo2/fallback/camera_imu_odom_candidate',Odometry,odom,queue_size=100)
rospy.Subscriber('/fast_livo2/fallback/camera_imu_status',String,status,queue_size=100)
rospy.Subscriber('/pose',rospy.AnyMsg,lambda _:forbidden.append(rospy.Time.now().to_sec()),queue_size=2)

def save():
    a={'odometry':records,'statuses':states,'forbidden_pose_count':len(forbidden),
       'candidate_count':len(records),
       'status_counts':dict(collections.Counter(z['text'].split()[0] for z in states))}
    Path(args.output).write_text(json.dumps(a,indent=2)+'\n')
rospy.on_shutdown(save)
rospy.spin()
