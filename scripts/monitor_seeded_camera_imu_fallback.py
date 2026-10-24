#!/usr/bin/env python3
import json,argparse,collections
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);args=parser.parse_args()
rospy.init_node('seeded_candidate_observer',anonymous=True)
poses=[];states=[];forbidden=[]
def odom(m):
    p=m.pose.pose.position
    poses.append(dict(stamp=m.header.stamp.to_sec(),frame=m.header.frame_id,xyz=[p.x,p.y,p.z]))
    if len(poses)%60==0:print('SEEDED_ODOM',len(poses),poses[-1]['stamp'],flush=True)
rospy.Subscriber('/fast_livo2/fallback/seeded_local_odom_candidate',Odometry,odom,queue_size=60)
rospy.Subscriber('/fast_livo2/fallback/seeded_local_odom_status',String,lambda m:states.append(m.data),queue_size=40)
rospy.Subscriber('/pose',rospy.AnyMsg,lambda m:forbidden.append(rospy.Time.now().to_sec()),queue_size=2)
def save():
    Path(args.out).write_text(json.dumps(dict(odometry=poses,states=states,forbidden_pose_count=len(forbidden)),indent=2)+'\n')
rospy.on_shutdown(save)
rospy.spin()
