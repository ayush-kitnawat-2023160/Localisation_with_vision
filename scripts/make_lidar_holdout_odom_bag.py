#!/usr/bin/env python3
"""Export previously verified LiDAR-IMU ROS1 odometry CSV to a timestamped rosbag.

This lets Step3 be evaluated on real, held-out LiDAR scans with independent
FAST-LIVO2 odometry without concurrent estimator load. It is NOT ground truth.
"""
import argparse
import csv
from pathlib import Path
import rosbag
import rospy
from nav_msgs.msg import Odometry


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--trajectory',required=True)
    ap.add_argument('--out',required=True)
    ap.add_argument('--topic',default='/aft_mapped_to_init')
    args=ap.parse_args()
    path=Path(args.out)
    if path.exists():raise SystemExit(f'Refusing overwrite: {path}')
    path.parent.mkdir(parents=True,exist_ok=True)
    count=0
    with rosbag.Bag(str(path),'w',compression='none') as out:
        for row in csv.DictReader(open(args.trajectory)):
            msg=Odometry()
            msg.header.stamp=rospy.Time.from_sec(float(row['timestamp']))
            msg.header.frame_id='camera_init'
            msg.child_frame_id='body'
            pos=msg.pose.pose.position
            pos.x,pos.y,pos.z=(float(row[k]) for k in ('x','y','z'))
            quat=msg.pose.pose.orientation
            quat.x,quat.y,quat.z,quat.w=(float(row[k]) for k in ('qx','qy','qz','qw'))
            out.write(args.topic,msg,t=msg.header.stamp)
            count+=1
    print(f'{path}: {count} odometry samples',flush=True)
    if count<100:raise SystemExit('Trajectory too short')


if __name__=='__main__':
    main()
