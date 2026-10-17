#!/usr/bin/env python3
"""Read-only ROS1 localization freshness probe.

Sample age = receiver ROS clock - ORIGINAL message header stamp. Never change
sensor data, pose, or map frame. For rosbag --clock this is *sim-time* staleness;
for synchronized live NUCs this approximates end-to-end message freshness.
Cannot measure actual vehicle network/actuator latency from offline playback.
"""
import argparse
import csv
import math
import signal
import time
from pathlib import Path

TOPICS = (
    ('lidar_raw','/lidar103/velodyne_points','cloud'),
    ('imu_raw','/mavros/imu/data','imu'),
    ('image_raw','/camera/color/image_raw','image'),
    ('lidar_adapted','/fast_livo2/input/lidar','cloud'),
    ('image_adapted','/fast_livo2/input/image','image'),
    ('fast_local','/aft_mapped_to_init','odom'),
    ('global_pose','/fast_livo2/step3_global_pose','pose'),
    ('imu_propagated','/LIVO2/imu_propagate','odom'),
    ('highrate_global','/fast_livo2/highrate_global_pose','pose'),
)

def safe_age(now, stamp):
    if not all(math.isfinite(x) for x in (now,stamp)) or now<1.0e8 or stamp<1.0e8:
        return None
    delta=now-stamp
    if delta<-0.10 or delta>10.0:
        return None
    return delta


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--csv',type=Path,required=True)
    p.add_argument('--global-pose-topic',default='/fast_livo2/step3_global_pose',help='Use /pose when probe runs on the actual planner NUC')
    p.add_argument('--sample-every',type=int,default=1)
    p.add_argument('--pose-only',action='store_true',help='Avoid deserializing large camera/LiDAR payloads in performance A/B')
    args=p.parse_args()
    import rospy
    from sensor_msgs.msg import PointCloud2,Imu,Image
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    typ={'cloud':PointCloud2,'imu':Imu,'image':Image,'odom':Odometry,'pose':PoseStamped}
    rospy.init_node('fast_livo2_pose_freshness_probe',anonymous=True,disable_signals=True)
    args.csv.parent.mkdir(parents=True,exist_ok=True)
    out=args.csv.open('w',newline='',buffering=1)
    w=csv.writer(out)
    w.writerow(['source','topic','header_stamp_ros_sec','receiver_ros_time_sec','age_ms','receiver_wall_time_sec','byte_size','frame_id'])
    seen={name:0 for name,_,_ in TOPICS}
    def listener(name,topic):
        def callback(msg):
            seen[name]+=1
            if seen[name]%max(1,args.sample_every):return
            now=rospy.Time.now().to_sec()
            stamp=msg.header.stamp.to_sec()
            age=safe_age(now,stamp)
            size=len(msg.data) if hasattr(msg,'data') and isinstance(msg.data,(bytes,bytearray)) else 0
            w.writerow([name,topic,f'{stamp:.9f}',f'{now:.9f}',
                '' if age is None else f'{1000*age:.4f}',f'{time.time():.6f}',size,msg.header.frame_id])
        return callback
    listeners=[]
    for name,topic,t in TOPICS:
        if name=='global_pose':topic=args.global_pose_topic
        if args.pose_only and name not in ('fast_local','global_pose','imu_propagated','highrate_global'):continue
        # Tiny queues drop old monitoring callbacks; do not back-pressure ROS.
        listeners.append(rospy.Subscriber(topic,typ[t],listener(name,topic),queue_size=1,
                                           buff_size=8*1024*1024,tcp_nodelay=True))
    def shutdown(*_):
        out.flush()
        out.close()
        raise SystemExit(0)
    signal.signal(signal.SIGINT,shutdown)
    signal.signal(signal.SIGTERM,shutdown)
    rospy.loginfo('FAST-LIVO2 read-only pose age probe started, csv=%s',args.csv)
    try:rospy.spin()
    finally:
        if not out.closed:out.close()

if __name__=='__main__':main()
