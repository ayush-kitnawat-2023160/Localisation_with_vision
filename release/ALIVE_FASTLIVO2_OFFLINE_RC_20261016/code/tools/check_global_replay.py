#!/usr/bin/env python3
"""Bounded ROS replay check; launches its own nodes against an existing roscore.

The initial pose must be supplied in the PCD map frame. Success means validated
scan matching and finite, monotonic pose output, not surveyed pose accuracy.
"""
import argparse
import json
import math
import os
import subprocess
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt64


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True, help='radians')
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--rate', type=float, default=0.5)
    parser.add_argument('--search-xy', type=float, default=5.)
    parser.add_argument('--search-yaw', type=float, default=20.)
    parser.add_argument('--map-path', help='override the map path inside the ROS environment')
    parser.add_argument('--max-step', type=float, default=2., help='maximum accepted inter-pose jump in metres')
    parser.add_argument('--max-tilt-change', type=float, default=25., help='maximum change in vehicle up direction, degrees')
    parser.add_argument('--full', action='store_true', help='also run FAST-LIVO2')
    parser.add_argument('--output', default='/tmp/floam-replay')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0. or not math.isfinite(args.duration) or args.duration <= 0.:
        parser.error('rate and duration must be positive finite values')
    os.makedirs(args.output, exist_ok=True)
    rospy.init_node('check_global_replay', anonymous=True)
    poses, states, native, scans = [], [], [], []
    prediction_counts = []
    def record(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses.append([msg.header.stamp.to_sec(), p.x, p.y, p.z,
                      q.x, q.y, q.z, q.w])
    rospy.Subscriber('/odom', Odometry, record, queue_size=1000)
    rospy.Subscriber('/fast_livo2/fast_prediction_count', UInt64,
                     lambda msg: prediction_counts.append(msg.data), queue_size=100)
    rospy.Subscriber('/laser_cloud_surf', PointCloud2,
                     lambda msg: scans.append(msg.header.stamp.to_sec()), queue_size=100)
    rospy.Subscriber('/aft_mapped_to_init', Odometry,
                     lambda msg: native.append(msg.header.stamp.to_sec()), queue_size=1000)
    rospy.Subscriber('/fast_livo2/localization_status', String,
                     lambda msg: states.append(msg.data), queue_size=100)
    publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                                queue_size=1, latch=True)
    launch_name = 'fast_livo2_full_rosbag.launch' if args.full else 'floam_global_rosbag.launch'
    with open(os.path.join(args.output, 'launch.log'), 'w') as log:
        launch_args = ['roslaunch', 'fast_livo2', launch_name, 'rviz:=false',
                       'initial_search_xy:=' + str(args.search_xy),
                       'initial_search_yaw:=' + str(args.search_yaw)]
        if args.map_path:
            launch_args.append('map_path:=' + args.map_path)
        launch = subprocess.Popen(launch_args,
                                  stdout=log, stderr=subprocess.STDOUT)
        play = None
        try:
            rospy.wait_for_message('/apriori_map', PointCloud2, timeout=90.)
            # No live scan is needed to load the map; no output pose should exist yet.
            assert not poses, 'global pose was published before scan validation'
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = 'map'
            seed.pose.pose.position.x, seed.pose.pose.position.y = args.x, args.y
            seed.pose.pose.orientation.z = math.sin(args.yaw / 2.)
            seed.pose.pose.orientation.w = math.cos(args.yaw / 2.)
            publisher.publish(seed)
            time.sleep(0.5)
            topics = ['/lidar103/velodyne_points']
            if args.full:
                topics += ['/mavros/imu/data', '/camera/color/image_raw']
            with open(os.path.join(args.output, 'play.log'), 'w') as play_log:
                play = subprocess.Popen(['rosbag', 'play', '--clock', '--quiet', '-r', str(args.rate),
                                         '--duration=' + str(args.duration), args.bag,
                                         '--topics'] + topics, stdout=play_log, stderr=subprocess.STDOUT)
                play.wait(timeout=args.duration / args.rate + 90.)
            time.sleep(3.)
            assert play.returncode == 0, 'rosbag playback failed'
            assert launch.poll() is None, 'required localization node exited'
            assert 'LOCALIZED' in states, 'FLOAM never validated map lock: ' + str(states[-3:])
            assert len(poses) >= 10, 'too few validated poses'
            assert states[-1] == 'LOCALIZED', 'replay ended without map lock: ' + states[-1]
            assert scans and scans[-1] - poses[-1][0] < 1., 'global output stopped before replay ended'
            assert len(poses) >= 0.7 * len(scans), 'less than 70% of scans produced validated poses'
            assert all(math.isfinite(v) for p in poses for v in p), 'non-finite pose'
            assert all(b[0] > a[0] for a, b in zip(poses, poses[1:])), 'pose timestamps not monotonic'
            max_step = max(math.dist(a[1:4], b[1:4]) for a, b in zip(poses, poses[1:]))
            assert max_step <= args.max_step, 'global trajectory jumped %.3f m' % max_step
            def up(p):
                x, y, z, w = p[4:]
                return (2. * (x*z + w*y), 2. * (y*z - w*x), 1. - 2. * (x*x + y*y))
            initial_up = up(poses[0])
            max_tilt = max(math.degrees(math.acos(max(-1., min(1., sum(a*b for a, b in zip(initial_up, up(p))))))) for p in poses)
            assert max_tilt <= args.max_tilt_change, 'vehicle up direction changed %.1f degrees' % max_tilt
            if args.full:
                assert len(native) >= 10, 'FAST-LIVO2 did not produce local odometry'
                assert prediction_counts and prediction_counts[-1] >= 10, 'FAST motion was never handed to FLOAM'
            print(json.dumps({'poses': len(poses), 'native_poses': len(native),
                              'fast_predictions': prediction_counts[-1] if prediction_counts else 0,
                              'first': poses[0], 'last': poses[-1],
                              'max_step_m': max_step, 'max_tilt_change_deg': max_tilt,
                              'states': list(dict.fromkeys(states))}, indent=2))
        finally:
            if play is not None and play.poll() is None:
                play.terminate()
                play.wait(timeout=10.)
            launch.terminate()
            launch.wait(timeout=20.)
            with open(os.path.join(args.output, 'poses.json'), 'w') as output:
                json.dump({'poses': poses, 'states': states, 'native': native,
                           'scans': scans, 'fast_prediction_counts': prediction_counts}, output)


if __name__ == '__main__':
    main()
