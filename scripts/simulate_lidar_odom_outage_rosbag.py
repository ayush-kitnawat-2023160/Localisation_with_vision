#!/usr/bin/env python3
"""TEST FIXTURE: only relay saved FAST odom before 20s; publish fake failure flag.
Do NOT use its flag as a real vehicle sensor-health attestation.
"""
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
rospy.init_node('offline_only_lidar_odom_outage_fixture')
start=float(rospy.get_param('~bag_first_lidar_stamp'))
a=float(rospy.get_param('~outage_start_sec',20.0))
b=float(rospy.get_param('~outage_end_sec',55.0))
fast_pub=rospy.Publisher('/fallback_test/healthy_fast_odom',Odometry,queue_size=30)
health_pub=rospy.Publisher('/fallback_test/lidar_failed_simulated',Bool,queue_size=5,latch=True)
last=None

def odom(m):
    stamp=m.header.stamp.to_sec()-start
    if stamp<a:fast_pub.publish(m)


def publish(_):
    global last
    t=rospy.Time.now().to_sec()-start
    state=a<=t<b
    if state!=last:
        health_pub.publish(Bool(data=state));last=state
        rospy.logwarn('[SIMULATED_LIDAR_FAILURE] t=%.2f failed=%s',t,state)

rospy.Subscriber('/aft_mapped_to_init',Odometry,odom,queue_size=50)
rospy.Timer(rospy.Duration(.075),publish)
rospy.spin()
