#!/usr/bin/env python3
"""Synthetic ROS1 integration test of watchdog (not hardware stop test).

Requires a ROS master and the opt-in watchdog node. Sends healthy/stale/jump
mock trajectories and verifies /fast_livo2/localization_stop_required.
No CAN, vehicle control, or navigation publishers are used.
"""
import time
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

rospy.init_node('test_localization_watchdog_ros',anonymous=True)
trust=rospy.Publisher('/fast_livo2/step3_pose_trusted',Bool,queue_size=4)
untrusted=rospy.Publisher('/fast_livo2/step3_outage_smoothing_untrusted',Bool,queue_size=4)
pose=rospy.Publisher('/fast_livo2/step3_global_pose',PoseStamped,queue_size=4)
events=[]
rospy.Subscriber('/fast_livo2/localization_stop_required',Bool,
                 lambda msg: events.append((time.monotonic(),msg.data)),queue_size=30)

def assert_state(expected,label,timeout=2.0):
    start=time.monotonic()
    while time.monotonic()-start<timeout:
        if events and events[-1][1] is expected:
            print('PASS',label,'stop_required=',expected,flush=True)
            return
        time.sleep(0.025)
    raise AssertionError(f'{label}: expected {expected}, got {events[-5:]}')

def publish_frame(x):
    trust.publish(Bool(data=True))
    m=PoseStamped()
    m.header.stamp=rospy.Time.now()
    m.header.frame_id='map'
    m.pose.position.x=x
    m.pose.orientation.w=1.
    pose.publish(m)
    time.sleep(0.095)

assert_state(True,'startup fail closed',4.0)
for i in range(12):publish_frame(i*.10)
assert_state(False,'verified continuous motion opens gate')
time.sleep(.80)
assert_state(True,'stale pose/independent trust forces stop')
for i in range(12,25):publish_frame(i*.10)
assert_state(False,'rearm on five fresh verified poses')
trust.publish(Bool(data=False))
assert_state(True,'explicit localization trust loss forces stop')
for i in range(25,37):publish_frame(i*.10)
assert_state(False,'rearm after trust restored')
publish_frame(50.)
assert_state(True,'implausible map jump hard-latches stop')
for i in range(12):publish_frame(50.+i*.1)
assert_state(True,'jump must not silently auto-clear')
untrusted.publish(Bool(data=True))
time.sleep(.16)
assert_state(True,'unsafe smoothing cannot clear hard stop')
print('PASS ROS1 watchdog end-to-end topics/timeout/trust-loss/hard-fault (NO ACTUATOR TEST)',flush=True)
