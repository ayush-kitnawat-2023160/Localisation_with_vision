#!/usr/bin/env python3
"""GPS-free fail-closed, candidate-only ROS1 recovery handoff reviewer.

Does NOT publish /pose, a map->odom transform, any authority topic, or change
Step3 state. 'Eligible' is only a diagnostic to show a stopped-cart operator.
Vehicle stop interlocks / surveyed map verification do NOT exist here.
"""
from collections import deque
import math
import os
import sys
import rospy
import rospkg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from tf.transformations import euler_from_quaternion
# Catkin's devel-space generated relay may not add source/scripts to sys.path.
sys.path.insert(0,os.path.join(rospkg.RosPack().get_path('fast_livo2'),'scripts'))
from safe_handoff_rules import PoseSample, evaluate_review


class Review:
    def __init__(self):
        self.status = ''
        self.last_status_received = rospy.Time(0)
        self.lidar = deque(maxlen=40)
        self.visual = deque(maxlen=40)
        self.odom = deque(maxlen=180)
        self.previous = None
        self.external_stop = False
        self.external_stop_last = rospy.Time(0)
        self.review_status = rospy.Publisher('/fast_livo2/mh_handoff_review_status',String,queue_size=4,latch=True)
        rospy.Subscriber('/fast_livo2/step3_status',String,self.on_status,queue_size=4)
        rospy.Subscriber('/fast_livo2/step3_mh_recovery_candidate',PoseWithCovarianceStamped,
                         lambda msg:self.accept_pose(msg,self.lidar),queue_size=12)
        rospy.Subscriber('/fast_livo2/visual_global_candidate',PoseWithCovarianceStamped,
                         lambda msg:self.accept_pose(msg,self.visual),queue_size=12)
        rospy.Subscriber('/aft_mapped_to_init',Odometry,self.on_odom,queue_size=100)
        # Must originate from an independently verified vehicle-control stop
        # interlock, never from estimated FAST-LIVO2 speed or camera odometry.
        # It is absent in offline bags and so defaults to fail closed.
        rospy.Subscriber('/fast_livo2/external_stop_verified',Bool,self.on_external_stop,queue_size=2)
        rospy.Timer(rospy.Duration(.5),self.check)
        rospy.logwarn('[MH HANDOFF REVIEW] diagnostics ONLY; no /pose; no autonomous handoff')

    def on_status(self,msg):
        self.status = msg.data
        self.last_status_received = rospy.Time.now()
    def on_external_stop(self,msg):
        self.external_stop = bool(msg.data)
        self.external_stop_last = rospy.Time.now()
    def accept_pose(self,msg,buffer):
        if msg.header.frame_id!='map' or msg.header.stamp.is_zero():return
        p=msg.pose.pose
        q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
        if not all(math.isfinite(x) for x in q):return
        y=euler_from_quaternion(q)[2]
        t=msg.header.stamp.to_sec()
        if buffer and t<=buffer[-1].stamp:return
        buffer.append(PoseSample(t,(p.position.x,p.position.y,p.position.z),y))
    def on_odom(self,msg):
        p=msg.pose.pose; q=p.orientation
        t=msg.header.stamp.to_sec()
        if self.odom and t<=self.odom[-1].stamp:return
        tw=msg.twist.twist
        speed=math.sqrt(tw.linear.x**2+tw.linear.y**2+tw.linear.z**2)
        self.odom.append(PoseSample(t,(p.position.x,p.position.y,p.position.z),
                       euler_from_quaternion([q.x,q.y,q.z,q.w])[2],speed,tw.angular.z))
    def check(self,_):
        now=rospy.Time.now().to_sec()
        if now<=0:return
        external_fresh=(self.external_stop and not self.external_stop_last.is_zero() and
                        0 <= now-self.external_stop_last.to_sec() <= 1.0)
        status_fresh=(not self.last_status_received.is_zero() and
                      0<=now-self.last_status_received.to_sec()<=2.5)
        status=self.status if status_fresh else 'STALE_STEP3_GLOBAL_STATUS'
        ok,why,detail=evaluate_review(status,list(self.lidar),list(self.visual),
                                       list(self.odom),now,externally_verified_stop=external_fresh)
        result=('ELIGIBLE_FOR_OFFLINE_MANUAL_REVIEW' if ok else 'UNTRUSTED')+' '+why
        if result!=self.previous:
            self.review_status.publish(String(data=result))
            rospy.logwarn('[MH HANDOFF REVIEW] %s; %s',result,detail)
            self.previous=result

if __name__=='__main__':
    rospy.init_node('fresh_safe_handoff_review')
    Review()
    rospy.spin()
