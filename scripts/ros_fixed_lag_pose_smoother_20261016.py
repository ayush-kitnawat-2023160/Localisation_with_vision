#!/usr/bin/env python3
"""ROS1 SHADOW visualisation of map-frame smoothing, never command authority.

Consumes existing global PoseStamped, publishes new topic ONLY.
Inputs:   /fast_livo2/step3_global_pose
Preview:  /fast_livo2/step3_global_pose_smoothed_preview
Flags:    /fast_livo2/step3_visual_smoother_numeric_unsafe
          /fast_livo2/step3_visual_smoother_disagreement_m

This node is not localisation. 'numerically_continuous' is not map trust and
cannot clear the real stop-required watchdog. No /cmd_vel, DBW, or nav output.
"""
import os
import sys
sys.path.insert(0,os.path.dirname(__file__))
from robust_fixed_lag_pose_smoother_20261016 import FixedLagPoseSmoother

def main():
 import rospy
 from geometry_msgs.msg import PoseStamped
 from std_msgs.msg import Bool,Float32
 rospy.init_node('step3_fixed_lag_smoother_preview',anonymous=False)
 lag=rospy.get_param('~lag_sec',.23)
 window=rospy.get_param('~fit_window_sec',1.15)
 degree=rospy.get_param('~fit_degree',1)
 filt=FixedLagPoseSmoother(lag=lag,window=window,degree=degree,
   max_gap=rospy.get_param('~max_gap_sec',.35),
   max_speed=rospy.get_param('~max_numeric_speed_mps',5.0),
   max_raw_disagreement=rospy.get_param('~max_disagreement_m',1.0))
 topic_in=rospy.get_param('~source','/fast_livo2/step3_global_pose')
 pub=rospy.Publisher('/fast_livo2/step3_global_pose_smoothed_preview',PoseStamped,queue_size=10)
 unsafe_pub=rospy.Publisher('/fast_livo2/step3_visual_smoother_numeric_unsafe',Bool,queue_size=10)
 disagreement_pub=rospy.Publisher('/fast_livo2/step3_visual_smoother_disagreement_m',Float32,queue_size=10)
 state={'samples':0,'continuity_faults':0}
 def cb(m):
  if m.header.frame_id not in ('map','global_map'):
   unsafe_pub.publish(Bool(data=True))
   rospy.logwarn_throttle(2,'[SMOOTH PREVIEW] Unknown map frame; not publishing')
   return
  p=m.pose.position; q=m.pose.orientation
  result=filt.ingest(m.header.stamp.to_sec(),
    [p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
  if result is None:
   unsafe_pub.publish(Bool(data=True))
   return
  out=PoseStamped()
  out.header.stamp=rospy.Time.from_sec(result.stamp)
  out.header.frame_id=m.header.frame_id
  out.pose.position.x=float(result.xyz[0])
  out.pose.position.y=float(result.xyz[1])
  out.pose.position.z=float(result.xyz[2])
  out.pose.orientation.x=float(result.quat[0])
  out.pose.orientation.y=float(result.quat[1])
  out.pose.orientation.z=float(result.quat[2])
  out.pose.orientation.w=float(result.quat[3])
  pub.publish(out)
  unsafe_pub.publish(Bool(data=not result.continuous))
  disagreement_pub.publish(Float32(data=result.raw_disagreement_m))
  state['samples']+=1
  if not result.continuous:
   state['continuity_faults']+=1
   rospy.logwarn_throttle(1,'[SMOOTH PREVIEW] NUMERICALLY_UNSAFE: '+result.reason)
  if state['samples']%100==0:
   rospy.loginfo('[SMOOTH PREVIEW] %d poses, %d numeric discontinuities, fixed-lag %.2fs (NOT MAP TRUST)',
     state['samples'],state['continuity_faults'],lag)
 rospy.Subscriber(topic_in,PoseStamped,cb,queue_size=120,tcp_nodelay=True)
 rospy.logwarn('Shadow smoothing only. Delayed map pose is NOT navigation-ready; do not remap vehicle/planner pose to this preview.')
 rospy.spin()
if __name__=='__main__':main()
