#!/usr/bin/env python3
"""ROS1 synthetic safety test, no cart/DBW/brakes ever contacted."""
import rospy,time
from std_msgs.msg import Bool,String
from geometry_msgs.msg import PoseStamped

rospy.init_node('test_watchdog_smoother_ros',anonymous=True)
trust=rospy.Publisher('/fast_livo2/step3_pose_trusted',Bool,queue_size=3)
pose=rospy.Publisher('/fast_livo2/step3_global_pose',PoseStamped,queue_size=3)
unsafe=rospy.Publisher('/fast_livo2/step3_outage_smoothing_untrusted',Bool,queue_size=3)
stops=[];reasons=[]
rospy.Subscriber('/fast_livo2/localization_stop_required',Bool,lambda x:stops.append(x.data))
rospy.Subscriber('/fast_livo2/localization_trust_reason',String,lambda x:reasons.append(x.data))
def expect(value,title,timeout=2.5):
 t=time.monotonic()
 while time.monotonic()-t<timeout:
  if stops and stops[-1] is value:
   print('PASS',title,'stop=',value,flush=True)
   return
  time.sleep(.03)
 raise AssertionError((title,stops[-8:],reasons[-8:]))
def fresh(i):
 trust.publish(Bool(data=True))
 m=PoseStamped()
 m.header.stamp=rospy.Time.now()
 m.header.frame_id='map'
 m.pose.position.x=i*.07
 m.pose.orientation.w=1.0
 pose.publish(m)
 time.sleep(.095)
expect(True,'startup fail-closed',4)
for i in range(13):fresh(i)
expect(False,'healthy independent-trust + continuous pose')
unsafe.publish(Bool(data=True))
expect(True,'independent smoothing-unsafe flag causes stop')
unsafe.publish(Bool(data=False))
for i in range(13,27):fresh(i)
expect(True,'false smoothing-unsafe flag cannot auto-clear hard stop')
assert any('HARD_POSE_FAULT' in x for x in reasons[-30:]),reasons[-10:]
print('PASS: ROS safety flag latches after smoothing error (SIGNALLING ONLY, NO ACTUATOR TEST)',flush=True)
