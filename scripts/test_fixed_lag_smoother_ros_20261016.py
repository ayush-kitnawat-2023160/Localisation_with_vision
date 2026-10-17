#!/usr/bin/env python3
"""End-to-end ROS1 shadow fixed-lag publisher sanity check. NO actuators."""
import math,time
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
rospy.init_node('test_fixed_lag_smoother_preview',anonymous=True)
pub=rospy.Publisher('/fast_livo2/step3_global_pose',PoseStamped,queue_size=150)
received=[];flags=[]
rospy.Subscriber('/fast_livo2/step3_global_pose_smoothed_preview',PoseStamped,
    lambda m:received.append((m.header.stamp.to_sec(),m.pose.position.x,m.pose.position.y)))
rospy.Subscriber('/fast_livo2/step3_visual_smoother_numeric_unsafe',Bool,
    lambda m:flags.append(m.data))
start=time.monotonic()
while pub.get_num_connections()<1 and time.monotonic()-start<8:time.sleep(.05)
assert pub.get_num_connections()>=1,'Shadow node not connected'
base=1788852570.0
times=[.1*i for i in range(25)]+[2.5+.2*i for i in range(16)]+[5.7+.1*i for i in range(22)]
for i,t in enumerate(times):
 m=PoseStamped();m.header.stamp=rospy.Time.from_sec(base+t);m.header.frame_id='map'
 m.pose.position.x=t*.9
 m.pose.position.y=0.08*math.sin(t)
 m.pose.orientation.w=1
 if 3.3<=t<3.65:m.pose.position.x+=2.6
 pub.publish(m)
 time.sleep(.027)
time.sleep(.8)
assert len(received)>35,('Not enough published frames',len(received))
ts=np.array([x[0] for x in received]);x=np.array([x[1] for x in received])
assert np.diff(ts).min()>0,'Out-of-order preview stamps'
assert abs(np.median(np.diff(ts))-.1)<.03,'Timestamp drift'
assert np.isfinite(x).all()
assert any(flags), 'Synthetic map spike must set numerical unsafe'
assert min(ts)>base and max(ts)<base+times[-1]
assert len(received)>len(times)*.73,'Unexpected preview poses dropped when source rate is 5Hz'
print('PASS ROS1 preview publishes',len(received),'of',len(times),'source poses, unsafe flag count',sum(flags),'pose stamp monotonic, 10Hz/5Hz rate supported, NO DBW touched',flush=True)
