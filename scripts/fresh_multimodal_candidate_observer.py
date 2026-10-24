#!/usr/bin/env python3
"""Read-only ROS1 visual-vs-LiDAR-hypothesis observer.
Emits String diagnostics ONLY; never publishes an anchor, /pose or mode change.
The Step3 bank can legitimately contain just ONE branch; in that case this
observer MUST NOT claim that visual sensing resolved an ambiguity.
"""
import collections,re
import numpy as np,rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

PAT=re.compile(r'\[id=(?P<id>-?\d+) hits=(?P<hits>\d+) fit=(?P<fit>[-+\d.eE]+) xy=(?P<x>[-+\d.eE]+),(?P<y>[-+\d.eE]+)\]')
class Observer:
 def __init__(self):
  self.max_age=rospy.Duration(float(rospy.get_param('~max_bank_age_sec',2.5)));self.good_radius=float(rospy.get_param('~visual_agreement_m',2.5));self.bad_radius=float(rospy.get_param('~visual_disagreement_m',8.0));self.min_votes=int(rospy.get_param('~min_visual_votes',2))
  self.max_sensor_offset=float(rospy.get_param('~max_bank_to_visual_sensor_offset_sec',1.5))
  self.bank_time=None;self.bank_sensor_stamp=None;self.bank=[];self.vote_history=collections.deque(maxlen=6)
  self.pub=rospy.Publisher('/fast_livo2/multimodal_branch_decision',String,queue_size=6)
  rospy.Subscriber('/fast_livo2/step3_mh_status',String,self.on_bank,queue_size=10)
  rospy.Subscriber('/fast_livo2/visual_global_candidate',PoseWithCovarianceStamped,self.on_visual,queue_size=10)
 def event(self,message):
  self.pub.publish(String(data='DIAGNOSTIC_ONLY '+message))
  rospy.loginfo('[CROSS-SENSOR] %s',message)
 def on_bank(self,m):
  # Step3 includes the LiDAR header timestamp in this status. Receipt time
  # alone is not a trustworthy proxy for measurement time in delayed replay.
  stamp=re.search(r'\bstamp=([-+\d.eE]+)',m.data)
  try:source_stamp=float(stamp.group(1)) if stamp else None
  except (TypeError,ValueError):source_stamp=None
  if source_stamp is None or not np.isfinite(source_stamp):
   self.bank=[];self.bank_sensor_stamp=None;self.vote_history.clear()
   self.event('UNTRUSTED_MISSING_LIDAR_HYPOTHESIS_SENSOR_STAMP');return
  if self.bank_sensor_stamp is not None and source_stamp<self.bank_sensor_stamp-1e-3:
   self.event('UNTRUSTED_OUT_OF_ORDER_LIDAR_BANK');return
  pairs=[]
  for match in PAT.finditer(m.data):
   d=match.groupdict();pairs.append({'id':int(d['id']),'hits':int(d['hits']),'fitness':float(d['fit']),'xy':np.array([float(d['x']),float(d['y'])])})
  self.bank=pairs;self.bank_time=rospy.Time.now();self.bank_sensor_stamp=source_stamp
 def on_visual(self,m):
  if self.bank_time is None or rospy.Time.now()-self.bank_time>self.max_age:
   self.vote_history.clear();self.event('UNTRUSTED_NO_FRESH_LIDAR_HYPOTHESES');return
  if not self.bank:
   self.vote_history.clear();self.event('UNTRUSTED_NO_LIDAR_HYPOTHESES');return
  if self.bank_sensor_stamp is None or abs(m.header.stamp.to_sec()-self.bank_sensor_stamp)>self.max_sensor_offset:
   self.vote_history.clear();self.event('UNTRUSTED_LIDAR_BANK_SENSOR_TIMESTAMP_MISMATCH');return
  xy=np.array([m.pose.pose.position.x,m.pose.pose.position.y],float)
  scored=sorted(((float(np.linalg.norm(xy-k['xy'])),k) for k in self.bank),key=lambda a:a[0])
  dist,top=scored[0]
  if dist>self.good_radius:
   self.vote_history.clear();self.event('UNTRUSTED_VISUAL_DISAGREES_WITH_ALL_LIDAR_BRANCHES nearest_id=%d error=%.2fm'%(top['id'],dist));return
  if len(scored)<2:
   self.vote_history.clear()
   self.event('VISUAL_SUPPORTS_ONLY_ONE_AVAILABLE_BRANCH id=%d error=%.2fm; ambiguity NOT resolved'%(top['id'],dist));return
  if top['hits']<2:
   self.vote_history.clear();self.event('UNTRUSTED_LIDAR_BRANCH_NOT_TEMPORALLY_SUPPORTED');return
  far=[(d,k) for d,k in scored[1:] if d>self.bad_radius]
  if not far:
   self.vote_history.clear();self.event('AMBIGUOUS_BRANCHES_NOT_WELL_SEPARATED visual_best_id=%d'%top['id']);return
  if not any(other['hits']>=2 for _,other in far):
   self.vote_history.clear();self.event('UNTRUSTED_COMPETING_LIDAR_BRANCH_NOT_TEMPORALLY_SUPPORTED');return
  stamp=m.header.stamp.to_sec();
  if self.vote_history and stamp<=self.vote_history[-1][0]:return
  self.vote_history.append((stamp,top['id']))
  votes=sum(k==top['id'] for _,k in self.vote_history)
  if votes>=self.min_votes:
   self.event('VISUAL_PREFERENCE_FOR_LIDAR_HYPOTHESIS id=%d votes=%d visual_error=%.2fm competing_sep=%.2fm -- NO POSE AUTHORITY'%(top['id'],votes,dist,far[0][0]))
  else:self.event('TEMPORAL_ARBITRATION_PENDING visual_support_id=%d votes=%d/%d'%(top['id'],votes,self.min_votes))
if __name__=='__main__':
 rospy.init_node('fresh_multimodal_candidate_observer');Observer();rospy.spin()
