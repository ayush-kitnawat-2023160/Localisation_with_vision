#!/usr/bin/env python3
"""GPS-FREE asynchronous ROS1 ZED X -> learned visual candidate bridge.

A separate localhost Torch inference worker does global ORB-LSH place retrieval,
learned local matching, and metric 2D/3D PnP. This ROS Noetic node is deliberately
fail-closed: stale/noisy/ambiguous results never create /pose and require 3
independent image confirmations before publishing the experiment-only candidate.
"""
import collections,json,queue,threading,time,urllib.request,urllib.error
import cv2,numpy as np,rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_matrix,quaternion_from_matrix

def rigid(R,p):
 T=np.eye(4);T[:3,:3]=np.asarray(R,float).reshape(3,3);T[:3,3]=p;return T
class Bridge:
 def __init__(self):
  self.worker_url=rospy.get_param('~worker_url','http://127.0.0.1:18090/localize')
  if not self.worker_url.startswith('http://127.0.0.1:') and not self.worker_url.startswith('http://localhost:'):
   raise RuntimeError('Experimental worker MUST be localhost-only')
  self.timeout=float(rospy.get_param('~request_timeout_sec',1.5));self.max_age=float(rospy.get_param('~max_sensor_age_sec',1.5));self.period=1./max(.2,float(rospy.get_param('~query_hz',1.0)))
  self.confirm=int(rospy.get_param('~confirm_frames',3))
  # Shadow-only background place checks are read-only, do not trigger /pose,
  # relocalization, anchor resets, or estimator input.
  self.always=bool(rospy.get_param('~always_evaluate_shadow',
                                  rospy.get_param('~always_evaluate_offline',False)))
  self.queue=queue.Queue(maxsize=1);self.shutdown=False;self.recent=collections.deque(maxlen=max(4,self.confirm));self.last_requested=rospy.Time(0);self.mode='FULL_LIVO'
  self.camera_gap_timeout=rospy.Duration(float(rospy.get_param('~camera_gap_timeout_sec',2.0)));self.camera_gap_reported=False
  self.bridge=CvBridge();self.status=rospy.Publisher('/fast_livo2/visual_match_status',String,queue_size=10)
  self.candidate=rospy.Publisher('/fast_livo2/visual_global_candidate',PoseWithCovarianceStamped,queue_size=4)
  self.stats=collections.Counter()
  Rcl=rospy.get_param('/extrin_calib/Rcl');Pcl=rospy.get_param('/extrin_calib/Pcl')
  Ril=rospy.get_param('/extrin_calib/extrinsic_R');Til=rospy.get_param('/extrin_calib/extrinsic_T')
  self.T_camera_body=rigid(Rcl,Pcl)@np.linalg.inv(rigid(Ril,Til))
  rospy.Subscriber(rospy.get_param('~camera_topic','/zedx/zed_node/left/image_rect_color'),Image,self.image_cb,queue_size=1,buff_size=8*1024*1024)
  rospy.Subscriber('/fast_livo2/failsafe_mode',String,self.mode_cb,queue_size=2)
  rospy.Timer(rospy.Duration(.6),self.watchdog)
  threading.Thread(target=self.process,daemon=True).start()
  rospy.logwarn('[VISUAL LEARNED] experimental GPS-FREE candidate-only mode, /pose NOT published; backend %s',self.worker_url)
 def mode_cb(self,msg):self.mode=msg.data
 def report(self,text):
  self.status.publish(String(data='LEARNED_VISUAL '+text))
 def age(self,stamp):
  now=rospy.Time.now()
  return (now-stamp).to_sec() if not now.is_zero() else 0
 def watchdog(self,event):
  if not self.last_requested.is_zero() and rospy.Time.now()-self.last_requested>self.camera_gap_timeout:
   self.recent.clear()
   if not self.camera_gap_reported:
    self.camera_gap_reported=True;self.report('CAMERA_GAP_UNTRUSTED: no pose authority')
 def image_cb(self,msg):
  if not self.always and self.mode not in ('VI_LIDAR_FAILED','IMU_ONLY_BRIDGE','GLOBAL_RELOCALIZING','RELOCALIZING'):return
  if self.age(msg.header.stamp)>self.max_age or self.age(msg.header.stamp)<-.25:self.stats['stale_input']+=1;self.report('REJECT_STALE_INPUT');return
  if not self.last_requested.is_zero() and (msg.header.stamp-self.last_requested).to_sec()<self.period:return
  self.last_requested=msg.header.stamp;self.camera_gap_reported=False
  try:
   if self.queue.full():self.queue.get_nowait();self.stats['drop_old_image']+=1
   self.queue.put_nowait(msg)
  except queue.Full:self.stats['drop_new_image']+=1
 def process(self):
  while not rospy.is_shutdown() and not self.shutdown:
   try:msg=self.queue.get(timeout=.25)
   except queue.Empty:continue
   if self.age(msg.header.stamp)>self.max_age:self.report('REJECT_STALE_QUEUED');continue
   try:
    img=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8');ok,enc=cv2.imencode('.jpg',img,[cv2.IMWRITE_JPEG_QUALITY,87]);
    if not ok:continue
    req=urllib.request.Request(self.worker_url,data=enc.tobytes(),headers={'Content-Type':'image/jpeg','X-Sensor-Stamp':str(msg.header.stamp.to_sec())},method='POST')
    t0=time.monotonic();data=json.load(urllib.request.urlopen(req,timeout=self.timeout));elapsed=(time.monotonic()-t0)*1000
   except Exception as e:
    self.stats['service_error']+=1;self.recent.clear();self.report('WORKER_UNAVAILABLE '+str(e)[:60]);continue
   if abs(data.get('stamp_s',-1)-msg.header.stamp.to_sec())>1e-3 or self.age(msg.header.stamp)>self.max_age:
    self.recent.clear();self.report('REJECT_STALE_RESPONSE');continue
   if not data.get('matched',False) or data.get('ambiguous',False) or not data.get('best'):
    self.recent.clear();self.report('UNTRUSTED_NO_MATCH_OR_AMBIGUOUS');continue
   pose=data['best'];q=np.array(pose['T_map_camera_qxyzw'],float)
   if len(q)!=4 or np.linalg.norm(q)<.9:self.recent.clear();continue
   T_map_cam=quaternion_matrix(q);T_map_cam[:3,3]=pose['T_map_camera_xyz']
   T_map_body=T_map_cam@self.T_camera_body
   if not np.isfinite(T_map_body).all():self.recent.clear();continue
   stamp=msg.header.stamp.to_sec();pos=T_map_body[:3,3].copy();rot=T_map_body[:3,:3].copy()
   if self.recent:
    prev_t,prev_pos,prev_rot=self.recent[-1];dt=stamp-prev_t
    d=float(np.linalg.norm(pos-prev_pos));ang=np.degrees(np.arccos(np.clip((np.trace(prev_rot.T@rot)-1)/2,-1,1)))
    if dt<=0 or dt>4.0 or d>1.0+5.0*dt or ang>15.0+70.0*dt:
     self.stats['motion_reject']+=1;self.recent.clear();self.report('REJECT_TEMPORAL_MOTION');continue
   self.recent.append((stamp,pos,rot))
   if len(self.recent)<self.confirm:
    self.report('VERIFYING %d/%d inliers=%d ms=%.0f'%(len(self.recent),self.confirm,pose['inliers'],elapsed));continue
   # Candidate only: covariance is conservative heuristic, not calibrated.
   out=PoseWithCovarianceStamped();out.header=msg.header;out.header.frame_id='map'
   out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,pos)
   pq=quaternion_from_matrix(T_map_body)
   out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,pq)
   for j in range(6):out.pose.covariance[j*6+j]=4.0 if j<3 else .3
   self.candidate.publish(out);self.stats['confirmed_candidate']+=1
   self.report('TEMPORALLY_CONFIRMED_CANDIDATE keyframe=%d inliers=%d reproj=%.2fpx latency=%.0fms'%(pose['id'],pose['inliers'],pose['reprojection_px'],elapsed))
if __name__=='__main__':
 rospy.init_node('fresh_visual_global_learned_bridge');Bridge();rospy.spin()
