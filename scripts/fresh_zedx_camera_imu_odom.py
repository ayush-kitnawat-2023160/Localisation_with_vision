#!/usr/bin/env python3
"""ROS Noetic candidate-only ZED X RGB-D + IMU relative odometry.

No LiDAR, GPS, global map, FAST-LIVO2 state, or /pose required. Publishes an
isolated `camera_imu_odom` trajectory in a *private local frame*, not a TF and
not vehicle odometry authority. The inherited body<->camera transform is for
OFFLINE tests only and must be physically verified before live use.

Health messages deliberately distinguish vision + gyro from visual-only,
failed depth/flow, stale IMU, and stopped output. A failure never invents
forward motion from a stale velocity.
"""
import os,sys,threading,math,time
from collections import OrderedDict
import numpy as np,cv2,rospy
from sensor_msgs.msg import Image,Imu,PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix

sys.path.insert(0,os.path.join(os.path.dirname(__file__)))
from zedx_rgbd_imu_odom_core import ZedRGBDIMUOdometry,pose


class CameraIMUOdometry:
    def __init__(self):
        self.lock=threading.RLock()
        self.width=int(rospy.get_param('/laserMapping/cam_width',960))
        self.height=int(rospy.get_param('/laserMapping/cam_height',600))
        fx,fy,cx,cy=[float(rospy.get_param('/laserMapping/'+k)) for k in ('cam_fx','cam_fy','cam_cx','cam_cy')]
        self.scale=float(rospy.get_param('~image_scale',.65))
        self.period=float(rospy.get_param('~frame_period_sec',.065))
        self.max_output_gap=float(rospy.get_param('~max_output_gap_sec',.65))
        self.max_image_silence=float(rospy.get_param('~max_image_silence_sec',.7))
        self.max_pair_offset=float(rospy.get_param('~max_pair_offset_sec',.005))
        self.require_imu=bool(rospy.get_param('~require_imu_for_healthy',True))
        self.lidar_missing_after=float(rospy.get_param('~lidar_missing_after_sec',.65))
        self.offline_only=bool(rospy.get_param('~offline_only',True))
        self.shadow_only=bool(rospy.get_param('~shadow_only',True))
        if not self.shadow_only:
            raise ValueError('KLT fallback can only run in isolated, NON-AUTHORITATIVE SHADOW mode')
        if not self.offline_only and not bool(rospy.get_param('~explicit_live_shadow_ack',False)):
            raise ValueError('Live shadow KLT requires ~explicit_live_shadow_ack; never /pose or TF')
        K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]],float);K[:2,:]*=self.scale
        # Do NOT hard-code transforms inferred from an old rosbag. Current
        # offline calibration provides a clearly labeled provisional matrix.
        # A live shadow requires its own physically checked profile.
        imu_opt=rospy.get_param('/camera_imu_fallback/imu_to_optical_rotation',None)
        source=str(rospy.get_param('/camera_imu_fallback/rotation_source','UNSPECIFIED'))
        if not isinstance(imu_opt,list) or len(imu_opt)!=9:
            raise ValueError('Missing 3x3 camera_imu_fallback/imu_to_optical_rotation; calibration required')
        Roptimu=np.asarray(imu_opt,float).reshape(3,3)
        if not np.isfinite(Roptimu).all() or \
                abs(np.linalg.det(Roptimu)-1)>1e-3 or \
                np.linalg.norm(Roptimu.T@Roptimu-np.eye(3))>1e-3:
            raise ValueError('Invalid optical<-IMU rotation matrix')
        if not self.offline_only and not source.upper().startswith('PHYSICALLY_MEASURED_'):
            raise ValueError('Live shadow requires traceable PHYSICALLY_MEASURED_ ZED camera/IMU calibration')
        self.core=ZedRGBDIMUOdometry(K,R_opt_imu=Roptimu,track_points=650)
        # Express the initial optical-camera VO frame in the assumed body frame,
        # then return a body trajectory. This must never be TF broadcast until
        # the physical sensor->body installation has been remeasured.
        bRl=np.asarray(rospy.get_param('/extrin_calib/extrinsic_R'),float).reshape(3,3)
        btl=np.asarray(rospy.get_param('/extrin_calib/extrinsic_T'),float)
        cRl=np.asarray(rospy.get_param('/extrin_calib/Rcl'),float).reshape(3,3)
        ctl=np.asarray(rospy.get_param('/extrin_calib/Pcl'),float)
        self.T_body_cam=pose(bRl,btl)@np.linalg.inv(pose(cRl,ctl))
        if not np.all(np.isfinite(self.T_body_cam)):
            raise ValueError('Invalid camera/body extrinsics')
        self.T_cam_body=np.linalg.inv(self.T_body_cam)
        self.depth=OrderedDict();self.rgb_buffer=OrderedDict();self.last_frame=-1e30
        self.last_lidar=None;self.last_imu=None
        self.valid=0;self.invalid=0;self.processed=0;self.last_status='INIT'
        self.last_published_stamp=None
        self.last_image_received_stamp=None
        self.continuity_lost=False
        self.odom_pub=rospy.Publisher('/fast_livo2/fallback/camera_imu_odom_candidate',Odometry,queue_size=10)
        self.status_pub=rospy.Publisher('/fast_livo2/fallback/camera_imu_status',String,queue_size=15,latch=True)
        image_topic=rospy.get_param('~image_topic','/zedx/zed_node/left/image_rect_color')
        depth_topic=rospy.get_param('~depth_topic','/zedx/zed_node/depth/depth_registered')
        imu_topic=rospy.get_param('~imu_topic','/zedx/zed_node/imu/data')
        lidar_topic=rospy.get_param('~lidar_topic','/lidar103/velodyne_points')
        self.ims=rospy.Subscriber(image_topic,Image,self.on_color,queue_size=2,buff_size=6*1024*1024)
        self.deps=rospy.Subscriber(depth_topic,Image,self.on_depth,queue_size=2,buff_size=6*1024*1024)
        self.imus=rospy.Subscriber(imu_topic,Imu,self.on_imu,queue_size=90)
        self.lidars=rospy.Subscriber(lidar_topic,PointCloud2,self.on_lidar,queue_size=1)
        self.health_timer=rospy.Timer(rospy.Duration(.2),self.on_health_watchdog)
        rospy.logwarn('[CAMERA_IMU_ODOM] %s CANDIDATE ONLY. No TF, /pose, FAST-LIVO2 estimator writes or controller outputs. source=%s',
                      'OFFLINE' if self.offline_only else 'LIVE_SHADOW', source)
        self.publish_status('WARMUP_NOT_TRUSTED',0,0,0)

    @staticmethod
    def rgb(m):
        a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)
        if m.encoding=='bgra8':return cv2.cvtColor(a[:,:m.width*4].reshape(m.height,m.width,4),cv2.COLOR_BGRA2BGR)
        if m.encoding=='bgr8':return a[:,:m.width*3].reshape(m.height,m.width,3).copy()
        if m.encoding=='rgb8':return cv2.cvtColor(a[:,:m.width*3].reshape(m.height,m.width,3),cv2.COLOR_RGB2BGR)
        raise ValueError('Unrecognized ZED X image encoding: '+m.encoding)

    def publish_status(self,state,stamp,inliers,reproj,parallax_deg=float('nan')):
        # Reporting absence of LiDAR does NOT downgrade optical visual tracking.
        if self.last_lidar is None:lidar_condition='LIDAR_NOT_OBSERVED'
        else:
            dt=stamp-self.last_lidar
            lidar_condition='LIDAR_MISSING' if dt>self.lidar_missing_after else 'LIDAR_PRESENT'
        # Low parallax also occurs while STOPPED. This is not a safety gate;
        # it is telemetry for offline estimator benchmarking/health analysis.
        parallax_category=('UNKNOWN' if not math.isfinite(parallax_deg) else
                           ('LOW_PARALLAX_OR_STATIONARY' if parallax_deg<.10 else 'MEASURABLE'))
        message=('state=%s lidar=%s stamp=%.9f inliers=%d reproj_px=%.3f accepted=%d rejected=%d '
                 'parallax_deg=%.4f parallax_observability=%s '
                 'authority=FALSE offline_only=%s' %
                 (state,lidar_condition,stamp,inliers,reproj,self.valid,self.invalid,
                  parallax_deg,parallax_category,str(self.offline_only).upper()))
        self.status_pub.publish(String(data=message))
        self.last_status=state

    def on_health_watchdog(self,_):
        with self.lock:
            if self.last_image_received_stamp is None or self.last_published_stamp is None:return
            now=rospy.Time.now().to_sec()
            if now<1 or not math.isfinite(now):return
            dt=now-self.last_image_received_stamp
            # Sensor stream delayed through ROS? Never fabricate local motion.
            # A long lapse permanently invalidates the old session, so a new
            # image cannot silently resume navigation in a stale local frame.
            if dt>self.max_image_silence and not self.continuity_lost:
                self.continuity_lost=True
                self.publish_status('UNTRUSTED_CAMERA_TIMEOUT_REARM_REQUIRED',now,0,float('inf'))

    def on_lidar(self,m):
        with self.lock:self.last_lidar=m.header.stamp.to_sec()
    def on_imu(self,m):
        o=m.angular_velocity;t=m.header.stamp.to_sec()
        with self.lock:
            self.core.add_imu(t,[o.x,o.y,o.z]);self.last_imu=t

    def on_color(self,m):
        with self.lock:
            key=round(m.header.stamp.to_sec(),5)
            other=self.depth.pop(key,None)
            if other is None:
                self.rgb_buffer[key]=m
                while len(self.rgb_buffer)>7:self.rgb_buffer.popitem(last=False)
                return
            self.handle(m,other)

    def on_depth(self,m):
        if m.encoding!='32FC1':return
        with self.lock:
            key=round(m.header.stamp.to_sec(),5)
            other=self.rgb_buffer.pop(key,None)
            if other is None:
                self.depth[key]=m
                while len(self.depth)>7:self.depth.popitem(last=False)
                return
            self.handle(other,m)

    def handle(self,im,dep):
        stamp=im.header.stamp.to_sec()
        if abs(stamp-dep.header.stamp.to_sec())>self.max_pair_offset:return
        if stamp<=self.last_frame:return
        if stamp-self.last_frame<self.period:return
        self.last_frame=stamp
        self.last_image_received_stamp=stamp
        if self.continuity_lost:
            self.publish_status('REARM_REQUIRED_AFTER_CAMERA_ODOM_GAP',stamp,0,float('inf'))
            return
        try:
            color=self.rgb(im)
            depth=np.frombuffer(dep.data,np.float32).reshape(dep.height,dep.step//4)[:,:dep.width].copy()
            if self.scale!=1.0:
                color=cv2.resize(color,None,fx=self.scale,fy=self.scale,interpolation=cv2.INTER_AREA)
                depth=cv2.resize(depth,(color.shape[1],color.shape[0]),interpolation=cv2.INTER_NEAREST)
            res=self.core.process(stamp,color,depth)
        except Exception as exc:
            self.invalid+=1
            rospy.logerr_throttle(2.,'[CAMERA_IMU_ODOM] failed RGB-D processing: %s',exc)
            self.publish_status('UNTRUSTED_PROCESSING_ERROR',stamp,0,float('inf'))
            return
        self.processed+=1
        if res.state not in ('VISION_IMU_TRACKING','VISION_ONLY_IMU_STALE'):
            self.invalid+=1
            self.publish_status('UNTRUSTED_'+res.state,stamp,res.inliers,res.reproj_px)
            return
        if res.state=='VISION_ONLY_IMU_STALE' and self.require_imu:
            self.invalid+=1
            self.publish_status('UNTRUSTED_GYRO_UNAVAILABLE',stamp,res.inliers,res.reproj_px)
            return
        if (self.last_published_stamp is not None and
            stamp-self.last_published_stamp>self.max_output_gap):
            self.continuity_lost=True
            self.invalid+=1
            self.publish_status('REARM_REQUIRED_AFTER_CAMERA_ODOM_GAP',stamp,res.inliers,res.reproj_px)
            return
        self.valid+=1
        self.last_published_stamp=stamp
        # T_initialBody_currentBody. A private frame, no map authority.
        T=self.T_body_cam@res.transform@self.T_cam_body
        q=quaternion_from_matrix(T)
        out=Odometry();out.header.stamp=rospy.Time.from_sec(stamp)
        out.header.frame_id='zedx_camera_imu_initial_body'
        out.child_frame_id='zedx_body_candidate_not_base_link'
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,T[:3,3])
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
        # Conservative heuristic covariance: not filter covariance. Grows
        # monotonically with accumulated travel; never a global-localization
        # confidence score or sensor-level safety certificate.
        travelled=self.core.accepted
        variance=max(.04,.02*travelled+.10*(1.-min(res.inliers/100.,1.)))
        out.pose.covariance[0]=variance
        out.pose.covariance[7]=variance
        out.pose.covariance[14]=2.*variance
        out.pose.covariance[21]=.04+.01*travelled
        out.pose.covariance[28]=.04+.01*travelled
        out.pose.covariance[35]=.06+.015*travelled
        # No twist message is synthesized from a missing or rejected vision
        # update. Downstream must obtain a true velocity filter separately.
        out.twist.covariance[0]=1000.
        out.twist.covariance[7]=1000.
        out.twist.covariance[14]=1000.
        self.odom_pub.publish(out)
        self.publish_status('TRACKING_VISION_GYRO' if res.state=='VISION_IMU_TRACKING' else 'DEGRADED_VISION_ONLY',stamp,res.inliers,res.reproj_px,parallax_deg=res.parallax_deg)


if __name__=='__main__':
    rospy.init_node('fresh_zedx_camera_imu_odom')
    CameraIMUOdometry()
    rospy.spin()
