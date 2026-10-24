#!/usr/bin/env python3
"""Bounded, GPS-assisted visual relocalization against offline RGB-D 3D landmarks.

No second LiDAR/IMU estimator. Only activates on LiDAR loss; uses the last
LiDAR-verified map<-odom anchor with FAST-LIVO2 local odometry to bound search.
GPS helps propose candidates only. ORB provides the dependency-free baseline.
Map landmarks must be created from the SAME optimized world/map poses as LiDAR.
Publishes candidate geometry; a separate pose authority decides acceptance.
"""
import csv
import math
from pathlib import Path
import threading
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String


def pose_T(p):
    q = p.orientation
    from tf.transformations import quaternion_matrix
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def stamp_T(T, stamp):
    from tf.transformations import quaternion_from_matrix
    p = PoseStamped()
    p.header.stamp = stamp
    p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, T[:3, 3])
    q = quaternion_from_matrix(T)
    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = map(float, q)
    return p


class VisualGlobal:
    def __init__(self):
        root = Path(rospy.get_param("~visual_map"))
        rows = list(csv.DictReader((root/"keyframes.csv").open()))
        if not rows:
            raise RuntimeError("Empty visual keyframe map")
        self.entries = []
        for r in rows:
            feat = np.load(root/r["landmarks"])
            xyz, desc = feat["xyz"].astype(np.float32), feat["desc"].astype(np.uint8)
            if len(xyz) < 12 or len(desc) != len(xyz):
                continue
            g = (float(r["gps_lat"]), float(r["gps_lon"])) if r["gps_lat"] and r["gps_lon"] else None
            self.entries.append((np.array([float(r[k]) for k in ("x", "y", "z")]),
                                 g, xyz, desc, int(r["id"])))
        if not self.entries:
            raise RuntimeError("No depth-backed visual landmarks")
        self.locations = np.array([e[0] for e in self.entries])
        self.current_odom = None
        self.anchor = None
        self.last_lidar_lock = rospy.Time(0)
        self.mode = "FULL_LIVO"
        self.gps = None
        self.last_processed = rospy.Time(0)
        self.last_image = rospy.Time(0)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.orb = cv2.ORB_create(nfeatures=int(rospy.get_param("~orb_features", 1200)), fastThreshold=10)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.fx = float(rospy.get_param("/laserMapping/cam_fx"))
        self.fy = float(rospy.get_param("/laserMapping/cam_fy"))
        self.cx = float(rospy.get_param("/laserMapping/cam_cx"))
        self.cy = float(rospy.get_param("/laserMapping/cam_cy"))
        Rcl = np.asarray(rospy.get_param("/extrin_calib/Rcl"),float).reshape(3,3)
        Pcl = np.asarray(rospy.get_param("/extrin_calib/Pcl"),float)
        Ril = np.asarray(rospy.get_param("/extrin_calib/extrinsic_R"),float).reshape(3,3)
        Til = np.asarray(rospy.get_param("/extrin_calib/extrinsic_T"),float)
        T_camera_lidar = np.eye(4); T_camera_lidar[:3,:3]=Rcl;T_camera_lidar[:3,3]=Pcl
        T_imu_lidar = np.eye(4); T_imu_lidar[:3,:3]=Ril;T_imu_lidar[:3,3]=Til
        self.T_camera_body = T_camera_lidar @ np.linalg.inv(T_imu_lidar)
        self.hz = float(rospy.get_param("~query_hz", 1.0))
        self.local_radius = float(rospy.get_param("~local_radius_m", 15.0))
        self.max_radius = float(rospy.get_param("~max_local_radius_m", 60.0))
        self.max_candidates = int(rospy.get_param("~max_candidates", 28))
        self.min_inliers = int(rospy.get_param("~min_inliers", 18))
        self.min_coverage = float(rospy.get_param("~min_feature_spread_px", 95.0))
        self.max_reproj = float(rospy.get_param("~max_reproj_px", 3.0))
        self.max_age_sec = float(rospy.get_param("~max_age_sec", 0.35))
        self.confirm = int(rospy.get_param("~confirm_frames", 2))
        self.streak = 0
        self.last_candidate = None
        self.last_candidate_ts = None
        self.pub = rospy.Publisher("/fast_livo2/visual_global_candidate", PoseWithCovarianceStamped, queue_size=4)
        self.status = rospy.Publisher("/fast_livo2/visual_match_status", String, queue_size=5)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/aft_mapped_to_init"),Odometry,self.odom_cb,queue_size=30)
        rospy.Subscriber("/fast_livo2/step3_gicp_map_pose",PoseStamped,self.lidar_cb,queue_size=3)
        rospy.Subscriber("/fast_livo2/failsafe_mode",String,self.mode_cb,queue_size=3)
        rospy.Subscriber(rospy.get_param("~gps_topic","/mavros/global_position/raw/fix"),NavSatFix,self.gps_cb,queue_size=3)
        rospy.Subscriber(rospy.get_param("~camera_topic","/zedx/zed_node/left/image_rect_color"),Image,self.image_cb,queue_size=1,buff_size=9*1024*1024)
        rospy.loginfo("visual map matcher: loaded %d usable keyframes (ORB+metric PnP)",len(self.entries))

    def odom_cb(self, m):
        with self.lock:
            self.current_odom = (m.header.stamp, pose_T(m.pose.pose))

    def lidar_cb(self, m):
        with self.lock:
            if self.mode != "FULL_LIVO" or self.current_odom is None:
                return
            stamp, cur = self.current_odom
            if abs((m.header.stamp-stamp).to_sec()) > .18:
                return
            self.anchor = pose_T(m.pose) @ np.linalg.inv(cur)
            self.last_lidar_lock = m.header.stamp
            self.streak = 0
            self.last_candidate = None

    def mode_cb(self, m):
        with self.lock:
            self.mode = m.data
            if self.mode == "FULL_LIVO":
                self.streak = 0

    def gps_cb(self, m):
        if m.status.status < 0:
            return
        with self.lock:
            self.gps=(m.header.stamp,float(m.latitude),float(m.longitude),
                      max(16.,float(m.position_covariance[0]+m.position_covariance[4])))

    def candidates(self, stamp, current_pose, anchor, last_lock, gps):
        if current_pose is not None and anchor is not None and not last_lock.is_zero():
            predicted = (anchor @ current_pose)[:3,3]
            since = max(0., (stamp-last_lock).to_sec())
            # Uncertainty grows while the map anchor is stale; do NOT search
            # arbitrary old keyframe IDs or trust GPS as a hard constraint.
            radius = min(self.max_radius, self.local_radius + 0.45*since)
            dist=np.linalg.norm(self.locations[:,:2]-predicted[None,:2],axis=1)
            ids=np.argsort(dist)
            ids=[int(i) for i in ids if dist[i] <=radius]
            if ids:
                return ids[:self.max_candidates], predicted
        if gps and abs((stamp-gps[0]).to_sec()) < 2.0:
            lat,lon=gps[1:3]
            def meters(g):
                if g is None:return float("inf")
                return math.hypot((g[0]-lat)*111132.0,
                                  (g[1]-lon)*111320.0*math.cos(math.radians(lat)))
            scores = sorted(((meters(e[1]),i) for i,e in enumerate(self.entries)))
            # Soft regional ranking, not a hard GPS position correction.
            if scores and scores[0][0] < 120.0:
                return [i for _,i in scores[:self.max_candidates]],None
        return [],None  # No cold global retrieval on a real-time fallback worker.

    def image_cb(self, m):
        stamp=m.header.stamp
        with self.lock:
            mode=self.mode
            if mode not in ("VI_LIDAR_FAILED","IMU_ONLY_BRIDGE"):
                return
            if not self.last_processed.is_zero() and (stamp-self.last_processed).to_sec() < 1.0/max(.2,self.hz):
                return
            self.last_processed = stamp
            current=self.current_odom[1].copy() if self.current_odom is not None else None
            anchor=self.anchor.copy() if self.anchor is not None else None
            last_lock=self.last_lidar_lock
            gps=self.gps
        if abs((rospy.Time.now()-stamp).to_sec()) > self.max_age_sec:
            self.status.publish(String(data="stale image; no global fix"))
            return
        candidates,predicted=self.candidates(stamp,current,anchor,last_lock,gps)
        if not candidates:
            self.status.publish(String(data="no local/GPS search region"))
            return
        try:
            gray=self.bridge.imgmsg_to_cv2(m,desired_encoding="mono8")
        except Exception:
            return
        kp,desc=self.orb.detectAndCompute(gray,None)
        if desc is None or len(kp)<self.min_inliers:
            return
        K=np.array([[self.fx,0,self.cx],[0,self.fy,self.cy],[0,0,1]],dtype=np.float64)
        winner=None
        for idx in candidates:
            center,g,xyz,refdesc,kid=self.entries[idx]
            pairs=self.matcher.knnMatch(refdesc,desc,k=2)
            matches=[m[0] for m in pairs if len(m)==2 and m[0].distance<.72*m[1].distance]
            if len(matches)<self.min_inliers:
                continue
            obj=np.array([xyz[x.queryIdx] for x in matches],dtype=np.float64)
            pix=np.array([kp[x.trainIdx].pt for x in matches],dtype=np.float64)
            ok,rvec,tvec,inliers=cv2.solvePnPRansac(obj,pix,K,None,iterationsCount=80,
                reprojectionError=self.max_reproj,confidence=.995,flags=cv2.SOLVEPNP_EPNP)
            if not ok or inliers is None or len(inliers)<self.min_inliers:
                continue
            I=inliers[:,0]
            if np.ptp(pix[I,0])<self.min_coverage or np.ptp(pix[I,1])<self.min_coverage*.65:
                continue
            ok,rvec,tvec=cv2.solvePnP(obj[I],pix[I],K,None,rvec,tvec,True,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:continue
            proj,_=cv2.projectPoints(obj[I],rvec,tvec,K,None)
            err=float(np.median(np.linalg.norm(proj.reshape(-1,2)-pix[I],axis=1)))
            if err>self.max_reproj: continue
            R,_=cv2.Rodrigues(rvec)
            cam_map=np.eye(4);cam_map[:3,:3]=R;cam_map[:3,3]=tvec[:,0]
            T_map_body=np.linalg.inv(cam_map)@self.T_camera_body
            if predicted is not None:
                innovation=float(np.linalg.norm(T_map_body[:3,3]-predicted))
                if innovation > min(self.max_radius, self.local_radius+.7*max(0,(stamp-last_lock).to_sec())):
                    continue
            score=(len(I),-err)
            if winner is None or score>winner[0]:
                winner=(score,T_map_body,kid,err)
        if winner is None:
            self.streak=0
            self.status.publish(String(data="no geometrically verified visual match"))
            return
        _,T_map_body,kid,err=winner
        if (self.last_candidate is not None and self.last_candidate_ts is not None and
                (stamp-self.last_candidate_ts).to_sec()<2.5 and
                np.linalg.norm(T_map_body[:3,3]-self.last_candidate[:3,3])<3.5):
            self.streak+=1
        else:
            self.streak=1
        self.last_candidate=T_map_body
        self.last_candidate_ts=stamp
        if self.streak < self.confirm:
            self.status.publish(String(data="visual candidate waiting temporal confirmation"))
            return
        p=stamp_T(T_map_body,stamp)
        out=PoseWithCovarianceStamped()
        out.header=p.header;out.pose.pose=p.pose
        sigma=max(.15, min(2.0,.05*err+1./max(1.,winner[0][0])))
        for i in range(6):out.pose.covariance[7*i]=sigma*sigma if i<3 else .04
        self.pub.publish(out)
        self.status.publish(String(data="verified visual keyframe %d inliers=%d reproj=%.2fpx" %
                                   (kid,winner[0][0],err)))


if __name__=="__main__":
    rospy.init_node("fresh_visual_global_localizer")
    VisualGlobal()
    rospy.spin()
