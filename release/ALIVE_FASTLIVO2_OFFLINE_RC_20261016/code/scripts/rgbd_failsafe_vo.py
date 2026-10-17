#!/usr/bin/env python3
import threading
import math
from collections import deque
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped


def mat3(v):
    a = np.asarray(v, dtype=np.float64)
    return a.reshape(3, 3)


def vec3(v):
    return np.asarray(v, dtype=np.float64).reshape(3)


def T_from(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


class RGBDFailsafeVO:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        # Serialize VO processing against mode-transition reset. rospy may run
        # subscriber callbacks concurrently; without this guard FULL_LIVO
        # recovery could clear prev_kp/prev_desc while color_cb was using them.
        self.processing_lock = threading.Lock()
        self.mode = "FULL_LIVO"
        self.latest_depth = None
        self.latest_depth_rx = rospy.Time(0)
        self.latest_depth_stamp = rospy.Time(0)
        # Short event-time history: callback arrival order is not sensor-time order
        # under rosbag/CPU load. Nearest-time pairing prevents valid RGB-D frames
        # from being rejected solely because a newer depth callback arrived late.
        self.depth_buffer = deque(maxlen=max(4, int(rospy.get_param("~depth_buffer_size", 12))))
        self.depth_clock_offset = None
        self.prev_gray = None
        self.prev_depth_color = None
        self.prev_stamp = None
        self.prev_kp = None
        self.prev_desc = None
        # The standalone RGB-D failure experiment was only stable once the
        # relative motion itself was made causal. Keep the same filter here so
        # an isolated PnP/3D-3D outlier cannot inject a 6-12 m/s velocity into
        # the persistent FAST-LIVO2 ESIKF.
        self.filt_v = None
        self.filt_w = None
        # Keep the runtime front-end numerically identical to the standalone
        # RGB-D fallback that achieved ~0.70 m RMSE.  In particular it operates
        # on every third RGB frame and filters the CAMERA-frame SE(3) increment
        # before converting that increment into the FAST-LIVO2 body frame.
        self.max_linear_accel = float(rospy.get_param("~max_linear_accel", 5.0))
        self.max_angular_accel = float(rospy.get_param("~max_angular_accel", 1.6))
        self.filter_alpha = float(rospy.get_param("~filter_alpha", 0.45))
        self.bootstrap_max_speed = float(rospy.get_param("~bootstrap_max_speed", 5.0))
        self.bootstrap_max_omega = float(rospy.get_param("~bootstrap_max_omega", 1.2))

        self.color_topic = rospy.get_param("~color_topic", "/fast_livo2/input/image")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
        self.depth_info_topic = rospy.get_param("~depth_info_topic", "/camera/depth/camera_info")
        self.mode_topic = rospy.get_param("~failsafe_mode_topic", "/fast_livo2/failsafe_mode")
        # Step-3 keeps the FAST-LIVO2 visual update stream CPU-throttled, but
        # RGB-D odometry needs the native RealSense cadence.  When this frontend
        # is pointed at the raw camera topic, rebase its hardware-clock stamp
        # with the same offset published by fast_livo2_sensor_adapter.  Depth
        # already uses this exact offset in depth_cb().
        self.color_rebase_with_shared_offset = bool(
            rospy.get_param("~color_rebase_with_shared_offset", False))

        self.fx = float(rospy.get_param("/laserMapping/cam_fx", 641.710876464844))
        self.fy = float(rospy.get_param("/laserMapping/cam_fy", 640.919189453125))
        self.cx = float(rospy.get_param("/laserMapping/cam_cx", 650.866943359375))
        self.cy = float(rospy.get_param("/laserMapping/cam_cy", 360.883941650391))
        self.base_w = int(rospy.get_param("/laserMapping/cam_width", 1280))
        self.base_h = int(rospy.get_param("/laserMapping/cam_height", 720))
        self.dist = np.array([
            float(rospy.get_param("/laserMapping/cam_d0", -0.057067841291)),
            float(rospy.get_param("/laserMapping/cam_d1", 0.071866385639)),
            float(rospy.get_param("/laserMapping/cam_d2", 0.000707329484)),
            float(rospy.get_param("/laserMapping/cam_d3", 0.000918560720)),
            float(rospy.get_param("/laserMapping/cam_d4", -0.023856583983)),
        ], dtype=np.float64)

        self.dfx = float(rospy.get_param("~depth_fx", 436.7371826171875))
        self.dfy = float(rospy.get_param("~depth_fy", 436.7371826171875))
        self.dcx = float(rospy.get_param("~depth_cx", 426.9660339355469))
        self.dcy = float(rospy.get_param("~depth_cy", 242.29881286621094))
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.depth_min = float(rospy.get_param("~depth_min_m", 0.5))
        self.depth_max = float(rospy.get_param("~depth_max_m", 12.0))
        self.depth_sync_max_age = float(rospy.get_param("~depth_sync_max_age", 0.05))
        self.max_frame_dt = float(rospy.get_param("~max_frame_dt", 0.35))
        self.depth_alignment_stride = max(1, int(
            rospy.get_param("~depth_alignment_stride", 2)))

        self.Rdc = mat3(rospy.get_param("~depth_to_color_R", [
            0.9999934434890747, 0.003619089489802718, 0.000103622158349026,
            -0.003618920221924782, 0.9999921917915344, -0.0015905441250652075,
            -0.00010937767365248874, 0.0015901586739346385, 0.9999987483024597]))
        self.tdc = vec3(rospy.get_param("~depth_to_color_T",
            [-0.05904686823487282, -0.00036308413837105036, 0.00008832781168166548]))

        Rbl = mat3(rospy.get_param("/extrin_calib/extrinsic_R"))
        tbl = vec3(rospy.get_param("/extrin_calib/extrinsic_T"))
        Rcl = mat3(rospy.get_param("/extrin_calib/Rcl"))
        Pcl = vec3(rospy.get_param("/extrin_calib/Pcl"))
        T_bl = T_from(Rbl, tbl)
        T_cl = T_from(Rcl, Pcl)
        self.T_bc = T_bl @ np.linalg.inv(T_cl)
        self.T_cb = np.linalg.inv(self.T_bc)

        self.orb = cv2.ORB_create(
            nfeatures=int(rospy.get_param("~orb_features", 1500)),
            fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.ratio = float(rospy.get_param("~match_ratio", 0.78))
        self.min_matches = int(rospy.get_param("~min_matches", 25))
        self.min_inliers = int(rospy.get_param("~min_inliers", 18))
        self.max_reproj = float(rospy.get_param("~max_reprojection_px", 3.0))
        self.process_every_n = max(1, int(rospy.get_param("~process_every_n", 3)))
        self.frame_count = 0

        self.pub = rospy.Publisher("/fast_livo2/relative_motion_hint",
                                   TwistStamped, queue_size=10)
        # Dedicated mirror for the map-localizer's relative-history transport.
        # The shared topic remains the ESIKF measurement path; this second
        # publisher carries the exact same delta and is NOT a second estimator.
        self.map_history_pub = rospy.Publisher(
            "/fast_livo2/rgbd_relative_motion_hint", TwistStamped, queue_size=20)
        self.depth_sub = None
        self.color_sub = None
        self.streams_active = False
        self.mode_sub = rospy.Subscriber(self.mode_topic, String, self.mode_cb, queue_size=10)
        # CameraInfo is tiny and infrequent, so keep only this metadata stream
        # subscribed while healthy. Full RGB/depth transport is attached only
        # in degraded modes to avoid rospy deserializing ~100 MB/s needlessly.
        self.depth_info_sub = rospy.Subscriber(
            self.depth_info_topic, CameraInfo, self.depth_info_cb, queue_size=1)

        rospy.loginfo("[FAILSAFE RGBD] ready dormant; streams attach only on sensor failure")

    def set_streams_active(self, active):
        if active == self.streams_active:
            return
        if active:
            self.depth_sub = rospy.Subscriber(
                self.depth_topic, Image, self.depth_cb,
                queue_size=1, buff_size=8 * 1024 * 1024)
            self.color_sub = rospy.Subscriber(
                self.color_topic, Image, self.color_cb,
                queue_size=1, buff_size=12 * 1024 * 1024)
            self.streams_active = True
            rospy.loginfo("[FAILSAFE RGBD] subscribed to RGB/depth streams")
        else:
            if self.depth_sub is not None:
                self.depth_sub.unregister()
            if self.color_sub is not None:
                self.color_sub.unregister()
            self.depth_sub = None
            self.color_sub = None
            self.streams_active = False
            rospy.loginfo("[FAILSAFE RGBD] unsubscribed from RGB/depth streams")

    def reset(self):
        self.prev_gray = None
        self.prev_depth_color = None
        with self.lock:
            self.latest_depth = None
            self.latest_depth_rx = rospy.Time(0)
            self.latest_depth_stamp = rospy.Time(0)
            self.depth_buffer.clear()
        self.prev_stamp = None
        self.prev_kp = None
        self.prev_desc = None
        self.filt_v = None
        self.filt_w = None

    def mode_cb(self, msg):
        old = self.mode
        self.mode = msg.data
        degraded = self.mode in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE", "LV_IMU_FAILED")
        if old != self.mode:
            rospy.loginfo("[FAILSAFE RGBD] mode %s -> %s", old, self.mode)
        # Subscribe before processing can start in degraded mode; unsubscribe
        # first on recovery so no more heavyweight image callbacks can enter.
        self.set_streams_active(degraded)
        if not degraded and old != self.mode:
            with self.processing_lock:
                self.reset()

    def depth_info_cb(self, msg):
        if msg.K[0] > 0 and msg.K[4] > 0:
            self.dfx, self.dfy, self.dcx, self.dcy = \
                float(msg.K[0]), float(msg.K[4]), float(msg.K[2]), float(msg.K[5])

    def depth_cb(self, msg):
        # This frontend is failure-only. In FULL_LIVO the depth stream can be
        # 30 Hz and converting/copying every 848x480 frame wastes substantial
        # CPU/memory bandwidth, which competes directly with Step-3 GICP at
        # real-time playback. Wake on the next depth frame as soon as a relevant
        # degraded mode is latched; no healthy-state depth history is needed.
        if self.mode not in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE", "LV_IMU_FAILED"):
            return
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception:
            return
        # RGB and depth are produced by the same RealSense hardware clock.
        # The sensor adapter already computes the authoritative camera->ROS/bag
        # clock offset for RGB. Reuse EXACTLY that offset for depth instead of
        # estimating a second one from callback receipt time (which varied by
        # 90-113 ms under CPU load and could reject every RGB-D frame).
        now = rospy.Time.now()
        shared_offset = rospy.get_param("/fast_livo2/camera_clock_offset_sec", None)
        depth_stamp = msg.header.stamp
        if shared_offset is not None and not msg.header.stamp.is_zero():
            self.depth_clock_offset = rospy.Duration.from_sec(float(shared_offset))
            depth_stamp = msg.header.stamp + self.depth_clock_offset
        elif not now.is_zero() and not msg.header.stamp.is_zero():
            # Startup-only fallback before the first RGB frame establishes the
            # shared clock parameter. It is replaced as soon as the adapter has
            # published the authoritative offset.
            candidate_offset = now - msg.header.stamp
            if (self.depth_clock_offset is None or
                    candidate_offset.to_sec() < self.depth_clock_offset.to_sec()):
                self.depth_clock_offset = candidate_offset
            depth_stamp = msg.header.stamp + self.depth_clock_offset
        elif depth_stamp.is_zero():
            depth_stamp = now

        with self.lock:
            depth_copy = np.asarray(d).copy()
            self.latest_depth = depth_copy
            self.latest_depth_rx = now
            self.latest_depth_stamp = depth_stamp
            self.depth_buffer.append((depth_stamp, now, depth_copy))

    def aligned_depth(self, depth, color_shape, K, dist):
        # Avoid cv2.rgbd.registerDepth here. OpenCV 4.2's native RGB-D path has
        # repeatedly segfaulted this ROS Python process on the recorded
        # RealSense stream (exit -11). Use the same explicit projection path as
        # visual_place_hint.py: depth pixel -> 3-D -> depth/color extrinsic ->
        # distorted color pixel, then z-buffer nearest depth.
        H, W = color_shape[:2]
        stride = self.depth_alignment_stride
        if stride > 1:
            sampled = depth[::stride, ::stride]
            z = sampled.astype(np.float32) * self.depth_scale
            valid = (z >= self.depth_min) & (z <= self.depth_max)
            if np.count_nonzero(valid) < 500:
                return None
            v_axis = np.arange(0, depth.shape[0], stride, dtype=np.float32)
            u_axis = np.arange(0, depth.shape[1], stride, dtype=np.float32)
            uu, vv = np.meshgrid(u_axis, v_axis)
        else:
            z = depth.astype(np.float32) * self.depth_scale
            valid = (z >= self.depth_min) & (z <= self.depth_max)
            if np.count_nonzero(valid) < 500:
                return None
            vv, uu = np.indices(depth.shape, dtype=np.float32)
        u = uu[valid].astype(np.float64)
        v = vv[valid].astype(np.float64)
        zd = z[valid].astype(np.float64)
        pd = np.column_stack(((u - self.dcx) * zd / self.dfx,
                              (v - self.dcy) * zd / self.dfy,
                              zd))
        pc = (self.Rdc @ pd.T).T + self.tdc
        front = pc[:, 2] > 0.15
        pc = pc[front]
        if pc.shape[0] < 500:
            return None

        # projectPoints applies the recorded color-camera distortion model.
        proj, _ = cv2.projectPoints(pc.reshape(-1, 1, 3),
                                    np.zeros(3), np.zeros(3), K, dist)
        uv = np.rint(proj.reshape(-1, 2)).astype(np.int32)
        zc = pc[:, 2].astype(np.float32)
        inside = ((uv[:, 0] >= 0) & (uv[:, 0] < W) &
                  (uv[:, 1] >= 0) & (uv[:, 1] < H))
        uv = uv[inside]
        zc = zc[inside]
        if uv.shape[0] < 500:
            return None
        flat = np.full(H * W, np.inf, dtype=np.float32)
        idx = uv[:, 1] * W + uv[:, 0]
        np.minimum.at(flat, idx, zc)
        out = flat.reshape(H, W)
        out[~np.isfinite(out)] = 0.0
        return out if np.count_nonzero(out) >= 500 else None

    @staticmethod
    def sample_depth(depth, u, v, radius=3):
        h, w = depth.shape
        x = int(round(u)); y = int(round(v))
        x0=max(0,x-radius); x1=min(w,x+radius+1)
        y0=max(0,y-radius); y1=min(h,y+radius+1)
        vals = depth[y0:y1, x0:x1]
        vals = vals[vals > 0]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    def color_cb(self, msg):
        # A mode callback can arrive while a camera callback is still executing.
        # Keep reset and all prev-frame/filter accesses mutually exclusive.
        with self.processing_lock:
            self._color_cb_locked(msg)

    def _color_cb_locked(self, msg):
        if self.mode not in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE", "LV_IMU_FAILED"):
            return
        rospy.loginfo_throttle(1.0, "[FAILSAFE RGBD] degraded RGB callback active in %s", self.mode)
        self.frame_count += 1
        if self.frame_count % self.process_every_n:
            return

        now = rospy.Time.now()
        # The normal FAST-LIVO2 image input is already rebased by the adapter.
        # The failure-only RGB-D frontend can instead subscribe to the RAW
        # 30-Hz RealSense RGB topic so the mapper may remain throttled at 5 Hz.
        # Rebase that raw stamp with the adapter's one authoritative camera
        # offset; never estimate a second RGB clock from callback arrival time.
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else now
        if self.color_rebase_with_shared_offset and not msg.header.stamp.is_zero():
            shared_offset = rospy.get_param("/fast_livo2/camera_clock_offset_sec", None)
            if shared_offset is None:
                rospy.logwarn_throttle(
                    1.0, "[FAILSAFE RGBD] waiting for shared camera clock offset")
                return
            stamp = msg.header.stamp + rospy.Duration.from_sec(float(shared_offset))
        with self.lock:
            if not self.depth_buffer:
                rospy.logwarn_throttle(1.0, "[FAILSAFE RGBD] waiting for depth")
                return
            best_stamp, best_rx, best_depth = min(
                self.depth_buffer,
                key=lambda item: abs((stamp - item[0]).to_sec())
                    if (not stamp.is_zero() and not item[0].is_zero())
                    else abs((now - item[1]).to_sec()))
            depth_stamp = best_stamp
            age = (abs((stamp - depth_stamp).to_sec())
                   if not depth_stamp.is_zero()
                   else abs((now - best_rx).to_sec()))
            if age > self.depth_sync_max_age:
                rospy.logwarn_throttle(1.0, "[FAILSAFE RGBD] RGB/depth event-time mismatch %.3fs", age)
                return
            depth = best_depth.copy()

        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception:
            return
        gray = np.asarray(gray)
        H, W = gray.shape[:2]
        sx = float(W) / float(self.base_w)
        sy = float(H) / float(self.base_h)
        K = np.array([[self.fx*sx, 0, self.cx*sx],
                      [0, self.fy*sy, self.cy*sy],
                      [0, 0, 1]], dtype=np.float64)

        aligned = self.aligned_depth(depth, gray.shape, K, self.dist)
        if aligned is None:
            rospy.logwarn_throttle(1.0, "[FAILSAFE RGBD] aligned depth unavailable")
            return

        kp, desc = self.orb.detectAndCompute(gray, None)
        if desc is None or len(kp) < self.min_matches:
            rospy.logwarn_throttle(1.0, "[FAILSAFE RGBD] insufficient ORB features: %d", 0 if kp is None else len(kp))
            return

        if self.prev_desc is None:
            self.prev_gray, self.prev_depth_color = gray, aligned
            self.prev_stamp, self.prev_kp, self.prev_desc = stamp, kp, desc
            return

        dt = (stamp - self.prev_stamp).to_sec()
        if dt <= 0.025 or dt >= self.max_frame_dt:
            rospy.logwarn_throttle(
                1.0, "[FAILSAFE RGBD] frame dt %.3fs outside VO window", dt)
            self.prev_gray, self.prev_depth_color = gray, aligned
            self.prev_stamp, self.prev_kp, self.prev_desc = stamp, kp, desc
            return

        knn = self.matcher.knnMatch(self.prev_desc, desc, k=2)
        good = []
        for m_n in knn:
            if len(m_n) != 2:
                continue
            m, n = m_n
            if m.distance < self.ratio * n.distance:
                good.append(m)
        if len(good) < self.min_matches:
            rospy.logwarn_throttle(1.0, "[FAILSAFE RGBD] insufficient descriptor matches: %d", len(good))
            self.prev_gray, self.prev_depth_color = gray, aligned
            self.prev_stamp, self.prev_kp, self.prev_desc = stamp, kp, desc
            return

        prev_uv = []
        cur_uv = []
        prev_z = []
        cur_z = []
        for m in good:
            up, vp = self.prev_kp[m.queryIdx].pt
            uc, vc = kp[m.trainIdx].pt
            zp = self.sample_depth(self.prev_depth_color, up, vp)
            zc = self.sample_depth(aligned, uc, vc)
            if (zp is None or zc is None or
                    zp < self.depth_min or zp > self.depth_max or
                    zc < self.depth_min or zc > self.depth_max):
                continue
            prev_uv.append([up, vp])
            cur_uv.append([uc, vc])
            prev_z.append(zp)
            cur_z.append(zc)

        if len(prev_uv) < self.min_matches:
            rospy.logwarn_throttle(1.0, "[FAILSAFE RGBD] insufficient depth-backed matches: %d", len(prev_uv))
            self.prev_gray, self.prev_depth_color = gray, aligned
            self.prev_stamp, self.prev_kp, self.prev_desc = stamp, kp, desc
            return

        prev_px = np.asarray(prev_uv, dtype=np.float64).reshape(-1,1,2)
        cur_px = np.asarray(cur_uv, dtype=np.float64).reshape(-1,1,2)
        prev_norm = cv2.undistortPoints(prev_px, K, self.dist).reshape(-1,2)
        cur_norm = cv2.undistortPoints(cur_px, K, self.dist).reshape(-1,2)
        prev_zv = np.asarray(prev_z, dtype=np.float64)
        cur_zv = np.asarray(cur_z, dtype=np.float64)
        prev3 = np.column_stack((prev_norm[:,0]*prev_zv,
                                 prev_norm[:,1]*prev_zv,
                                 prev_zv)).astype(np.float64)
        cur3 = np.column_stack((cur_norm[:,0]*cur_zv,
                                cur_norm[:,1]*cur_zv,
                                cur_zv)).astype(np.float64)

        # Primary metric estimate: depth-backed 3D<->3D RANSAC. This avoids
        # the scale bias seen when PnP relied on only the previous depth frame.
        affine_ok, affine, affine_inliers = cv2.estimateAffine3D(
            prev3, cur3, ransacThreshold=0.22, confidence=0.995)

        T_cur_prev = None
        inlier_count = 0
        total_count = len(prev3)
        method = "3d3d"
        if affine_ok and affine is not None and affine_inliers is not None:
            A = affine[:, :3]
            t_aff = affine[:, 3]
            U, sv, Vt = np.linalg.svd(A)
            R = U @ Vt
            if np.linalg.det(R) < 0:
                U[:, -1] *= -1.0
                R = U @ Vt
            inl = affine_inliers.reshape(-1).astype(bool)
            inlier_count = int(np.count_nonzero(inl))
            # Reject affine solutions whose scale/shear is inconsistent with
            # rigid camera motion.
            if (inlier_count >= self.min_inliers and
                    np.all(np.isfinite(sv)) and
                    np.max(np.abs(sv - 1.0)) < 0.25):
                resid = np.linalg.norm((R @ prev3[inl].T).T + t_aff - cur3[inl], axis=1)
                if len(resid) and float(np.median(resid)) < 0.20:
                    T_cur_prev = T_from(R, t_aff)

        # PnP is retained as a fallback for frames where current depth is sparse.
        if T_cur_prev is None:
            method = "pnp"
            obj3 = prev3
            img2 = np.asarray(cur_uv, dtype=np.float64)
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj3, img2, K, self.dist,
                iterationsCount=120,
                reprojectionError=self.max_reproj,
                confidence=0.995,
                flags=cv2.SOLVEPNP_EPNP)
            if not ok or inliers is None or len(inliers) < self.min_inliers:
                self.prev_gray, self.prev_depth_color = gray, aligned
                self.prev_stamp, self.prev_kp, self.prev_desc = stamp, kp, desc
                return
            inl_idx = inliers.reshape(-1)
            ok2, rvec, tvec = cv2.solvePnP(
                obj3[inl_idx], img2[inl_idx], K, self.dist,
                rvec, tvec, True, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok2:
                return
            R_cur_prev, _ = cv2.Rodrigues(rvec)
            T_cur_prev = T_from(R_cur_prev, tvec.reshape(3))
            inlier_count = len(inl_idx)
            total_count = len(obj3)

        T_prev_cur = np.linalg.inv(T_cur_prev)

        # IMPORTANT: the successful standalone fallback filters the relative
        # motion in the CAMERA frame first.  The old integrated version first
        # converted to body velocity and then filtered it; that changed the
        # lever-arm component and, together with 30-Hz updates, produced the
        # severe speed/scale underestimate seen in the failed 26 m run.
        v_cam = T_prev_cur[:3, 3] / dt
        r_cam, _ = cv2.Rodrigues(T_prev_cur[:3, :3])
        omega_cam = r_cam.reshape(3) / dt

        speed = float(np.linalg.norm(v_cam))
        wnorm = float(np.linalg.norm(omega_cam))
        inlier_ratio = float(inlier_count) / float(max(1, total_count))
        if (not np.all(np.isfinite(v_cam)) or not np.all(np.isfinite(omega_cam)) or
                speed > 12.0 or wnorm > 2.5 or inlier_ratio < 0.30):
            self.prev_gray, self.prev_depth_color = gray, aligned
            self.prev_stamp, self.prev_kp, self.prev_desc = stamp, kp, desc
            return

        # Same acceleration/angular-acceleration limiter used by the stable
        # standalone RGB-D-only trajectory.  The raw registration is still
        # allowed to be noisy, but its *change* is bounded before it reaches
        # FAST-LIVO2.  This preserves turns/stops while rejecting one-frame
        # velocity spikes.
        if self.filt_v is None:
            if speed > self.bootstrap_max_speed or wnorm > self.bootstrap_max_omega:
                self.prev_gray, self.prev_depth_color = gray, aligned
                self.prev_stamp, self.prev_kp, self.prev_desc = stamp, kp, desc
                return
            self.filt_v = v_cam.copy()
            self.filt_w = omega_cam.copy()
        else:
            dv = v_cam - self.filt_v
            lim = self.max_linear_accel * dt
            ndv = float(np.linalg.norm(dv))
            if ndv > lim and ndv > 1.0e-9:
                dv *= lim / ndv
            dw = omega_cam - self.filt_w
            limw = self.max_angular_accel * dt
            ndw = float(np.linalg.norm(dw))
            if ndw > limw and ndw > 1.0e-9:
                dw *= limw / ndw
            candidate_v = self.filt_v + dv
            candidate_w = self.filt_w + dw
            a = max(0.0, min(1.0, self.filter_alpha))
            self.filt_v = (1.0 - a) * self.filt_v + a * candidate_v
            self.filt_w = (1.0 - a) * self.filt_w + a * candidate_w

        # Reconstruct exactly the filtered SE(3) increment used by the
        # standalone implementation, then convert that increment to body frame.
        Df_cam = np.eye(4, dtype=np.float64)
        Df_cam[:3, 3] = self.filt_v * dt
        Rf_cam, _ = cv2.Rodrigues((self.filt_w * dt).reshape(3, 1))
        Df_cam[:3, :3] = Rf_cam
        Df_body = self.T_bc @ Df_cam @ self.T_cb
        t_body = Df_body[:3, 3]
        r_body, _ = cv2.Rodrigues(Df_body[:3, :3])
        v_body_out = t_body / dt
        omega_out = r_body.reshape(3) / dt

        hint = TwistStamped()
        hint.header.stamp = stamp
        hint.header.frame_id = "rgbd_visual_motion"
        hint.twist.linear.x, hint.twist.linear.y, hint.twist.linear.z = map(float, v_body_out)
        hint.twist.angular.x, hint.twist.angular.y, hint.twist.angular.z = map(float, omega_out)
        self.pub.publish(hint)

        # Do not throw away the quantity that made the standalone fallback
        # accurate.  Publish the filtered body-frame SE(3) increment itself.
        # LIVMapper applies this increment directly to the ONE persistent
        # FAST-LIVO2 state; it is not a second odometry trajectory.
        delta = TwistStamped()
        delta.header.stamp = stamp
        delta.header.frame_id = "rgbd_relative_delta"
        delta.twist.linear.x, delta.twist.linear.y, delta.twist.linear.z = map(float, t_body)
        delta.twist.angular.x, delta.twist.angular.y, delta.twist.angular.z = map(float, r_body.reshape(3))
        self.pub.publish(delta)
        self.map_history_pub.publish(delta)

        rospy.loginfo_throttle(
            0.5,
            "[FAILSAFE RGBD] %s inliers=%d/%d dt=%.3f raw=%.2fm/s filtered=(%.2f,%.2f,%.2f)m/s w=%.1fdeg/s" %
            (method, inlier_count, total_count, dt, speed,
             float(v_body_out[0]), float(v_body_out[1]), float(v_body_out[2]),
             math.degrees(float(np.linalg.norm(omega_out)))))

        self.prev_gray, self.prev_depth_color = gray, aligned
        self.prev_stamp, self.prev_kp, self.prev_desc = stamp, kp, desc


if __name__ == "__main__":
    rospy.init_node("fast_livo2_rgbd_failsafe_vo")
    RGBDFailsafeVO()
    rospy.spin()
