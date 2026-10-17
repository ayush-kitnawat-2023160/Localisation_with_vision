#!/usr/bin/env python3
"""
Visual place-recognition front-end for degraded FAST-LIVO2.

This node is NOT an estimator and owns no persistent navigation state.
It only publishes a small ordered set of coarse map<-LiDAR pose hypotheses.
failsafe_map_localizer geometrically verifies/refines them before the one
FAST-LIVO2 ESIKF may consume a correction.
"""
import os, math, csv, threading
from collections import OrderedDict, deque
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String

# OpenCV 4.2 can oversubscribe/occasionally crash when ROS Python processes
# concurrently run ORB/BFMatcher and RGB-D registration.  The validated
# standalone pipeline is single-process/single-query, so one OpenCV worker also
# keeps the live behavior deterministic without changing any matching math.
cv2.setNumThreads(1)


def mat_from(v):
    return np.asarray(v, dtype=np.float64).reshape(3, 3)


def tf(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64)
    return T


def quat_to_R(qx, qy, qz, qw):
    q = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = q / n
    return np.asarray([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ], dtype=np.float64)


def R_to_quat(R):
    # OpenCV Rodrigues -> quaternion without scipy dependency.
    rvec, _ = cv2.Rodrigues(R)
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    axis = rvec.reshape(3) / theta
    s = math.sin(theta * 0.5)
    return float(axis[0]*s), float(axis[1]*s), float(axis[2]*s), math.cos(theta*0.5)


class VisualPlaceHint:
    def __init__(self):
        self.image_dir = Path(rospy.get_param("~image_dir"))
        self.trajectory_csv = Path(rospy.get_param("~trajectory_csv"))
        self.color_topic = rospy.get_param("~color_topic", "/fast_livo2/input/image")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
        self.mode_topic = rospy.get_param("~failsafe_mode_topic", "/fast_livo2/failsafe_mode")
        self.output_topic = rospy.get_param("~output_topic", "/fast_livo2/place_pose_candidates")
        self.map_pose_topic = rospy.get_param("~map_pose_topic", "/fast_livo2/map_pose")
        self.query_hz = float(rospy.get_param("~query_hz", 2.0))
        self.enable_on_healthy = bool(rospy.get_param("~enable_on_healthy", True))
        self.healthy_depth_pnp_enable = bool(rospy.get_param("~healthy_depth_pnp_enable", True))
        self.healthy_query_hz = float(rospy.get_param("~healthy_query_hz", 1.0))
        self.healthy_pool_size = int(rospy.get_param("~healthy_pool_size", 48))
        # Opt-in precomputed, *exact* ORB retrieval descriptors for the static
        # map images. A stale/mismatched cache always falls back to the original
        # feature extraction path; no approximate descriptor matching is used.
        self.orb_retrieval_cache_path = str(rospy.get_param(
            "~orb_retrieval_cache_path", ""))
        self.enable_on_lidar_fail = bool(rospy.get_param("~enable_on_lidar_fail", False))
        # Unlike the former appearance-only LiDAR-loss seed, this uses RGB-D
        # feature correspondences against *saved image keyframes* to estimate
        # the current metric map pose.  The C++ map localizer still performs
        # independent depth-map GICP and owns the final acceptance decision.
        self.lidar_fail_feature_pnp_enable = bool(rospy.get_param(
            "~lidar_fail_feature_pnp_enable", False))
        self.lidar_fail_feature_use_akaze = bool(rospy.get_param(
            "~lidar_fail_feature_use_akaze", False))
        self.lidar_fail_feature_akaze_cache = str(rospy.get_param(
            "~lidar_fail_feature_akaze_cache", ""))
        self.lidar_fail_feature_min_inliers = int(rospy.get_param(
            "~lidar_fail_feature_min_inliers", 18))
        self.lidar_fail_feature_min_ratio = float(rospy.get_param(
            "~lidar_fail_feature_min_ratio", 0.45))
        self.lidar_fail_feature_max_reproj_px = float(rospy.get_param(
            "~lidar_fail_feature_max_reproj_px", 2.8))
        self.lidar_fail_feature_confirmations = int(rospy.get_param(
            "~lidar_fail_feature_confirmations", 2))
        self.lidar_fail_feature_unique_ratio = float(rospy.get_param(
            "~lidar_fail_feature_unique_ratio", 1.12))
        self.lidar_fail_feature_history = None
        self.lidar_fail_feature_streak = 0
        self.lidar_fail_query_hz = float(rospy.get_param("~lidar_fail_query_hz", 1.0))
        self.lidar_fail_pool_size = int(rospy.get_param("~lidar_fail_pool_size", 48))
        self.top_retrieval = int(rospy.get_param("~top_retrieval", 8))
        self.max_candidates = int(rospy.get_param("~max_candidates", 4))
        self.min_retrieval_matches = int(rospy.get_param("~min_retrieval_matches", 8))
        self.min_pnp_inliers = int(rospy.get_param("~min_pnp_inliers", 18))
        self.imu_local_pool_size = int(rospy.get_param("~imu_local_pool_size", 24))
        self.imu_expanded_pool_size = int(rospy.get_param("~imu_expanded_pool_size", 72))
        self.imu_expand_after_misses = int(rospy.get_param("~imu_expand_after_misses", 2))
        self.place_miss_streak = 0
        self.last_healthy_query = rospy.Time(0)
        self.last_lidar_fail_query = rospy.Time(0)

        fx = float(rospy.get_param("/laserMapping/cam_fx"))
        fy = float(rospy.get_param("/laserMapping/cam_fy"))
        cx = float(rospy.get_param("/laserMapping/cam_cx"))
        cy = float(rospy.get_param("/laserMapping/cam_cy"))
        self.K = np.asarray([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.dist = np.asarray([
            float(rospy.get_param("/laserMapping/cam_d0", 0.0)),
            float(rospy.get_param("/laserMapping/cam_d1", 0.0)),
            float(rospy.get_param("/laserMapping/cam_d2", 0.0)),
            float(rospy.get_param("/laserMapping/cam_d3", 0.0)),
            float(rospy.get_param("/laserMapping/cam_d4", 0.0))
        ], dtype=np.float64)

        self.dfx = float(rospy.get_param("~depth_fx"))
        self.dfy = float(rospy.get_param("~depth_fy"))
        self.dcx = float(rospy.get_param("~depth_cx"))
        self.dcy = float(rospy.get_param("~depth_cy"))
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.depth_min = float(rospy.get_param("~depth_min_m", 0.5))
        self.depth_max = float(rospy.get_param("~depth_max_m", 12.0))
        self.Rdc = mat_from(rospy.get_param("~depth_to_color_R"))
        self.Tdc = np.asarray(rospy.get_param("~depth_to_color_T"), dtype=np.float64)

        Rbl = mat_from(rospy.get_param("/extrin_calib/extrinsic_R"))
        Tbl = np.asarray(rospy.get_param("/extrin_calib/extrinsic_T"), dtype=np.float64)
        Rcl = mat_from(rospy.get_param("/extrin_calib/Rcl"))
        Pcl = np.asarray(rospy.get_param("/extrin_calib/Pcl"), dtype=np.float64)
        self.Tbc = tf(Rbl, Tbl) @ np.linalg.inv(tf(Rcl, Pcl))  # body <- color
        self.Tcl = tf(Rcl, Pcl)  # color <- lidar

        self.bridge = CvBridge()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        # Match the validated standalone place-retrieval pipeline exactly:
        # half-resolution ORB for retrieval, then a denser ORB pass for PnP.
        self.orb_ret = cv2.ORB_create(nfeatures=700, fastThreshold=12)
        self.orb_full = cv2.ORB_create(nfeatures=2200, fastThreshold=10)
        self.illumination_normalizer = cv2.createCLAHE(
            clipLimit=2.0, tileGridSize=(8, 8))
        self.akaze = cv2.AKAZE_create(threshold=0.00035)

        self.traj = self._load_traj()
        self.db = self._build_db()
        self.akaze_db = {}
        self.akaze_local_index_signature = None
        self.akaze_local_index_matcher = None
        self.akaze_local_index_offsets = None
        self.akaze_local_index_frames = None
        if self.lidar_fail_feature_pnp_enable and self.lidar_fail_feature_use_akaze:
            if not self.lidar_fail_feature_akaze_cache:
                raise RuntimeError("AKAZE mode requires map-only feature cache")
            with np.load(self.lidar_fail_feature_akaze_cache, allow_pickle=False) as cache:
                stems = cache['image_stems']
                starts = cache['offsets']
                descriptors = cache['descriptors']
                if len(starts) != len(stems) + 1 or starts[-1] != len(descriptors):
                    raise RuntimeError("Invalid saved AKAZE feature cache")
                # All descriptor slices own the same bounded uint8 backing
                # array; no keypoint/image objects are kept per map frame.
                self.akaze_db = {str(stem): descriptors[starts[i]:starts[i+1]]
                                 for i, stem in enumerate(stems)}
            rospy.loginfo("[FAILSAFE PLACE FEATURE] loaded %d cached saved-map AKAZE frames",
                          len(self.akaze_db))
        # Keypoint objects are large. Keep a bounded cache so a long route does
        # not retain dense ORB state for every database image forever.
        self.full_cache = OrderedDict()
        self.full_cache_limit = int(rospy.get_param("~full_cache_limit", 64))

        self.mode = "FULL_LIVO"
        self.latest_map_xyz = None
        self.failure_prior_xyz = None
        self.failure_start = rospy.Time(0)
        self.latest_depth = None
        # Color is throttled by the FAST-LIVO2 adapter while RealSense depth
        # arrives more often. Matching the *two latest* frames silently pairs
        # RGB to a future depth frame by 0.10-0.30 s. Keep a bounded timestamped
        # ring and choose the depth exposure nearest to the actual query RGB.
        self.depth_frames = deque(maxlen=32)
        self.latest_depth_recv = rospy.Time(0)
        self.latest_depth_stamp = rospy.Time(0)
        self.depth_clock_offset = None
        self.latest_color = None
        self.latest_color_stamp = rospy.Time(0)
        self.latest_color_recv = rospy.Time(0)
        self.lock = threading.Lock()
        self.processing_lock = threading.Lock()

        self.pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=2)
        rospy.Subscriber(self.mode_topic, String, self.mode_cb, queue_size=5)
        rospy.Subscriber(self.map_pose_topic, PoseStamped, self.map_pose_cb, queue_size=5)
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1,
                         buff_size=2**22)
        rospy.Subscriber(self.color_topic, Image, self.color_cb, queue_size=1,
                         buff_size=2**22)
        self.worker_timer = rospy.Timer(
            rospy.Duration(1.0 / max(0.2, self.query_hz)), self.process_latest)

        rospy.set_param("~ready", True)
        rospy.loginfo("[FAILSAFE PLACE] ready: %d visual keyframes from %s",
                      len(self.db), str(self.image_dir))

    def _load_traj(self):
        rows = []
        with open(self.trajectory_csv, newline='') as f:
            for r in csv.DictReader(f):
                try:
                    T = tf(quat_to_R(float(r['qx']), float(r['qy']), float(r['qz']), float(r['qw'])),
                           [float(r['x']), float(r['y']), float(r['z'])])
                    rows.append((float(r['timestamp']), T))
                except Exception:
                    pass
        rows.sort(key=lambda x: x[0])
        if not rows:
            raise RuntimeError("empty trajectory database")
        return rows

    def _interp_body(self, t):
        rows = self.traj
        if t <= rows[0][0]:
            return rows[0][1].copy()
        if t >= rows[-1][0]:
            return rows[-1][1].copy()
        lo, hi = 0, len(rows)-1
        while lo+1 < hi:
            m = (lo+hi)//2
            if rows[m][0] <= t:
                lo = m
            else:
                hi = m
        ta, A = rows[lo]
        tb, B = rows[hi]
        u = (t-ta)/(tb-ta)
        ra, _ = cv2.Rodrigues(A[:3, :3])
        rb, _ = cv2.Rodrigues(B[:3, :3])
        R, _ = cv2.Rodrigues((1-u)*ra + u*rb)
        out = np.eye(4)
        out[:3, :3] = R
        out[:3, 3] = (1-u)*A[:3, 3] + u*B[:3, 3]
        return out

    def _build_db(self):
        db = []
        files = []
        for p in self.image_dir.glob("*.png"):
            try:
                files.append((float(p.stem), p))
            except Exception:
                continue
        files.sort()
        cached_descriptors = None
        cached_offsets = None
        if self.orb_retrieval_cache_path:
            try:
                with np.load(self.orb_retrieval_cache_path, allow_pickle=False) as cache:
                    stems = np.asarray([p.stem for _, p in files])
                    sizes = np.asarray([p.stat().st_size for _, p in files], dtype=np.int64)
                    mtimes = np.asarray([p.stat().st_mtime_ns for _, p in files], dtype=np.int64)
                    cached_offsets = cache["offsets"]
                    cached_descriptors = cache["descriptors"]
                    valid = (
                        cache["opencv_version"].item() == cv2.__version__ and
                        np.array_equal(cache["orb_config"], [700, 12, 640, 360]) and
                        np.array_equal(cache["image_stems"], stems) and
                        np.array_equal(cache["image_sizes"], sizes) and
                        np.array_equal(cache["image_mtimes_ns"], mtimes) and
                        len(cached_offsets) == len(files) + 1 and
                        cached_offsets[0] == 0 and
                        np.all(np.diff(cached_offsets) >= 0) and
                        cached_descriptors.dtype == np.uint8 and
                        cached_descriptors.ndim == 2 and
                        cached_descriptors.shape[1] == 32 and
                        cached_offsets[-1] == len(cached_descriptors))
                if not valid:
                    raise ValueError("ORB cache metadata does not match saved images/OpenCV")
                rospy.loginfo("[FAILSAFE PLACE] loaded exact ORB map descriptor cache: %d frames",
                              len(files))
            except (OSError, ValueError, KeyError, IndexError) as exc:
                rospy.logwarn("[FAILSAFE PLACE] ORB cache rejected; extracting from images: %s", exc)
                cached_descriptors = None
                cached_offsets = None
        for i, (ts, p) in enumerate(files):
            if cached_descriptors is not None:
                des = cached_descriptors[cached_offsets[i]:cached_offsets[i + 1]]
            else:
                im = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
                if im is None:
                    continue
                small = cv2.resize(im, (640, 360))
                _, des = self.orb_ret.detectAndCompute(small, None)
            if des is not None and len(des) >= 30:
                map_xyz = self._interp_body(ts)[:3, 3].copy()
                db.append((ts, p, des, map_xyz))
        if not db:
            raise RuntimeError("empty visual keyframe database")
        return db


    def mode_cb(self, msg):
        previous = self.mode
        self.mode = msg.data
        degraded = self.mode in ("VI_LIDAR_FAILED", "LV_IMU_FAILED", "IMU_ONLY_BRIDGE")
        was_degraded = previous in ("VI_LIDAR_FAILED", "LV_IMU_FAILED", "IMU_ONLY_BRIDGE")
        if degraded and not was_degraded:
            with self.lock:
                self.failure_prior_xyz = (None if self.latest_map_xyz is None
                                          else self.latest_map_xyz.copy())
            self.failure_start = rospy.Time.now()
            self.lidar_fail_feature_history = None
            self.lidar_fail_feature_streak = 0
            with self.lock:
                self.depth_frames.clear()
        elif not degraded:
            self.failure_prior_xyz = None
            self.failure_start = rospy.Time(0)
            self.lidar_fail_feature_history = None
            self.lidar_fail_feature_streak = 0

    def map_pose_cb(self, msg):
        p = msg.pose.position
        xyz = np.asarray([p.x, p.y, p.z], dtype=np.float64)
        if not np.all(np.isfinite(xyz)):
            return
        with self.lock:
            self.latest_map_xyz = xyz

    def retrieval_pool(self, now):
        """Return the visual retrieval database for the current degraded mode.

        IMU failure must match the validated standalone pipeline: search all
        saved keyframes globally. The live map pose is not a safe prior here
        because the IMU-failed state is exactly the quantity that can drift.
        LiDAR failure may still use the cheaper local pool because healthy IMU
        keeps metric attitude/short-term motion reliable.
        """
        # IMU failure starts from a valid global localization. A 615-keyframe
        # brute-force query takes ~2.2 s on the cart/laptop CPU, which lets the
        # LiDAR-only relative trajectory leave the standalone matcher's 4 m
        # innovation basin before the first correction arrives. Use the last
        # healthy map position as a strong retrieval prior for *all* single-
        # sensor outages; LiDAR GICP still independently verifies every visual
        # hypothesis before it can correct the ESIKF.

        """Bound real-time place retrieval around the pre-failure map lock.

        Global search is useful for initial acquisition, but during a single
        sensor outage the car was localized immediately before failure.  Use
        that map position as a strong prior and expand the search radius with
        outage duration.  This preserves recovery from motion while avoiding
        a 615-keyframe brute-force query every second.
        """
        with self.lock:
            frozen = (None if self.failure_prior_xyz is None
                      else self.failure_prior_xyz.copy())
            live = (None if self.latest_map_xyz is None
                    else self.latest_map_xyz.copy())
        if frozen is None:
            # Healthy operation has a trustworthy map pose from the last
            # verified global lock. Search locally around it so global place
            # verification stays real-time instead of scanning all keyframes.
            if self.mode == "FULL_LIVO" and self.enable_on_healthy and live is not None:
                # Use a deliberately wide local pool. 48 keyframes was too
                # narrow once the global anchor drifted several metres; the full
                # 615-frame database took ~5 s/query. 128 keeps the correct loop
                # segment in the candidate set while remaining real-time enough
                # for LiDAR verification.
                scored = [(float(np.linalg.norm(item[3] - live)), item)
                          for item in self.db]
                scored.sort(key=lambda x: x[0])
                return [item for _, item in scored[:max(128, self.healthy_pool_size)]]
            return self.db

        elapsed = 0.0 if self.failure_start.is_zero() else max(
            0.0, (now - self.failure_start).to_sec())
        # A 25 m local basin already covers several seconds of cart motion.
        # Expand at 5 m/s, and use the current map pose as a second center only
        # while it remains physically plausible relative to the frozen lock.
        radius = min(120.0, 25.0 + 5.0 * elapsed)
        centers = [frozen]
        if live is not None and np.linalg.norm(live - frozen) <= 12.0 + 8.0 * elapsed:
            centers.append(live)

        scored = []
        for item in self.db:
            xyz = item[3]
            d = min(float(np.linalg.norm(xyz - c)) for c in centers)
            if d <= radius:
                scored.append((d, item))
        if len(scored) < 24:
            scored = [(min(float(np.linalg.norm(item[3] - c)) for c in centers), item)
                      for item in self.db]
        scored.sort(key=lambda x: x[0])
        # Keep IMU-loss retrieval comfortably below the 0.5 s query period.
        # The pool expands spatially above, so 48 nearest keyframes preserve
        # route coverage without the 615-frame brute-force latency.
        if self.mode == "LV_IMU_FAILED":
            limit = (self.imu_expanded_pool_size
                     if self.place_miss_streak >= self.imu_expand_after_misses
                     else self.imu_local_pool_size)
        else:
            limit = self.lidar_fail_pool_size
        return [item for _, item in scored[:max(8, limit)]]

    def depth_cb(self, msg):
        # RGB-D feature/PnP verification requires synchronized current depth.
        # This does not replace the independent saved-voxel-map GICP verifier.
        active = (
            self.enable_on_healthy and self.healthy_depth_pnp_enable and
            self.mode == "FULL_LIVO"
        ) or (self.enable_on_lidar_fail and
              self.lidar_fail_feature_pnp_enable and
              self.mode in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE"))
        if not active:
            return
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception:
            return
        now = rospy.Time.now()
        # Use the same RealSense camera hardware->ROS/bag-time offset as RGB.
        # A separate receipt-time estimate is scheduling-dependent and can move
        # depth by ~0.1 s relative to the color frame.
        shared_offset = rospy.get_param("/fast_livo2/camera_clock_offset_sec", None)
        depth_stamp = msg.header.stamp
        if shared_offset is not None and not msg.header.stamp.is_zero():
            self.depth_clock_offset = rospy.Duration.from_sec(float(shared_offset))
            depth_stamp = msg.header.stamp + self.depth_clock_offset
        elif not now.is_zero() and not msg.header.stamp.is_zero():
            candidate = now - msg.header.stamp
            if (self.depth_clock_offset is None or
                    candidate.to_sec() < self.depth_clock_offset.to_sec()):
                self.depth_clock_offset = candidate
            depth_stamp = msg.header.stamp + self.depth_clock_offset
        elif depth_stamp.is_zero():
            depth_stamp = now

        with self.lock:
            self.latest_depth = np.asarray(d).copy()
            self.latest_depth_recv = now
            self.latest_depth_stamp = depth_stamp
            self.depth_frames.append((depth_stamp, self.latest_depth))

    def aligned_depth(self, depth, H, W):
        # Use the exact projection path from the validated standalone matcher.
        # OpenCV 4.2's cv2.rgbd.registerDepth has been observed to segfault the
        # ROS Python process (exit -11) on this RealSense stream; Python cannot
        # catch that native crash.  Explicit unproject -> extrinsic -> project
        # gives the same depth-to-color geometry without that native code path.
        vv, uu = np.indices(depth.shape, dtype=np.float32)
        z = depth.astype(np.float32) * self.depth_scale
        valid = (z >= self.depth_min) & (z <= self.depth_max)
        if np.count_nonzero(valid) < 500:
            return np.zeros((H, W), dtype=np.float32)

        u = uu[valid].astype(np.float64)
        v = vv[valid].astype(np.float64)
        zd = z[valid].astype(np.float64)
        pd = np.column_stack(((u - self.dcx) * zd / self.dfx,
                              (v - self.dcy) * zd / self.dfy,
                              zd))
        pc = (self.Rdc @ pd.T).T + self.Tdc
        pc = pc[pc[:, 2] > 0.15]
        if pc.shape[0] < 500:
            return np.zeros((H, W), dtype=np.float32)
        proj, _ = cv2.projectPoints(pc.reshape(-1, 1, 3),
                                    np.zeros(3), np.zeros(3),
                                    self.K, self.dist)
        uv = np.rint(proj.reshape(-1, 2)).astype(np.int32)
        zc = pc[:, 2].astype(np.float32)
        inside = ((uv[:, 0] >= 0) & (uv[:, 0] < W) &
                  (uv[:, 1] >= 0) & (uv[:, 1] < H))
        uv = uv[inside]
        zc = zc[inside]
        flat = np.full(H * W, np.inf, dtype=np.float32)
        idx = uv[:, 1] * W + uv[:, 0]
        np.minimum.at(flat, idx, zc)
        out = flat.reshape(H, W)
        out[~np.isfinite(out)] = 0.0
        return out

    @staticmethod
    def sample_depth(d, u, v, rad=3):
        x, y = int(round(u)), int(round(v))
        h, w = d.shape
        a = d[max(0, y-rad):min(h, y+rad+1),
              max(0, x-rad):min(w, x+rad+1)]
        a = a[a > 0]
        return None if a.size == 0 else float(np.median(a))

    def color_cb(self, msg):
        active = (
            (self.enable_on_healthy and self.mode == "FULL_LIVO") or
            self.mode == "LV_IMU_FAILED" or
            (self.enable_on_lidar_fail and self.mode in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE"))
        )
        if not active:
            return
        try:
            color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        now = rospy.Time.now()
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else now
        # The archived RealSense RGB clock is ~70M s away from rosbag /clock.
        # Normally /fast_livo2/input/image is already rebased by the sensor
        # adapter, but make this frontend robust to either clock domain.  Never
        # add the offset twice: only rebase when the incoming stamp is clearly
        # not in the current ROS/bag-time epoch.
        shared_offset = rospy.get_param("/fast_livo2/camera_clock_offset_sec", None)
        if (shared_offset is not None and not stamp.is_zero() and not now.is_zero() and
                abs((stamp - now).to_sec()) > 5.0):
            stamp = stamp + rospy.Duration.from_sec(float(shared_offset))
        with self.lock:
            self.latest_color = np.asarray(color).copy()
            self.latest_color_stamp = stamp
            self.latest_color_recv = now

    def process_latest(self, _event):
        active = (
            (self.enable_on_healthy and self.mode == "FULL_LIVO") or
            self.mode == "LV_IMU_FAILED" or
            (self.enable_on_lidar_fail and self.mode in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE"))
        )
        if not active:
            return
        if not self.processing_lock.acquire(False):
            return
        try:
            # Snapshot the mode for this whole query.  A ROS mode callback can
            # otherwise switch FULL_LIVO -> VI_LIDAR_FAILED after we snapshot
            # RGB but before the depth-dependent branch below, making `depth`
            # legitimately None and crashing aligned_depth().  A query belongs
            # to exactly one health epoch; the next timer tick handles a new mode.
            query_mode = self.mode
            now = rospy.Time.now()
            if query_mode == "FULL_LIVO":
                if (not self.last_healthy_query.is_zero() and
                        (now - self.last_healthy_query).to_sec() <
                        1.0 / max(0.2, self.healthy_query_hz)):
                    return
                self.last_healthy_query = now
            elif query_mode in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE"):
                if (not self.last_lidar_fail_query.is_zero() and
                        (now - self.last_lidar_fail_query).to_sec() <
                        1.0 / max(0.2, self.lidar_fail_query_hz)):
                    return
                self.last_lidar_fail_query = now
            with self.lock:
                if self.latest_color is None:
                    if query_mode in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE"):
                        rospy.logwarn_throttle(2.0, "[FAILSAFE PLACE FEATURE] waiting for input color")
                    return
                color = self.latest_color.copy()
                color_stamp = self.latest_color_stamp
                color_recv = self.latest_color_recv
                # Degraded fast-seed retrieval is appearance-only; metric
                # geometry is supplied by downstream depth->visual-map GICP.
                feature_lidar_fail = (
                    self.enable_on_lidar_fail and self.lidar_fail_feature_pnp_enable and
                    query_mode in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE"))
                needs_depth = (
                    (query_mode == "FULL_LIVO" and self.healthy_depth_pnp_enable) or
                    feature_lidar_fail)
                if needs_depth and self.latest_depth is None:
                    if feature_lidar_fail:
                        rospy.logwarn_throttle(2.0, "[FAILSAFE PLACE FEATURE] waiting for input depth")
                    return
                if needs_depth and self.depth_frames and not color_stamp.is_zero():
                    depth_stamp, nearest_depth = min(
                        self.depth_frames,
                        key=lambda pair: abs((pair[0]-color_stamp).to_sec()))
                    depth = nearest_depth.copy()
                else:
                    depth = None if self.latest_depth is None else self.latest_depth.copy()
                    depth_stamp = self.latest_depth_stamp

            # RGB-D pairing is only required by optional healthy metric PnP.
            if (needs_depth and not color_stamp.is_zero() and not depth_stamp.is_zero() and
                    abs((color_stamp-depth_stamp).to_sec()) > 0.08):
                if feature_lidar_fail:
                    rospy.logwarn_throttle(2.0,
                        "[FAILSAFE PLACE FEATURE] color/depth clock mismatch dt=%.3fs color=%.3f depth=%.3f",
                        (color_stamp-depth_stamp).to_sec(), color_stamp.to_sec(), depth_stamp.to_sec())
                return

            # Never spend CPU on an already-stale input.
            input_lag = abs((now-color_stamp).to_sec()) if not color_stamp.is_zero() else abs((now-color_recv).to_sec())
            if input_lag > 0.35:
                if feature_lidar_fail:
                    rospy.logwarn_throttle(2.0, "[FAILSAFE PLACE FEATURE] input too old lag=%.3fs", input_lag)
                return

            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            # Exact retrieval stage from rgbd_place_hypotheses.py.  Do not use
            # the previous 64x36 global-image shortcut: on the two-loop campus
            # route it could rank a visually similar but wrong repeated section.
            qsmall = cv2.resize(gray, (640, 360))
            use_akaze = feature_lidar_fail and self.lidar_fail_feature_use_akaze
            _, qret = (self.akaze.detectAndCompute(
                self.illumination_normalizer.apply(qsmall), None)
                if use_akaze else self.orb_ret.detectAndCompute(qsmall, None))
            if qret is None:
                if feature_lidar_fail:
                    rospy.logwarn_throttle(2.0, "[FAILSAFE PLACE FEATURE] no query ORB descriptors")
                return
            scores = []
            pool = self.retrieval_pool(now)
            if use_akaze:
                # Build a 0.05-s LSH index over only locally reachable saved
                # keyframes; global 615-frame / 755k-descriptor searches took
                # 4-6 seconds and their measurements were stale on arrival.
                # The frozen pre-failure lock supplies this *coarse* search
                # bound; metric RGB-D PnP and depth-map GICP still verify pose.
                sig = tuple(p.stem for _ts, p, _desc, _xyz in pool)
                if sig != self.akaze_local_index_signature:
                    items = [(ts, p, self.akaze_db[p.stem])
                             for ts, p, _desc, _xyz in pool
                             if p.stem in self.akaze_db and
                             len(self.akaze_db[p.stem]) >= 2]
                    if not items:
                        return
                    pool_descriptors = np.concatenate([item[2] for item in items], axis=0)
                    lengths = [len(item[2]) for item in items]
                    self.akaze_local_index_offsets = np.cumsum([0] + lengths)
                    self.akaze_local_index_frames = [(item[0], item[1]) for item in items]
                    self.akaze_local_index_matcher = cv2.FlannBasedMatcher(
                        dict(algorithm=6, table_number=7, key_size=17,
                             multi_probe_level=2), dict(checks=32))
                    self.akaze_local_index_matcher.add([pool_descriptors])
                    self.akaze_local_index_matcher.train()
                    self.akaze_local_index_signature = sig
                votes = {}
                try:
                    nearest = self.akaze_local_index_matcher.knnMatch(qret, k=8)
                except cv2.error as exc:
                    rospy.logwarn_throttle(1.0, "[FAILSAFE PLACE FEATURE] local LSH retrieval failed: %s", str(exc))
                    return
                for neighbors in nearest:
                    visited = set()
                    for match in neighbors:
                        if match.distance >= 100:
                            continue
                        candidate_idx = int(np.searchsorted(
                            self.akaze_local_index_offsets, match.trainIdx,
                            side='right') - 1)
                        if candidate_idx in visited:
                            continue
                        visited.add(candidate_idx)
                        votes[candidate_idx] = votes.get(candidate_idx, 0) + 1
                for candidate_idx, count in votes.items():
                    if count >= self.min_retrieval_matches:
                        ts, p = self.akaze_local_index_frames[candidate_idx]
                        scores.append((count, ts, p))
            else:
                for cts, p, cdes, _xyz in pool:
                    try:
                        knn = self.bf.knnMatch(qret, cdes, k=2)
                    except Exception:
                        continue
                    good_count = sum(
                        1 for z in knn
                        if len(z) == 2 and z[0].distance < 0.75 * z[1].distance)
                    if good_count >= self.min_retrieval_matches:
                        scores.append((good_count, cts, p))
            scores.sort(reverse=True, key=lambda x: x[0])
            top = scores[:min(self.top_retrieval, 5) if use_akaze else self.top_retrieval]
            if not top:
                self.place_miss_streak += 1
                if feature_lidar_fail:
                    rospy.logwarn_throttle(2.0, "[FAILSAFE PLACE FEATURE] no archived image retrieval matches pool=%d",len(pool))
                return
            if feature_lidar_fail:
                rospy.loginfo_throttle(1.0,
                    "[FAILSAFE PLACE FEATURE] retrieval candidates=%d top_score=%d",
                    len(top), top[0][0])

            # Visual retrieval is a REGION proposal; current sensor geometry
            # remains the metric authority. For LiDAR loss this is especially
            # important on the cart CPU: the previous second dense ORB+PnP stage
            # made absolute fixes sparse by many seconds. Map RGB keyframes are
            # ~0.5 s apart, so their loop-closed map<-LiDAR poses are excellent
            # coarse seeds for the existing current-depth coarse+fine GICP.
            # This keeps the standalone principle (appearance proposes, geometry
            # verifies) without running two metric-registration stages.
            if (query_mode in ("VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE", "LV_IMU_FAILED")
                    and not feature_lidar_fail or
                    (query_mode == "FULL_LIVO" and not self.healthy_depth_pnp_enable)):
                poses = []
                debug = []
                T_body_lidar = self.Tbc @ self.Tcl
                for retrieval_good, cts, _p in top:
                    Tml = self._interp_body(cts) @ T_body_lidar
                    pmsg = Pose()
                    pmsg.position.x, pmsg.position.y, pmsg.position.z = map(float, Tml[:3, 3])
                    qx, qy, qz, qw = R_to_quat(Tml[:3, :3])
                    pmsg.orientation.x, pmsg.orientation.y = qx, qy
                    pmsg.orientation.z, pmsg.orientation.w = qz, qw
                    poses.append(pmsg)
                    debug.append((retrieval_good, 0, cts))
                    if len(poses) >= self.max_candidates:
                        break
                if not poses:
                    self.place_miss_streak += 1
                    return
                self.place_miss_streak = 0
                arr = PoseArray()
                arr.header.stamp = color_stamp
                arr.header.frame_id = "saved_map_lidar_candidates"
                arr.poses = poses
                self.pub.publish(arr)
                lag = abs((rospy.Time.now()-color_stamp).to_sec())
                rospy.loginfo_throttle(
                    0.5,
                    "[FAILSAFE PLACE] %s fast-seed candidates=%d retrieval=%d key=%.3f lag=%.3fs",
                    query_mode, len(poses), debug[0][0], debug[0][2], lag)
                return

            aligned = self.aligned_depth(depth, gray.shape[0], gray.shape[1])
            dense_gray = (self.illumination_normalizer.apply(gray)
                          if use_akaze else gray)
            qkp, qdes = (self.akaze.detectAndCompute(dense_gray, None)
                         if use_akaze else self.orb_full.detectAndCompute(gray, None))
            if qdes is None:
                return

            poses = []
            debug = []
            metric_hypotheses = []
            max_raw_matches = 0
            max_depth_matches = 0
            max_pnp_inliers = 0
            for retrieval_good, cts, p in top:
                key = ("akaze:" if use_akaze else "orb:") + str(p)
                if key not in self.full_cache:
                    cim = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
                    if cim is None:
                        continue
                    if use_akaze:
                        cim = self.illumination_normalizer.apply(cim)
                    self.full_cache[key] = (
                        self.akaze.detectAndCompute(cim, None) if use_akaze
                        else self.orb_full.detectAndCompute(cim, None))
                    while len(self.full_cache) > max(8, self.full_cache_limit):
                        self.full_cache.popitem(last=False)
                else:
                    self.full_cache.move_to_end(key)
                ckp, cdes = self.full_cache[key]
                if cdes is None:
                    continue
                try:
                    knn = self.bf.knnMatch(qdes, cdes, k=2)
                except Exception:
                    continue
                good = [z[0] for z in knn
                        if len(z) == 2 and z[0].distance <
                        (0.78 if use_akaze else 0.74)*z[1].distance]
                if use_akaze and len(good) >= 12:
                    ptsq = np.float32([qkp[m.queryIdx].pt for m in good])
                    ptsm = np.float32([ckp[m.trainIdx].pt for m in good])
                    _, mask = cv2.findFundamentalMat(
                        ptsq, ptsm, cv2.FM_RANSAC, 2.0, 0.995)
                    if mask is None or len(mask) != len(good):
                        continue
                    good = [g for g, flag in zip(good, mask.ravel()) if flag]
                if feature_lidar_fail:
                    max_raw_matches = max(max_raw_matches, len(good))
                obj, img = [], []
                for m in good:
                    u, v = qkp[m.queryIdx].pt
                    z = self.sample_depth(aligned, u, v)
                    if z is None or z < self.depth_min or z > self.depth_max:
                        continue
                    norm = cv2.undistortPoints(
                        np.asarray([[[u, v]]], dtype=np.float64),
                        self.K, self.dist).reshape(2)
                    obj.append([norm[0]*z, norm[1]*z, z])
                    img.append(ckp[m.trainIdx].pt)
                if feature_lidar_fail:
                    max_depth_matches = max(max_depth_matches, len(obj))
                if len(obj) < (22 if feature_lidar_fail else 25):
                    continue
                obj = np.asarray(obj, dtype=np.float64)
                img = np.asarray(img, dtype=np.float64)
                ok, rvec, tvec, inl = cv2.solvePnPRansac(
                    obj, img, self.K, self.dist,
                    iterationsCount=160, reprojectionError=3.0,
                    confidence=0.995, flags=cv2.SOLVEPNP_EPNP)
                if not ok or inl is None or len(inl) < self.min_pnp_inliers:
                    continue
                ii = inl.reshape(-1)
                if feature_lidar_fail:
                    max_pnp_inliers = max(max_pnp_inliers, len(ii))
                ok2, rvec, tvec = cv2.solvePnP(
                    obj[ii], img[ii], self.K, self.dist,
                    rvec, tvec, True, flags=cv2.SOLVEPNP_ITERATIVE)
                if not ok2:
                    continue

                if feature_lidar_fail:
                    # In repeated-looking campus roads GICP fitness alone can
                    # be almost perfect at the *wrong* place. Require a large
                    # spatially distributed set of genuine image correspondences,
                    # not just a tiny patch of similar asphalt/foliage.
                    ratio = float(len(ii)) / max(1, len(obj))
                    if (len(ii) < self.lidar_fail_feature_min_inliers or
                            ratio < self.lidar_fail_feature_min_ratio):
                        continue
                    inlier_uv = img[ii]
                    xspan = float(np.ptp(inlier_uv[:, 0])) / max(1, float(self.K[0, 2] * 2))
                    yspan = float(np.ptp(inlier_uv[:, 1])) / max(1, float(self.K[1, 2] * 2))
                    rospy.loginfo_throttle(1.0,
                        "[FAILSAFE PLACE FEATURE GEOM] inliers=%d ratio=%.2f support=%.2f/%.2f",
                        len(ii),ratio,xspan,yspan)
                    if xspan < 0.18 or yspan < 0.12:
                        continue
                    projected, _ = cv2.projectPoints(
                        obj[ii].reshape(-1, 1, 3), rvec, tvec, self.K, self.dist)
                    err = np.linalg.norm(projected.reshape(-1, 2) - img[ii], axis=1)
                    reproj = float(np.median(err))
                    if not np.isfinite(reproj) or reproj > self.lidar_fail_feature_max_reproj_px:
                        continue

                Rc_q, _ = cv2.Rodrigues(rvec)
                T_candidate_query = tf(Rc_q, tvec.reshape(3))
                Tmb = self._interp_body(cts)
                Tmc = Tmb @ self.Tbc
                Tmq = Tmc @ T_candidate_query
                Tml = Tmq @ self.Tcl

                if feature_lidar_fail:
                    # Retain map position, not only an image keyframe index:
                    # adjacent saved frames form the same physical place while
                    # distant visually similar frames are genuine competitors.
                    metric_hypotheses.append((len(ii), ratio, reproj,
                                              retrieval_good, Tml, cts))
                    continue

                pmsg = Pose()
                pmsg.position.x, pmsg.position.y, pmsg.position.z = map(float, Tml[:3, 3])
                qx, qy, qz, qw = R_to_quat(Tml[:3, :3])
                pmsg.orientation.x, pmsg.orientation.y = qx, qy
                pmsg.orientation.z, pmsg.orientation.w = qz, qw
                poses.append(pmsg)
                debug.append((retrieval_good, len(ii), cts))
                if len(poses) >= self.max_candidates:
                    break

            if feature_lidar_fail:
                if not metric_hypotheses:
                    self.place_miss_streak += 1
                    self.lidar_fail_feature_streak = 0
                    self.lidar_fail_feature_history = None
                    rospy.loginfo_throttle(1.0, "[FAILSAFE PLACE FEATURE] no geometrically verified RGB-D image keyframe")
                    rospy.loginfo_throttle(1.0,
                        "[FAILSAFE PLACE FEATURE STATS] max2d=%d max3d=%d maxPnP=%d",
                        max_raw_matches, max_depth_matches, max_pnp_inliers)
                    return
                metric_hypotheses.sort(key=lambda h: (-h[0], -h[1], h[2]))
                best = metric_hypotheses[0]
                # Do not allow ambiguous high-score image hypotheses from
                # geographically different places to become global authority.
                ambiguous = any(
                    np.linalg.norm(alt[4][:3, 3] - best[4][:3, 3]) > 7.0 and
                    best[0] < self.lidar_fail_feature_unique_ratio * alt[0]
                    for alt in metric_hypotheses[1:])
                if ambiguous:
                    self.lidar_fail_feature_streak = 0
                    self.lidar_fail_feature_history = None
                    rospy.logwarn_throttle(1.0, "[FAILSAFE PLACE FEATURE] ambiguous visually similar map regions; withholding absolute fix")
                    return
                consistent = False
                if self.lidar_fail_feature_history is not None:
                    prev_stamp, prev_T = self.lidar_fail_feature_history
                    dt = (color_stamp - prev_stamp).to_sec()
                    if 0.20 < dt <= 5.0:
                        travel = float(np.linalg.norm(best[4][:3, 3] - prev_T[:3, 3]))
                        relative_R = prev_T[:3, :3].T @ best[4][:3, :3]
                        yaw_delta = float(np.linalg.norm(cv2.Rodrigues(relative_R)[0]))
                        consistent = (travel <= 1.0 + 5.0 * dt and
                                      yaw_delta <= math.radians(12.0 + 38.0 * dt))
                self.lidar_fail_feature_streak = (
                    self.lidar_fail_feature_streak + 1 if consistent else 1)
                self.lidar_fail_feature_history = (color_stamp, best[4])
                rospy.loginfo_throttle(
                    0.5, "[FAILSAFE PLACE FEATURE] inliers=%d ratio=%.2f reproj=%.2f streak=%d/%d key=%.3f",
                    best[0], best[1], best[2], self.lidar_fail_feature_streak,
                    self.lidar_fail_feature_confirmations, best[5])
                if self.lidar_fail_feature_streak < max(2, self.lidar_fail_feature_confirmations):
                    return
                Tml = best[4]
                pmsg = Pose()
                pmsg.position.x, pmsg.position.y, pmsg.position.z = map(float, Tml[:3, 3])
                qx, qy, qz, qw = R_to_quat(Tml[:3, :3])
                pmsg.orientation.x, pmsg.orientation.y = qx, qy
                pmsg.orientation.z, pmsg.orientation.w = qz, qw
                poses = [pmsg]
                debug = [(best[3], best[0], best[5])]

            if not poses:
                self.place_miss_streak += 1
                return
            self.place_miss_streak = 0
            arr = PoseArray()
            # The candidate is computed from this query image. Geometric
            # refinement will use a newer live cloud, so the map localizer
            # treats it as a coarse region rather than the final pose.
            arr.header.stamp = color_stamp
            arr.header.frame_id = (
                "saved_map_lidar_rgbd_metric_candidates"
                if query_mode == "FULL_LIVO" or feature_lidar_fail
                else "saved_map_lidar_candidates")
            arr.poses = poses
            self.pub.publish(arr)
            lag = abs((rospy.Time.now()-color_stamp).to_sec())
            rospy.loginfo_throttle(
                0.5,
                "[FAILSAFE PLACE] %s candidates=%d retrieval=%d pnp=%d key=%.3f lag=%.3fs",
                query_mode, len(poses), debug[0][0], debug[0][1],
                debug[0][2], lag)
            rospy.loginfo_throttle(
                1.0, "[FAILSAFE PLACE] retrieval pool=%d/%d", len(pool), len(self.db))
        finally:
            self.processing_lock.release()


if __name__ == "__main__":
    rospy.init_node("fast_livo2_visual_place_hint")
    try:
        VisualPlaceHint()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("[FAILSAFE PLACE] %s", exc)
        raise
