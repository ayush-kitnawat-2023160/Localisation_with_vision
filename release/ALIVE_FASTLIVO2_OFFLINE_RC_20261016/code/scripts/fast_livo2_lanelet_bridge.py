#!/usr/bin/env python3
"""GPS/Lanelet initialisation bridge for a relative FAST-LIVO2 trajectory.

This node deliberately does *not* load a PCD, run registration, or modify
FAST-LIVO2.  It estimates one road-plane pose from GNSS plus Lanelet geometry,
then publishes the composition

    map -> fastlivo_odom -> base_link

where FAST-LIVO2 owns the second, full-6DoF transform.
"""

import math
import statistics
import threading
import xml.etree.ElementTree as ET
from collections import deque

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String
import tf2_ros


EARTH_A = 6378137.0
EARTH_E2 = 0.0066943799901413165


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def planar_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def median(values):
    return statistics.median(values) if values else float('nan')


def utm(latitude, longitude, zone):
    """WGS84 UTM (metres), equivalent to Lanelet2's UTM projector here."""
    latitude = math.radians(latitude)
    longitude = math.radians(longitude)
    eprime = EARTH_E2 / (1.0 - EARTH_E2)
    central_meridian = math.radians((zone - 1) * 6 - 180 + 3)
    n = EARTH_A / math.sqrt(1.0 - EARTH_E2 * math.sin(latitude) ** 2)
    t = math.tan(latitude) ** 2
    c = eprime * math.cos(latitude) ** 2
    a = math.cos(latitude) * (longitude - central_meridian)
    m = EARTH_A * ((1 - EARTH_E2 / 4 - 3 * EARTH_E2 ** 2 / 64 -
                    5 * EARTH_E2 ** 3 / 256) * latitude -
                   (3 * EARTH_E2 / 8 + 3 * EARTH_E2 ** 2 / 32 +
                    45 * EARTH_E2 ** 3 / 1024) * math.sin(2 * latitude) +
                   (15 * EARTH_E2 ** 2 / 256 + 45 * EARTH_E2 ** 3 / 1024) *
                   math.sin(4 * latitude) -
                   (35 * EARTH_E2 ** 3 / 3072) * math.sin(6 * latitude))
    east = 0.9996 * n * (a + (1 - t + c) * a ** 3 / 6 +
                          (5 - 18 * t + t * t + 72 * c - 58 * eprime) *
                          a ** 5 / 120) + 500000.0
    north = 0.9996 * (m + n * math.tan(latitude) *
                      (a * a / 2 + (5 - t + 9 * c + 4 * c * c) * a ** 4 / 24 +
                       (61 - 58 * t + t * t + 600 * c - 330 * eprime) *
                       a ** 6 / 720))
    if latitude < 0:
        north += 10000000.0
    return east, north


def closest_on_segment(point, first, second):
    dx, dy = second[0] - first[0], second[1] - first[1]
    length2 = dx * dx + dy * dy
    fraction = 0.0 if length2 < 1e-12 else max(0.0, min(1.0,
        ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / length2))
    projected = (first[0] + fraction * dx, first[1] + fraction * dy)
    return planar_distance(point, projected), math.atan2(dy, dx), projected


class LaneletMap:
    """Small read-only Lanelet OSM reader for centerline candidate selection."""

    def __init__(self, filename, origin_lat, origin_lon, zone):
        self.origin_east, self.origin_north = utm(origin_lat, origin_lon, zone)
        root = ET.parse(filename).getroot()
        nodes = {}
        for node in root.findall('node'):
            east, north = utm(float(node.attrib['lat']), float(node.attrib['lon']), zone)
            nodes[node.attrib['id']] = (east - self.origin_east, north - self.origin_north)
        ways = {}
        for way in root.findall('way'):
            points = [nodes[member.attrib['ref']] for member in way.findall('nd')
                      if member.attrib['ref'] in nodes]
            if len(points) >= 2:
                ways[way.attrib['id']] = points
        self.lanelets = []
        for relation in root.findall('relation'):
            tags = {tag.attrib.get('k'): tag.attrib.get('v') for tag in relation.findall('tag')}
            if tags.get('type') != 'lanelet':
                continue
            members = {member.attrib.get('role'): member.attrib.get('ref')
                       for member in relation.findall('member')
                       if member.attrib.get('type') == 'way'}
            left, right = ways.get(members.get('left')), ways.get(members.get('right'))
            if not left or not right:
                continue
            # Lanelet boundaries normally have matching direction. Correct the
            # occasional reversed right boundary before calculating midpoints.
            if planar_distance(left[0], right[-1]) + planar_distance(left[-1], right[0]) < \
               planar_distance(left[0], right[0]) + planar_distance(left[-1], right[-1]):
                right = list(reversed(right))
            count = max(len(left), len(right))
            center = []
            for index in range(count):
                lp = left[round(index * (len(left) - 1) / max(1, count - 1))]
                rp = right[round(index * (len(right) - 1) / max(1, count - 1))]
                center.append(((lp[0] + rp[0]) * .5, (lp[1] + rp[1]) * .5))
            if len(center) >= 2:
                self.lanelets.append((relation.attrib['id'], center))
        if not self.lanelets:
            raise ValueError('no Lanelet relations with usable boundaries')

    def candidates(self, point, radius):
        result = []
        for lanelet_id, centerline in self.lanelets:
            best = min((closest_on_segment(point, centerline[index], centerline[index + 1])
                        for index in range(len(centerline) - 1)), key=lambda value: value[0])
            if best[0] <= radius:
                result.append((lanelet_id, best[0], best[1], best[2]))
        return result


def matrix_from_pose(pose):
    matrix = tf.transformations.quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                                    pose.orientation.z, pose.orientation.w))
    matrix[0, 3], matrix[1, 3], matrix[2, 3] = pose.position.x, pose.position.y, pose.position.z
    return matrix


def pose_from_matrix(matrix):
    pose = PoseStamped().pose
    pose.position.x, pose.position.y, pose.position.z = matrix[0, 3], matrix[1, 3], matrix[2, 3]
    quaternion = tf.transformations.quaternion_from_matrix(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    return pose


def planar_pose_matrix(x, y, yaw):
    return tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix((x, y, 0.0)),
        tf.transformations.euler_matrix(0.0, 0.0, yaw))


class LaneletBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~fastlivo_odom_frame', 'fastlivo_odom')
        self.native_frame = rospy.get_param('~native_frame', 'camera_init')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.native_odom_topic = rospy.get_param('~native_odom_topic', '/aft_mapped_to_init')
        self.gps_topic = rospy.get_param('~gps_topic', '/mavros/global_position/global')
        self.lanelet_file = rospy.get_param('~lanelet_map_file')
        self.origin_lat = float(rospy.get_param('~lanelet_origin_lat', 28.54875901))
        self.origin_lon = float(rospy.get_param('~lanelet_origin_lon', 77.27399459))
        self.utm_zone = int(rospy.get_param('~utm_zone', 43))
        self.gps_min_samples = int(rospy.get_param('~gps_min_samples', 20))
        self.gps_outlier_m = float(rospy.get_param('~gps_outlier_threshold_m', 8.0))
        self.gps_max_sigma = float(rospy.get_param('~gps_max_sigma_m', 15.0))
        self.search_radius = float(rospy.get_param('~lanelet_search_radius_m', 20.0))
        self.min_displacement = float(rospy.get_param('~minimum_course_displacement_m', 6.0))
        self.heading_weight = float(rospy.get_param('~heading_weight', 8.0))
        self.distance_weight = float(rospy.get_param('~distance_weight', 1.0))
        self.route_weight = float(rospy.get_param('~route_weight', 3.0))
        self.confidence_min = float(rospy.get_param('~initialization_confidence_threshold', .55))
        self.drift_enabled = bool(rospy.get_param('~gps_drift_correction_enabled', False))
        self.route_ids = set(str(value) for value in rospy.get_param('~preferred_lanelet_ids', []))
        self.map = LaneletMap(self.lanelet_file, self.origin_lat, self.origin_lon, self.utm_zone)
        self.gps = deque(maxlen=max(self.gps_min_samples * 20, 400))
        self.local_start = None
        self.local_latest = None
        self.anchor = None
        # Optional operator-provided RViz estimate.  It is deliberately kept
        # separate from the GPS/Lanelet path: an operator can seed the global
        # frame when GNSS is unavailable or when the recorded bag is from a
        # different map revision.
        self.manual_initial_pose = None
        self.matched_lanelet = None
        self.path = Path(header=Header(frame_id=self.map_frame))
        self.tf = tf2_ros.TransformBroadcaster()
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        # Keep the native cloud anchor on a separate latched broadcaster; a
        # ROS1 static broadcaster retains its last message.
        self.native_static_tf = tf2_ros.StaticTransformBroadcaster()
        self.status_pub = rospy.Publisher('/fast_livo2/localization_status', String, queue_size=2, latch=True)
        self.match_pub = rospy.Publisher('/fast_livo2/matched_lanelet', String, queue_size=2, latch=True)
        self.gps_pose_pub = rospy.Publisher('/fast_livo2/gps_map_pose', PoseStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=20)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=20)
        self.path_pub = rospy.Publisher('/fast_livo2/global_trajectory', Path, queue_size=2)
        rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_cb, queue_size=100)
        rospy.Subscriber(self.native_odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped,
                         self.initial_pose_cb, queue_size=2)
        self.status('WAITING_FOR_GPS_AND_FASTLIVO_ODOM')

    def status(self, text):
        self.status_pub.publish(String(data=text))
        rospy.loginfo('FAST-LIVO2 Lanelet bridge: %s', text)

    def gps_cb(self, message):
        if message.status.status < 0 or not math.isfinite(message.latitude) or not math.isfinite(message.longitude):
            return
        variance = message.position_covariance[0]
        if variance > 0.0 and math.sqrt(variance) > self.gps_max_sigma:
            return
        east, north = utm(message.latitude, message.longitude, self.utm_zone)
        point = (east - self.map.origin_east, north - self.map.origin_north)
        with self.lock:
            self.gps.append((message.header.stamp, point))

    def robust_gps(self):
        if len(self.gps) < self.gps_min_samples:
            return None
        x = median([sample[1][0] for sample in self.gps])
        y = median([sample[1][1] for sample in self.gps])
        accepted = [sample for sample in self.gps if planar_distance(sample[1], (x, y)) <= self.gps_outlier_m]
        if len(accepted) < self.gps_min_samples:
            return None
        return median([sample[1][0] for sample in accepted]), median([sample[1][1] for sample in accepted])

    def courses(self):
        if not self.local_start or not self.local_latest or len(self.gps) < 2:
            return None, None
        local_delta = (self.local_latest.pose.pose.position.x - self.local_start.pose.pose.position.x,
                       self.local_latest.pose.pose.position.y - self.local_start.pose.pose.position.y)
        first, last = self.gps[0][1], self.gps[-1][1]
        gps_delta = (last[0] - first[0], last[1] - first[1])
        local_distance, gps_distance = math.hypot(*local_delta), math.hypot(*gps_delta)
        local_course = math.atan2(local_delta[1], local_delta[0]) if local_distance >= self.min_displacement else None
        gps_course = math.atan2(gps_delta[1], gps_delta[0]) if gps_distance >= self.min_displacement else None
        return local_course, gps_course

    def try_initialize(self):
        if self.anchor is not None or self.local_latest is None:
            return
        if self.manual_initial_pose is not None:
            # RViz publishes map->base_link.  FAST-LIVO2 publishes the
            # corresponding fastlivo_odom->base_link pose, so their
            # composition gives the required map->fastlivo_odom anchor.
            global_base = matrix_from_pose(self.manual_initial_pose.pose.pose)
            local = matrix_from_pose(self.local_latest.pose.pose)
            self.anchor = global_base.dot(tf.transformations.inverse_matrix(local))
            self.send_static_tf(self.map_frame, self.odom_frame, self.anchor)
            self.send_native_static_tf(self.map_frame, self.native_frame, self.anchor)
            self.status('MANUAL_INITIALIZED map_xy=(%.2f,%.2f) yaw=%.3f' %
                        (global_base[0, 3], global_base[1, 3],
                         tf.transformations.euler_from_matrix(global_base)[2]))
            return
        point = self.robust_gps()
        if point is None:
            return
        self.publish_gps(point)
        candidates = self.map.candidates(point, self.search_radius)
        if not candidates:
            self.status('REJECTED_NO_LANELET_WITHIN_%.1fm_OF_GPS x=%.2f y=%.2f' %
                        (self.search_radius, point[0], point[1]))
            return
        local_course, gps_course = self.courses()
        if local_course is None:
            self.status('WAITING_FOR_FASTLIVO_MOTION min_displacement=%.1fm candidates=%d' %
                        (self.min_displacement, len(candidates)))
            return
        scored = []
        for lanelet_id, distance, tangent, projection in candidates:
            for lane_yaw in (tangent, wrap(tangent + math.pi)):
                heading_error = abs(wrap(lane_yaw - gps_course)) if gps_course is not None else math.pi / 2
                route_penalty = 0.0 if not self.route_ids or lanelet_id in self.route_ids else 1.0
                score = self.distance_weight * distance + self.heading_weight * heading_error + self.route_weight * route_penalty
                scored.append((score, lanelet_id, distance, lane_yaw, heading_error, projection))
        scored.sort(key=lambda value: value[0])
        best = scored[0]
        runner_up = scored[1] if len(scored) > 1 else None
        margin = 1.0 if runner_up is None else max(0.0, min(1.0, (runner_up[0] - best[0]) / max(1.0, runner_up[0])))
        confidence = (1.0 / (1.0 + best[0] / max(1.0, self.search_radius))) * (.5 + .5 * margin)
        if confidence < self.confidence_min:
            self.status('AMBIGUOUS_LANELET best=%s score=%.2f confidence=%.2f threshold=%.2f' %
                        (best[1], best[0], confidence, self.confidence_min))
            return
        local = matrix_from_pose(self.local_latest.pose.pose)
        global_base = planar_pose_matrix(best[5][0], best[5][1], best[3])
        self.anchor = global_base.dot(tf.transformations.inverse_matrix(local))
        self.send_static_tf(self.map_frame, self.odom_frame, self.anchor)
        self.send_native_static_tf(self.map_frame, self.native_frame, self.anchor)
        self.matched_lanelet = best
        self.match_pub.publish(String(data='lanelet_id=%s distance_m=%.3f heading_error_rad=%.3f confidence=%.3f' %
                                      (best[1], best[2], best[4], confidence)))
        self.status('LOCALIZED lanelet=%s confidence=%.2f map_xy=(%.2f,%.2f) yaw=%.3f' %
                    (best[1], confidence, best[5][0], best[5][1], best[3]))

    def publish_gps(self, point):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.pose.position.x, message.pose.position.y = point
        message.pose.orientation.w = 1.0
        self.gps_pose_pub.publish(message)

    def send_tf(self, parent, child, matrix, stamp):
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, parent, child
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = matrix[0, 3], matrix[1, 3], matrix[2, 3]
        quaternion = tf.transformations.quaternion_from_matrix(matrix)
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = quaternion
        self.tf.sendTransform(transform)

    def send_static_tf(self, parent, child, matrix):
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = rospy.Time(0), parent, child
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = matrix[0, 3], matrix[1, 3], matrix[2, 3]
        quaternion = tf.transformations.quaternion_from_matrix(matrix)
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = quaternion
        self.static_tf.sendTransform(transform)

    def send_native_static_tf(self, parent, child, matrix):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time(0)
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.transform.translation.x = matrix[0, 3]
        transform.transform.translation.y = matrix[1, 3]
        transform.transform.translation.z = matrix[2, 3]
        quaternion = tf.transformations.quaternion_from_matrix(matrix)
        transform.transform.rotation.x, transform.transform.rotation.y = quaternion[0], quaternion[1]
        transform.transform.rotation.z, transform.transform.rotation.w = quaternion[2], quaternion[3]
        self.native_static_tf.sendTransform(transform)

    def odom_cb(self, message):
        with self.lock:
            self.local_latest = message
            if self.local_start is None:
                self.local_start = message
            self.try_initialize()
            if self.anchor is None:
                return
            local = matrix_from_pose(message.pose.pose)
            global_pose = self.anchor.dot(local)
            self.send_tf(self.odom_frame, self.base_frame, local, message.header.stamp)
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = message.header.stamp, self.map_frame
            pose.pose = pose_from_matrix(global_pose)
            odom = Odometry()
            odom.header, odom.child_frame_id, odom.pose.pose, odom.twist = pose.header, self.base_frame, pose.pose, message.twist
            self.pose_pub.publish(pose)
            self.odom_pub.publish(odom)
            self.path.header.stamp = message.header.stamp
            self.path.poses.append(pose)
            if len(self.path.poses) > 10000:
                self.path.poses.pop(0)
            if len(self.path.poses) % 5 == 0:
                self.path_pub.publish(self.path)

    def initial_pose_cb(self, message):
        """Accept RViz's 2D Pose Estimate as a global map seed.

        The next FAST-LIVO2 odometry sample completes the transform, making
        this safe even when the button is pressed before the first LiDAR frame.
        """
        frame = message.header.frame_id.lstrip('/') or self.map_frame
        if frame != self.map_frame:
            self.status('IGNORED_INITIALPOSE_FRAME expected=%s got=%s' %
                        (self.map_frame, frame))
            return
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            self.status('IGNORED_INITIALPOSE_NONFINITE')
            return
        with self.lock:
            self.manual_initial_pose = message
            # Allow an operator to correct a previous seed while the node is
            # running.  The next odometry callback will publish the new anchor.
            self.anchor = None
            self.matched_lanelet = None
            self.path.poses = []
            self.status('RECEIVED_MANUAL_INITIALPOSE map_xy=(%.2f,%.2f)' %
                        (pose.position.x, pose.position.y))
            self.try_initialize()


if __name__ == '__main__':
    rospy.init_node('fast_livo2_lanelet_bridge')
    LaneletBridge()
    rospy.spin()
