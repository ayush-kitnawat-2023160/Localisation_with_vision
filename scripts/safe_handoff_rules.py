#!/usr/bin/env python3
"""Pure, deliberately conservative rules for an OFFLINE handoff review.

No ROS dependency here: replay and synthetic tests can exercise precisely the
same predicates. This NEVER controls a robot, resets an EKF, or emits /pose.
A review-ready result still requires independently verified physical stop,
calibration, surveyed map, and authorized vehicle control interlocks.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    xyz: tuple
    yaw: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0


def wrapped(a):
    return math.atan2(math.sin(a), math.cos(a))


def evaluate_review(status, lidar, visual, odom, now, *,
                    externally_verified_stop=False,
                    min_confirmations=3, min_stationary_sec=2.5,
                    max_candidate_age_sec=1.5, max_match_offset_sec=1.0,
                    max_cross_sensor_distance_m=2.0,
                    max_stationary_speed_mps=0.15,
                    max_stationary_yaw_rate_rps=0.08,
                    max_odom_interval_sec=0.35,
                    max_lidar_jump_m=1.4):
    """Return (eligible_for_manual_review, reason, diagnostics).

    Uses sensor time only. Every LiDAR image pairing must use distinct
    timestamped measurements; no duplication of the same camera frame.
    """
    detail = {'lidar_samples': len(lidar), 'visual_samples': len(visual), 'odom_samples': len(odom)}
    if status != 'RELOCALIZING: MH_HANDOFF_REQUIRED':
        return False, 'GLOBAL_STATUS_NOT_HANDOFF_REQUIRED', detail
    if not all(math.isfinite(x) for x in [now, *[s.stamp for s in lidar+visual+odom]]):
        return False, 'INVALID_SENSOR_TIME', detail
    if not lidar or not visual or not odom:
        return False, 'MISSING_SENSOR_EVIDENCE', detail
    # NaN comparisons against threshold are false in Python: checking only
    # delta > limit could incorrectly accept a physically invalid sample.
    if not all(math.isfinite(v)
               for item in lidar+visual+odom
               for v in (*item.xyz,item.yaw,item.speed,item.yaw_rate)):
        return False, 'INVALID_POSE_OR_MOTION_SAMPLE',detail
    ls = sorted(lidar, key=lambda x:x.stamp)
    vs = sorted(visual, key=lambda x:x.stamp)
    os = sorted(odom, key=lambda x:x.stamp)
    if now-ls[-1].stamp > max_candidate_age_sec or ls[-1].stamp-now > .15:
        return False, 'STALE_LIDAR_RECOVERY_CANDIDATE', detail
    if now-vs[-1].stamp > max_candidate_age_sec or vs[-1].stamp-now > .15:
        return False, 'STALE_VISUAL_CANDIDATE', detail
    if now-os[-1].stamp > .35 or os[-1].stamp-now > .15:
        return False, 'STALE_ODOMETRY', detail
    earliest = now-min_stationary_sec
    stationary = [x for x in os if earliest-.3 <= x.stamp <= now+.1]
    if len(stationary)<6 or stationary[0].stamp > earliest+.20:
        return False, 'INSUFFICIENT_STATIONARY_OBSERVATION_WINDOW', detail
    for a,b in zip(stationary,stationary[1:]):
        dt=b.stamp-a.stamp
        if dt<=0 or dt>max_odom_interval_sec:
            return False, 'GAPPED_OR_NONMONOTONIC_ODOMETRY', detail
        displacement=math.dist(a.xyz,b.xyz)
        if displacement/dt>max_stationary_speed_mps:
            return False, 'ROBOT_NOT_STATIONARY_POSITION',detail
        if abs(wrapped(b.yaw-a.yaw))/dt>max_stationary_yaw_rate_rps:
            return False, 'ROBOT_NOT_STATIONARY_ROTATION',detail
    if (any(abs(s.speed)>max_stationary_speed_mps or abs(s.yaw_rate)>max_stationary_yaw_rate_rps
            for s in stationary)):
        return False, 'ROBOT_NOT_STATIONARY_TWIST',detail
    if math.dist(stationary[0].xyz,stationary[-1].xyz)>.30:
        return False, 'ROBOT_STATIONARY_WINDOW_DRIFT',detail
    # Pair most recent independent LiDAR and camera observations, not repeated
    # references to one cached pose or to the same timestamp.
    candidates=[x for x in ls if now-5.0<=x.stamp<=now+.1]
    used_visual=set()
    pairs=[]
    for li in reversed(candidates):
        options=[(abs(li.stamp-vi.stamp),i,vi) for i,vi in enumerate(vs)
                 if i not in used_visual and abs(li.stamp-vi.stamp)<=max_match_offset_sec]
        if not options:
            continue
        delta,i,vi=min(options)
        dist=math.dist(li.xyz[:2],vi.xyz[:2]);detail['last_cross_sensor_xy_error_m']=round(dist,3)
        if dist>max_cross_sensor_distance_m:
            return False, 'VISUAL_LIDAR_BRANCH_DISAGREEMENT',detail
        used_visual.add(i)
        pairs.append((li,vi))
        if len(pairs)>=min_confirmations:break
    detail['independent_correspondence_pairs']=len(pairs)
    if len(pairs)<min_confirmations:
        return False, 'INSUFFICIENT_INDEPENDENT_LIDAR_VISUAL_CONFIRMATIONS', detail
    lp=sorted((li for li,_ in pairs),key=lambda x:x.stamp)
    if any(math.dist(a.xyz[:2],b.xyz[:2])>max_lidar_jump_m for a,b in zip(lp,lp[1:])):
        return False, 'LIDAR_CANDIDATES_INCONSISTENT_WHILE_STOPPED',detail
    # Motion estimates from the SAME failed odometry estimator are not an
    # independent physical stop signal. A supervised reset must additionally
    # require a separately verified STOP interlock (never available in bags).
    if not externally_verified_stop:
        return False, 'EXTERNAL_STOP_INTERLOCK_NOT_VERIFIED',detail
    return True, 'ELIGIBLE_FOR_OFFLINE_MANUAL_REVIEW_NOT_AUTHORITY',detail
