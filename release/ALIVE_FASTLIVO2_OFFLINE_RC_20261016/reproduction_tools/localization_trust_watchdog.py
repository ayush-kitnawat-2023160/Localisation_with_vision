#!/usr/bin/env python3
"""Opt-in fail-closed localization health watchdog. NOT a DBW stop command.

Publishes /fast_livo2/localization_stop_required=True unless ALL conditions
are satisfied: fresh independent map trust, fresh map pose, physically
plausible consecutive motion and multiple healthy samples. A separate
hardware-tested actuator arbiter MUST honor this before cart deployment.
"""
import math
import threading
import time


class TrustMonitor:
    """Pure, deterministic state machine. Times supplied by caller are monotonic."""
    def __init__(self, max_age=0.5, max_speed=5.0, step_slack=0.15,
                 rearm_samples=5, max_pose_gap=0.6,
                 max_yaw_rate_deg_s=120.0, yaw_slack_deg=8.0):
        self.max_age=float(max_age)
        self.max_speed=float(max_speed)
        self.step_slack=float(step_slack)
        self.rearm_samples=int(rearm_samples)
        self.max_pose_gap=float(max_pose_gap)
        self.max_yaw_rate=math.radians(float(max_yaw_rate_deg_s))
        self.yaw_slack=math.radians(float(yaw_slack_deg))
        self.last_trust_receipt=None
        self.last_pose_receipt=None
        self.last_trust=False
        self.last_pose_stamp=None
        self.last_xyz=None
        self.last_yaw=None
        self.pose_safe=False
        # A physically impossible map pose is not repaired merely by several
        # subsequent frames on the wrong branch. Require explicit watchdog
        # restart/independent recovery procedure, never time-only auto-clear.
        self.hard_fault_latched=False
        self.healthy_streak=0
        self.reason='STARTUP_NO_VERIFICATION'

    def trust(self, trusted, now):
        self.last_trust=bool(trusted)
        self.last_trust_receipt=float(now)
        if not trusted:
            self.healthy_streak=0
            self.pose_safe=False
            self.reason='INDEPENDENT_MAP_FIX_UNTRUSTED'

    def outage_smoothing_unsafe(self, unsafe):
        """Do not permit a smooth-looking but invalid map pose to re-arm.

        Independent of normal map-trust heartbeat. The trial producer may
        publish false again after an outage, but this hard latch requires
        operator inspection/restart and cannot clear from later frames.
        """
        if unsafe:
            self.hard_fault_latched=True
            self.pose_safe=False
            self.healthy_streak=0
            self.reason='SMOOTHED_POSE_MAP_BACKLOG_UNTRUSTED'

    def pose(self, stamp, xyz, now, yaw=None):
        stamp=float(stamp)
        now=float(now)
        if (not all(math.isfinite(v) for v in xyz) or not math.isfinite(stamp)
                or (yaw is not None and not math.isfinite(yaw))):
            self.healthy_streak=0
            self.pose_safe=False
            self.hard_fault_latched=True
            self.reason='NONFINITE_POSE'
            return
        safe=True
        if self.last_pose_stamp is not None:
            dt=stamp-self.last_pose_stamp
            if dt<=0 or dt>self.max_pose_gap:
                safe=False
                self.reason='POSE_TIMESTAMP_GAP'
            elif self.last_xyz is not None:
                d=math.dist(tuple(xyz), self.last_xyz)
                if d>self.max_speed*dt+self.step_slack:
                    safe=False
                    self.hard_fault_latched=True
                    self.reason='IMPLAUSIBLE_POSE_STEP'
                if yaw is not None and self.last_yaw is not None:
                    dyaw=math.atan2(math.sin(yaw-self.last_yaw),
                                     math.cos(yaw-self.last_yaw))
                    if abs(dyaw)>self.max_yaw_rate*dt+self.yaw_slack:
                        safe=False
                        self.hard_fault_latched=True
                        self.reason='IMPLAUSIBLE_YAW_JUMP'
        self.last_pose_stamp=stamp
        self.last_xyz=tuple(xyz)
        self.last_yaw=yaw
        self.last_pose_receipt=now
        trust_fresh=(self.last_trust and self.last_trust_receipt is not None and
                     0.0<=now-self.last_trust_receipt<=self.max_age)
        if safe and trust_fresh and not self.hard_fault_latched:
            self.pose_safe=True
            self.healthy_streak+=1
        else:
            self.pose_safe=False
            self.healthy_streak=0

    def verdict(self, now):
        now=float(now)
        if self.hard_fault_latched:
            return False,'HARD_POSE_FAULT_REQUIRES_OPERATOR_RESTART'
        if not self.last_trust or self.last_trust_receipt is None:
            return False,'INDEPENDENT_MAP_FIX_UNTRUSTED'
        if now-self.last_trust_receipt>self.max_age or self.last_trust_receipt>now:
            return False,'MAP_TRUST_HEARTBEAT_TIMEOUT'
        if self.last_pose_receipt is None or now-self.last_pose_receipt>self.max_age:
            return False,'POSE_HEARTBEAT_TIMEOUT'
        if not self.pose_safe:return False,self.reason
        if self.healthy_streak<self.rearm_samples:return False,'REARMING'
        return True,'VERIFIED_CONTINUOUS_MAP_POSE'


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String
    rospy.init_node('localization_trust_watchdog')
    monitor=TrustMonitor(
        max_age=rospy.get_param('~max_age_sec',0.5),
        max_speed=rospy.get_param('~max_speed_mps',5.0),
        step_slack=rospy.get_param('~pose_step_slack_m',0.15),
        rearm_samples=rospy.get_param('~rearm_samples',5),
        max_pose_gap=rospy.get_param('~max_pose_gap_sec',0.6),
        max_yaw_rate_deg_s=rospy.get_param('~max_yaw_rate_deg_s',120.0),
        yaw_slack_deg=rospy.get_param('~yaw_slack_deg',8.0))
    lock=threading.Lock()
    stop_pub=rospy.Publisher('/fast_livo2/localization_stop_required',Bool,queue_size=2,latch=True)
    reason_pub=rospy.Publisher('/fast_livo2/localization_trust_reason',String,queue_size=2,latch=True)
    stop_pub.publish(Bool(data=True))
    reason_pub.publish(String(data='STARTUP_NO_VERIFICATION'))

    def on_trust(msg):
        with lock:monitor.trust(msg.data,time.monotonic())

    def on_smoothing_unsafe(msg):
        with lock:monitor.outage_smoothing_unsafe(msg.data)

    def on_pose(msg):
        p=msg.pose.position
        q=msg.pose.orientation
        qnorm=q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z
        if not math.isfinite(qnorm) or abs(qnorm-1.0)>0.10:
            yaw=float('nan')
        else:
            yaw=math.atan2(2.0*(q.w*q.z+q.x*q.y),
                            1.0-2.0*(q.y*q.y+q.z*q.z))
        with lock:monitor.pose(msg.header.stamp.to_sec(),(p.x,p.y,p.z),time.monotonic(),yaw=yaw)

    def tick(_):
        with lock:safe,reason=monitor.verdict(time.monotonic())
        stop_pub.publish(Bool(data=not safe))
        reason_pub.publish(String(data=reason))
        if not safe:rospy.logwarn_throttle(1.0,'[TRUST WATCHDOG] stop required: %s',reason)

    rospy.Subscriber('/fast_livo2/step3_pose_trusted',Bool,on_trust,queue_size=1)
    rospy.Subscriber('/fast_livo2/step3_outage_smoothing_untrusted',Bool,
                     on_smoothing_unsafe,queue_size=2)
    rospy.Subscriber('/fast_livo2/step3_global_pose',PoseStamped,on_pose,queue_size=2)
    rospy.Timer(rospy.Duration(0.1),tick)
    rospy.spin()

if __name__=='__main__':main()
