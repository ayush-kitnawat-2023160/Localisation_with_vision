#!/usr/bin/env python3
"""Offline-only state machine for a candidate *manual* localization reset.

No ROS dependency, no actuator interfaces, no TF publisher, no /pose publisher,
no service callers. This prototype deliberately NEVER authorizes an actual
vehicle reset. A verified hardware-originated stop/brake interlock and an
operator-authenticated control channel would be separate deployment projects.

Times are in one trusted, monotonic clock domain (e.g. ROS bag simulation time
for experiments). Do not mix replay sensor time with wall clock.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
import math
import secrets
from typing import Dict, Optional, Tuple


class Phase(str, Enum):
    IDLE = 'IDLE'
    PREPARING = 'PREPARING'
    APPROVED = 'APPROVED'
    DRYRUN_READY = 'DRYRUN_READY'
    ABORTED = 'ABORTED'


# Attestations must be produced by SEPARATE hardware/control components.
# A ROS topic alone is NOT an authenticated proof of the claimed state.
EXPECTED_ACKS = {
    'planner': 'STOPPED',
    'controller': 'MOTION_INHIBITED',
    'brake': 'ENGAGED',
    'tf_consumers': 'READY_FOR_EPOCH',
}


@dataclass(frozen=True)
class Evidence:
    candidate_stamp: float
    visual_stamp: float
    review_stamp: float
    stop_stamp: float
    map_frame: str
    map_version: str
    current_epoch: int
    current_session: str
    candidate_xyz: Tuple[float, float, float]
    candidate_yaw: float
    review_status: str
    independent_stop_verified: bool
    # Corroboration must already have been checked by the upstream reviewer.
    lidar_visual_confirmations: int
    median_lidar_visual_xy_error_m: float


@dataclass(frozen=True)
class Acknowledgment:
    component: str
    value: str
    stamp: float
    target_epoch: int
    candidate_fingerprint: str


@dataclass
class Transaction:
    transaction_id: str
    fingerprint: str
    evidence: Evidence
    target_epoch: int
    started_at: float
    last_refresh_at: float
    phase: Phase = Phase.PREPARING
    acknowledgments: Dict[str, Acknowledgment] = field(default_factory=dict)
    approved_at: Optional[float] = None
    approved_by: Optional[str] = None
    reason: str = ''


class HandoffError(ValueError):
    pass


class DryRunHandoff:
    def __init__(self, *, max_evidence_age=1.5, max_stop_age=.8,
                 max_ack_age=2.0, max_review_duration=12.0,
                 max_candidate_xy_motion_m=.7, max_confirm_error_m=2.0):
        self.max_evidence_age = float(max_evidence_age)
        self.max_stop_age = float(max_stop_age)
        self.max_ack_age = float(max_ack_age)
        self.max_review_duration = float(max_review_duration)
        self.max_candidate_xy_motion_m = float(max_candidate_xy_motion_m)
        self.max_confirm_error_m = float(max_confirm_error_m)
        self.transaction: Optional[Transaction] = None
        self.used_transaction_ids = set()

    @staticmethod
    def _finite(*numbers):
        return all(math.isfinite(float(v)) for v in numbers)

    def _validate_evidence(self, e: Evidence, now: float):
        if not self._finite(now, e.candidate_stamp, e.visual_stamp, e.review_stamp,
                            e.stop_stamp, e.candidate_yaw,
                            e.median_lidar_visual_xy_error_m, *e.candidate_xyz):
            raise HandoffError('NONFINITE_EVIDENCE')
        if not e.map_version.strip() or not e.current_session.strip() or e.map_frame != 'map':
            raise HandoffError('MAP_VERSION_FRAME_OR_SESSION_INVALID')
        if isinstance(e.current_epoch, bool) or e.current_epoch<0 or not isinstance(e.current_epoch,int):
            raise HandoffError('CURRENT_EPOCH_INVALID')
        if e.review_status != 'ELIGIBLE_FOR_OFFLINE_MANUAL_REVIEW':
            raise HandoffError('REVIEW_NOT_ELIGIBLE')
        if e.lidar_visual_confirmations<3 or e.median_lidar_visual_xy_error_m<0 or e.median_lidar_visual_xy_error_m>self.max_confirm_error_m:
            raise HandoffError('INSUFFICIENT_LIDAR_VISUAL_CONSENSUS')
        if not e.independent_stop_verified:
            raise HandoffError('NO_INDEPENDENT_STOP')
        for stamp in (e.candidate_stamp,e.visual_stamp,e.review_stamp):
            if not 0<=now-stamp<=self.max_evidence_age:
                raise HandoffError('STALE_OR_FUTURE_SENSOR_EVIDENCE')
        if not 0<=now-e.stop_stamp<=self.max_stop_age:
            raise HandoffError('STALE_OR_FUTURE_STOP_PROOF')

    @staticmethod
    def _fingerprint(e: Evidence):
        data = {'map':e.map_version,'frame':e.map_frame,'epoch':e.current_epoch,
                'session':e.current_session,
                'candidate_xyz':[round(float(x),3) for x in e.candidate_xyz],
                'candidate_yaw':round(float(e.candidate_yaw),5),
                'candidate_stamp':round(float(e.candidate_stamp),4)}
        return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()

    def _active(self):
        t=self.transaction
        if t is None or t.phase in (Phase.IDLE,Phase.ABORTED,Phase.DRYRUN_READY):
            raise HandoffError('NO_ACTIVE_TRANSACTION')
        return t

    def abort(self, reason):
        if self.transaction and self.transaction.phase not in (Phase.ABORTED,Phase.DRYRUN_READY):
            self.transaction.phase=Phase.ABORTED
            self.transaction.reason=str(reason)
        return {'phase':self.transaction.phase.value if self.transaction else Phase.IDLE.value,
                'reason':str(reason)}

    def prepare(self, evidence: Evidence, now: float):
        if self.transaction and self.transaction.phase not in (Phase.IDLE,Phase.ABORTED,Phase.DRYRUN_READY):
            raise HandoffError('ACTIVE_TRANSACTION_EXISTS')
        if (self.transaction and self.transaction.phase == Phase.DRYRUN_READY and
            evidence.map_version == self.transaction.evidence.map_version and
            evidence.current_session == self.transaction.evidence.current_session and
            evidence.current_epoch < self.transaction.target_epoch):
            # Do not begin another reset against the same source epoch just
            # because a previous dry run returned a manifest. A real system
            # must persist the epoch journal across process restarts.
            raise HandoffError('PREVIOUS_EPOCH_NOT_ADVANCED')
        self._validate_evidence(evidence,now)
        tx_id=secrets.token_hex(16)
        while tx_id in self.used_transaction_ids:
            tx_id=secrets.token_hex(16)
        self.used_transaction_ids.add(tx_id)
        self.transaction=Transaction(
            transaction_id=tx_id,fingerprint=self._fingerprint(evidence),
            evidence=evidence,target_epoch=evidence.current_epoch+1,
            started_at=now,last_refresh_at=now)
        return self.diagnostic()

    def refresh(self, evidence: Evidence, now: float):
        t=self._active()
        try:self._validate_evidence(evidence,now)
        except HandoffError as exc:
            self.abort(str(exc));raise
        if now<t.last_refresh_at:
            self.abort('CLOCK_REGRESSION');raise HandoffError('CLOCK_REGRESSION')
        if now-t.started_at>self.max_review_duration:
            self.abort('TRANSACTION_DEADLINE');raise HandoffError('TRANSACTION_DEADLINE')
        if (evidence.map_version!=t.evidence.map_version or
            evidence.current_epoch!=t.evidence.current_epoch or
            evidence.current_session!=t.evidence.current_session or
            evidence.map_frame!=t.evidence.map_frame):
            self.abort('EPOCH_MAP_OR_SESSION_CHANGED');raise HandoffError('EPOCH_MAP_OR_SESSION_CHANGED')
        if math.dist(evidence.candidate_xyz[:2],t.evidence.candidate_xyz[:2])>self.max_candidate_xy_motion_m:
            self.abort('CANDIDATE_CHANGED_TOO_FAR');raise HandoffError('CANDIDATE_CHANGED_TOO_FAR')
        t.evidence=evidence
        t.last_refresh_at=now
        return self.diagnostic()

    def acknowledge(self, acknowledgment: Acknowledgment, now: float):
        t=self._active()
        if not self._finite(now,acknowledgment.stamp):
            raise HandoffError('INVALID_ACK_TIME')
        if not 0<=now-acknowledgment.stamp<=self.max_ack_age:
            raise HandoffError('STALE_OR_FUTURE_ACK')
        if acknowledgment.component not in EXPECTED_ACKS or acknowledgment.value!=EXPECTED_ACKS[acknowledgment.component]:
            raise HandoffError('UNEXPECTED_COMPONENT_OR_STATE')
        if acknowledgment.target_epoch!=t.target_epoch or acknowledgment.candidate_fingerprint!=t.fingerprint:
            raise HandoffError('ACK_FOR_DIFFERENT_EPOCH_OR_CANDIDATE')
        previous=t.acknowledgments.get(acknowledgment.component)
        if previous and acknowledgment.stamp<=previous.stamp:
            raise HandoffError('REPLAYED_ACKNOWLEDGMENT')
        t.acknowledgments[acknowledgment.component]=acknowledgment
        return self.diagnostic()

    def approve(self, operator: str, transaction_id: str, fingerprint: str, now: float,
                *, explicitly_confirmed: bool=False):
        t=self._active()
        if t.phase!=Phase.PREPARING:
            raise HandoffError('INVALID_APPROVAL_PHASE')
        if not isinstance(operator,str) or not operator.strip() or not explicitly_confirmed:
            raise HandoffError('MISSING_EXPLICIT_OPERATOR_APPROVAL')
        if transaction_id!=t.transaction_id or fingerprint!=t.fingerprint:
            raise HandoffError('APPROVAL_FOR_DIFFERENT_TRANSACTION')
        if not self._finite(now) or now<t.started_at or now-t.started_at>self.max_review_duration:
            self.abort('APPROVAL_EXPIRED');raise HandoffError('APPROVAL_EXPIRED')
        t.phase=Phase.APPROVED
        t.approved_by=operator
        t.approved_at=now
        return self.diagnostic()

    def finalize_dryrun(self, evidence: Evidence, now: float):
        t=self._active()
        if t.phase!=Phase.APPROVED:
            raise HandoffError('OPERATOR_APPROVAL_MISSING')
        self.refresh(evidence,now)
        if now-t.approved_at>self.max_review_duration:
            self.abort('APPROVAL_EXPIRED');raise HandoffError('APPROVAL_EXPIRED')
        for component in EXPECTED_ACKS:
            a=t.acknowledgments.get(component)
            if a is None:
                raise HandoffError('MISSING_ACK_'+component.upper())
            if not 0<=now-a.stamp<=self.max_ack_age:
                self.abort('ACK_EXPIRED_'+component.upper());raise HandoffError('ACK_EXPIRED_'+component.upper())
        t.phase=Phase.DRYRUN_READY
        # A purely descriptive manifest. No reset service, TF or actuator calls.
        return {'status':'DRYRUN_READY_NOT_AUTHORITY',
                'transaction_id':t.transaction_id,'old_epoch':t.evidence.current_epoch,
                'proposed_epoch':t.target_epoch,'candidate_fingerprint':t.fingerprint,
                'map_version':t.evidence.map_version,
                'operator':t.approved_by,
                'acknowledged_components':sorted(t.acknowledgments),
                'actions_performed':[],
                'requires_live_vehicle_stop_brake_planner_tf_controller_and_operator':True}

    def diagnostic(self):
        t=self.transaction
        if t is None:return {'phase':Phase.IDLE.value}
        return {'phase':t.phase.value,'transaction_id':t.transaction_id,
                'candidate_fingerprint':t.fingerprint,
                'map_version':t.evidence.map_version,'source_session':t.evidence.current_session,
                'old_epoch':t.evidence.current_epoch,'proposed_epoch':t.target_epoch,
                'acked':sorted(t.acknowledgments),
                'missing_acks':sorted(set(EXPECTED_ACKS)-set(t.acknowledgments)),
                'reason':t.reason}
