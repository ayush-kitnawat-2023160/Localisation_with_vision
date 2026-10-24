#!/usr/bin/env python3
"""Unit tests for dry-run handoff transaction; no ROS or vehicle access."""
import sys
import unittest
from dataclasses import replace
from supervised_handoff_transaction import DryRunHandoff,Evidence,Acknowledgment,HandoffError,Phase,EXPECTED_ACKS

class HandoffTest(unittest.TestCase):
    def setUp(self):
        self.now=100.0
        self.e=Evidence(candidate_stamp=99.7,visual_stamp=99.6,review_stamp=99.7,
            stop_stamp=99.8,map_frame='map',map_version='campus_full_lio_226s_20261019',
            current_epoch=4,current_session='20261020-session-001',
            candidate_xyz=(20.,-6.,1.5),candidate_yaw=.5,
            review_status='ELIGIBLE_FOR_OFFLINE_MANUAL_REVIEW',independent_stop_verified=True,
            lidar_visual_confirmations=3,median_lidar_visual_xy_error_m=.25)
        self.m=DryRunHandoff()
    def prepare(self):return self.m.prepare(self.e,self.now)
    def ack(self,component,stamp=100.1,**kwargs):
        t=self.m.transaction
        return self.m.acknowledge(Acknowledgment(component,EXPECTED_ACKS[component],stamp,
            t.target_epoch,t.fingerprint),stamp+.05,**kwargs)
    def full(self):
        self.prepare()
        for c in EXPECTED_ACKS:self.ack(c)
        t=self.m.transaction
        self.m.approve('test-human',t.transaction_id,t.fingerprint,100.3,explicitly_confirmed=True)
        e=replace(self.e,candidate_stamp=100.2,visual_stamp=100.2,review_stamp=100.2,stop_stamp=100.2)
        return self.m.finalize_dryrun(e,100.4)
    def bad(self,e,msg):
        with self.assertRaisesRegex(HandoffError,msg):self.m.prepare(e,self.now)
    def test_happy_dryrun_never_actuates(self):
        x=self.full();self.assertEqual(x['status'],'DRYRUN_READY_NOT_AUTHORITY');self.assertEqual(x['proposed_epoch'],5);self.assertEqual(x['actions_performed'],[])
    def test_no_stop(self):self.bad(replace(self.e,independent_stop_verified=False),'NO_INDEPENDENT_STOP')
    def test_wrong_review(self):self.bad(replace(self.e,review_status='GLOBAL_LOCKED'),'REVIEW_NOT_ELIGIBLE')
    def test_stale_camera(self):self.bad(replace(self.e,visual_stamp=92),'STALE_OR_FUTURE_SENSOR_EVIDENCE')
    def test_future_stop(self):self.bad(replace(self.e,stop_stamp=105),'STALE_OR_FUTURE_STOP_PROOF')
    def test_nan_pose(self):self.bad(replace(self.e,candidate_xyz=(float('nan'),0.,0.)),'NONFINITE_EVIDENCE')
    def test_no_map_version(self):self.bad(replace(self.e,map_version='  '),'MAP_VERSION_FRAME_OR_SESSION_INVALID')
    def test_too_few_confirmations(self):self.bad(replace(self.e,lidar_visual_confirmations=2),'INSUFFICIENT_LIDAR_VISUAL_CONSENSUS')
    def test_wrong_branch(self):self.bad(replace(self.e,median_lidar_visual_xy_error_m=12.),'INSUFFICIENT_LIDAR_VISUAL_CONSENSUS')
    def test_active_cannot_double_prepare(self):
        self.prepare()
        with self.assertRaisesRegex(HandoffError,'ACTIVE_TRANSACTION_EXISTS'):self.prepare()
    def test_old_epoch_ack_rejected(self):
        self.prepare();t=self.m.transaction
        with self.assertRaisesRegex(HandoffError,'ACK_FOR_DIFFERENT'):
            self.m.acknowledge(Acknowledgment('planner','STOPPED',100.1,4,t.fingerprint),100.15)
    def test_wrong_candidate_ack_rejected(self):
        self.prepare();t=self.m.transaction
        with self.assertRaisesRegex(HandoffError,'ACK_FOR_DIFFERENT'):
            self.m.acknowledge(Acknowledgment('brake','ENGAGED',100.1,5,'other'),100.15)
    def test_stale_ack_rejected(self):
        self.prepare();t=self.m.transaction
        with self.assertRaisesRegex(HandoffError,'STALE_OR_FUTURE_ACK'):
            self.m.acknowledge(Acknowledgment('planner','STOPPED',90,5,t.fingerprint),100)
    def test_replayed_ack_rejected(self):
        self.prepare();self.ack('planner')
        with self.assertRaisesRegex(HandoffError,'REPLAYED_ACKNOWLEDGMENT'):self.ack('planner')
    def test_unsigned_operator_approval_missing(self):
        self.prepare();t=self.m.transaction
        with self.assertRaisesRegex(HandoffError,'MISSING_EXPLICIT_OPERATOR_APPROVAL'):
            self.m.approve('operator',t.transaction_id,t.fingerprint,100.3)
    def test_wrong_token_approval_rejected(self):
        self.prepare();t=self.m.transaction
        with self.assertRaisesRegex(HandoffError,'APPROVAL_FOR_DIFFERENT'):
            self.m.approve('operator','wrong',t.fingerprint,100.3,explicitly_confirmed=True)
    def test_cannot_commit_without_acks(self):
        self.prepare();t=self.m.transaction
        self.m.approve('operator',t.transaction_id,t.fingerprint,100.2,explicitly_confirmed=True)
        with self.assertRaisesRegex(HandoffError,'MISSING_ACK'):
            self.m.finalize_dryrun(replace(self.e,candidate_stamp=100.2,visual_stamp=100.2,review_stamp=100.2,stop_stamp=100.2),100.3)
    def test_map_changed_aborts(self):
        self.prepare()
        with self.assertRaisesRegex(HandoffError,'EPOCH_MAP_OR_SESSION_CHANGED'):
            self.m.refresh(replace(self.e,map_version='different'),100.1)
        self.assertEqual(self.m.transaction.phase,Phase.ABORTED)
    def test_large_candidate_jump_aborts(self):
        self.prepare()
        with self.assertRaisesRegex(HandoffError,'CANDIDATE_CHANGED_TOO_FAR'):
            self.m.refresh(replace(self.e,candidate_xyz=(38.,-6.,1.5)),100.1)
        self.assertEqual(self.m.transaction.phase,Phase.ABORTED)
    def test_clock_regression_aborts(self):
        self.prepare()
        with self.assertRaisesRegex(HandoffError,'CLOCK_REGRESSION'):self.m.refresh(self.e,99.9)
    def test_new_session_aborts(self):
        self.prepare()
        with self.assertRaisesRegex(HandoffError,'EPOCH_MAP_OR_SESSION_CHANGED'):
            self.m.refresh(replace(self.e,current_session='new-session'),100.1)
    def test_lost_stop_proof_aborts(self):
        self.prepare()
        with self.assertRaisesRegex(HandoffError,'NO_INDEPENDENT_STOP'):
            self.m.refresh(replace(self.e,independent_stop_verified=False),100.1)
        self.assertEqual(self.m.transaction.phase,Phase.ABORTED)
    def test_ack_expiry_aborts(self):
        self.prepare()
        for c in EXPECTED_ACKS:self.ack(c)
        t=self.m.transaction;self.m.approve('operator',t.transaction_id,t.fingerprint,100.3,explicitly_confirmed=True)
        with self.assertRaisesRegex(HandoffError,'ACK_EXPIRED'):
            self.m.finalize_dryrun(replace(self.e,candidate_stamp=103.0,visual_stamp=103.0,review_stamp=103.0,stop_stamp=103.0),103.1)
    def test_one_time_transaction_cannot_reuse(self):
        x=self.full()
        with self.assertRaisesRegex(HandoffError,'NO_ACTIVE_TRANSACTION'):
            self.m.approve('operator',x['transaction_id'],x['candidate_fingerprint'],100.5,explicitly_confirmed=True)
    def test_bad_tf_ack_not_accepted(self):
        self.prepare();t=self.m.transaction
        with self.assertRaisesRegex(HandoffError,'UNEXPECTED_COMPONENT_OR_STATE'):
            self.m.acknowledge(Acknowledgment('tf_consumers','ALREADY_RESET',100.1,5,t.fingerprint),100.2)
    def test_only_map_frame(self):self.bad(replace(self.e,map_frame='odom'),'MAP_VERSION_FRAME_OR_SESSION_INVALID')
    def test_previous_epoch_must_advance(self):
        self.full()
        with self.assertRaisesRegex(HandoffError,'PREVIOUS_EPOCH_NOT_ADVANCED'):
            self.m.prepare(self.e,100.6)
    def test_proposed_epoch_can_progress(self):
        self.full()
        # This only permits the new *dry-run* proposal, it does not claim the
        # cart executed a TF reset. A live system needs persistent epoch proof.
        later=replace(self.e,current_epoch=5,candidate_stamp=100.4,
                      visual_stamp=100.4,review_stamp=100.4,stop_stamp=100.4)
        result=self.m.prepare(later,100.5)
        self.assertEqual(result['proposed_epoch'],6)

if __name__=='__main__':unittest.main(verbosity=2)
