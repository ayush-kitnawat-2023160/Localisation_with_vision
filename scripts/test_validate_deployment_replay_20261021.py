#!/usr/bin/env python3
"""The offline runner must never report rosbag completion as valid evidence."""
import json
import tempfile
import unittest
from pathlib import Path

from validate_deployment_replay_20261021 import check


class ReplayGates(unittest.TestCase):
    def setUp(self):
        self.dir=tempfile.TemporaryDirectory()
        self.path=Path(self.dir.name)/'monitor.json'
        self.d={
          'received':{'fast_odom':100,'klt_odom':120,'step3_status':4,
                      'health_status':90,'klt_continuation_status':5,
                      'xfeat_global':10,'offline_fault_events':1,
                      'klt_continuation':30},
          'status_counts':{'health_STOP_REQUIRED_REARM_LATCHED':7}}
    def tearDown(self):self.dir.cleanup()
    def run_check(self,visual=False,fault=False):
        self.path.write_text(json.dumps(self.d))
        return check(self.path,visual,fault)
    def test_valid_normal(self):self.assertTrue(self.run_check()['passed'])
    def test_valid_full_visual(self):self.assertTrue(self.run_check(True)['passed'])
    def test_valid_fault_candidate(self):self.assertTrue(self.run_check(False,True)['passed'])
    def test_pose_anytype_fail(self):
        self.d['received']['forbidden_pose']=1
        self.assertFalse(self.run_check()['passed'])
    def test_normal_fast_topic_leak_fails(self):
        self.d['received']['forbidden_canonical_fast']=1
        self.assertFalse(self.run_check()['passed'])
    def test_tf_leak_fails(self):
        self.d['received']['forbidden_canonical_tf']=1
        self.assertFalse(self.run_check()['passed'])
    def test_step3_anchor_leak_fails(self):
        self.d['received']['forbidden_step3_lock']=1
        self.assertFalse(self.run_check()['passed'])
    def test_cmd_vel_fail(self):
        self.d['received']['forbidden_cmd_vel']=1
        self.assertFalse(self.run_check()['passed'])
    def test_legacy_fused_fail(self):
        self.d['received']['unexpected_legacy_fused']=1
        self.assertFalse(self.run_check()['passed'])
    def test_initialpose_fail(self):
        self.d['received']['forbidden_initialpose']=1
        self.assertFalse(self.run_check()['passed'])
    def test_missing_estimator_fail(self):
        self.d['received']['fast_odom']=0
        self.assertFalse(self.run_check()['passed'])
    def test_xfeat_required_if_visual(self):
        self.d['received']['xfeat_global']=0
        self.assertFalse(self.run_check(True)['passed'])
        self.assertTrue(self.run_check(False)['passed'])
    def test_outage_continuation_required_if_fault(self):
        self.d['received']['klt_continuation']=0
        self.assertFalse(self.run_check(False,True)['passed'])
    def test_stop_latch_required_if_fault(self):
        self.d['status_counts']={}
        self.assertFalse(self.run_check(False,True)['passed'])
    def test_missing_monitor_fail(self):
        self.assertFalse(check(self.path)['passed'])


if __name__=='__main__':unittest.main(verbosity=2)
