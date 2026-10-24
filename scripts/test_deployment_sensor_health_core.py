#!/usr/bin/env python3
"""Deterministic tests for shadow-only, never-autonomous health decisions."""
import math
import unittest
from deployment_sensor_health_core import Health


class TestFailClosedHealth(unittest.TestCase):
    def setUp(self):
        self.h=Health()
        self.clock=100.
        self.recv=500.

    def add(self,key,t=None,r=None):
        return self.h.add(key,self.clock if t is None else t,
                          self.recv if r is None else r,self.clock)

    def nominal(self):
        for key in self.h.max_age:self.add(key)
        self.h.set_step3_status('GLOBAL_LOCKED')
        self.h.set_visual_odom_status('state=TRACKING_VISION_GYRO')

    def test_no_messages_stop(self):
        self.assertEqual(self.h.evaluate(self.clock,self.recv).mode,
                         'STOP_REQUIRED_NO_TRUSTED_LOCALIZATION')
    def test_nominal_shadow_only(self):
        self.nominal()
        s=self.h.evaluate(self.clock,self.recv)
        self.assertEqual(s.mode,'SHADOW_PRIMARY_LIDAR_GLOBAL')
        self.assertTrue(s.lidar_global)
        self.assertFalse(s.cross_sensor_visual_agreement)
        self.assertFalse(s.authority_enabled)
        self.assertFalse(s.autonomous_motion_authorized)
        self.assertFalse(s.may_auto_reset_map_anchor)
    def test_visual_candidate_not_independently_verified(self):
        self.nominal()
        a=self.h.evaluate(100,500)
        self.assertTrue(a.visual_global)
        self.assertFalse(a.cross_sensor_visual_agreement)
    def test_cross_sensor_confirmation_only_on_fresh_evidence(self):
        self.nominal()
        self.h.confirm_visual_lidar_agreement(100,500)
        a=self.h.evaluate(100,500)
        self.assertEqual(a.mode,'SHADOW_ALL_MODALITIES_CROSSCHECKED')
        self.assertTrue(a.cross_sensor_visual_agreement)
    def test_visual_crossmatch_expires_without_a_new_pair(self):
        self.nominal()
        self.h.confirm_visual_lidar_agreement(100,500)
        self.clock=103.;self.recv=503.
        for key in self.h.max_age:self.add(key)
        snap=self.h.evaluate(103,503)
        self.assertFalse(snap.cross_sensor_visual_agreement)
    def test_visual_crossmatch_cannot_be_replayed(self):
        self.nominal()
        self.assertTrue(self.h.confirm_visual_lidar_agreement(100.,500.))
        self.assertFalse(self.h.confirm_visual_lidar_agreement(99.9,501.))
    def test_new_visual_disagreement_invalidates_earlier_consensus(self):
        self.nominal()
        self.assertTrue(self.h.confirm_visual_lidar_agreement(100.,500.))
        self.assertTrue(self.h.evaluate(100,500).cross_sensor_visual_agreement)
        self.h.invalidate_visual_lidar_agreement()
        self.assertFalse(self.h.evaluate(100,500).cross_sensor_visual_agreement)
    def test_visual_match_missing_lidar_only(self):
        self.nominal();del self.h._data['visual_map_pose']
        self.assertEqual(self.h.evaluate(100,500).mode,'SHADOW_PRIMARY_LIDAR_GLOBAL')
    def test_lidar_drop_latches_stop_not_auto_switch(self):
        self.nominal();self.h.evaluate(100,500)
        self.clock=100.8;self.recv=500.8
        for key in self.h.max_age:
            if key!='lidar':self.add(key)
        result=self.h.evaluate(self.clock,self.recv)
        self.assertTrue(result.continuity_latched)
        self.assertEqual(result.mode,'STOP_REQUIRED_REARM_LATCHED')
        self.assertEqual(result.local_source,'KLT_RGBD_ODOM_CANDIDATE_ONLY')
        self.assertFalse(result.authority_enabled)
    def test_lidar_recovery_not_autorearmed(self):
        self.nominal();self.h.evaluate(100,500)
        self.h.evaluate(101.,501.)
        self.clock=101.1;self.recv=501.1
        for key in self.h.max_age:self.add(key)
        self.h.set_step3_status('GLOBAL_LOCKED')
        self.assertTrue(self.h.evaluate(101.1,501.1).continuity_latched)
    def test_camera_blackout(self):
        self.nominal();self.h.evaluate(100,500)
        self.assertFalse(self.h.evaluate(101,501).visual_odom)
    def test_missing_depth_blocks_visual(self):
        self.nominal();del self.h._data['depth']
        self.assertFalse(self.h.evaluate(100,500).visual_odom)
    def test_rgb_depth_offset_blocks_visual(self):
        self.nominal();self.h._data['depth'].stamp=99.8
        self.assertFalse(self.h.evaluate(100,500).visual_odom)
    def test_missing_zed_imu_blocks_visual(self):
        self.nominal();del self.h._data['zed_imu']
        self.assertFalse(self.h.evaluate(100,500).visual_odom)
    def test_missing_fast_imu_blocks_fast(self):
        self.nominal();del self.h._data['fast_imu']
        self.assertFalse(self.h.evaluate(100,500).lidar_global)
    def test_future_stamp_rejected(self):
        self.assertFalse(self.h.add('lidar',100.5,500.,100.))
    def test_negative_stamp_rejected(self):
        self.assertFalse(self.h.add('lidar',-1,500,100))
    def test_nan_stamp_rejected(self):
        self.assertFalse(self.h.add('rgb',float('nan'),500,100))
    def test_duplicate_source_stamp_rejected(self):
        self.assertTrue(self.add('lidar'))
        self.assertFalse(self.add('lidar'))
    def test_backward_receipt_rejected(self):
        self.assertTrue(self.add('lidar'))
        self.assertFalse(self.h.add('lidar',100.1,499.9,100.1))
    def test_stale_receipt_rejected_even_if_timestamp_fresh(self):
        self.nominal();self.h._data['lidar'].received=490.
        self.assertFalse(self.h.evaluate(100,500).lidar_global)
    def test_clock_rollback_latched(self):
        self.nominal();self.h.evaluate(100,500)
        self.assertFalse(self.h.add('lidar',98.,501.,98.))
        self.assertTrue(self.h.evaluate(98.,501.).continuity_latched)
    def test_untrusted_step3_status_blocks_lidar_pose(self):
        self.nominal();self.h.set_step3_status('RELOCALIZING: MH_HANDOFF_REQUIRED')
        self.assertFalse(self.h.evaluate(100,500).lidar_global)
    def test_step3_trust_loss_latched(self):
        self.nominal();self.h.evaluate(100,500)
        self.h.set_step3_status('SEARCHING')
        self.assertTrue(self.h.evaluate(100,500).continuity_latched)
    def test_stale_source_local_odom_blocks_global(self):
        self.nominal();del self.h._data['fast_odom']
        self.assertFalse(self.h.evaluate(100,500).lidar_global)
    def test_visual_odom_must_have_explicit_tracking_status(self):
        self.nominal()
        self.h.set_visual_odom_status('state=UNTRUSTED_GYRO_UNAVAILABLE')
        self.assertFalse(self.h.evaluate(100,500).visual_odom)
    def test_visual_odom_rearm_required_latches(self):
        self.nominal()
        self.h._last_local_source='KLT_RGBD_ODOM_CANDIDATE_ONLY'
        self.h.set_visual_odom_status('state=UNTRUSTED_CAMERA_TIMEOUT_REARM_REQUIRED')
        self.assertTrue(self.h.evaluate(100,500).continuity_latched)
    def test_warmup_visual_candidate_precedes_fast_without_fake_handoff(self):
        for key in ('lidar','fast_imu','rgb','depth','zed_imu','visual_odom'):
            self.add(key)
        self.h.set_visual_odom_status('state=TRACKING_VISION_GYRO')
        warmup=self.h.evaluate(100,500)
        self.assertEqual(warmup.local_source,'KLT_RGBD_ODOM_CANDIDATE_ONLY')
        self.add('fast_odom')
        now=self.h.evaluate(100,500)
        self.assertEqual(now.local_source,'FAST_LIVO2_ODOM')
        self.assertFalse(now.continuity_latched)
    def test_camera_timeout_does_not_invalidate_healthy_primary(self):
        self.nominal()
        self.h.evaluate(100,500)
        self.h.set_visual_odom_status('state=UNTRUSTED_CAMERA_TIMEOUT_REARM_REQUIRED')
        snap=self.h.evaluate(100,500)
        self.assertFalse(snap.continuity_latched)
        self.assertTrue(snap.lidar_global)
    def test_map_visual_alone_cannot_authorize(self):
        self.add('visual_map_pose')
        r=self.h.evaluate(100,500)
        self.assertFalse(r.map_pose_trusted)
        self.assertFalse(r.autonomous_motion_authorized)
    def test_no_ros_time(self):
        self.nominal()
        self.assertEqual(self.h.evaluate(0,500).mode,'STOP_REQUIRED')
    def test_distinct_external_discontinuity_always_latched(self):
        self.nominal()
        self.h.mark_external_discontinuity('BODY_CAMERA_EXTRINSICS_CHANGED')
        self.assertTrue(self.h.evaluate(100,500).continuity_latched)


if __name__=='__main__':unittest.main(verbosity=2)
