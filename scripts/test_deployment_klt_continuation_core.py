#!/usr/bin/env python3
"""Pure, deterministic LiDAR outage shadow candidate checks."""
import unittest
import numpy as np
from deployment_klt_continuation_core import CandidateContinuation


def pose(x=0.,y=0.,z=0.):
    a=np.eye(4);a[:3,3]=[x,y,z];return a


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.c=CandidateContinuation(pair_min_count=5,pair_duration_min=.25,
             max_lidar_age=.75,seed_age_max=.98)
    def seed(self):
        self.c.monitor_health(True)
        for i in range(12):
            t=100.+i*.10
            self.c.observe_lidar(t)
            self.c.camera_status(True,t)
            self.c.add_visual(t,pose(x=.5+i*.03))
            self.c.add_fast(t+.02,pose(x=1.5+i*.03))
            self.c.tick(t+.02)
        self.assertIsNotNone(self.c.anchor)
        self.assertAlmostEqual(self.c.anchor[0,3],1.,places=4)
        return self.c.last_lidar

    def test_empty_no_output(self):
        self.assertIsNone(self.c.add_visual(100,pose()))
        self.assertEqual(self.c.tick(101),'WARMUP_NO_SEED')
    def test_seed_from_independent_poses(self):
        self.seed()
        self.assertEqual(self.c.state,'WARMUP_TIME_ALIGNED_SEED_CANDIDATE')
    def test_no_seed_without_health_supervisor(self):
        for i in range(10):
            t=100+i*.1
            self.c.observe_lidar(t)
            self.c.camera_status(True,t)
            self.c.add_visual(t,pose(x=t*.1))
            self.c.add_fast(t,pose(x=2+t*.1))
        self.assertIsNone(self.c.anchor)
    def test_no_seed_without_camera_status(self):
        self.c.monitor_health(True)
        for i in range(10):
            t=100+i*.1
            self.c.observe_lidar(t)
            self.c.add_visual(t,pose())
            self.c.add_fast(t,pose())
        self.assertIsNone(self.c.anchor)
    def test_no_seed_skewed_clock(self):
        self.c.monitor_health(True)
        for i in range(10):
            t=100+i*.5
            self.c.observe_lidar(t)
            self.c.camera_status(True,t)
            self.c.add_visual(t,pose())
            self.c.add_fast(t+.25,pose())
        self.assertIsNone(self.c.anchor)
    def test_duplicate_pose_rejected(self):
        self.c.monitor_health(True)
        self.assertTrue(self.c.add_fast(100,pose()))
        self.assertFalse(self.c.add_fast(100,pose()))
    def test_invalid_pose_rejected(self):
        x=pose();x[0,3]=float('nan')
        self.assertFalse(self.c.add_fast(100,x))
        x=pose();x[:3,:3]=np.eye(3)*2
        self.assertIsNone(self.c.add_visual(100,x))
    def test_mismatched_motion_does_not_seed(self):
        self.c.monitor_health(True)
        for i in range(10):
            t=100+i*.1
            self.c.observe_lidar(t)
            self.c.camera_status(True,t)
            self.c.add_visual(t,pose())
            self.c.add_fast(t+.01,pose(x=i*2.))
        self.assertIsNone(self.c.anchor)
    def test_outage_continuation_no_phantom_map_reset(self):
        t=self.seed()
        self.c.camera_status(True,t+.80)
        self.c.add_visual(t+.80,pose(x=.95))
        self.assertEqual(self.c.tick(t+.85),'OUTAGE_SEEDED_CANDIDATE_ONLY')
        self.c.camera_status(True,t+.91)
        result=self.c.add_visual(t+.91,pose(x=1.0))
        self.assertIsNotNone(result)
        self.assertEqual(self.c.state,'CAMERA_CONTINUATION_CANDIDATE_ONLY')
        self.assertAlmostEqual(result.transform[0,3],2.,places=4)
        self.assertLess(result.outage_age,1.)
    def test_camera_capture_stamp_lags_clock_no_false_lidar_return(self):
        t=self.seed()
        self.c.camera_status(True,t+.78)
        self.c.add_visual(t+.78,pose(x=.9))
        self.assertEqual(self.c.tick(t+.85),'OUTAGE_SEEDED_CANDIDATE_ONLY')
        # Image was captured 200ms before ROS /clock now. LiDAR did NOT
        # reappear; the camera stamp must not trigger an artificial return.
        self.c.camera_status(True,t+.85)
        candidate=self.c.add_visual(t+.86,pose(x=1.))
        self.assertIsNotNone(candidate)
        self.assertFalse(self.c.terminal)
    def test_no_cold_start_auto_continuation(self):
        self.c.observe_lidar(100)
        self.c.camera_status(True,100)
        self.c.monitor_health(True)
        self.c.tick(100.9)
        self.assertTrue(self.c.terminal)
        self.assertEqual(self.c.state,'NO_FRESH_TIME_ALIGNED_PRE_OUTAGE_SEED')
    def test_stop_on_lidar_return(self):
        t=self.seed()
        self.c.camera_status(True,t+.80)
        self.c.add_visual(t+.80,pose(x=.95))
        self.c.tick(t+.85)
        self.c.observe_lidar(t+.9)
        self.c.tick(t+.91)
        self.assertTrue(self.c.terminal)
        self.assertIn('MANUAL_REJOIN',self.c.state)
    def test_stop_on_camera_failure(self):
        t=self.seed()
        self.c.tick(t+.85)
        self.c.camera_status(False,t+.88)
        self.assertTrue(self.c.terminal)
    def test_stop_on_video_gap(self):
        t=self.seed()
        self.c.tick(t+.85)
        self.c.camera_status(True,t+.90)
        self.c.add_visual(t+.90,pose(x=1.))
        self.c.tick(t+1.80)
        self.assertTrue(self.c.terminal)
    def test_stop_on_implausible_motion(self):
        t=self.seed()
        self.c.tick(t+.85)
        self.c.camera_status(True,t+.90)
        self.c.add_visual(t+.90,pose(x=1.))
        self.c.camera_status(True,t+.99)
        self.assertIsNone(self.c.add_visual(t+.99,pose(x=25.)))
        self.assertTrue(self.c.terminal)
    def test_stop_on_time_rollback(self):
        t=self.seed()
        self.c.tick(t-.8)
        self.assertTrue(self.c.terminal)
        self.assertEqual(self.c.state,'CLOCK_ROLLBACK_REARM_REQUIRED')
    def test_timeout_limits_outage_duration(self):
        t=self.seed()
        self.c.tick(t+.85)
        self.c.tick(t+20.)
        self.assertTrue(self.c.terminal)
    def test_repeated_outage_cannot_auto_rearm(self):
        t=self.seed()
        self.c.tick(t+.85)
        self.c.observe_lidar(t+.91)
        self.c.tick(t+.92)
        self.assertTrue(self.c.terminal)
        self.c.tick(t+2.2)
        self.assertFalse(self.c.active)


if __name__=='__main__':unittest.main(verbosity=2)
