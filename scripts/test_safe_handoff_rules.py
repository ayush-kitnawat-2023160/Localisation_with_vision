#!/usr/bin/env python3
import unittest
from safe_handoff_rules import PoseSample,evaluate_review

class TestHandoffReview(unittest.TestCase):
    def setUp(self):
        self.t=110.
        self.odom=[PoseSample(107+i*.1,(1.,2.,0.),0.,0.,0.) for i in range(31)]
        self.lidar=[PoseSample(107.5+i,(12.,8.,0.)) for i in range(3)]
        self.visual=[PoseSample(107.6+i,(12.1,8.1,0.)) for i in range(3)]
        self.status='RELOCALIZING: MH_HANDOFF_REQUIRED'
    def test_stationary_matching_manual_review_only(self):
        ok,why,_=evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t,
                                 externally_verified_stop=True)
        self.assertTrue(ok,why);self.assertIn('MANUAL_REVIEW',why)
    def test_no_independent_stop_interlock(self):
        self.assertEqual(evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t)[1],
                         'EXTERNAL_STOP_INTERLOCK_NOT_VERIFIED')
    def test_no_implicit_authority(self):
        self.assertFalse(evaluate_review('GLOBAL_LOCKED',self.lidar,self.visual,self.odom,self.t)[0])
    def test_stale_candidate(self):
        self.assertEqual(evaluate_review(self.status,self.lidar,self.visual,self.odom,115)[1],'STALE_LIDAR_RECOVERY_CANDIDATE')
    def test_wrong_branch(self):
        self.visual=[PoseSample(107.6+i,(40.,25.,0.)) for i in range(3)]
        self.assertEqual(evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t)[1],'VISUAL_LIDAR_BRANCH_DISAGREEMENT')
    def test_duplicate_single_lidar_cannot_confirm(self):
        self.assertEqual(evaluate_review(self.status,self.lidar[-1:],self.visual,self.odom,self.t)[1],
                         'INSUFFICIENT_INDEPENDENT_LIDAR_VISUAL_CONFIRMATIONS')
    def test_motion_blocks(self):
        self.odom=[PoseSample(107+i*.1,(i*.07,2.,0.),0.,.7,0.) for i in range(31)]
        self.assertIn('ROBOT_NOT_STATIONARY',evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t)[1])
    def test_angular_motion_blocks(self):
        self.odom=[PoseSample(107+i*.1,(1.,2.,0.),i*.02,0.,.2) for i in range(31)]
        self.assertIn('ROBOT_NOT_STATIONARY',evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t)[1])
    def test_lidar_pose_inconsistency(self):
        self.lidar[1]=PoseSample(108.5,(14.,10.,0.))
        self.assertEqual(evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t)[1],
                         'VISUAL_LIDAR_BRANCH_DISAGREEMENT')
    def test_odom_gap_blocks(self):
        self.odom=self.odom[:5]+self.odom[15:]
        self.assertEqual(evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t)[1],'GAPPED_OR_NONMONOTONIC_ODOMETRY')
    def test_nan_position_blocks(self):
        self.visual[-1]=PoseSample(109.6,(float('nan'),8.,0.))
        self.assertEqual(evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t)[1],
                         'INVALID_POSE_OR_MOTION_SAMPLE')
    def test_nan_twist_blocks(self):
        self.odom[-1]=PoseSample(110.,(1.,2.,0.),0.,float('nan'),0.)
        self.assertEqual(evaluate_review(self.status,self.lidar,self.visual,self.odom,self.t)[1],
                         'INVALID_POSE_OR_MOTION_SAMPLE')

if __name__=='__main__':unittest.main(verbosity=2)
