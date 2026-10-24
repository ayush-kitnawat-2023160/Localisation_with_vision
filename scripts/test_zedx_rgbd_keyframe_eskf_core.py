#!/usr/bin/env python3
"""Deterministic ESKF math/clone health tests; NO vehicle integration."""
import unittest
import numpy as np
from zedx_rgbd_keyframe_eskf_core import CloneESKF,pose,exp_so3,log_so3,ZedKeyframeESKFOdometry


class TestCloneESKF(unittest.TestCase):
    def test_stationary_imu_bootstrap_and_propagation(self):
        k=CloneESKF(np.eye(3))
        for i in range(170):k.add_imu(i*.005,[.0002,-.0001,0.],[0.,0.,9.81])
        self.assertTrue(k.initialized)
        self.assertLess(np.linalg.norm(k.p),.02)
        self.assertLess(np.linalg.norm(k.v),.05)
        self.assertLess(np.linalg.norm(k.R-np.eye(3)),.01)
        self.assertTrue(k.covariance_ok())
        self.assertGreater(k.n_imuprop,0)
    def test_clone_covariance_crosscorrelation_not_reset(self):
        k=CloneESKF(np.eye(3))
        k.initialized=True
        k.p=np.array([1.,2.,3.]);k.R=exp_so3([.03,-.02,.01]);k.P=np.eye(21)*.1
        k.clone_current_pose()
        self.assertTrue(np.allclose(k.P[:3,15:18],np.eye(3)*.1))
        self.assertTrue(np.allclose(k.P[6:9,18:21],np.eye(3)*.1))
        self.assertTrue(np.allclose(k.pk,k.p))
        self.assertTrue(k.covariance_ok())
    def test_relative_visual_update_reduces_error(self):
        k=CloneESKF(np.eye(3))
        k.initialized=True
        k.P=np.eye(21)*.2
        k.clone_current_pose()
        k.p=np.array([.25,-.18,.07]);k.R=exp_so3([.012,-.006,.002])
        # Visual PnP says -0.22m old<-current along x with small rotation.
        R,t=k.relative_prediction()
        measured=pose(R@exp_so3([.009,0,0]),t+np.array([.012,.003,-.005]))
        olddiff=np.linalg.norm(measured[:3,3]-t)
        ok,nis=k.visual_update(measured,100,.5,1.5)
        self.assertTrue(ok)
        self.assertLess(nis,22.46)
        pred2=k.relative_prediction()[1]
        self.assertLess(np.linalg.norm(measured[:3,3]-pred2),olddiff)
        self.assertTrue(k.covariance_ok())
    def test_visual_outlier_rejected_without_corrupting_covariance(self):
        k=CloneESKF(np.eye(3));k.initialized=True
        k.clone_current_pose();old=k.p.copy();oldP=k.P.copy()
        ok,nis=k.visual_update(pose(t=[15,0,0]),100,0.3,1.2)
        self.assertFalse(ok);self.assertGreater(nis,22.46)
        self.assertTrue(np.allclose(k.p,old))
        self.assertTrue(np.allclose(k.P,oldP))
    def test_missing_initial_still_untrusted(self):
        K=np.array([[240.,0,320.],[0,240.,200.],[0,0,1.]])
        k=ZedKeyframeESKFOdometry(K,np.eye(3))
        result=k.process(0.,np.zeros((400,640,3),np.uint8),np.ones((400,640),np.float32))
        self.assertEqual(result.state,'UNTRUSTED_INERTIAL_BOOTSTRAP')
    def test_imu_gap_fail_closed(self):
        k=CloneESKF(np.eye(3))
        for i in range(150):k.add_imu(i*.005,[0,0,0],[0,0,9.81])
        self.assertTrue(k.initialized)
        self.assertFalse(k.add_imu(2.,[0,0,0],[0,0,9.81]))
        self.assertTrue(k.failed_gap)


if __name__=='__main__':unittest.main(verbosity=2)
