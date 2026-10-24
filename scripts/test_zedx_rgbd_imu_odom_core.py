#!/usr/bin/env python3
"""Synthetic metric geometry and fail-closed behavior tests, not field accuracy."""
import sys,unittest
import numpy as np,cv2
from zedx_rgbd_imu_odom_core import ZedRGBDIMUOdometry,project_rigid,rotation_compensated_parallax_deg

class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.K=np.array([[300.,0,159.],[0,300.,119.],[0,0,1.]])
    def test_pnp_metric_transform(self):
        rng=np.random.RandomState(4);p=rng.uniform([-1,-.8,3],[1,.8,7],(250,3))
        R,_=cv2.Rodrigues(np.array([.01,-.028,.024]));t=np.array([.08,-.02,.12])
        q=p@R.T+t
        uv=q[:,:2]/q[:,2,None]*np.array([300.,300.])+[159.,119.]
        T,idx,err=project_rigid(p,uv,self.K)
        self.assertGreater(len(idx),240)
        np.testing.assert_allclose(T[:3,:3],R,atol=1e-5)
        np.testing.assert_allclose(T[:3,3],t,atol=1e-5)
        self.assertLess(err,1e-3)
    def test_insufficient_pnp_points(self):
        T,ids,e=project_rigid(np.zeros((6,3)),np.zeros((6,2)),self.K)
        self.assertIsNone(T)
    def test_gyro_integration(self):
        v=ZedRGBDIMUOdometry(self.K)
        for i in range(11):v.add_imu(i*.01,[0.,0.,.1])
        w,coverage=v.gyro_increment(0.,.10)
        np.testing.assert_allclose(w,[0,0,.01],atol=1e-6)
        self.assertAlmostEqual(coverage,1.)
    def test_out_of_order_imu_rejected(self):
        v=ZedRGBDIMUOdometry(self.K)
        v.add_imu(1.,[0,0,0]);v.add_imu(.9,[1,0,0]);self.assertEqual(len(v.gyro),1)
    def test_imu_nonfinite_rejected(self):
        v=ZedRGBDIMUOdometry(self.K)
        v.add_imu(1.,[float('nan'),0,0]);self.assertEqual(len(v.gyro),0)
    def test_invalid_rotation_input(self):
        with self.assertRaises(ValueError):ZedRGBDIMUOdometry(self.K,R_opt_imu=np.zeros((3,3)))
    def test_missing_depth_cannot_generate_motion(self):
        v=ZedRGBDIMUOdometry(self.K);frame=np.zeros((240,320,3),np.uint8)
        d=np.full((240,320),float('nan'),np.float32)
        v.process(0,frame,d)
        x=v.process(.15,frame,d)
        self.assertEqual(x.state,'UNTRUSTED_VISUAL')
        self.assertEqual(v.accepted,0)
        np.testing.assert_allclose(x.transform,np.eye(4))
    def test_camera_gap_quarantined(self):
        v=ZedRGBDIMUOdometry(self.K);f=np.zeros((240,320,3),np.uint8)
        d=np.ones((240,320),np.float32)*5
        v.process(0,f,d);x=v.process(2.1,f,d)
        self.assertEqual(x.state,'UNTRUSTED_IMAGE_GAP')
        self.assertEqual(v.accepted,0)
    def test_bad_image_dimensions(self):
        v=ZedRGBDIMUOdometry(self.K)
        with self.assertRaises(ValueError):v.process(0,np.zeros((120,120,3),np.uint8),np.ones((50,50),np.float32))
    def test_pure_rotation_has_negligible_derotated_parallax(self):
        uv=np.array([[110.,90.],[180.,90.],[110.,160.],[180.,160.]])
        R,_=cv2.Rodrigues(np.array([0.,0.,.06]))
        x=np.column_stack(((uv[:,0]-self.K[0,2])/300.,
                           (uv[:,1]-self.K[1,2])/300.,np.ones(len(uv))))
        v=(R@x.T).T
        q=v[:,:2]/v[:,2,None]*300.+[self.K[0,2],self.K[1,2]]
        self.assertLess(rotation_compensated_parallax_deg(uv,q,self.K,R),.001)
    def test_translation_generates_parallax(self):
        uv=np.array([[110.,90.],[180.,90.],[110.,160.],[180.,160.]])
        shifted=uv+np.array([8.,0.])
        self.assertGreater(rotation_compensated_parallax_deg(uv,shifted,self.K,np.eye(3)),.5)
    def test_nonfinite_parallax_input_is_unknown(self):
        self.assertTrue(np.isnan(rotation_compensated_parallax_deg(
            [[float('nan'),80]],[[10,80]],self.K,np.eye(3))))
    def test_no_fake_acceleration_between_rejections(self):
        v=ZedRGBDIMUOdometry(self.K);rng=np.random.RandomState(0)
        f=rng.randint(0,255,(240,320,3),dtype=np.uint8);d=np.zeros((240,320),np.float32)
        v.process(0,f,d)
        for i in range(1,5):
            q=v.process(i*.14,f,d)
            np.testing.assert_allclose(q.transform,np.eye(4))
        self.assertEqual(v.accepted,0)

if __name__=='__main__':unittest.main(verbosity=2)
