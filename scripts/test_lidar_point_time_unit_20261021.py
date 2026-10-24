#!/usr/bin/env python3
"""ROS Noetic offline test: explicit VLP-16 point-time unit is not guessed.

Does NOT connect to a ROS master; constructs tiny PointCloud2 messages and
invokes adapter callback with a captured publisher.
"""
import unittest
from unittest.mock import patch
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField
from fast_livo2_sensor_adapter import SensorAdapter


class Capture:
    def __init__(self):self.events=[]
    def publish(self,m):self.events.append(m)


def cloud(values):
    a=np.zeros(len(values),dtype=[('x','<f4'),('y','<f4'),
                                  ('z','<f4'),('time','<f4'),('ring','<u2'),('pad','<u2')])
    a['time']=values
    m=PointCloud2()
    m.header.stamp=rospy.Time.from_sec(100.)
    m.header.frame_id='source_lidar'
    m.height=1;m.width=len(values)
    m.is_bigendian=False;m.is_dense=True
    m.fields=[PointField(name='time',offset=12,datatype=PointField.FLOAT32,count=1),
              PointField(name='ring',offset=16,datatype=PointField.UINT16,count=1)]
    m.point_step=a.dtype.itemsize;m.row_step=len(values)*m.point_step
    m.data=a.tobytes()
    return m


def times(msg):
    return np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.width,msg.point_step)[:,12:16].copy().view('<f4').ravel()


class Tests(unittest.TestCase):
    def adapter(self,unit):
        a=SensorAdapter.__new__(SensorAdapter)
        a.lidar_time_unit=unit
        a.lidar_time_scale=1e6
        a.lidar_frame='velodyne'
        a.last_lidar_stamp=None
        a.lidar_pub=Capture()
        return a

    def test_seconds_shift_to_scan_start_then_microseconds(self):
        a=self.adapter('seconds')
        a.lidar_cb(cloud([-.10,-.05,0]))
        self.assertEqual(len(a.lidar_pub.events),1)
        out=a.lidar_pub.events[0]
        self.assertAlmostEqual(out.header.stamp.to_sec(),99.9,places=4)
        self.assertTrue(np.allclose(times(out),[0,50000,100000],atol=.04))
        self.assertEqual(out.header.frame_id,'velodyne')

    def test_microseconds_already_correct_not_scaled_twice(self):
        a=self.adapter('microseconds')
        a.lidar_cb(cloud([0,50000,100000]))
        out=a.lidar_pub.events[0]
        self.assertEqual(out.header.stamp.to_sec(),100.)
        self.assertTrue(np.allclose(times(out),[0,50000,100000]))

    def test_microseconds_negative_offsets_rebase_header(self):
        a=self.adapter('microseconds')
        a.lidar_cb(cloud([-100000,-50000,0]))
        out=a.lidar_pub.events[0]
        self.assertAlmostEqual(out.header.stamp.to_sec(),99.9,places=4)
        self.assertTrue(np.allclose(times(out),[0,50000,100000],atol=.1))

    def test_mismatched_declared_units_fail_closed(self):
        a=self.adapter('seconds')
        with patch('fast_livo2_sensor_adapter.rospy.logerr_throttle'):
            a.lidar_cb(cloud([0,50000,100000]))
        self.assertEqual(len(a.lidar_pub.events),0)

    def test_milliseconds_misdeclared_as_seconds_fail_closed(self):
        a=self.adapter('seconds')
        with patch('fast_livo2_sensor_adapter.rospy.logerr_throttle'):
            a.lidar_cb(cloud([0,50,100]))
        self.assertEqual(len(a.lidar_pub.events),0)


if __name__=='__main__':unittest.main(verbosity=2)
