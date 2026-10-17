#!/usr/bin/env python3
"""Compare the real ROS message serializer for old vs opt-in adapter.

Run inside Noetic (same interpreter/modules as ROS runtime). No ROS master,
physical hardware, bag playback or changes to production code are required.
"""
import importlib.util
import io
from pathlib import Path
import time
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField

def load(path,label):
    spec=importlib.util.spec_from_file_location(label,str(path))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.SensorAdapter

class StubPublisher:
    def __init__(self):self.last=None
    def publish(self,msg):self.last=msg

def create_adapter(cls):
    x=cls.__new__(cls)
    x.lidar_time_scale=1.0e6
    x.camera_mode='rebase'
    x.max_camera_hz=0.0
    x.last_camera_source_stamp=None
    x.camera_offset=rospy.Duration.from_sec(1.5)
    x.last_lidar_stamp=None
    x.lidar_frame='velodyne'
    x.camera_frame='camera_color_optical_frame'
    x.lidar_pub=StubPublisher()
    x.camera_pub=StubPublisher()
    return x

def wire(msg):
    sink=io.BytesIO()
    msg.serialize(sink)
    return sink.getvalue()

im=Image()
im.header.stamp=rospy.Time.from_sec(1788852570.0)
im.header.frame_id='camera_old'
im.width=1280;im.height=720;im.step=3840;im.encoding='bgr8';im.is_bigendian=0
im.data=bytes(np.random.default_rng(7).integers(0,256,size=im.step*im.height,dtype=np.uint8))

lidar=PointCloud2()
lidar.header.stamp=rospy.Time.from_sec(1788852570.0)
lidar.header.frame_id='lidar_old'
lidar.width=3000;lidar.height=1;lidar.point_step=20
lidar.row_step=lidar.point_step*lidar.width
lidar.is_bigendian=False;lidar.is_dense=True
lidar.fields=[PointField('x',0,PointField.FLOAT32,1),
              PointField('y',4,PointField.FLOAT32,1),
              PointField('z',8,PointField.FLOAT32,1),
              PointField('time',12,PointField.FLOAT32,1),
              PointField('ring',16,PointField.UINT16,1)]
cloud=bytearray(lidar.row_step)
values=np.ndarray((lidar.width,),dtype=np.float32,buffer=cloud,offset=12,strides=(20,))
values[:]=np.linspace(-.099,.001,lidar.width,dtype=np.float32)
lidar.data=bytes(cloud)

old=load('/old.py','baseline_adapter')
new=load('/new.py','shallow_adapter')
for message,callback,publisher in ((im,'camera_cb','camera_pub'),
                                   (lidar,'lidar_cb','lidar_pub')):
    original=wire(message)
    baseline=create_adapter(old)
    candidate=create_adapter(new)
    getattr(baseline,callback)(message)
    baseline_msg=getattr(baseline,publisher).last
    getattr(candidate,callback)(message)
    candidate_msg=getattr(candidate,publisher).last
    assert wire(baseline_msg)==wire(candidate_msg),callback
    assert wire(message)==original,'source message mutated'
    print(f'PASS: {callback} baseline and shallow serialized bytes identical; input unmodified',flush=True)

for cls in (old,new):
    obj=create_adapter(cls)
    for _ in range(5):obj.camera_cb(im);obj.lidar_cb(lidar)

for name,cls in (('deepcopy',old),('shallow',new)):
    obj=create_adapter(cls)
    count=160
    start=time.perf_counter()
    for _ in range(count):obj.camera_cb(im)
    t_img=time.perf_counter()-start
    start=time.perf_counter()
    for _ in range(count):obj.lidar_cb(lidar)
    t_lidar=time.perf_counter()-start
    print(f'{name}: {count} images={t_img:.3f}s, {count} clouds={t_lidar:.3f}s',flush=True)
