#!/usr/bin/env python3
"""Visualize ORB 2D-3D PnP inlier correspondence at ~63.3s in localization_3."""
import argparse,csv,json
from pathlib import Path
import numpy as np,cv2,rosbag,rospy,yaml

a=argparse.ArgumentParser();a.add_argument('--bag',required=True);a.add_argument('--visual-map',required=True);a.add_argument('--calibration',required=True);a.add_argument('--output',required=True);a.add_argument('--bag-camera-offset',type=float,default=63.2983);a.add_argument('--keyframe',type=int,default=196);a.add_argument('--first-camera-stamp',type=float,required=True);z=a.parse_args()
root=Path(z.visual_map);rows=list(csv.DictReader(open(root/'keyframes.csv')));entry=rows[z.keyframe];feature=np.load(root/entry['landmarks']);refimg=cv2.imread(str(root/entry['image']))
target=z.first_camera_stamp+z.bag_camera_offset;best=None;bestdiff=float('inf')
with rosbag.Bag(z.bag) as bag:
    for _,im,_ in bag.read_messages(topics=['/zedx/zed_node/left/image_rect_color'],start_time=rospy.Time.from_sec(target-.1),end_time=rospy.Time.from_sec(target+1.0)):
        err=abs(im.header.stamp.to_sec()-target)
        if err<bestdiff:bestdiff=err;best=im
if best is None or bestdiff>0.18:raise RuntimeError('Cannot locate selected query RGB image in bag; closest error '+str(bestdiff))
a=np.frombuffer(best.data,np.uint8).reshape(best.height,best.step)
if best.encoding=='bgra8':query=cv2.cvtColor(a[:,:best.width*4].reshape(best.height,best.width,4),cv2.COLOR_BGRA2BGR)
elif best.encoding=='bgr8':query=a[:,:best.width*3].reshape(best.height,best.width,3)
else:raise RuntimeError(best.encoding)
orb=cv2.ORB_create(nfeatures=2200,fastThreshold=8);kp,desc=orb.detectAndCompute(cv2.cvtColor(query,cv2.COLOR_BGR2GRAY),None)
matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
raw=matcher.knnMatch(feature['desc'],desc,k=2)
choices=[v[0] for v in raw if len(v)==2 and v[0].distance<.78*v[1].distance and v[0].distance<74]
byquery={}
for m in sorted(choices,key=lambda x:x.distance):
    if m.trainIdx not in byquery:byquery[m.trainIdx]=m
matches=list(byquery.values());obj=np.array([feature['xyz'][m.queryIdx] for m in matches],np.float32);uv=np.array([kp[m.trainIdx].pt for m in matches],np.float32)
c=yaml.safe_load(open(z.calibration))['laserMapping'];fx,fy,cx,cy=[float(c[k]) for k in ('cam_fx','cam_fy','cam_cx','cam_cy')];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],float)
ok,rvec,tvec,I=cv2.solvePnPRansac(obj,uv,K,None,iterationsCount=130,reprojectionError=3.5,confidence=.995,flags=cv2.SOLVEPNP_EPNP)
if not ok or I is None:raise RuntimeError('PnP returned no inliers')
I=I[:,0];kept=[matches[i] for i in I]
# Prefer broad image coverage, not visually overwhelming lines.
kept=kept[::max(1,len(kept)//36)][:40]
refkp=[cv2.KeyPoint(float(x),float(y),2) for x,y in feature['uv']]
out=cv2.drawMatches(refimg,refkp,query,kp,kept,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
cv2.rectangle(out,(0,0),(out.shape[1],35),(0,0,0),-1)
cv2.putText(out,'Reference map KF %d  -> held-out localization_3 at %.2fs   |   RANSAC inliers: %d'%(z.keyframe,z.bag_camera_offset,len(I)),(10,25),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,255,255),2)
Path(z.output).parent.mkdir(parents=True,exist_ok=True);cv2.imwrite(z.output,out,[cv2.IMWRITE_JPEG_QUALITY,88]);print('WROTE',z.output,'inliers',len(I),'raw desc matches',len(matches),'query_timestamp_difference_s',bestdiff)
