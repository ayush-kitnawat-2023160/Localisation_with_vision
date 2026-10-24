#!/usr/bin/env python3
"""Exploratory (NOT authoritative) LiDAR ground-plane vs IMU gravity alignment.

Fits plane only in the *predicted ground vicinity*. Assumes the first few seconds
are near stationary; fitted plane can be biased by ramps/curbs and this prior.
"""
import argparse,json,math
from pathlib import Path
import rosbag,numpy as np,yaml

def points(m):
 names={x.name:x for x in m.fields}
 if not all(k in names for k in ('x','y','z')):return np.empty((0,3))
 dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[names[x].offset for x in ('x','y','z')],'itemsize':m.point_step})
 out=np.frombuffer(m.data,dtype=dtype,count=m.width*m.height)
 xyz=np.column_stack([out[x] for x in ('x','y','z')]).astype(float)
 return xyz[np.isfinite(xyz).all(axis=1)]

def main():
 p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--profile',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 with open(a.profile) as f:c=yaml.safe_load(f)
 R=np.array(c['extrin_calib']['extrinsic_R'],float).reshape(3,3)
 imu=[];scans=[]
 with rosbag.Bag(a.bag) as b:
  t0=b.get_start_time()
  for topic,m,r in b.read_messages(topics=['/mavros/imu/data','/lidar103/velodyne_points']):
   t=m.header.stamp.to_sec()-t0
   if t>7:break
   if topic=='/mavros/imu/data' and 0<t<3:
    x=m.linear_acceleration;g=m.angular_velocity;imu.append([x.x,x.y,x.z,g.x,g.y,g.z])
   if topic=='/lidar103/velodyne_points' and t>1 and len(scans)<4 and (not scans or t-scans[-1][0]>0.8):
    scans.append((t,points(m)))
 arr=np.array(imu)
 if len(arr)<50:raise SystemExit('No stationary IMU samples available')
 acc=np.mean(arr[:,:3],axis=0);imu_up=acc/np.linalg.norm(acc);lidar_ground_pred=R.T@imu_up
 out=[]
 for ts,xyz in scans:
  dist=np.linalg.norm(xyz,axis=1);height=xyz@lidar_ground_pred
  selector=(dist>2)&(dist<25)&(height>-2.3)&(height<-.6)
  pts=xyz[selector]
  # Match lowest/ground slab using independent local planar PCA; selection
  # is gravity-prior dependent, so not an independent calibration solution.
  if len(pts)<100:continue
  # Fixed seed for deterministic sampling.
  rng=np.random.default_rng(12);pts=pts[rng.choice(len(pts),size=min(5000,len(pts)),replace=False)]
  winner=None
  for i in range(200):
   tri=pts[rng.integers(0,len(pts),size=3)];normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);norm=np.linalg.norm(normal)
   if norm<1e-5:continue
   normal/=norm
   if np.dot(normal,lidar_ground_pred)<0:normal*=-1
   if np.dot(normal,lidar_ground_pred)<.82:continue
   d=pts@normal
   offset=np.median(d);mask=np.abs(d-offset)<.13
   if winner is None or mask.sum()>winner[0]:winner=(int(mask.sum()),normal,mask)
  if winner is None:continue
  support,n,mask=winner
  centered=pts[mask]-pts[mask].mean(axis=0)
  eigval,eigvec=np.linalg.eigh(centered.T@centered);n=eigvec[:,0]
  if np.dot(n,lidar_ground_pred)<0:n*=-1
  angle=math.degrees(math.acos(float(np.clip(np.dot(n,lidar_ground_pred),-1,1))))
  out.append(dict(scan_sec=ts,eligible_points=int(len(pts)),ransac_inliers=support,inlier_fraction=float(support/len(pts)),ground_plane_vs_IMU_and_profile_angle_deg=angle,normal_lidar=n.tolist(),expected_normal_lidar=lidar_ground_pred.tolist(),signed_offset_m=float(np.median(pts[mask]@n))))
 result=dict(bag=Path(a.bag).name,imu_samples=len(arr),imu_accel_mean=acc.tolist(),imu_accel_std=np.std(arr[:,:3],axis=0).tolist(),imu_gyro_norm_median=float(np.median(np.linalg.norm(arr[:,3:],axis=1))),predicted_ground_normal_lidar=lidar_ground_pred.tolist(),scan_plane_samples=out,median_plane_angle_deg=float(np.median([x['ground_plane_vs_IMU_and_profile_angle_deg'] for x in out])) if out else None,caveats=['Predicted-ground-slab plane fitting is an exploratory consistency check and is biased toward the assumed calibration','MAVROS IMU acceleration vector is not independent tilt ground truth while accelerating','Cannot infer LiDAR-IMU translation from plane normals or confirm physical mount','Segmentation can select ramps or other horizontal structures rather than road ground'])
 Path(a.out).parent.mkdir(parents=True,exist_ok=True);Path(a.out).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
