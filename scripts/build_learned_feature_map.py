#!/usr/bin/env python3
"""Offline, pose-aligned RGB-D SuperPoint or XFeat feature map from cached ZED X depth.
Preserves the same unoptimized map frame and verified image/depth timestamps.
Never use GPS or overwrite the original ORB / LiDAR maps.
"""
import argparse,csv,json,time
from pathlib import Path
import cv2,numpy as np,torch,yaml
from scipy.spatial.transform import Rotation
from learned_visual_backends import load_backend,extract

def rotT(xyz,q):
 T=np.eye(4);T[:3,:3]=Rotation.from_quat(q).as_matrix();T[:3,3]=xyz;return T

def main():
 p=argparse.ArgumentParser();p.add_argument('--source-visual-map',required=True);p.add_argument('--source-depth',required=True);p.add_argument('--calibration',required=True);p.add_argument('--vendor',required=True);p.add_argument('--backend',required=True,choices=['superpoint_lightglue','xfeat_mnn','xfeat_lighterglue']);p.add_argument('--out',required=True);p.add_argument('--max-frames',type=int,default=0);p.add_argument('--max-kpts',type=int,default=1100);p.add_argument('--resize-width',type=int,default=640);p.add_argument('--device',default='cpu');a=p.parse_args()
 source=Path(a.source_visual_map)/'visual';rows=list(csv.DictReader(open(source/'keyframes.csv')));depthdir=Path(a.source_depth);dest=Path(a.out);assert not dest.exists() or not list(dest.iterdir()),'Refuse overwrite learned map'
 (dest/'visual'/'landmarks').mkdir(parents=True,exist_ok=True)
 b=load_backend(a.backend,a.vendor,a.device,max_kpts=a.max_kpts,resize_width=a.resize_width)
 C=yaml.safe_load(open(a.calibration))['laserMapping'];fx,fy,cx,cy=[float(C[k]) for k in ('cam_fx','cam_fy','cam_cx','cam_cy')]
 feats=[];outrows=[];time_start=time.perf_counter();N=min(len(rows),a.max_frames) if a.max_frames else len(rows)
 for i,row in enumerate(rows[:N]):
  t0=time.perf_counter();im=cv2.imread(str(source/row['image']));depth=np.load(depthdir/f'{i:05d}.npz')['depth'];assert depth.shape==im.shape[:2]
  ex=extract(b,im);uv=ex['uv'];xy=np.rint(uv).astype(np.int32);inside=(xy[:,0]>=1)&(xy[:,0]<depth.shape[1]-1)&(xy[:,1]>=1)&(xy[:,1]<depth.shape[0]-1)
  z=depth[np.clip(xy[:,1],0,depth.shape[0]-1),np.clip(xy[:,0],0,depth.shape[1]-1)]
  good=inside&np.isfinite(z)&(z>=.6)&(z<=25.0)
  d=z[good];xyz_cam=np.column_stack(((uv[good,0]-cx)*d/fx,(uv[good,1]-cy)*d/fy,d))
  T=rotT([float(row[k]) for k in ('x','y','z')],[float(row[k]) for k in ('qx','qy','qz','qw')]);xyz=xyz_cam@T[:3,:3].T+T[:3,3]
  np.savez_compressed(str(dest/'visual'/'landmarks'/f'{i:05d}.npz'),uv=uv[good],xyz=xyz.astype(np.float32),desc=ex['desc'][good].astype(np.float16),scores=ex['scores'][good],all_uv=uv,all_desc=ex['desc'].astype(np.float16),all_scores=ex['scores'],image_size=np.asarray(ex['shape'],np.int32))
  cur=dict(row);cur.update(id=i,landmarks=f'landmarks/{i:05d}.npz',raw_learned_features=int(len(uv)),descriptors=int(good.sum()),upper_image_features=int(np.count_nonzero(uv[good,1]<.4*im.shape[0])),lower_image_features=int(np.count_nonzero(uv[good,1]>.6*im.shape[0])),extract_ms=round(ex['extract_ms'],3))
  outrows.append(cur)
  if i%25==0:print('MAP',a.backend,'frame',i,'of',N,'learned raw',len(uv),'depth3d',good.sum(),'lat_ms',round(ex['extract_ms']),flush=True)
 with (dest/'visual'/'keyframes.csv').open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(outrows[0]));w.writeheader();w.writerows(outrows)
 a0=np.array([r['raw_learned_features'] for r in outrows]);b0=np.array([r['descriptors'] for r in outrows]);
 result={'type':'pose-aligned sparse RGB-D learned features; not BA/loop-closed','feature_backend':a.backend,'source_orb_map':a.source_visual_map,'depth_frames_exactly_timestamp_matched':True,'device':a.device,'max_keypoints_per_image':a.max_kpts,'model_input_resize_width':a.resize_width,'map_image_dimensions':[960,600],'frames':N,'raw_keypoint_observations':int(a0.sum()),'depth_supported_observations':int(b0.sum()),'mean_depth_support':float(np.mean(b0/a0)),'median_depth_support_per_frame':float(np.median(b0)),'min_depth_support_per_frame':int(b0.min()),'mean_extract_ms':float(np.mean([r['extract_ms'] for r in outrows])),'median_extract_ms':float(np.median([r['extract_ms'] for r in outrows])),'elapsed_wall_s':time.perf_counter()-time_start,'gps_used':False,'surveyed_ground_truth':False,'independent_landmark_merging':False,'model_architecture_descriptor_dim':int(np.load(dest/'visual'/'landmarks'/'00000.npz')['desc'].shape[1])}
 (dest/'metadata.json').write_text(json.dumps(result,indent=2)+'\n');print('BUILD_COMPLETE',json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
