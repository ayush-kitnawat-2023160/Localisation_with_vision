#!/usr/bin/env python3
"""Build a new visual-only RGB-D 2D/3D ORB map, aligned to the saved LiDAR map.

Input: localization_full.bag ZED X registered depth and rectified left RGB,
       the SAME 225s LiDAR-only odometry CSV used by the LiDAR map,
       and offline-only camera/LiDAR calibration.
Output: images, per-keyframe depth-anchored world XYZ+ORB Hamming descriptors,
        full-image ORB appearance descriptors, quality telemetry and sparse RGB PCD.
Not loop-closed/surveyed and must never feed a live vehicle pose authority.
"""
import argparse,csv,json,math,collections,struct
from pathlib import Path
import cv2,numpy as np,rosbag,yaml
from tf.transformations import quaternion_matrix,quaternion_from_matrix,quaternion_slerp

def traj_load(p):
    x=list(csv.DictReader(open(p)))
    t=np.asarray([float(r['timestamp']) for r in x]);xyz=np.asarray([[float(r[k]) for k in ('x','y','z')] for r in x]);q=np.asarray([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in x]);order=np.argsort(t);return t[order],xyz[order],q[order]
def pose_at(t,data,max_gap=.18):
    ts,xx,qq=data
    j=int(np.searchsorted(ts,t));j=max(1,min(j,len(ts)-1))
    if min(abs(ts[j]-t),abs(ts[j-1]-t))>max_gap or t<ts[0] or t>ts[-1]:return None
    u=(t-ts[j-1])/(ts[j]-ts[j-1]);q=quaternion_slerp(qq[j-1],qq[j],u)
    T=quaternion_matrix(q);T[:3,3]=xx[j-1]*(1-u)+xx[j]*u
    return T
def rigid(R,t):
    T=np.eye(4);T[:3,:3]=np.array(R).reshape(3,3);T[:3,3]=t;return T
def img_bgr(m):
    arr=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)
    if m.encoding=='bgra8':return cv2.cvtColor(arr[:,:m.width*4].reshape(m.height,m.width,4),cv2.COLOR_BGRA2BGR)
    if m.encoding=='bgr8':return arr[:,:m.width*3].reshape(m.height,m.width,3).copy()
    if m.encoding=='rgb8':return cv2.cvtColor(arr[:,:m.width*3].reshape(m.height,m.width,3),cv2.COLOR_RGB2BGR)
    raise ValueError('unexpected RGB encoding '+m.encoding)
def rgb_pcd(path,xyz,bgr):
    xyz=np.asarray(xyz,dtype='<f4');c=np.asarray(bgr,dtype=np.uint8)
    rgb=(c[:,2].astype(np.uint32)<<16)|(c[:,1].astype(np.uint32)<<8)|c[:,0].astype(np.uint32)
    rows=np.empty((len(xyz),4),dtype='<f4');rows[:,:3]=xyz;rows[:,3]=rgb.view('<f4')
    header=(f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n').encode()
    path.write_bytes(header+rows.tobytes())
def main():
    a=argparse.ArgumentParser();a.add_argument('--bag',required=True);a.add_argument('--trajectory',required=True);a.add_argument('--calibration',required=True);a.add_argument('--out',required=True);a.add_argument('--frame-period',type=float,default=1.0);a.add_argument('--orb-features',type=int,default=2200);a.add_argument('--max-frames',type=int,default=0);args=a.parse_args()
    root=Path(args.out);assert not root.exists() or not list(root.iterdir()),'Refusing to overwrite '+str(root)
    vis=root/'visual';(vis/'images').mkdir(parents=True);(vis/'landmarks').mkdir(parents=True)
    calib=yaml.safe_load(open(args.calibration));intr=calib['laserMapping'];fx,fy,cx,cy=[float(intr[k]) for k in ['cam_fx','cam_fy','cam_cx','cam_cy']]
    ec=calib['extrin_calib'];T_imu_lidar=rigid(ec['extrinsic_R'],ec['extrinsic_T']);T_cam_lidar=rigid(ec['Rcl'],ec['Pcl']);T_lidar_cam=np.linalg.inv(T_cam_lidar)
    trajectory=traj_load(args.trajectory);orb=cv2.ORB_create(nfeatures=args.orb_features,fastThreshold=8)
    image_topic='/zedx/zed_node/left/image_rect_color';depth_topic='/zedx/zed_node/depth/depth_registered'
    depth_cache=collections.OrderedDict();rgb_pending=collections.OrderedDict();rows=[];preview_xyz=[];preview_rgb=[];stats=[]
    seen=collections.Counter();last_saved=-1e15
    def do_pair(im,dep,t):
        nonlocal last_saved
        if t-last_saved<args.frame_period:return
        pose=pose_at(t,trajectory)
        if pose is None:seen['pose_unavailable']+=1;return
        if dep.encoding!='32FC1':seen['unexpected_depth_encoding']+=1;return
        bgr=img_bgr(im)
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        kp,d=orb.detectAndCompute(gray,None)
        if d is None or len(kp)<40:seen['low_raw_orb']+=1;return
        depth=np.frombuffer(dep.data,np.float32).reshape(dep.height,dep.step//4)[:,:dep.width]
        uv=np.array([k.pt for k in kp],dtype=np.float32);ix=np.rint(uv[:,0]).astype(int);iy=np.rint(uv[:,1]).astype(int)
        inside=(ix>=1)&(ix<dep.width-1)&(iy>=1)&(iy<dep.height-1)
        safeix=np.clip(ix,0,dep.width-1);safeiy=np.clip(iy,0,dep.height-1)
        z=depth[safeiy,safeix];good=inside&np.isfinite(z)&(z>=.6)&(z<=25.0)
        if int(good.sum())<18:seen['low_depth_support']+=1;return
        z=z[good];points_cam=np.column_stack(((uv[good,0]-cx)*z/fx,(uv[good,1]-cy)*z/fy,z))
        T_map_cam=pose@T_imu_lidar@T_lidar_cam;points_map=points_cam@T_map_cam[:3,:3].T+T_map_cam[:3,3]
        idx=len(rows);relimg=f'images/{idx:05d}.jpg';relfeat=f'landmarks/{idx:05d}.npz'
        cv2.imwrite(str(vis/relimg),bgr,[cv2.IMWRITE_JPEG_QUALITY,83]);
        np.savez_compressed(str(vis/relfeat),uv=uv[good].astype(np.float32),xyz=points_map.astype(np.float32),desc=d[good],all_uv=uv,all_desc=d)
        q=quaternion_from_matrix(T_map_cam);uvs=uv[good];n=int(len(uvs));raw=int(len(uv))
        upper=int(np.sum(uvs[:,1]<0.40*im.height));lower=int(np.sum(uvs[:,1]>0.60*im.height))
        rows.append(dict(id=idx,timestamp=t,x=float(T_map_cam[0,3]),y=float(T_map_cam[1,3]),z=float(T_map_cam[2,3]),qx=float(q[0]),qy=float(q[1]),qz=float(q[2]),qw=float(q[3]),descriptors=n,raw_orb_features=raw,depth_supported_features=n,upper_image_features=upper,lower_image_features=lower,image=relimg,landmarks=relfeat))
        # Preview subsampling is NOT a global uniqueness analysis of landmarks.
        pick=np.arange(0,n,3);preview_xyz.append(points_map[pick].astype(np.float32));preview_rgb.append(bgr[safeiy[good][pick],safeix[good][pick]])
        stats.append([raw,n,upper,lower]);last_saved=t;seen['frames_exported']+=1
        if idx%25==0:print(f'VISUAL frame={idx:04d} bag_s={t-trajectory[0][0]:.1f} ORB={raw} validXYZ={n}',flush=True)
    with rosbag.Bag(args.bag) as bag:
        for topic,m,record_stamp in bag.read_messages(topics=[image_topic,depth_topic]):
            t=m.header.stamp.to_sec();key=round(t,5)
            if topic==image_topic:
                seen['rgb_messages']+=1
                if t-last_saved<args.frame_period:continue
                dep=depth_cache.pop(key,None)
                if dep is not None:seen['exact_rgbd_pairs_selected']+=1;do_pair(m,dep,t)
                else:rgb_pending[key]=(t,m)
            else:
                seen['depth_messages']+=1
                im=rgb_pending.pop(key,None)
                if im is not None:seen['exact_rgbd_pairs_selected']+=1;do_pair(im[1],m,t)
                else:depth_cache[key]=m
            while len(depth_cache)>12:depth_cache.popitem(last=False)
            while len(rgb_pending)>12:
                rgb_pending.popitem(last=False);seen['unpaired_selected_rgb']+=1
            if args.max_frames>0 and len(rows)>=args.max_frames:break
    if not rows:raise RuntimeError('No keyframes selected')
    with (vis/'keyframes.csv').open('w',newline='') as f:
        wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
    xyz=np.vstack(preview_xyz);colors=np.vstack(preview_rgb)
    # Fast global voxel preview (not unique 3D structure points).
    vox=np.floor(xyz/.18).astype(np.int32);_,indices=np.unique(vox,axis=0,return_index=True);indices.sort();rgb_pcd(vis/'landmarks_preview.pcd',xyz[indices],colors[indices]);S=np.array(stats)
    meta=dict(map_type='metric sparse RGB-D ORB keyframe/3D-landmark observation map',source_bag=Path(args.bag).name,source_trajectory=args.trajectory,alignment='same RAW 225s LiDAR-only FAST-LIVO2 trajectory as campus_full_lio_226s_20261019',gps_used=False,trajectory_is_loop_closed=False,surveyed_metric_global_accuracy=False,physical_imu_lidar_extrinsics_validated=False,camera_lidar_reprojection_median_px_previous=3.50,counts=dict(seen),visual_keyframes=len(rows),source_rgb_messages=seen['rgb_messages'],source_depth_messages=seen['depth_messages'],trajectory_first_sec=float(trajectory[0][0]),trajectory_last_sec=float(trajectory[0][-1]),map_first_sec=rows[0]['timestamp'],map_last_sec=rows[-1]['timestamp'],map_coverage_sec=rows[-1]['timestamp']-rows[0]['timestamp'],sum_raw_orb_observations=int(S[:,0].sum()),sum_depth_valid_3d_orb_observations=int(S[:,1].sum()),raw_orb_per_frame={'min':int(S[:,0].min()),'median':float(np.median(S[:,0])),'p95':float(np.percentile(S[:,0],95)),'max':int(S[:,0].max())},depth_valid_orb_per_frame={'min':int(S[:,1].min()),'median':float(np.median(S[:,1])),'p95':float(np.percentile(S[:,1],95)),'max':int(S[:,1].max())},upper_image_3d_observations=int(S[:,2].sum()),lower_image_3d_observations=int(S[:,3].sum()),preview_unique_voxel_points=int(len(indices)),orb_nfeatures=args.orb_features,feature_quantization='32-byte binary ORB descriptors, BF Hamming and PnP',feature_geometry='registered ZED X 32FC1 depth (0.6-25m) -> T_map_camera via LiDAR odometry and inherited LiDAR/IMU',note='3D observations are not necessarily distinct physical landmarks. Preview voxel count is not a unique landmark count. No image-based BA, loop closure, or metric ground-truth validation.')
    (root/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n');print('FINISHED',json.dumps({k:meta[k] for k in ['visual_keyframes','map_coverage_sec','sum_raw_orb_observations','sum_depth_valid_3d_orb_observations','preview_unique_voxel_points']},indent=2),flush=True)
if __name__=='__main__':main()
