#!/usr/bin/env python3
"""Build Step-3 global-localization keyframes directly from loop-closed chunks.

Each input chunk is already expressed in the final loop-closed map frame.
We select spatial/yaw keyframes, transform that chunk back into the matching
local FAST-LIVO2 body frame, and store the optimized T_map_body pose.
This preserves the exact target-map coordinates without re-deskewing a bag.
"""
import argparse, csv, json, math, shutil
from pathlib import Path
import numpy as np


def qmat(q):
    x,y,z,w=np.asarray(q,dtype=float); n=np.linalg.norm([x,y,z,w]);
    if n < 1e-12: return np.eye(3)
    x,y,z,w=np.array([x,y,z,w])/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]],dtype=float)

def yaw(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

def adiff(a,b): return abs(math.atan2(math.sin(a-b),math.cos(a-b)))

def read_pcd(path):
    raw=Path(path).read_bytes(); marker=b'DATA binary\n'; k=raw.find(marker)
    if k<0: raise RuntimeError(f'{path}: binary PCD required')
    he=k+len(marker); hdr=raw[:he].decode('ascii'); meta={}
    for line in hdr.splitlines():
        s=line.split(maxsplit=1)
        if len(s)==2: meta[s[0]]=s[1]
    fields=meta['FIELDS'].split(); sizes=list(map(int,meta['SIZE'].split())); types=meta['TYPE'].split(); counts=list(map(int,meta.get('COUNT',' '.join(['1']*len(fields))).split()))
    fmts=[]; offs=[]; off=0; tm={('F',4):'<f4',('F',8):'<f8',('U',1):'u1',('U',2):'<u2',('U',4):'<u4',('I',4):'<i4'}
    for s,t,c in zip(sizes,types,counts):
        if c!=1: raise RuntimeError('multi-count fields unsupported')
        fmts.append(tm[(t,s)]); offs.append(off); off+=s
    dt=np.dtype({'names':fields,'formats':fmts,'offsets':offs,'itemsize':off})
    a=np.frombuffer(raw[he:],dtype=dt,count=int(meta['POINTS'])).copy()
    xyz=np.column_stack([a[k] for k in ('x','y','z')]).astype(float)
    intensity=np.asarray(a['intensity'],dtype=np.float32) if 'intensity' in fields else np.zeros(len(a),np.float32)
    if 'rgb' in fields: intensity=np.asarray(a['rgb'].view('<f4') if a['rgb'].dtype.itemsize==4 else np.zeros(len(a)),dtype=np.float32)
    good=np.isfinite(xyz).all(1); return xyz[good],intensity[good]

def voxel(xyz,intensity,leaf):
    if leaf<=0 or not len(xyz): return xyz,intensity
    keys=np.floor(xyz/leaf).astype(np.int64); _,idx=np.unique(keys,axis=0,return_index=True);idx.sort();return xyz[idx],intensity[idx]

def write_pcd(path,xyz,intensity):
    out=np.zeros((len(xyz),8),dtype='<f4');out[:,:3]=xyz.astype(np.float32);out[:,3]=intensity.astype(np.float32)
    h=("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
       "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
       "SIZE 4 4 4 4 4 4 4 4\nTYPE F F F F F F F F\nCOUNT 1 1 1 1 1 1 1 1\n"
       f"WIDTH {len(out)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(out)}\nDATA binary\n").encode()
    Path(path).write_bytes(h+out.tobytes())

ap=argparse.ArgumentParser();ap.add_argument('--trajectory',required=True);ap.add_argument('--chunks',required=True);ap.add_argument('--output',required=True);ap.add_argument('--distance',type=float,default=1.5);ap.add_argument('--yaw-deg',type=float,default=10.0);ap.add_argument('--voxel',type=float,default=.20);ap.add_argument('--min-range',type=float,default=1.0);ap.add_argument('--max-range',type=float,default=80.0);ap.add_argument('--overwrite',action='store_true');args=ap.parse_args()
out=Path(args.output)
if out.exists():
    if not args.overwrite: raise SystemExit(f'{out} exists; use --overwrite')
    shutil.rmtree(out)
(out/'pcd').mkdir(parents=True)
rows=list(csv.DictReader(open(args.trajectory))); chunks=Path(args.chunks)
prev_p=None;prev_yaw=None;manifest=[];poses=[];kid=0
for ri,r in enumerate(rows):
    t=float(r['timestamp']);p=np.array([float(r[k]) for k in ('x','y','z')]);q=np.array([float(r[k]) for k in ('qx','qy','qz','qw')]); y=yaw(q)
    if prev_p is not None and np.linalg.norm(p-prev_p)<args.distance and adiff(y,prev_yaw)<math.radians(args.yaw_deg): continue
    src=chunks/f'{t:.6f}.pcd'
    if not src.exists(): continue
    xyz_w,intensity=read_pcd(src); R=qmat(q)
    xyz_local=(xyz_w-p)@R
    rr=np.linalg.norm(xyz_local,axis=1); keep=np.isfinite(rr)&(rr>=args.min_range)&(rr<=args.max_range)
    xyz_local,intensity=voxel(xyz_local[keep],intensity[keep],args.voxel)
    if len(xyz_local)<500: continue
    write_pcd(out/'pcd'/f'{kid}.pcd',xyz_local,intensity)
    poses.append(f'{p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {q[3]:.12f} {q[0]:.12f} {q[1]:.12f} {q[2]:.12f}')
    manifest.append({'id':kid,'trajectory_row':ri,'timestamp_sec':t,'source_chunk':str(src),'stored_points':len(xyz_local)})
    prev_p,prev_yaw=p,y;kid+=1
(out/'pose.json').write_text('\n'.join(poses)+'\n')
with open(out/'keyframes.csv','w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=manifest[0].keys());w.writeheader();w.writerows(manifest)
(out/'metadata.json').write_text(json.dumps({'format':'STEP3 loop-closed local keyframes','trajectory':args.trajectory,'chunks':args.chunks,'keyframes':kid,'distance_threshold_m':args.distance,'yaw_threshold_deg':args.yaw_deg,'voxel_leaf_m':args.voxel,'pose_convention':'T_map_body tx ty tz qw qx qy qz','pcd_frame':'local FAST-LIVO2 body frame recovered from optimized world chunk'},indent=2)+'\n')
print(f'Complete: {kid} keyframes from {len(rows)} optimized poses')
