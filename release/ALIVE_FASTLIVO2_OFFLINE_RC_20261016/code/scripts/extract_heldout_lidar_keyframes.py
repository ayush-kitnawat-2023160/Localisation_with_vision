#!/usr/bin/env python3
"""Extract independently held-out September bag LiDAR scans for offline reference.

Run inside ROS Noetic Docker. No FAST-LIVO2 global poses or evaluated failure
trajectories are used to construct these scan observations.
"""
import argparse
import csv
from pathlib import Path

import numpy as np
import rosbag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--topic', default='/lidar103/velodyne_points')
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--voxel', type=float, default=0.45)
    parser.add_argument('--max-range', type=float, default=48.0)
    args = parser.parse_args()
    dest = Path(args.output)
    dest.mkdir(parents=True, exist_ok=True)
    # ALIVE_RECORDED_CART_2026-08-26_PASS4 -- lidar -> base_link.
    R = np.array([[0.925416516, -0.173648251, 0.336824223],
                  [0.163175972, 0.984807740, 0.059391224],
                  [-0.342020284, 0., 0.939692570]], dtype=np.float64)
    if abs(np.linalg.det(R)-1.) > 1e-4:
        raise RuntimeError('Invalid LiDAR->base rotation')
    bag_start = None
    last_out = -1e30
    rows = []
    with rosbag.Bag(args.bag, 'r') as bag:
        bag_start = bag.get_start_time()
        for _, msg, stamp in bag.read_messages(topics=[args.topic]):
            t = msg.header.stamp.to_sec()
            if t - last_out < args.interval - .025:
                continue
            dt = {'names': ['x','y','z'], 'formats': ['<f4']*3,
                  'offsets':[next(f.offset for f in msg.fields if f.name==n)
                             for n in ('x','y','z')], 'itemsize': msg.point_step}
            pts = np.frombuffer(msg.data, dtype=np.dtype(dt),
                                count=msg.width*msg.height)
            lidar = np.column_stack([pts[n] for n in ('x','y','z')]).astype(np.float32)
            good = np.isfinite(lidar).all(axis=1)
            r = np.linalg.norm(lidar[:,:2],axis=1)
            good &= (r>2.0)&(r<args.max_range)
            body = (lidar[good] @ R.T).astype(np.float32)
            good_z = (body[:,2]>-3.5)&(body[:,2]<5.0)
            body = body[good_z]
            if len(body)<300:continue
            _,keep=np.unique(np.floor(body/args.voxel).astype(np.int32),
                             axis=0, return_index=True)
            body=body[np.sort(keep)]
            seq=len(rows)
            np.save(dest/f'{seq:04d}.npy',body)
            rows.append({'seq':seq,'timestamp':f'{t:.9f}',
                         'bag_relative_s':f'{t-bag_start:.3f}',
                         'scan_points':len(lidar),'selected_points':len(body),
                         'frame':msg.header.frame_id})
            last_out=t
            if seq%25==0:
                print('LiDAR',seq,'at',round(t-bag_start,2),'sec',
                      'points',len(body),flush=True)
    if not rows:raise RuntimeError('No LiDAR scans extracted')
    with (dest/'keyframes.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
    print('Wrote',len(rows),'LiDAR observations in',dest,flush=True)


if __name__=='__main__':main()
