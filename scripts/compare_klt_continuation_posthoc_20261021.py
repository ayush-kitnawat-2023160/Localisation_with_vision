#!/usr/bin/env python3
"""POST-HOC KLT continuation drift vs uninterrupted LIO-ONLY odometry.

NEVER an estimation input. This separately recorded uninterrupted FAST-LIVO2
bag is an ODOMETRY PROXY, not ground truth; 3D SE(3) initial pose alignment
measures only RELATIVE trajectory divergence during an injected LiDAR blackout.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import rosbag
from tf.transformations import quaternion_matrix,quaternion_slerp


def read_csv(path):
    rows=[]
    with open(path) as f:
        for line in csv.DictReader(f):
            t=float(line['stamp'])
            q=np.array([float(line[k]) for k in ('qx','qy','qz','qw')])
            T=quaternion_matrix(q)
            T[:3,3]=[float(line[k]) for k in ('x','y','z')]
            if np.isfinite(T).all() and np.linalg.norm(q)>.99:rows.append((t,T,q))
    return sorted(rows,key=lambda x:x[0])


def read_reference(path):
    rows=[]
    with rosbag.Bag(str(path),'r') as bag:
        for _,msg,_ in bag.read_messages(topics=['/aft_mapped_to_init']):
            t=msg.header.stamp.to_sec()
            p=msg.pose.pose.position
            o=msg.pose.pose.orientation
            q=np.array([o.x,o.y,o.z,o.w],float)
            T=quaternion_matrix(q)
            T[:3,3]=[p.x,p.y,p.z]
            if np.isfinite(T).all():rows.append((t,T,q))
    return rows


def interp(rows, t, max_gap=.32):
    ts=[x[0] for x in rows]
    j=int(np.searchsorted(ts,t))
    if j==0 or j>=len(rows):return None
    a,b=rows[j-1],rows[j]
    dt=b[0]-a[0]
    if dt<=0 or dt>max_gap:return None
    w=(t-a[0])/dt
    T=quaternion_matrix(quaternion_slerp(a[2],b[2],w))
    T[:3,3]=(1-w)*a[1][:3,3]+w*b[1][:3,3]
    return T


def rotation_error_deg(a,b):
    R=a[:3,:3].T@b[:3,:3]
    return math.degrees(math.acos(np.clip((float(np.trace(R))-1)*.5,-1,1)))


def metrics(rows,aligned,reference):
    xyz_e=np.array([p[:3,3] for p in aligned])
    xyz_r=np.array([p[:3,3] for p in reference])
    errors=np.linalg.norm(xyz_e-xyz_r,axis=1)
    orient=np.array([rotation_error_deg(a,b) for a,b in zip(aligned,reference)])
    stamps=np.array([x[0] for x in rows])
    out={'paired':len(rows),'duration_s':float(stamps[-1]-stamps[0]),
         'relative_position_RMSE_m':float(np.sqrt(np.mean(errors*errors))),
         'relative_position_median_m':float(np.median(errors)),
         'relative_position_p95_m':float(np.percentile(errors,95)),
         'relative_position_endpoint_m':float(errors[-1]),
         'max_position_divergence_m':float(np.max(errors)),
         'orientation_divergence_endpoint_deg':float(orient[-1]),
         'orientation_divergence_p95_deg':float(np.percentile(orient,95))}
    for window in (1.,3.,5.):
        diffs=[]
        for k,t in enumerate(stamps):
            j=int(np.searchsorted(stamps,t+window))
            if j>=len(stamps) or abs(stamps[j]-(t+window))>.16:continue
            de=np.linalg.inv(aligned[k])@aligned[j]
            dr=np.linalg.inv(reference[k])@reference[j]
            diffs.append(np.linalg.norm(de[:3,3]-dr[:3,3]))
        out[f'RPE_{window:g}s_translation_median_m']=(float(np.median(diffs)) if diffs else None)
        out[f'RPE_{window:g}s_translation_p95_m']=(float(np.percentile(diffs,95)) if diffs else None)
    return out,errors,orient


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--run',required=True)
    ap.add_argument('--lidar-only-odometry-bag',required=True)
    ap.add_argument('--output',required=True)
    a=ap.parse_args()
    root=Path(a.run)
    out=Path(a.output)
    manifest=dict(line.split('=',1) for line in
        (root/'experiment_manifest.env').read_text().splitlines() if '=' in line)
    if manifest.get('INJECT_OFFLINE_LIDAR_FAULT')!='true':
        raise RuntimeError('Refusing to claim blackout drift without explicit fault manifest')
    if manifest.get('VISUAL') not in ('true','false'):
        raise RuntimeError('Malformed visualization/worker manifest')
    bagname=Path(a.lidar_only_odometry_bag).name
    if not bagname.startswith(manifest['BAG']+'_'):
        raise RuntimeError('Reference bag does not match the recorded fault bag')
    trajectory=read_csv(root/'klt_continuation_trajectory.csv')
    if len(trajectory)<15:raise RuntimeError('Too few continuation candidates')
    ref=read_reference(a.lidar_only_odometry_bag)
    pairs=[(t,T,interp(ref,t)) for t,T,q in trajectory]
    valid=[(t,T,R) for t,T,R in pairs if R is not None]
    if len(valid)<15:raise RuntimeError('Too few timestamp-overlapping reference odometry poses')
    ts=np.array([x[0] for x in valid]);pred=[x[1] for x in valid]
    target=[x[2] for x in valid]
    first_pose_map=pred[0]@np.linalg.inv(target[0])
    reference=[first_pose_map@T for T in target]
    summary,err,orientation=metrics(valid,pred,reference)
    summary.update({'source_run':str(root),'reference_file':bagname,
        'mapping_bag_used_for_reference':False,
        'reference_is_undisturbed_independent_LIDAR_ODOm_not_ground_truth':True,
        'reference_used_during_filtering':False,
        'reference_used_only_posthoc':True,
        'initial_alignment':'one SE(3) transform at first candidate observation',
        'calibration_mount_verified':False,
        'visual_odom_is_driving_authority':False,
        'fault_window_record_seconds':[float(manifest['FAULT_START']),float(manifest['FAULT_END'])],
        'candidate_start_unix_stamp':float(ts[0]),
        'candidate_end_unix_stamp':float(ts[-1]),
        'not_surveyed_ground_truth':True})
    out.mkdir(parents=True,exist_ok=True)
    (out/'posthoc_lio_reference_metrics.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (out/'posthoc_lio_reference_errors.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['stamp','elapsed_sec','position_divergence_m','orientation_divergence_deg'])
        writer.writerows(zip(ts,ts-ts[0],err,orientation))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
