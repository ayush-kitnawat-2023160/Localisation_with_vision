#!/usr/bin/env python3
"""Evaluate private seeded candidate against saved LiDAR odometry proxy.

Reference is NOT independently surveyed ground truth. The LiDAR bag is opened
only after ROS1 visual/IMU seed-fallback replay is complete. No pose changes.
"""
import argparse,json,math,csv
from pathlib import Path
import numpy as np,rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--events',required=True);p.add_argument('--reference',required=True)
p.add_argument('--out',required=True);p.add_argument('--outage-start',type=float,default=20.)
p.add_argument('--outage-end',type=float,default=55.)
p.add_argument('--first-lidar',type=float,required=True)
a=p.parse_args();d=json.load(open(a.events));rows=d['odometry']
ref_st=[];ref_xyz=[]
with rosbag.Bag(a.reference) as bag:
    for _,m,_ in bag.read_messages(topics=['/aft_mapped_to_init']):
        t=m.pose.pose.position
        ref_st.append(m.header.stamp.to_sec());ref_xyz.append([t.x,t.y,t.z])
ref_st=np.array(ref_st);ref_xyz=np.array(ref_xyz)
vt=np.array([x['stamp'] for x in rows]);vp=np.array([x['xyz'] for x in rows]);
ref=np.column_stack([np.interp(vt,ref_st,ref_xyz[:,k]) for k in range(3)])
errors=np.linalg.norm(vp-ref,axis=1)
mask=(vt>=a.first_lidar+a.outage_start)&(vt<=a.first_lidar+a.outage_end)
dt=np.diff(vt);steps=np.linalg.norm(np.diff(vp,axis=0),axis=1)
lat=np.array([float(x['variance_x']) for x in rows if 'variance_x' in x])
metrics=dict(name=Path(a.out).name,
    reference_is_independently_surveyed_truth=False,
    source_lidar_used_for_estimation_during_outage=False,
    source_gps_used=False,
    anchor_source='last time-aligned healthy FAST-LIVO2 odom and camera-IMU VO',
    estimated_frames=len(rows),
    estimated_period_median_sec=float(np.median(dt)),
    estimated_gap_max_sec=float(max(dt)),
    max_position_step_m=float(max(steps)),
    first_output_bag_sec=float(vt[0]-a.first_lidar),
    last_output_bag_sec=float(vt[-1]-a.first_lidar),
    outage_rmse_3d_vs_lidar_imu_odom_proxy_m=float(np.sqrt(np.mean(errors[mask]**2))),
    outage_median_error_vs_lidar_imu_proxy_m=float(np.median(errors[mask])),
    outage_p95_error_vs_lidar_imu_proxy_m=float(np.percentile(errors[mask],95)),
    outage_last_error_vs_lidar_imu_proxy_m=float(errors[mask][-1]),
    restarted_after_simulated_lidar_return=bool(np.any(vt>a.first_lidar+a.outage_end+.2)),
    forbidden_pose_published=d['forbidden_pose_count'],
    source_statuses=d['states'],
    caveats=['Comparison trajectory is saved LiDAR/IMU SLAM, not surveyed truth',
             'Seed is obtained from pre-failure LiDAR odometry: this is NOT a cold-start VIO localization solution',
             'Inherit camera/body calibration offline: not physically verified for current mount',
             'No FAST-LIVO2 ESIKF replacement, navigation authority, map correction or TF broadcast'])
assert len(rows)>50
assert d['forbidden_pose_count']==0
assert not metrics['restarted_after_simulated_lidar_return']
out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
(out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
plt.figure(figsize=(9,5))
plt.plot(ref_xyz[:,0],ref_xyz[:,1],label='Saved LiDAR/IMU odom reference (not ground truth)',lw=1.4,alpha=.8)
plt.plot(vp[:,0],vp[:,1],label='Seeded ZED X RGB-D/IMU outage candidate',lw=1.4)
plt.scatter([vp[0,0]],[vp[0,1]],s=55,label='Outage candidate begins')
plt.axis('equal');plt.grid(alpha=.25);plt.xlabel('Local odom x (m)');plt.ylabel('Local odom y (m)');plt.legend(fontsize=8)
plt.title('LiDAR 20–55s simulated dropout: local odometry comparison')
plt.tight_layout();plt.savefig(out/'seeded_odom_vs_lidar_proxy.png',dpi=150);plt.close()
print('SEEDED_COMPARISON',json.dumps({k:v for k,v in metrics.items() if k in ('estimated_frames','estimated_period_median_sec','estimated_gap_max_sec','max_position_step_m','outage_rmse_3d_vs_lidar_imu_odom_proxy_m','outage_last_error_vs_lidar_imu_proxy_m','first_output_bag_sec','last_output_bag_sec','restarted_after_simulated_lidar_return','forbidden_pose_published')},indent=2))
