#!/usr/bin/env python3
"""Diagnostic-only FAST-LIVO2 pose sanity report; does NOT infer absolute accuracy.

Usage: python3 scripts/analyze_zedx_baseline.py \
  --trajectory outputs/baseline_full_20261019/trajectory.csv \
  --old-trajectory outputs/full_20261019/trajectory.csv \
  --out outputs/baseline_full_20261019
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

START = 1792222492.825261
DURATION = 226.090772


def read(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["timestamp"]) for r in rows])
    xyz = np.array([[float(r[k]) for k in ("x", "y", "z")] for r in rows])
    quat = np.array([[float(r[k]) for k in ("qx", "qy", "qz", "qw")] for r in rows])
    good = np.isfinite(t) & np.isfinite(xyz).all(axis=1) & np.isfinite(quat).all(axis=1)
    t, xyz, quat = t[good], xyz[good], quat[good]
    return t, xyz, quat


def stats(t, xyz, quat, name):
    dt = np.diff(t)
    dp = np.diff(xyz, axis=0)
    speed = np.linalg.norm(dp, axis=1) / np.maximum(dt, 1e-5)
    xy_speed = np.linalg.norm(dp[:, :2], axis=1) / np.maximum(dt, 1e-5)
    yaw = np.unwrap(np.arctan2(
        2 * (quat[:, 3] * quat[:, 2] + quat[:, 0] * quat[:, 1]),
        1 - 2 * (quat[:, 1] ** 2 + quat[:, 2] ** 2)))
    # Derivatives of noisy pose trajectories are observations, not cart velocities.
    return dict(
        name=name, pose_count=len(t), first_stamp=float(t[0]), last_stamp=float(t[-1]),
        first_offset_from_bag_sec=float(t[0]-START),
        last_offset_from_bag_sec=float(t[-1]-START),
        pose_span_sec=float(t[-1]-t[0]),
        coverage_fraction_of_bag_span=float((t[-1]-t[0])/DURATION),
        fraction_bag_elapsed_at_last_pose=float((t[-1]-START)/DURATION),
        max_pose_stamp_gap_sec=float(np.max(dt)),
        median_pose_stamp_gap_sec=float(np.median(dt)),
        num_gaps_over_0p3s=int(np.count_nonzero(dt>.3)),
        num_gaps_over_1s=int(np.count_nonzero(dt>1)),
        num_nonpositive_dt=int(np.count_nonzero(dt<=0)),
        path_length_3d_m=float(np.linalg.norm(dp,axis=1).sum()),
        path_length_xy_m=float(np.linalg.norm(dp[:,:2],axis=1).sum()),
        displacement_xy_m=float(np.linalg.norm(xyz[-1,:2]-xyz[0,:2])),
        x_range_m=[float(xyz[:,0].min()),float(xyz[:,0].max())],
        y_range_m=[float(xyz[:,1].min()),float(xyz[:,1].max())],
        z_range_m=[float(xyz[:,2].min()),float(xyz[:,2].max())],
        z_span_m=float(np.ptp(xyz[:,2])),
        start_end_z_change_m=float(xyz[-1,2]-xyz[0,2]),
        median_derived_speed_mps=float(np.median(speed)),
        p95_derived_speed_mps=float(np.percentile(speed,95)),
        max_derived_speed_mps=float(speed.max()),
        num_speed_over_5_mps=int((speed>5).sum()),
        num_speed_over_8_mps=int((speed>8).sum()),
        p95_horizontal_speed_mps=float(np.percentile(xy_speed,95)),
        abs_yaw_change_deg=float(np.degrees(yaw[-1]-yaw[0])),
        max_position_step_m=float(np.linalg.norm(dp,axis=1).max()),
        max_quaternion_norm_error=float(np.max(np.abs(np.linalg.norm(quat,axis=1)-1))),
    )


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trajectory",type=Path,required=True)
    ap.add_argument("--old-trajectory",type=Path)
    ap.add_argument("--out",type=Path,required=True)
    args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    t,p,q=read(args.trajectory)
    if len(t)<20:raise RuntimeError("Not enough pose measurements")
    data={"baseline":stats(t,p,q,"standard_fast_livo2"),
          "interpretation":"Odometry self-consistency and physically plausible motion checks only; NO surveyed ground truth, loop closure, or absolute map alignment."
          }
    if args.old_trajectory and args.old_trajectory.exists():
        to,po,qo=read(args.old_trajectory)
        data["prior_failsafe_mapping"]=stats(to,po,qo,"previous_map_build")
        a=max(t[0],to[0]);b=min(t[-1],to[-1])
        if b>a+5:
            times=np.arange(a,b,.1)
            X=np.column_stack([np.interp(times,t,p[:,j]) for j in range(3)])
            Y=np.column_stack([np.interp(times,to,po[:,j]) for j in range(3)])
            subset=times<=a+5
            xx=X[subset,:2];yy=Y[subset,:2]
            U,s,Vt=np.linalg.svd((xx-xx.mean(axis=0)).T@(yy-yy.mean(axis=0)))
            R=U@np.diag([1,np.linalg.det(U@Vt)])@Vt
            Xal=(X[:,:2]-xx.mean(axis=0))@R+yy.mean(axis=0)
            residual=np.linalg.norm(Xal-Y[:,:2],axis=1)
            data["relative_trajectory_comparison"]={
                "overlap_duration_sec":float(b-a),
                "alignment":"Rigid XY alignment using first 5 seconds of shared timestamps, not ground truth",
                "median_horizontal_difference_m":float(np.median(residual)),
                "p95_horizontal_difference_m":float(np.percentile(residual,95)),
                "end_horizontal_difference_m":float(residual[-1]),
                "z_span_difference_m":float(np.ptp(p[:,2])-np.ptp(po[:,2])),
                "warning":"Different playback configurations and initialization; disagreement is not an absolute accuracy metric."
            }
    (args.out/"baseline_diagnostics.json").write_text(json.dumps(data,indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.figure(figsize=(9,8))
    plt.plot(p[:,0],p[:,1],lw=1.25,label="Standard FAST-LIVO2")
    plt.scatter(p[0,0],p[0,1],s=70,marker='o',label="Start")
    plt.scatter(p[-1,0],p[-1,1],s=70,marker='x',label="End")
    plt.xlabel("Local odom X (m)");plt.ylabel("Local odom Y (m)")
    plt.title("Localization_full — FAST-LIVO2 local odometry (not ground truth)")
    plt.axis("equal");plt.grid(alpha=.2);plt.legend();plt.tight_layout()
    plt.savefig(args.out/"baseline_xy.png",dpi=160);plt.close()
    plt.figure(figsize=(10,4))
    plt.plot(t-START,p[:,2],lw=1)
    plt.xlabel("ROS bag elapsed (s)");plt.ylabel("Estimated Z (m)")
    plt.title("Estimated vertical position (not independently verified)")
    plt.grid(alpha=.2);plt.tight_layout()
    plt.savefig(args.out/"baseline_z.png",dpi=160);plt.close()
    dt=np.diff(t);speed=np.linalg.norm(np.diff(p,axis=0),axis=1)/np.maximum(dt,1e-5)
    plt.figure(figsize=(10,4))
    plt.plot(t[1:]-START,speed,lw=.8)
    plt.axhline(5,ls="--",lw=1,label="5 m/s diagnostic threshold")
    plt.xlabel("ROS bag elapsed (s)");plt.ylabel("Pose-derived speed (m/s)")
    plt.title("Motion discontinuity diagnostic (not wheel-speed ground truth)")
    plt.grid(alpha=.2);plt.legend();plt.tight_layout()
    plt.savefig(args.out/"baseline_speed.png",dpi=160);plt.close()
    print(json.dumps(data,indent=2))


if __name__=="__main__":
    main()
