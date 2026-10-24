#!/usr/bin/env python3
"""Static matching-rate plots (same recording, distinct estimator configurations)."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'outputs'/'camera_diagnostics_20261019';out.mkdir(exist_ok=True)
labels={'lio':'LiDAR/IMU 10 Hz','scheduler':'Camera subscribed + 10 Hz LiDAR sync','track':'LIVO tracking without visual EKF','fusion':'Native visual EKF at 5 Hz','fusion10':'Native visual EKF at 10 Hz'}
runs={}
for mode in labels:
 p=(ROOT/'outputs'/'diag_loc_native_camera10hz_20261019'/'trajectory.csv' if mode=='fusion10'
    else ROOT/'outputs'/f'diag_loc_match075_{mode}_20261019'/'trajectory.csv')
 if not p.exists():continue
 rr=list(csv.DictReader(p.open()));t=np.array([float(x['timestamp']) for x in rr]);pos=np.array([[float(x[k]) for k in ('x','y','z')] for x in rr]);runs[mode]=(t,pos)
base=min(t[0] for t,_ in runs.values())
for name in ['xy','z','speed','pose_interval']:
 plt.figure(figsize=(9,6) if name=='xy' else (11,5))
 for m,(t,p) in runs.items():
  if name=='xy':plt.plot(p[:,0],p[:,1],label=labels[m],linewidth=1.2)
  elif name=='z':plt.plot(t-base,p[:,2],label=labels[m],linewidth=1.2)
  elif name=='speed':plt.plot(t[1:]-base,np.linalg.norm(np.diff(p,axis=0),axis=1)/np.maximum(np.diff(t),1e-6),label=labels[m],linewidth=1.2)
  elif name=='pose_interval':plt.plot(t[1:]-base,np.diff(t),label=labels[m],linewidth=1)
 plt.grid(alpha=.3)
 if name=='xy':plt.axis('equal');plt.xlabel('Local estimated X (m)');plt.ylabel('Local estimated Y (m)')
 elif name=='z':plt.xlabel('Elapsed bag sensor time (s)');plt.ylabel('Estimated Z (m)')
 elif name=='speed':plt.axhline(5,color='grey',linestyle='--',label='5m/s diagnostic threshold');plt.xlabel('Elapsed bag sensor time (s)');plt.ylabel('Pose-derived speed (m/s)')
 elif name=='pose_interval':plt.xlabel('Elapsed bag sensor time (s)');plt.ylabel('Time between published poses (s)')
 plt.title('Localization.bag, 0.75x playback — '+name.replace('_',' ').title()+' (not ground truth)');plt.legend(fontsize=9);plt.tight_layout()
 path=out/f'matched_localization_{name}.png';plt.savefig(path,dpi=155);plt.close();print(path)
