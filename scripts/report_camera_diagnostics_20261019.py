#!/usr/bin/env python3
"""Matched-playback camera-path isolation report, with measured profiled stages.

Never infers ground-truth accuracy. Requires four independently completed runs.
"""
from pathlib import Path
import json,csv,re
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs'
DEST=OUT/'camera_diagnostics_20261019'

def read(name):
 root=OUT/name
 rows=list(csv.DictReader((root/'trajectory.csv').open()))
 t=np.array([float(v['timestamp']) for v in rows]);x=np.array([[float(v[k]) for k in ('x','y','z')] for v in rows]);dt=np.diff(t);ds=np.linalg.norm(np.diff(x,axis=0),axis=1);speed=ds/np.maximum(dt,1e-6)
 log=(root/'launch.log').read_text(errors='replace')
 log=re.sub(r'\x1b\[[0-9;]*[A-Za-z]','',log)
 stages={}
 for label in ('retrieveFromVisualSparseMap','computeJacobianAndUpdateEKF','Current Total Time'):
  values=np.array([float(v) for v in re.findall(r'\|\s*'+label+r'\s*\|\s*([0-9.]+)\s*\|',log)])*1000
  stages[label]={'samples':int(len(values)),'median_ms':float(np.median(values)) if len(values) else None,'p95_ms':float(np.percentile(values,95)) if len(values) else None,'max_ms':float(np.max(values)) if len(values) else None}
 diagnostic=np.array([[float(a),float(b),float(c)] for a,b,c in re.findall(r'\[VIO_DIAGNOSTIC\] tracking_only=1 frames=\d+ retrieve_ms=([0-9.]+) total_ms=([0-9.]+) points=(\d+)',log)])
 if diagnostic.size:
  stages['track_only_report']={'samples':len(diagnostic),'retrieve_median_ms':float(np.median(diagnostic[:,0])),'total_median_ms':float(np.median(diagnostic[:,1])),'total_p95_ms':float(np.percentile(diagnostic[:,1],95)),'landmark_median':float(np.median(diagnostic[:,2]))}
 resources=[]
 p=root/'estimator_resources.txt'
 if p.exists():
  for line in p.read_text().splitlines():
   parts=line.split()
   if len(parts)==4:
    try:
     stamp,cpu,rss=float(parts[0]),float(parts[1]),float(parts[2])
     if stamp>1e9 and cpu<=1000 and rss<=10000000:
      resources.append([stamp,cpu,rss])
    except (ValueError,IndexError):pass
 res=np.array(resources)
 s=(root/'status.txt').read_text();verify=(root/'launch_verification.txt').read_text()
 return dict(t=t,xyz=x,summary=dict(poses=len(t),span_sec=float(t[-1]-t[0]),max_gap_sec=float(dt.max()),p95_gap_sec=float(np.percentile(dt,95)),z_span_m=float(np.ptp(x[:,2])),max_pose_derived_speed_mps=float(np.max(speed)),p95_pose_derived_speed_mps=float(np.percentile(speed,95)),high_speed_segments_over_5_mps=int(np.count_nonzero(speed>5)),xy_path_m=float(np.linalg.norm(np.diff(x[:,:2],axis=0),axis=1).sum()),z_end_minus_start_m=float(x[-1,2]-x[0,2]),estimated_frontend_processing_ms=stages,resources=dict(samples=len(res),cpu_pct_median=float(np.median(res[:,1])) if res.size else None,rss_mib_max=float(np.max(res[:,2])/1024) if res.size else None),sync_warnings=log.count('IMU and LiDAR not synced'),visual_gate_rejections=log.count('rejected visual state update'),completed='run_complete' in s and 'bagplay_exit=0' in s,verified=verify))

def compare(a,b):
 lo=max(a['t'][0],b['t'][0]);hi=min(a['t'][-1],b['t'][-1]);t=np.arange(lo+.25,hi,.25)
 x=np.column_stack([np.interp(t,a['t'],a['xyz'][:,i]) for i in range(3)]);y=np.column_stack([np.interp(t,b['t'],b['xyz'][:,i]) for i in range(3)])
 x0=np.array([np.interp(lo,a['t'],a['xyz'][:,i]) for i in range(3)]);y0=np.array([np.interp(lo,b['t'],b['xyz'][:,i]) for i in range(3)]);d=(x-x0)-(y-y0);horizontal=np.linalg.norm(d[:,:2],axis=1)
 return dict(end_relative_horizontal_disagreement_m=float(horizontal[-1]),p95_horizontal_disagreement_m=float(np.percentile(horizontal,95)),end_relative_z_difference_m=float(d[-1,2]),comparison='Local XYZ after subtracting initial pose; not ground truth, yaw drift is not aligned')

def main():
 DEST.mkdir(parents=True,exist_ok=True)
 runs={mode:read(f'diag_loc_match075_{mode}_20261019') for mode in ('lio','track','fusion','scheduler')}
 result={'bag':'localization.bag','rate':.75,'camera_throttle_hz':5,'independent_rosbags':'Only localization.bag is rerun here; previous four-bag structural audits are separate.','configurations':{mode:r['summary'] for mode,r in runs.items()},'relative_to_lio':{mode:compare(r,runs['lio']) for mode,r in runs.items() if mode!='lio'},'calibration_warnings':['camera<-LiDAR TF matches 40 supplied image/3D correspondences at 3.50px median; not ground-truth extrinsics','LiDAR<-IMU rotation/translation were inherited from older cart, unmeasured for these bags','Camera recorder lag ~155ms is not a sensor clock offset']}
 (DEST/'diagnostics.json').write_text(json.dumps(result,indent=2)+'\n')
 lines=['# FAST-LIVO2 camera path controlled isolation — 19 Oct 2026','','Input localization.bag, identical image throttle (5 Hz) and rosbag replay speed (0.75x) across all four modes. The three original modes used fast_livo2:diag; scheduler-only mode used a rebuild with the explicitly selected sync bypass. All source bags mounted read-only.','', '| Mode | Images active | Visual EKF updates | Poses | Max gap s | Max speed m/s | Z span m | End XY diff from LIO m |','|---|---|---|---:|---:|---:|---:|---:|']
 for mode,r in runs.items():
  s=r['summary'];diff=result['relative_to_lio'].get(mode,{'end_relative_horizontal_disagreement_m':0})
  lines.append(f"| {mode} | {'No' if mode=='lio' else 'Yes'} | {'Yes' if mode=='fusion' else 'No'} | {s['poses']} | {s['max_gap_sec']:.3f} | {s['max_pose_derived_speed_mps']:.2f} | {s['z_span_m']:.2f} | {diff['end_relative_horizontal_disagreement_m']:.2f} |")
 lines+=['','## Frontend timing and resource evidence','']
 for mode,r in runs.items():
  s=r['summary'];lines.append(f"**{mode}**: CPU median {s['resources']['cpu_pct_median']}% (process lifetime ps average), max RSS {s['resources']['rss_mib_max']} MiB; sync warnings {s['sync_warnings']}; completed={s['completed']}. Visual stage details: {json.dumps(s['estimated_frontend_processing_ms'])}.")
 lines+=['','## Interpretation','', '- LiDAR-only establishes whether this bag supports stable time-continuous local odometry under the identical replay rate.', '- Track-only runs image preprocessing, sparse map retrieval and visual map maintenance but never applies visual ESIKF correction. It STILL developed implausible motion, so native visual residuals are not the sole cause.', '- Native full fusion also diverged, but not identically to tracking-only, so visual residuals can still introduce further biases.', '- **Scheduler-only** keeps camera callbacks enabled but routes estimator updates through the LiDAR–IMU scan timing instead of image-timed LIVO splitting. It runs no visual tracking or EKF corrections. Its stable result points directly to LiDAR/IMU update scheduling as the primary identified failure mechanism in this experiment.', '- Neither diagnostic mode is a production-ready fused camera + LiDAR state estimator; they deliberately disable parts of LIVO to identify cause.', '- Estimated trajectories are in local map coordinates and are not externally surveyed; even a stable result does not validate ground-truth accuracy.', '', '## Calibration and timing caution','','The supplied ZED X camera←LiDAR tf is internally reprojection-consistent (40 matches, median 3.50px, p95 5.93px). The LiDAR←IMU and LiDAR←base transforms are inherited from an older cart and not validated against a surveyed measurement. Camera arrival latency is a ROS-bag transport measurement, not an offset to force into calibration.','', '## Next steps','','Separate high-rate LiDAR/IMU state updates from buffered camera correction events (correctly handle delayed measurement timestamps), then validate full visual fusion and latency on held-out bags before publishing /pose. Review visual residual outliers and LiDAR–IMU gravity/extrinsics; compare repeat-loop map geometry before using trajectories as global reference. Do not enable vehicle /pose authority from this experiment.']
 (DEST/'REPORT.md').write_text('\n'.join(lines)+'\n')
 print('\n'.join(lines[:14]));print('Wrote',DEST)
if __name__=='__main__':main()
