#!/usr/bin/env python3
"""Analyze independent cold-start LiDAR global localization experiments."""
import csv,json,math,re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1]
OUT=R/'outputs/localization3_alternate_cold_start_20261019'
NAMES=['start_10s_14s_gps_off','start_22s_14s_gps_off','start_36s_14s_gps_off','start_45s_14s_gps_off','start_50s_14s_gps_off','start_52s_12s_gps_off','start_54s_10s_gps_off','start_22s_42s_gps_off']
ro=[]
for name in NAMES:
    path=OUT/name
    if not (path/'status.txt').exists():continue
    status=(path/'status.txt').read_text()
    evt=json.loads((path/'events.json').read_text())
    txt=re.sub(r'\x1b\[[0-9;]*[A-Za-z]','',(path/'step3.log').read_text(errors='replace'))
    locks=re.findall(r'GLOBAL LOCK map<-fast xyz=.*?keyframe=(\d+)',txt)
    verified=re.findall(r'\[STEP3\] verified id=(\d+) sc=([\d.]+) gicp=([\d.]+) consistent=(\d+/\d+)',txt)
    p=np.asarray(evt['map_pose_trace'],float)
    m={'name':name,'start_sec':int(name.split('s_')[0].split('start_')[-1]),'duration_sec':int(name.split('s_')[1].split('s_')[0]),
       'complete':'run_complete' in status and 'playback_exit=0' in status,
       'first_lock_wall_sec':evt['first_lock_wall_s_after_first_lidar'],
       'first_lock_bag_sec':evt['first_lock_sensor_s_after_bag_start'],
       'keyframe_first_lock':int(locks[0]) if locks else None,'candidate_geom':verified[:3],
       'status_end':evt['last_status'],'map_fixes':evt['counts'].get('accepted_map_pose',0),
       'status_events':[{'bag_sec':z['sensor_s_since_bag_start'],'status':z['details']} for z in evt['events'] if z['kind']=='status' and z['sensor_s_since_bag_start'] is not None and z['sensor_s_since_bag_start']>0],
       'pose_count':len(p)}
    if len(p)>1:
        dt=np.diff(p[:,0]);ds=np.linalg.norm(np.diff(p[:,1:4],axis=0),axis=1)
        keep=dt>0.03
        m.update({'largest_map_pose_step_m':float(ds.max()),'maximum_map_pose_speed_mps':float((ds[keep]/dt[keep]).max()),'pose_trace_start_bag_sec':float(p[0,0]),'pose_trace_end_bag_sec':float(p[-1,0])})
    ro.append((m,p))
# Compare cold starts by their absolute estimated map coordinates on the SAME bag timestamps.
def overlap(a,b):
    t0=max(a[0,0],b[0,0]);t1=min(a[-1,0],b[-1,0]);
    if t1-t0<0.5:return None
    t=np.arange(t0,t1,0.1)
    x=np.column_stack([np.interp(t,a[:,0],a[:,i]) for i in (1,2,3)])
    y=np.column_stack([np.interp(t,b[:,0],b[:,i]) for i in (1,2,3)])
    dis=np.linalg.norm(x-y,axis=1)
    return {'overlap_sec':float(t1-t0),'median_position_disagreement_m':float(np.median(dis)),'p95_disagreement_m':float(np.percentile(dis,95)),'max_disagreement_m':float(dis.max())}
lookup={m['name']:p for m,p in ro}
long=lookup['start_22s_42s_gps_off']
pairs={}
for name in ['start_36s_14s_gps_off','start_45s_14s_gps_off','start_50s_14s_gps_off','start_52s_12s_gps_off','start_54s_10s_gps_off']:
    pairs[name]=overlap(long,lookup[name])
# GPS distances: associated GPS from nearest XY map keyframe is an independent weak check, NOT survey truth.
map_rows=list(csv.DictReader((R/'maps/campus_full_lio_226s_20261019/keyframes.csv').open()))
map_xy=np.array([[float(x[k]) for k in ('x','y')] for x in map_rows]);gps_rows=list(csv.DictReader((R/'outputs/camera_diagnostics_20261019/localization_3_gps.csv').open()))
gps_t=np.array([float(g['timestamp']) for g in gps_rows]);first_stamp=float(json.load((R/'outputs/looooo_integrity_20261019/localization_3.json').open())['sensor_timestamps']['lidar']['first_stamp'])
def gps_compare(name,t):
    p=lookup[name]
    if t<p[0,0] or t>p[-1,0]:return None
    xy=np.array([np.interp(t,p[:,0],p[:,i]) for i in (1,2)])
    i=int(np.argmin(np.linalg.norm(map_xy-xy,axis=1)));row=map_rows[i]
    g=gps_rows[int(np.argmin(np.abs(gps_t-(first_stamp+t))))]
    lat=float(g['latitude']);lon=float(g['longitude'])
    delta=math.hypot((float(row['gps_lat'])-lat)*111132.0,(float(row['gps_lon'])-lon)*111320.0*math.cos(math.radians(lat)))
    return {'at_bag_sec':t,'estimated_map_xy_m':xy.tolist(),'nearest_map_keyframe':i,'gps_distance_to_keyframe_m':delta,'gps_covariance_xx_m2':float(g['covariance_xx'])}
comparison={name:[gps_compare(name,t) for t in (62.,63.)] for name in ('start_22s_42s_gps_off','start_54s_10s_gps_off')}
report={'runs':[m for m,p in ro],'cross_start_comparison_against_22s_full_suffix':pairs,'late_gps_map_keyframe_comparison':comparison,'limitations':['No independently measured global pose, so map hypothesis correctness is unproven.','The replayed FAST-LIVO2 odometry is not ground truth.','The map is raw, unoptimized, no physical extrinsic calibration verification.','A run that ends immediately after its first lock cannot establish long-term stability.','The last relocalizing status at 63-64s may reflect insufficient post-end observations.']}
(OUT/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
plt.figure(figsize=(8,7))
for name in ['start_22s_42s_gps_off','start_36s_14s_gps_off','start_45s_14s_gps_off','start_50s_14s_gps_off','start_52s_12s_gps_off','start_54s_10s_gps_off']:
    p=lookup[name];plt.plot(p[:,1],p[:,2],label=name.replace('_gps_off','').replace('start_','start '),linewidth=1.7 if name=='start_22s_42s_gps_off' else 1.1)
    plt.scatter(p[0,1],p[0,2],s=16)
plt.axis('equal');plt.xlabel('Unsurveyed map X (m)');plt.ylabel('Unsurveyed map Y (m)');plt.title('Independent cold starts: map-frame trajectories (not ground truth)');plt.legend(loc='best',fontsize=8);plt.grid(alpha=.3);plt.tight_layout();plt.savefig(OUT/'alternate_cold_start_xy.png',dpi=160);plt.close()
plt.figure(figsize=(10,5))
for name in ['start_22s_42s_gps_off','start_36s_14s_gps_off','start_45s_14s_gps_off','start_50s_14s_gps_off','start_52s_12s_gps_off','start_54s_10s_gps_off']:
    p=lookup[name];plt.plot(p[:,0],p[:,2],label=name.replace('_gps_off','').replace('start_','start '))
plt.xlabel('Seconds into localization_3.bag');plt.ylabel('Estimated map Y (m)');plt.title('Branch disagreement near the end of bag');plt.legend(fontsize=8);plt.grid(alpha=.3);plt.tight_layout();plt.savefig(OUT/'alternate_cold_start_map_y_time.png',dpi=160);plt.close()
lines=['# LiDAR global localization at alternative poses in localization_3.bag','', '## Method', '', 'Each row uses a NEW matcher process and a separate ROS master. `rosbag play --start <seconds> --duration <seconds>` begins from a different sensor acquisition point. No previous map←odom/global lock is supplied. Camera disabled; GPS candidate prior **disabled**. Same unoptimized LiDAR map (`campus_full_lio_226s_20261019`, 319 keyframes), saved independent LiDAR–IMU odometry, Scan Context + GICP thresholds, and 0.75× replay as previous holdouts.', '', 'Map coordinate output is a hypothesis, not surveyed truth. `first_lock_wall_sec` measures wall time after first delivered LiDAR scan, including replay speed, matching, and two-consistency requirement.', '', '## Recorded cold-start results', '', '| Replay start | Duration | First lock wall time | Locked keyframe | Accepted map poses | End state | Max pose-derived speed |', '|---:|---:|---:|---:|---:|---|---:|']
for m,p in ro:
    lines.append(f"| {m['start_sec']}s | {m['duration_sec']}s | {m['first_lock_wall_sec']:.2f}s | {m['keyframe_first_lock']} | {m['map_fixes']} | {m['status_end']} | {m.get('maximum_map_pose_speed_mps',float('nan')):.2f} m/s |")
lines+=['', '## Are independently initialized map poses consistent?', '', 'The full-suffix replay starting at 22 s covers the late recording. Compared with that run over overlapping time windows:','','| Comparison run start | Overlap time | Median 3D position difference | p95 difference |', '|---|---:|---:|---:|']
for name,m in pairs.items():
    if m: lines.append(f"| {name.split('s_')[0].replace('start_','')}s | {m['overlap_sec']:.1f}s | {m['median_position_disagreement_m']:.2f} m | {m['p95_disagreement_m']:.2f} m |")
lines+=['', '**Interpretation:** Independent 22s, 36s, 45s, 50s, 52s starts often select a very similar branch in common intervals, while the 54s near-end cold start selects keyframe 290 and diverges by roughly 20 m from the continuously tracked branch. The old whole-bag run also ultimately searches keyframes 292–294 near the end. This is evidence of competing map-segment associations, not proof of which is correct.', '', '## Independent weak GPS cross-check at t≈63s', '']
for name,items in comparison.items():
    z=items[-1];lines.append(f"- {name}: map pose XY {tuple(round(x,2) for x in z['estimated_map_xy_m'])}, nearest map keyframe {z['nearest_map_keyframe']}, separation between mapped-keyframe GPS and heldout GPS {z['gps_distance_to_keyframe_m']:.2f} m.")
lines+=['', 'This comparison is coarse and does not validate absolute accuracy: keyframes are not continuously distributed and both recordings share GNSS systematics and map distortions. GPS was not fed to the matcher in any of these runs.', '', '## Outcome and limitations', '', '- Alternate *cold initialization* is possible at every tested point. It is NOT necessary to reuse the very first global pose selected at the beginning of localization_3.bag.', '- The 22s-to-end, 50s-to-end and 52s-to-end replays still end in `RELOCALIZING: verifying` around 63–64s. The final segment may be censored by bag termination, leaving too few further scans for the two-match gate.', '- The 54s start locks to a geographically different map branch but has only a few accepted updates before recording ends. Do not treat this as validated long-term tracking.', '- Most important next engineering step: identify the true road branch using an independently checked map/pose, log top-N Scan Context+GICP alternatives and their geometry, and fail-closed when two geometrically viable locations disagree by many meters.', '- Physical LiDAR–IMU/base extrinsics, 23m map Z variation, and full 3D loop-closure remain unresolved. Do not enable `/pose` for autonomous driving.', '', '## Artifacts', '', '- `summary.json` detailed case metrics and cross-start comparisons.', '- `alternate_cold_start_xy.png` and `alternate_cold_start_map_y_time.png` visualization.', '- Each `start_*` directory includes `step3.log`, `events.json`, `status.txt`, and rosbag replay record.', '- Re-run with `./scripts/run_localization3_cold_starts.sh <start> <duration>`; default GPS mode is off.']
(OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines[8:25]));print('outputs:',OUT)
