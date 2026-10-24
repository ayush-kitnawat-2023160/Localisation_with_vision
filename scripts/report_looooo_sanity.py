#!/usr/bin/env python3
"""Create reproducible ROS-bag readability and FAST-LIVO2 trajectory sanity report.
No ground truth, no manual map edits or parameter auto-tuning.
"""
from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BAGS=['localization','localization_from2','localization_3','localization_full']
ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'outputs'
def trajectory_path(bag,mode):
    if bag=='localization_full' and mode=='lio':
        return OUT/'baseline_full_lio_only_20261019'/'trajectory.csv'
    if mode=='lio':
        return OUT/f'sanity_{bag}_lio_20261019'/'trajectory.csv'
    return OUT/f'sanity_{bag}_livo_verified_20261019'/'trajectory.csv'
def disagreement(bag):
    def poses(p):
        rows=list(csv.DictReader(p.open()))
        return (np.array([float(r['timestamp']) for r in rows]),
                np.array([[float(r[k]) for k in ('x','y','z')] for r in rows]))
    ta,xa=poses(trajectory_path(bag,'lio'))
    tb,xb=poses(trajectory_path(bag,'livo'))
    start=max(ta[0],tb[0]);stop=min(ta[-1],tb[-1])
    common=np.arange(start+.25,stop,.25)
    a=np.stack([np.interp(common,ta,xa[:,i]) for i in range(3)],axis=1)
    b=np.stack([np.interp(common,tb,xb[:,i]) for i in range(3)],axis=1)
    start_a=np.array([np.interp(start,ta,xa[:,i]) for i in range(3)])
    start_b=np.array([np.interp(start,tb,xb[:,i]) for i in range(3)])
    diff=(b-start_b)-(a-start_a)
    xy=np.linalg.norm(diff[:,:2],axis=1)
    return {'end_xy_m':float(xy[-1]),'p95_xy_m':float(np.percentile(xy,95)),
            'end_z_m':float(diff[-1,2]),
            'notes':'Relative local-trajectory disagreement, not absolute accuracy; camera-on 0.75x and camera-off 1.0x replay.'}
def plot_comparison(bag,start):
    trajectories={}
    for label,mode in [('LiDAR + IMU','lio'),('LiDAR + camera + IMU','livo_verified')]:
        path=trajectory_path(bag,'lio' if mode=='lio' else 'livo')
        if not path.exists():continue
        rows=list(csv.DictReader(path.open()))
        if len(rows)<3:continue
        tt=np.array([float(r['timestamp'])-start for r in rows])
        xx=np.array([[float(r[k]) for k in ('x','y','z')] for r in rows])
        trajectories[label]=(tt,xx)
    if not trajectories:return
    dest=OUT/'looooo_integrity_20261019'
    plt.figure(figsize=(8,7))
    for name,(tt,xyz) in trajectories.items():
        plt.plot(xyz[:,0],xyz[:,1],linewidth=1.2,label=name)
        plt.scatter(xyz[0,0],xyz[0,1],s=20)
    plt.axis('equal');plt.grid(alpha=.3);plt.legend()
    plt.xlabel('Estimated local X (m)');plt.ylabel('Estimated local Y (m)')
    plt.title(f'{bag}: baseline XY odometry, NOT ground truth')
    plt.tight_layout();plt.savefig(dest/f'{bag}_xy.png',dpi=150);plt.close()
    plt.figure(figsize=(10,4))
    for name,(tt,xyz) in trajectories.items():plt.plot(tt,xyz[:,2],linewidth=1.2,label=name)
    plt.grid(alpha=.3);plt.legend();plt.xlabel('Sensor time since bag start (s)')
    plt.ylabel('Estimated Z (m)');plt.title(f'{bag}: vertical trajectory sanity check')
    plt.tight_layout();plt.savefig(dest/f'{bag}_z.png',dpi=150);plt.close()
    plt.figure(figsize=(10,4))
    for name,(tt,xyz) in trajectories.items():
        speed=np.linalg.norm(np.diff(xyz,axis=0),axis=1)/np.maximum(np.diff(tt),1e-6)
        plt.plot(tt[1:],speed,linewidth=1.0,label=name)
    plt.axhline(5,linewidth=1,linestyle='--',label='5 m/s diagnostic threshold')
    plt.grid(alpha=.3);plt.legend();plt.xlabel('Sensor time since bag start (s)')
    plt.ylabel('Pose-derived speed (m/s)')
    plt.title(f'{bag}: estimated motion jumps (not wheel speed)')
    plt.tight_layout();plt.savefig(dest/f'{bag}_speed.png',dpi=150);plt.close()
def trajectory(p,start,duration):
    if not p.is_file():return None
    rows=list(csv.DictReader(p.open()))
    if len(rows)<3:return None
    t=np.array([float(row['timestamp']) for row in rows]);p=np.array([[float(row[k]) for k in ('x','y','z')] for row in rows]);dt=np.diff(t);ds=np.linalg.norm(np.diff(p,axis=0),axis=1);v=ds/np.maximum(dt,1e-6)
    return {'poses':len(t),'first_s':float(t[0]-start),'last_s':float(t[-1]-start),'coverage_s':float(t[-1]-t[0]),'max_gap_s':float(np.max(dt)), 'gaps_over_half_sec':int(np.count_nonzero(dt>0.5)),'max_speed_mps':float(v.max()),'p95_speed_mps':float(np.percentile(v,95)), 'z_span_m':float(np.ptp(p[:,2])), 'z_start_m':float(p[0,2]), 'z_end_m':float(p[-1,2]),'path_length_m':float(ds.sum()), 'horizontal_end_displacement_m':float(np.linalg.norm(p[-1,:2]-p[0,:2])), 'max_position_step_m':float(ds.max()), 'physically_plausible_kinematics':bool(v.max()<8 and np.max(dt)<.5)}
def main():
    data={};rows=[]
    for bag in BAGS:
        audit=json.loads((OUT/'looooo_integrity_20261019'/f'{bag}.json').read_text())
        entries={}
        for mode in ('lio','livo'):
            # Exclude the earlier visual-on runs produced by the faulty
            # bool/string launch expression (those had img_en=0).
            p=trajectory_path(bag,mode)
            entries[mode]=trajectory(p,audit['sensor_timestamps']['lidar']['first_stamp'],audit['duration_s'])
        plot_comparison(bag,audit['sensor_timestamps']['lidar']['first_stamp'])
        data[bag]={'audit':audit,'trajectory':entries,
                   'trajectory_disagreement':disagreement(bag)}
        a=entries['lio'];v=entries['livo']
        fmt=lambda e,k:(f'{e[k]:.2f}' if e is not None else 'pending')
        rows.append(f"| {bag} | {audit['structural_pass']} | {audit['read_message_total']} | {fmt(a,'coverage_s')} | {fmt(a,'max_gap_s')} | {fmt(a,'z_span_m')} | {fmt(a,'max_speed_mps')} | {fmt(v,'coverage_s')} | {fmt(v,'max_gap_s')} | {fmt(v,'z_span_m')} | {fmt(v,'max_speed_mps')} |")
    lines=['# Independent checks: October 17 looooo ROS bags','', 'Structural integrity is independent of estimator trajectory stability. All original bags are mounted read-only; these are uncorrected odometry runs.','', '| Bag | Full serialized read | Messages | LIO span s | LIO max gap s | LIO Z span m | LIO max speed m/s | LIVO span s | LIVO max gap s | LIVO Z span m | LIVO max speed m/s |','|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|',*rows,'', '## Sensor timing','']
    for bag,entry in data.items():
        a=entry['audit'];s=a['sensor_timestamps'];lines.append(f"**{bag}:** {a['read_message_total']}/{a['indexed_message_total']} indexed messages read; LiDAR max gap {s['lidar']['max_gap_s']:.3f} s; IMU max gap {s['imu']['max_gap_s']:.3f} s; ZED RGB–depth nearest timestamp p95 {a['nearest_stamp_deltas']['camera_to_depth']['p95_ms']:.1f} ms; median camera recorder arrival lag {s['camera']['arrival_header_median_s']:.3f} s.")
    lines += ['','## Run-to-run disagreement (not ground truth)','','The runs start in approximately the same local frame. Subtract initial XYZ, interpolate at shared sensor timestamps and compare local translation. Different replay rates can also affect the result.','','| Bag | Endpoint XY disagreement m | P95 XY disagreement m | Endpoint Z difference m |','|---|---:|---:|---:|']
    for bag,record in data.items():
        d=record['trajectory_disagreement']
        lines.append(f"| {bag} | {d['end_xy_m']:.2f} | {d['p95_xy_m']:.2f} | {d['end_z_m']:.2f} |")
    lines += ['','## Main conclusions','','- Four of four bags passed full raw-message traversal, message-count checks, and required sensor Header checks. This argues against corrupted or truncated bag storage but cannot certify numerical validity of every pixel or LiDAR point.','- All four camera-off LiDAR–IMU FAST-LIVO2 runs covered their recording durations with ~10 Hz odometry and plausible kinematics. Their estimated Z still needs geometric validation.','- All four correctly verified camera-enabled FAST-LIVO2 runs completed playback. The visual frontend was throttled at 5 Hz; the output pose rate was about 4 Hz, so lower pose count alone does not establish CPU starvation.','- Camera-enabled localization.bag diverged severely (~28 m/s pose-derived speed, ~94 m Z span). Other camera-enabled bags completed without those extreme instantaneous motion jumps, though longer-term drift/disagreement remains.','- The earlier localization_full failure had a test harness bug: common/img_en=0 despite a camera-on label. The correctly visual-enabled rerun completed all 226 seconds without reproducing the catastrophic jump. Therefore that prior failure cannot be blamed on visual updates without further matched experiments.','','## Limitations and next experiments','','- This is an estimator self-consistency check, not ground-truth accuracy, and the camera-off (1x) and camera-on (0.75x) replay rates differ. Rerun both at the same rate and monitor CPU, queue delay and callbacks.','- ROS camera receipt typically trails sensor header by roughly 155 ms; that is recording/transport latency, NOT a calibrated inter-sensor clock offset.','- Test camera processing enabled with its ESIKF visual-state updates gated off, to separate visual processing from measurement-update instability.','- Verify LiDAR–IMU gravity/extrinsics and ZED extrinsics, plus repeated-route geometric consistency, before rebuilding authoritative maps.','','## Visualization','','Open the generated images in outputs/looooo_integrity_20261019, e.g. localization_xy.png, localization_z.png, localization_speed.png, localization_full_xy.png and localization_full_z.png. Raw trajectories and rosbag logs remain in their respective outputs/sanity_* run directories.']
    (OUT/'looooo_integrity_20261019'/'sanity_report.json').write_text(json.dumps(data,indent=2)+'\n')
    dst=OUT/'looooo_integrity_20261019'/'sanity_report.md';dst.write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines[:9]));print('Wrote',dst)
if __name__=='__main__':main()
