#!/usr/bin/env python3
"""Compare run-by-run GPS-soft and LiDAR-only heldouts without claiming accuracy."""
import json, csv, re
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1]
ROOT=R/'outputs/gps_soft_prior_20261019'
BASE=json.loads((R/'outputs/lidar_global_holdout_20261019/run_summary.json').read_text())
BASES={r['bag'].removesuffix('.bag'):r for r in BASE['heldout_runs']}
def summarize(bag,mode):
    d=ROOT/f'{bag}_{mode}_20261019'
    if not (d/'events.json').exists():return {'bag':bag,'mode':mode,'available':False}
    evt=json.loads((d/'events.json').read_text());p=np.array(evt.get('map_pose_trace',[]),float)
    s=(d/'status.txt').read_text() if (d/'status.txt').exists() else ''
    statuses=[(e.get('sensor_s_since_bag_start'),e.get('details')) for e in evt['events'] if e['kind']=='status']
    log=(d/'step3.log').read_text(errors='replace')
    num=len(p);stat={'samples':num}
    if num>1:
        dt=np.diff(p[:,0]);xyz=np.diff(p[:,1:4],axis=0)
        dist=np.linalg.norm(xyz,axis=1)
        good=dt>.03
        speed=dist[good]/dt[good];yaw=np.unwrap(2*np.arctan2(p[:,6],p[:,7]))
        dyaw=np.abs(np.diff(yaw))
        stat.update({'timestamp_span_sec':float(p[-1,0]-p[0,0]),'largest_step_m':float(dist.max()),'largest_speed_mps_dt_gt_30ms':float(speed.max()) if len(speed) else None,'p95_speed_mps_dt_gt_30ms':float(np.percentile(speed,95)) if len(speed) else None,'steps_over_3m':int(sum(dist>3)),'steps_over_20deg_yaw':int(sum(dyaw>np.deg2rad(20))),'nonmonotonic_timestamps':int(sum(dt<=0))})
    return {'bag':bag,'mode':mode,'available':True,'completed':'run_complete' in s and 'playback_exit=0' in s,'first_lock_wall_s_after_first_lidar':evt.get('first_lock_wall_s_after_first_lidar'),'first_lock_sensor_s_after_bag_start':evt.get('first_lock_sensor_s_after_bag_start'),'final_status':evt.get('last_status'),'status_transitions':statuses,'map_pose_continuity':stat,'gps_ranking_log_events':len(re.findall(r'\[STEP3 GPS\] reacquire',log)), 'initial_gps_scored_candidates':len(re.findall(r'gps_distance=',log)),'large_reanchor_warnings':len(re.findall(r'immediate internal re-anchor',log)),'reference':{'lock_wall_s':BASES[bag]['first_lock_wall_after_first_lidar_sec'],'final_status':BASES[bag]['final_status']}}
def main():
    runs=[summarize(b,m) for b in ['localization_3','localization','localization_from2'] for m in ['on']]
    (ROOT/'validation_summary.json').write_text(json.dumps(runs,indent=2)+'\n')
    lines=['# GPS soft-prior LiDAR localization: held-out replays','', 'The raw 226s LiDAR map, 319 keyframes, was unchanged. Same 0.75x bag rate and Step3 geometry thresholds as baseline. Only the soft GPS preference differs. All coordinates are un-surveyed. GPS is never accepted as a pose measurement.','', '| Bag | Replay completed | First lock wall s (baseline → GPS) | Final status (baseline → GPS) | GPS-ranked reacquire events | Largest published map step m |', '|---|---|---|---|---:|---:|']
    for r in runs:
        if not r['available']:continue
        c=r['map_pose_continuity'];lines.append(f"| {r['bag']} | {r['completed']} | {r['reference']['lock_wall_s']:.2f} → {r['first_lock_wall_s_after_first_lidar']:.2f} | {r['reference']['final_status']} → {r['final_status']} | {r['gps_ranking_log_events']} | {c.get('largest_step_m','—')} |")
        lines.append(f"\n{r['bag']}: map pose continuity {c}; status {r['status_transitions']}.")
    lines+=['','GPS NavSatStatus 0 is STATUS_FIX in ROS1, not NO_FIX; covariance/staleness gates determine use. GPS is a bounded ranking preference inside Scan Context candidate retrieval; invalid/unavailable GPS falls back to unmodified LiDAR-only retrieval. GICP, physical/motion consistency, and two-match global confirmation remain mandatory.','', 'A published map-pose jump is not a localization accuracy measurement; no surveyed pose ground truth exists. Never enable autonomous /pose on this evidence.']
    (ROOT/'REPORT.md').write_text('\n'.join(lines)+'\n');print('\n'.join(lines[:14]))
if __name__=='__main__':main()
