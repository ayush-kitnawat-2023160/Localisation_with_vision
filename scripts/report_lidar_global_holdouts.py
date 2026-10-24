#!/usr/bin/env python3
"""Compare three independent ScanContext/GICP held-out ROS1 replays."""
import json
import re
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/lidar_global_holdout_20261019'
MAP=ROOT/'maps/campus_full_lio_226s_20261019'
BAGS=['localization','localization_from2','localization_3']

def run_stats(name):
    path=OUT/f'global_{name}_rate075_20261019'
    status=(path/'status.txt').read_text() if (path/'status.txt').exists() else ''
    events=json.loads((path/'events.json').read_text()) if (path/'events.json').exists() else {}
    log=(path/'step3.log').read_text(errors='replace') if (path/'step3.log').exists() else ''
    accepted=re.findall(r'\[STEP3 TRACK\] ACCEPT id=(\d+) fitness=([\d.]+)',log)
    fits=[float(x[1]) for x in accepted]
    timestamps=[e['sensor_s_since_bag_start'] for e in events.get('events',[])
                if e['kind']=='accepted_map_pose']
    gaps=np.diff(timestamps) if len(timestamps)>1 else []
    return dict(bag=name,complete='run_complete' in status and 'bagplay_exit=0' in status,
                global_lock=events.get('first_lock_observed',False),
                lock_wall_s=events.get('first_lock_wall_s_after_first_lidar'),
                lock_sensor_s=events.get('first_lock_sensor_s_after_bag_start'),
                lidar_msgs=events.get('counts',{}).get('lidar_scans',0),
                odom_msgs=events.get('counts',{}).get('odom_poses',0),
                tracking_accepts=len(accepted),tracking_pose_published=len(timestamps),
                fitness_median=float(np.median(fits)) if fits else None,
                fitness_p95=float(np.percentile(fits,95)) if fits else None,
                max_tracking_pose_gap_s=float(max(gaps)) if len(gaps) else None,
                last_status=events.get('last_status'))

def main():
    mapmeta=json.loads((MAP/'metadata.json').read_text())
    cache=json.loads((MAP/'cache/metadata.json').read_text())
    results=[run_stats(b) for b in BAGS]
    gps_path=OUT/'gps_independent_sanity.json'
    gps=json.loads(gps_path.read_text()) if gps_path.exists() else {}
    data=dict(map=mapmeta,cache=cache,holdouts=results,
              gps_runtime_coarse_prior_used=False,
              gps_posthoc_sanity={b:{k:v for k,v in d.items() if k!='measurements'} for b,d in gps.items()},
              matching='Existing Step3 Scan Context retrieval, then GICP',
              odom_source='Independent LiDAR-IMU odometry saved from each heldout bag',
              accuracy_caveat='No surveyed ground truth or loop-closed reference; GPS not used by matcher.')
    (OUT/'summary.json').write_text(json.dumps(data,indent=2)+'\n')
    lines=['# LiDAR map and three independent held-out global localization tests','',
           'Map source: full localization_full.bag, LiDAR-IMU-only replay. Raw odometry is NOT loop closed.','',
           'Keyframes: '+str(mapmeta['lidar_keyframes'])+
           ', cached global points: '+str(cache['global_points'])+
           ', GPS metadata records: '+str(mapmeta['gps_messages'])+'.','',
           '| Holdout | Complete | Initial global lock | Lock sensor time s | Lock wall time s | GICP track accepts | GICP fitness median | Max track pose gap s | Final status |',
           '|---|---|---|---:|---:|---:|---:|---:|---|']
    for r in results:
        f=lambda x,dec=2:'N/A' if x is None else f'{x:.{dec}f}'
        lines.append(f"| {r['bag']} | {r['complete']} | {r['global_lock']} | {f(r['lock_sensor_s'])} | {f(r['lock_wall_s'])} | {r['tracking_accepts']} | {f(r['fitness_median'],4)} | {f(r['max_tracking_pose_gap_s'])} | {r['last_status']} |")
    lines+=['','## Time definition','',
            'Sensor time is the first accepted lock header timestamp minus first LiDAR header timestamp. Wall time is elapsed monotonic time between receiving first LiDAR callback and first accepted global lock callback. Both include retrieval, GICP, two-match consistency, playback and ROS scheduling, not only GICP solver execution.','',
            '## GPS coarse search','','NOT USED. Map GPS metadata exists; Step3 does not subscribe to NavSatFix, and its initial place retrieval searches Scan Context descriptors across the map before GICP. The GPS topic may be replayed for diagnostic purposes but does not affect matcher candidates.','',
            '## Independent, POST-HOC GPS consistency (GPS never influenced matching)','',
            '| Holdout | GPS pairs | Median map-vs-holdout GPS difference m | P95 GPS difference m |',
            '|---|---:|---:|---:|']
    for bag,d in gps.items():
        lines.append(f"| {bag} | {d['comparisons']} | {d['gps_distance_median_m']:.2f} | {d['gps_distance_p95_m']:.2f} |")
    lines+=['','This post-hoc check picks the nearest map keyframe in local XY for every accepted map pose and compares that keyframe GPS reading with the held-out GPS measurement at the pose timestamp. Both GPS measurements can drift; these differences are NOT global localization accuracy and do not demonstrate GPS-assisted search.','',
            '## Relocalization warning','','The localization_3 holdout temporarily entered RELOCALIZING around 31 seconds of sensor time. It resumed accepted tracking after a 7.26-second gap, but its relock log reports a large published-pose continuity-ramp offset of approximately 23.44 m and 133.9 degrees, with internal re-anchor 12.68 m / 9.9 degrees. A low GICP fitness alone does not make this safe. A pose-authority consumer must reject or quarantine such large corrections until route continuity and physical/map alignment are established. The next map pose also agrees only loosely with noisy GPS and requires further verification.','',
            '## Limits','','Saved LiDAR-IMU poses from the held-out bags are used as the independent local odometry input, not as ground truth. Map is from unoptimized local odometry; true global accuracy and false-match rate need independent surveyed or high-confidence reference. This test does not validate live estimator contention, sensor failure recovery or pose authority.','',
            'All run logs and ROS monitor events are in the individual global_<bag>_rate075_20261019 folders.']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))

if __name__=='__main__':main()
