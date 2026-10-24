#!/usr/bin/env python3
"""Non-authoritative GPS cross-check of ScanContext/GICP poses.

GPS is evaluated AFTER matching and never sent to the Step3 matcher.
Compares each accepted map pose to the nearest exported map keyframe's GPS.
"""
import csv
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/lidar_global_holdout_20261019'
MAP=ROOT/'maps/campus_full_lio_226s_20261019'

def main():
    ref=list(csv.DictReader((MAP/'keyframes.csv').open()))
    mxy=np.array([[float(r['x']),float(r['y'])] for r in ref])
    all_data={}
    for bag in ('localization','localization_from2','localization_3'):
        gps=list(csv.DictReader((ROOT/'outputs/camera_diagnostics_20261019'/f'{bag}_gps.csv').open()))
        gt=np.array([float(r['timestamp']) for r in gps])
        evt=json.loads((OUT/f'global_{bag}_rate075_20261019'/'events.json').read_text())
        start=json.loads((ROOT/'outputs/looooo_integrity_20261019'/f'{bag}.json').read_text())['sensor_timestamps']['lidar']['first_stamp']
        measured=[]
        for e in evt['events']:
            if e['kind']!='accepted_map_pose':continue
            xy=np.array(e['details'][:2])
            mid=int(np.argmin(np.linalg.norm(mxy-xy,axis=1)))
            row=ref[mid]
            if not row['gps_lat'] or not row['gps_lon']:continue
            ts=start+e['sensor_s_since_bag_start']
            gi=int(np.argmin(np.abs(gt-ts)))
            if abs(gt[gi]-ts)>.5:continue
            obs=gps[gi]
            lat=float(obs['latitude']);lon=float(obs['longitude'])
            nlat=float(row['gps_lat']);nlon=float(row['gps_lon'])
            horizontal=math.hypot((nlat-lat)*111132.,(nlon-lon)*111320.*math.cos(math.radians(lat)))
            measured.append({'sensor_time_from_start_s':e['sensor_s_since_bag_start'],
                             'selected_nearest_keyframe_id':mid,
                             'map_nearest_xy_distance_m':float(np.linalg.norm(mxy[mid]-xy)),
                             'distance_between_unfiltered_gps_fixes_m':horizontal,
                             'query_gps_status':int(obs['fix_status'])})
        distances=[v['distance_between_unfiltered_gps_fixes_m'] for v in measured]
        all_data[bag]={'comparisons':len(measured),
                       'gps_distance_median_m':float(np.median(distances)) if distances else None,
                       'gps_distance_p95_m':float(np.percentile(distances,95)) if distances else None,
                       'gps_distance_max_m':float(max(distances)) if distances else None,
                       'gps_was_runtime_matcher_input':False,
                       'measurements':measured}
        s=all_data[bag]
        print(bag,s['comparisons'],s['gps_distance_median_m'],s['gps_distance_p95_m'])
    (OUT/'gps_independent_sanity.json').write_text(json.dumps(all_data,indent=2)+'\n')

if __name__=='__main__':
    main()
