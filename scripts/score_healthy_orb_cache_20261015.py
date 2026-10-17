#!/usr/bin/env python3
"""Conservative diagnostic gate for opt-in exact ORB map cache (90s, 1x).

Uses a same-bag loop-closed proxy, not independent ground truth. Because ORB
cache is startup-only here, never claim it reduces sustained localisation CPU.
"""
import csv
import json
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parents[1]
b=root/'results/camera_lidar_imu_gps/HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015'
ref=list(csv.DictReader((root/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv').open()))
ref_t=np.array([float(r['timestamp']) for r in ref]); origin=ref_t[0]
ref_xyz=np.array([[float(r[k]) for k in 'xyz'] for r in ref])

def read(path):
    rows=list(csv.DictReader(path.open()))
    t=np.array([float(z['field.header.stamp'])*1e-9 for z in rows])
    xyz=np.array([[float(z['field.pose.position.'+c]) for c in 'xyz'] for z in rows])
    return t,xyz

def score(name):
    folder=b/name
    t,xyz=read(folder/'global_pose.csv')
    select=(t>=origin)&(t<origin+90)
    t=t[select];xyz=xyz[select]
    assert len(t)>700 and t[-1]-origin>85 and np.all(np.diff(t)>0) and np.isfinite(xyz).all()
    gt=np.column_stack([np.interp(t,ref_t,ref_xyz[:,k]) for k in range(3)])
    error=np.linalg.norm(xyz-gt,axis=1)
    step=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
    g,_=read(folder/'gicp_map_pose.csv')
    g=g[(g>=origin)&(g<origin+90)]
    assert len(g)>50
    lines=(folder/'launch.log').read_text(errors='replace')
    return {'reference_proxy_rmse3d_m':round(float(np.sqrt(np.mean(error**2))),3),
            'loop_40_60s_proxy_rmse3d_m':round(float(np.sqrt(np.mean(error[(t>=origin+40)&(t<origin+60)]**2))),3),
            'max_published_step_m':round(float(np.max(step)),3),
            'gicp_max_gap_s':round(float(np.max(np.diff(g))),3),
            'gicp_accepted_fixes':len(g),
            'fast_pose_gap_events':lines.count('[STEP3 FAST-GUARD] FAST pose gap'),
            'published_points':len(t),
            'elapsed_s':round(float(t[-1]-origin),3)}

old=score('HEALTHY_BASELINE_90S')
new=score('HEALTHY_ORB_CACHE_90S')
quality=(new['reference_proxy_rmse3d_m']<=old['reference_proxy_rmse3d_m']+.10 and
         new['loop_40_60s_proxy_rmse3d_m']<=old['loop_40_60s_proxy_rmse3d_m']+.10 and
         new['max_published_step_m']<=old['max_published_step_m']+.10 and
         new['gicp_max_gap_s']<=max(3.8,old['gicp_max_gap_s']+.25) and
         new['gicp_accepted_fixes']>=old['gicp_accepted_fixes']-3 and
         new['fast_pose_gap_events']==0)
text=(b/'HEALTHY_ORB_CACHE_90S/launch.log').read_text(errors='replace')
launched='orb_retrieval_cache_path: /visual_orb_cache' in text
out={'reference_independent_ground_truth':False,
     'tested': 'first 90 seconds of healthy camera_lidar_imu_gps.bag at 1x',
     'cache_configured_in_live_launch':launched,
     'baseline':old,'cached':new,'quality_gate_passed':bool(quality),
     'note':'Exact ORB descriptor cache is startup-only for healthy launch with visual_place_enable_on_healthy=false. No failover acceptance claimed.'}
(b/'orb_cache_healthy_90s_gate.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
