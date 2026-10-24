#!/usr/bin/env python3
"""Vertical self-consistency and GPS *proxy* checks, not altitude ground truth.

Input CSV lidar odometry in local XYZ and GPS time/lat/lon/altitude/status/covariance.
"""
from pathlib import Path
import argparse,csv,json, numpy as np
from scipy.spatial import cKDTree

def read_csv(p):return list(csv.DictReader(Path(p).open()))
def main():
 a=argparse.ArgumentParser();a.add_argument('--trajectory',required=True);a.add_argument('--gps',required=True);a.add_argument('--out',required=True);q=a.parse_args()
 traj=read_csv(q.trajectory);gps=read_csv(q.gps)
 t=np.array([float(v['timestamp']) for v in traj]);p=np.array([[float(v[k]) for k in ('x','y','z')] for v in traj]);
 g=np.array([[float(v[k]) for k in ('timestamp','latitude','longitude','altitude')] for v in gps]);good=np.isfinite(g).all(axis=1);g=g[good];g=g[np.r_[True,np.diff(g[:,0])>1e-5]]
 stamp=t[(t>=g[0,0])&(t<=g[-1,0])];zs=p[(t>=g[0,0])&(t<=g[-1,0]),2];ga=np.interp(stamp,g[:,0],g[:,3]);# reset relative initial independent of unknown reference height
 liodar_rel=zs-zs[0]; gps_rel=ga-ga[0]
 delta=liodar_rel-gps_rel
 # XY near-revisits after at least 20 seconds provide a local map self-consistency check.
 tree=cKDTree(p[:,:2]); pairs=tree.query_pairs(r=2.0,output_type='ndarray')
 if len(pairs):pairs=pairs[np.abs(t[pairs[:,0]]-t[pairs[:,1]])>20.0]
 # Avoid making ~millions of correlated pairs dominate; deterministic max sample.
 if len(pairs)>15000:pairs=pairs[np.linspace(0,len(pairs)-1,15000).astype(int)]
 err=np.abs(p[pairs[:,0],2]-p[pairs[:,1],2]) if len(pairs) else np.array([])
 result=dict(trajectory=Path(q.trajectory).name,gps_file=Path(q.gps).name,
  lio_z_span_m=float(np.ptp(p[:,2])),lio_end_relative_z_m=float(p[-1,2]-p[0,2]),
  gps_altitude_span_m=float(np.ptp(g[:,3])),gps_end_relative_altitude_m=float(g[-1,3]-g[0,3]),
  relative_lio_vs_gps_altitude_end_diff_m=float(delta[-1]),
  relative_lio_vs_gps_altitude_median_absolute_diff_m=float(np.median(abs(delta))),
  gps_lio_relative_altitude_correlation=float(np.corrcoef(liodar_rel,gps_rel)[0,1]) if len(stamp)>3 else None,
  revisits_xy_within_2m_after_20s_pairs=int(len(pairs)),
  revisit_z_median_absolute_difference_m=float(np.median(err)) if len(err) else None,
  revisit_z_p95_absolute_difference_m=float(np.percentile(err,95)) if len(err) else None,
  caveats=['GPS altitude is not an independent surveyed reference; multipath and covariance can be large.',
           'Near-repeat pairs based on estimated XY alone are not guaranteed to be the same surface or elevation.',
           'The LiDAR/base rotation was inherited from the older mount, not physically validated for current recordings.',
           'Neither global accuracy nor true campus grade is established.'])
 Path(q.out).parent.mkdir(parents=True,exist_ok=True);Path(q.out).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
