#!/usr/bin/env python3
"""Publish a map-supported, continuity-smoothed September-18 reference proxy.

Input is experimental multi-start scan/map constrained V3; this step
regularizes only the top-right corner without forcing a false global loop.
The previous reference and source matching diagnostics stay unchanged.
"""
import csv,json
from pathlib import Path
import numpy as np
from scipy.linalg import solve
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build_18sep_heldout_map_reference import read_pcd_xyz,vox

root=Path('results/18_sep_2026_combined/HELDOUT_LIDAR_MAP_REFERENCE_20261009')
source=root/'corner_graph_candidate_v3/reference_candidate_v3.csv'
oldpath=root/'REFERENCE_18SEP_LIDAR_TO_LOOPCLOSED_PCD_20261009.csv'
oldrows=list(csv.DictReader(open(oldpath)))
rows=list(csv.DictReader(open(source)))
out=root/'CORNER_FIXED_REFERENCE_20261009'
out.mkdir(exist_ok=True)
times=np.array([float(r['timestamp']) for r in rows]);p=np.array([[float(r[k]) for k in 'xyz'] for r in rows]);old=np.array([[float(r[k]) for k in 'xyz'] for r in oldrows]);
qs=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows]);old_q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in oldrows]);

# Reconcile matching hypotheses with physically continuous derivatives, while
# strongly preserving poses outside the 108-145s localized uncertainty zone.
a,b=108,145;n=b-a+1
w=np.ones(n)*2.0
w[:4]=80.0;w[-5:]=80.0
w[4:9]=2.0;w[-10:-5]=2.0
D=np.zeros((n-2,n))
for i in range(n-2):D[i,i:i+3]=[1,-2,1]
A=np.diag(w)+20.0*(D.T@D)
adjusted=p.copy()
for xyz in range(3):adjusted[a:b+1,xyz]=solve(A,w*p[a:b+1,xyz],assume_a='pos')

# Keep mapped yaw and IMU tilt from original ICP graph hypotheses. The
# smoothing above modifies only within <0.4 m, so do not synthesize extra yaw.
def kinematic_metrics(x):
    d=np.diff(x[:,:2],axis=0)
    speed=np.linalg.norm(d,axis=1)/np.diff(times)
    ang=np.unwrap(np.arctan2(d[:,1],d[:,0]))
    return speed,np.abs(np.degrees(np.diff(ang)))

def heldout_geometry(x,rot):
    # Validate only the corrected physical section, against points that were
    # withheld from each scan-to-map ICP fit.
    cloud=read_pcd_xyz('step3_global/map_db_sweeps_fast/cache/global_map.pcd').astype(np.float32)
    cloud=cloud[np.isfinite(cloud).all(1)]
    cloud=cloud[(cloud[:,0]>50)&(cloud[:,0]<125)&(cloud[:,1]>-58)&(cloud[:,1]<25)&(cloud[:,2]>-7)&(cloud[:,2]<20)]
    cloud=vox(cloud,.55)
    tree=cKDTree(cloud.astype(float))
    metrics=[]
    for i in range(111,141):
        scan=np.load(root/'scans'/f'{i:04d}.npy')[1::2]
        pts=scan@Rotation.from_quat(rot[i]).as_matrix().T+x[i]
        dist,_=tree.query(pts,k=1,workers=2)
        metrics.append([i,float(np.median(dist)),float(np.mean(dist<1.))])
    return np.array(metrics),cloud

vold,angleold=kinematic_metrics(old)
vnew,anglenew=kinematic_metrics(adjusted)
geomold,cloud=heldout_geometry(old,old_q)
geomnew,_=heldout_geometry(adjusted,qs)
region=slice(110,140)
def section_stats(v,turn,g):
    return {'median_withheld_map_residual_m':float(np.median(g[:,1])),
            'median_withheld_1m_overlap':float(np.median(g[:,2])),
            'max_adjacent_turn_deg':float(np.max(turn[110:140])),
            'min_speed_mps':float(np.min(v[region])),
            'max_speed_mps':float(np.max(v[region]))}
metrics={'previous_reference':section_stats(vold,angleold,geomold),
         'new_reference':section_stats(vnew,anglenew,geomnew),
         'max_xy_adjustment_from_v3_m':float(np.max(np.linalg.norm(adjusted[a:b+1,:2]-p[a:b+1,:2],axis=1))),
         'max_xy_adjustment_from_previous_reference_m':float(np.max(np.linalg.norm(adjusted[a:b+1,:2]-old[a:b+1,:2],axis=1))),
         'modified_index_range':[a,b],
         'source':'18 Sep raw LiDAR map ICP multi-start / pose graph + regularized position; NOT surveyed ground truth',
         'independent_gt':False}

# Guard: never silently promote a smooth-looking path if map agreement loses
# meaningful support, or if reference starts teleporting across timestamps.
assert metrics['new_reference']['median_withheld_map_residual_m'] < metrics['previous_reference']['median_withheld_map_residual_m']+0.02
assert metrics['new_reference']['max_adjacent_turn_deg'] < 7.5
assert metrics['new_reference']['max_speed_mps'] < 2.8
assert metrics['new_reference']['min_speed_mps'] > 1.0
assert metrics['max_xy_adjustment_from_v3_m']<0.75

result=out/'REFERENCE_18SEP_LIDAR_PCD_CORNER_FIXED_20261009.csv'
with result.open('w',newline='') as f:
    writer=csv.writer(f)
    writer.writerow(['timestamp','x','y','z','qx','qy','qz','qw','reference_quality'])
    for i,r in enumerate(rows):
        label=('corner_multistart_map_verified' if 118<=i<=133 else
               'corner_transition_smoothing' if a<=i<=b else
               oldrows[i]['reference_quality'])
        writer.writerow([f'{times[i]:.9f}',*[f'{v:.8f}' for v in adjusted[i]],
                         *[f'{v:.9f}' for v in qs[i]],label])
(out/'validation.json').write_text(json.dumps(metrics,indent=2)+'\n')
with (out/'heldout_corner_quality.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['frame','timestamp','before_median_m','after_median_m','before_overlap','after_overlap'])
    for x,y in zip(geomold,geomnew):
        i=int(x[0]);w.writerow([i,times[i],x[1],y[1],x[2],y[2]])

global_map_points=read_pcd_xyz('step3_global/map_db_sweeps_fast/cache/global_map.pcd').astype(np.float32)
global_map_points=global_map_points[np.isfinite(global_map_points).all(1)]
global_map_points=global_map_points[(global_map_points[:,0]>-8)&(global_map_points[:,0]<140)&
                                   (global_map_points[:,1]>-190)&(global_map_points[:,1]<24)&
                                   (global_map_points[:,2]>-7)&(global_map_points[:,2]<20)]
global_map_points=vox(global_map_points,.72)

def plot(ax,box=False):
    map_points=cloud if box else global_map_points
    if box:
        sel=(map_points[:,0]>75)&(map_points[:,0]<114)&(map_points[:,1]>-47)&(map_points[:,1]<20)
    else:sel=np.ones(len(map_points),bool)
    aa=map_points[sel][::3]
    ax.scatter(aa[:,0],aa[:,1],s=.26,c='.72',alpha=.35,rasterized=True,label='Loop-closed saved PCD points')
    if box:
        ax.plot(old[105:145,0],old[105:145,1],'--',lw=1.6,c='tab:red',alpha=.7,label='Earlier kinked proxy')
        ax.plot(adjusted[105:145,0],adjusted[105:145,1],lw=2.7,c='tab:orange',label='Refined same-bag LiDAR proxy')
        ax.set_xlim(75,114);ax.set_ylim(-47,19)
    else:
        ax.plot(adjusted[:,0],adjusted[:,1],lw=2.7,c='tab:orange',label='Refined same-bag LiDAR proxy')
        ax.scatter(adjusted[0,0],adjusted[0,1],c='green',s=50,label='Start')
        ax.scatter(adjusted[-1,0],adjusted[-1,1],c='black',s=50,label='End')
        ax.set_xlim(-6,138);ax.set_ylim(-187,22)
    ax.set_aspect('equal','box');ax.grid(alpha=.08)
    ax.legend(loc='best',fontsize=8)
    ax.set_xlabel('Saved PCD X (m)');ax.set_ylabel('Saved PCD Y (m)')

fig,ax=plt.subplots(figsize=(9.5,10.5),dpi=180)
plot(ax,True)
ax.set_title('18 Sep upper-right bend — LiDAR scan-map candidate repair\nNo forced loop closure; not ground truth')
fig.tight_layout();fig.savefig(out/'01_corner_before_after.png');plt.close(fig)
fig,ax=plt.subplots(figsize=(10,12),dpi=165)
plot(ax)
ax.set_title('18 Sep same-bag LiDAR-to-PCD reference — refined bend\nNot ground truth')
fig.tight_layout();fig.savefig(out/'02_full_reference_on_pcd_clean.png');plt.close(fig)
print(json.dumps(metrics,indent=2))
print('SAVED',result)
