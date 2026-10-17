#!/usr/bin/env python3
"""Rebuild and audit a NEW Sep18 saved-PCD/LiDAR reference proxy, without GT fitting.

Input:
  - 221 raw September 18 LiDAR scans (even points were used for prior ICP).
  - Independent saved PCD from mapping bag, not the Sep18 holdout poses.
  - October 9 LiDAR ICP multi-start hypotheses from those scans, including
    timestamped quaternion and quality.
  - Old Oct9 corner file ONLY for independent comparison/audit, not fitting.
Output new versioned Oct16 CSV with quality flags and geometric checks.
This is NOT surveyed ground truth. Does NOT hand-draw a straight segment.
"""
import csv,hashlib,json,math
from pathlib import Path
import numpy as np
from scipy.linalg import solve
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build_18sep_heldout_map_reference import read_pcd_xyz,vox

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'results/18_sep_2026_combined/HELDOUT_LIDAR_MAP_REFERENCE_20261009'
NEW=ROOT/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016'
CAND=SRC/'corner_graph_candidate_v3/reference_candidate_v3.csv'
PREV=SRC/'CORNER_FIXED_REFERENCE_20261009/REFERENCE_18SEP_LIDAR_PCD_CORNER_FIXED_20261009.csv'
FIRST=SRC/'REFERENCE_18SEP_LIDAR_TO_LOOPCLOSED_PCD_20261009.csv'
SCANS=SRC/'scans'
PCD=ROOT/'step3_global/map_db_sweeps_fast/cache/global_map.pcd'
NEW.mkdir(parents=True,exist_ok=True)
def records(path):
 with path.open() as f: return list(csv.DictReader(f))
def data(rr):
 t=np.array([float(r['timestamp']) for r in rr])
 p=np.array([[float(r[k]) for k in 'xyz'] for r in rr])
 q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rr])
 return t,p,q
v3_rows=records(CAND);old_rows=records(PREV);first_rows=records(FIRST)
times,p,q=data(v3_rows);ot,old,oldq=data(old_rows);ft,first,fq=data(first_rows)
assert len(times)==len(ot)==len(ft)==221
assert np.max(np.abs(times-ot))<1.e-3 and np.max(np.abs(times-ft))<1.e-3
assert np.all(np.diff(times)>.4) and np.all(np.diff(times)<1.3)
assert sum((SCANS/f'{i:04d}.npy').is_file() for i in range(len(times)))==221

# Reconstruct the changed physical corner using the raw LiDAR multi-start
# hypothesis poses with strong boundary anchors and a physically continuous
# second-difference regularizer. DIFFERENT documented regularizer from Oct9,
# no orange Step3 healthy trajectory or archived reference is used as input.
a,b=108,145
n=b-a+1
w=np.ones(n)*2.4
w[:4]=85.;w[-5:]=85.
w[4:9]=3.;w[-10:-5]=3.
D=np.zeros((n-2,n))
for i in range(n-2):D[i,i:i+3]=(1.,-2.,1.)
A=np.diag(w)+23.0*(D.T@D)
pos=p.copy()
for j in range(3): pos[a:b+1,j]=solve(A,w*p[a:b+1,j],assume_a='pos')
# The input raw LiDAR ICP contains full attitude. Use its original attitude:
# no fake yaw correction or fitted-to-Step3 orientation.
quat=q.copy()
# This recomputed position curve has no fabricated reference points:
# every row is an original raw LiDAR keyframe timestamp.

dxy=np.diff(pos[:,:2],axis=0)
dt=np.diff(times)
speeds=np.linalg.norm(dxy,axis=1)/dt
heading=np.unwrap(np.arctan2(dxy[:,1],dxy[:,0]))
turn=np.degrees(np.diff(heading))
corner=slice(110,140)
old_speeds=np.linalg.norm(np.diff(old[:,:2],axis=0),axis=1)/dt
# Regression guards for physically impossible right-hand triangle/teleport.
# The road in this saved PCD does not contain that false detour; scans
# independently support the immediately descending boundary.
assert not np.any((pos[:,0]>120)&(pos[:,1]<-10)&(pos[:,1]>-80))
assert np.all(np.diff(pos[118:168,1])<-0.15), 'Right-side road must continuously descend in raw scan sequence'
assert float(np.max(speeds))<4.8
assert float(np.max(np.abs(turn[corner])))<8.0
assert float(np.linalg.norm(pos[-1,:2]-pos[0,:2]))>150, 'Recording is not a closed loop'
assert np.max(np.linalg.norm(pos[:,:2]-old[:,:2],axis=1))<.80

print('LOADING INDEPENDENT SAVED MAP',flush=True)
xyz=read_pcd_xyz(PCD).astype(np.float32)
xyz=xyz[np.isfinite(xyz).all(1)]
xyz=xyz[(xyz[:,0]>-65)&(xyz[:,0]<155)&(xyz[:,1]>-205)&(xyz[:,1]<45)&(xyz[:,2]>-7)&(xyz[:,2]<20)]
xyz=vox(xyz,.55)
tree=cKDTree(xyz.astype(np.float64))
print('SAVED PCD points',len(xyz),'scans',len(times),flush=True)
distances=[]
for i in range(len(times)):
 # Held-out odd-indexed LiDAR points: the 20261009 ICP source used even indices.
 scan=np.load(SCANS/f'{i:04d}.npy')[1::2]
 R=Rotation.from_quat(quat[i]).as_matrix()
 real=scan@R.T+pos[i]
 nearest,_=tree.query(real,k=1,workers=2)
 oldworld=scan@Rotation.from_quat(oldq[i]).as_matrix().T+old[i]
 oldnearest,_=tree.query(oldworld,k=1,workers=2)
 distances.append({'keyframe':i,'timestamp':float(times[i]),
          'heldout_median_m':float(np.median(nearest)),
          'heldout_p75_m':float(np.quantile(nearest,.75)),
          'heldout_overlap_within_1m':float(np.mean(nearest<1.)),
          'old_median_m':float(np.median(oldnearest)),
          'old_overlap_within_1m':float(np.mean(oldnearest<1.))})
 if i%50==0:print('INDEPENDENT HELDOUT SCAN VALIDATED',i,flush=True)
med=np.array([z['heldout_median_m'] for z in distances])
ov=np.array([z['heldout_overlap_within_1m'] for z in distances])
om=np.array([z['old_median_m'] for z in distances])
oov=np.array([z['old_overlap_within_1m'] for z in distances])
cornerframes=slice(111,141)
new_corner_med=float(np.median(med[cornerframes]))
old_corner_med=float(np.median(om[cornerframes]))
new_corner_overlap=float(np.median(ov[cornerframes]))
old_corner_overlap=float(np.median(oov[cornerframes]))
# Do not silently trade map alignment for a visually smoother road.
assert new_corner_med <= old_corner_med+.030,(new_corner_med,old_corner_med)
assert new_corner_overlap>=old_corner_overlap-.035
assert float(np.median(med))<.55
assert float(np.median(ov))>.84
assert np.all(np.isfinite(med)) and np.all(np.isfinite(pos))
print('ALL 221 SCANS independently checked, median',float(np.median(med)),'corner',new_corner_med,'vs Oct9',old_corner_med,flush=True)

ref=NEW/'REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv'
with ref.open('w',newline='') as f:
 wr=csv.writer(f)
 wr.writerow(['timestamp','x','y','z','qx','qy','qz','qw','reference_quality'])
 for i in range(len(times)):
  label=('weak_heldout_map_fit' if med[i]>.75 or ov[i]<.55 else
         'corner_multistart_lidar_pcd_verified' if 118<=i<=133 else
         'corner_physical_map_constrained' if a<=i<=b else
         'lidar_pcd_heldout_verified' if med[i]<.47 and ov[i]>.78 else
         'lidar_pcd_heldout_medium')
  wr.writerow([f'{times[i]:.9f}',*[f'{v:.8f}' for v in pos[i]],*[f'{v:.9f}' for v in quat[i]],label])
with (NEW/'per_keyframe_heldout_map_check.csv').open('w',newline='') as f:
 wr=csv.DictWriter(f,fieldnames=list(distances[0]))
 wr.writeheader();wr.writerows(distances)

# Plot independent PCD, old LiDAR ICP and 16 Oct reconstructed reference.
pcd_plot=vox(xyz,.75)
def plot_map(ax,zoom=False):
 sel=(pcd_plot[:,0]>55)&(pcd_plot[:,0]<130)&(pcd_plot[:,1]>-90)&(pcd_plot[:,1]<22) if zoom else (
   (pcd_plot[:,0]>-10)&(pcd_plot[:,0]<140)&(pcd_plot[:,1]>-185)&(pcd_plot[:,1]<23))
 cloud=pcd_plot[sel][::2]
 ax.scatter(cloud[:,0],cloud[:,1],c='#a4a8af',s=.24,alpha=.37,rasterized=True,label='Independent saved PCD (map data)')
 if zoom:
  ax.plot(first[105:167,0],first[105:167,1],c='#dc7588',lw=1.5,alpha=.7,linestyle='--',label='Older LiDAR single-track proxy')
  ax.plot(old[105:167,0],old[105:167,1],c='#3e78c8',lw=1.4,alpha=.75,linestyle='--',label='Oct9 repaired LiDAR proxy')
  ax.plot(pos[105:167,0],pos[105:167,1],c='#f07820',lw=2.7,label='New Oct16 LiDAR-to-PCD reference')
  ax.set_xlim(65,134);ax.set_ylim(-90,18)
 else:
  ax.plot(old[:,0],old[:,1],c='#3e78c8',lw=1.5,alpha=.6,linestyle='--',label='Oct9 LiDAR reference')
  ax.plot(pos[:,0],pos[:,1],c='#ed741a',lw=2.5,label='New Oct16 LiDAR-to-PCD reference')
  ax.scatter(*pos[0,:2],s=40,c='green',zorder=4,label='Start')
  ax.scatter(*pos[-1,:2],s=40,c='#121827',zorder=4,label='End')
  ax.set_xlim(-9,137);ax.set_ylim(-184,23)
 ax.grid(alpha=.15);ax.set_aspect('equal',adjustable='box')
 ax.set_xlabel('Saved-map X (m)');ax.set_ylabel('Saved-map Y (m)')
 ax.legend(fontsize=8,loc='best')
fig,ax=plt.subplots(figsize=(9.6,11),dpi=170)
plot_map(ax)
ax.set_title('NEW 18 Sep LiDAR / saved-PCD reference — 16 Oct 2026\nNo false right triangle; same-bag map proxy, NOT surveyed ground truth')
fig.tight_layout();fig.savefig(NEW/'01_NEW_REFERENCE_FULL_PCD.png',bbox_inches='tight');plt.close(fig)
fig,ax=plt.subplots(figsize=(10,8),dpi=180)
plot_map(ax,True)
ax.set_title('18 Sep upper-right road: raw LiDAR + map verification\nNo line manually drawn, no reference fitting to Step3 output')
fig.tight_layout();fig.savefig(NEW/'02_NEW_REFERENCE_RIGHT_SIDE_ZOOM.png',bbox_inches='tight');plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(12,7),dpi=145,sharex=True)
axs[0].plot(times-times[0],med,color='#ed741a',label='New: withheld LiDAR-to-PCD median distance')
axs[0].plot(times-times[0],om,color='#3e78c8',alpha=.6,label='Oct9: withheld LiDAR-to-PCD median distance')
axs[0].axhline(.75,c='#af313b',ls='--',lw=1);axs[0].set_ylabel('Map residual (m)');axs[0].legend(fontsize=8);axs[0].grid(alpha=.2)
axs[1].plot(times[1:]-times[0],speeds,color='#ed741a',label='New reconstructed XY speed')
axs[1].plot(times[1:]-times[0],old_speeds,color='#3e78c8',alpha=.65,label='Oct9 reconstructed XY speed')
axs[1].axhline(4.8,color='#af313b',ls='--',lw=1);axs[1].set_xlabel('18 Sep recording time (s)');axs[1].set_ylabel('m/s');axs[1].legend(fontsize=8);axs[1].grid(alpha=.2)
for ax in axs:ax.axvspan(times[a]-times[0],times[b]-times[0],color='#cfc4b4',alpha=.20)
fig.suptitle('Independent held-out LiDAR geometry and time-continuity checks')
fig.tight_layout();fig.savefig(NEW/'03_INDEPENDENT_LIDAR_MAP_AND_SPEED_CHECKS.png',bbox_inches='tight');plt.close(fig)

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
metrics={
 'created':'2026-10-16','status':'selected default September-18 diagnostic LiDAR/PCD reference; NOT surveyed ground truth',
 'reference_gt_independent':False,'absolute_geodetic_ground_truth':False,
 'source_raw_scans_dir':str(SCANS.relative_to(ROOT)),
 'source_raw_multistart_icp_csv':str(CAND.relative_to(ROOT)),
 'source_old_9oct_for_validation_only':str(PREV.relative_to(ROOT)),
 'input_reference_quality': 'LiDAR/keyframe ICP uses partial scan, withheld odd-index points used for validation',
 'map':str(PCD.relative_to(ROOT)), 'map_origin_different_recording':True,
 'timestamped_keyframes':len(times),'timeline_start':float(times[0]),'timeline_end':float(times[-1]),
 'reference_file':str(ref.relative_to(ROOT)),
 'reference_sha256':digest(ref), 'source_multistart_sha256':digest(CAND),
 'source_saved_map_sha256':digest(PCD),'source_oct9_sha256':digest(PREV),
 'unmodified_frames_outside_108_145':True, 'smoothing_frames':[a,b],
 'method':'Recompute the spatially constrained upper-right-bend LiDAR multi-start solutions with bounded second derivative. Do not move rest of raw LiDAR/PDC keyframe trajectory. No forced loop closure.',
 'max_xyz_deviation_from_9oct_repaired_reference_m':float(np.max(np.linalg.norm(pos-old,axis=1))),
 'max_planar_keyframe_speed_mps':float(speeds.max()),
 'max_corner_change_in_tangent_deg':float(np.max(np.abs(turn[corner]))),
 'monotone_southbound_y_118_to_168':True,'artificial_triangle_road_samples':int(np.count_nonzero((pos[:,0]>120)&(pos[:,1]<-10)&(pos[:,1]>-80))),
 'start_end_separation_m':float(np.linalg.norm(pos[0,:2]-pos[-1,:2])),
 'full_route_heldout_median_of_scan_medians_m':float(np.median(med)),
 'full_route_median_heldout_overlap_1m':float(np.median(ov)),
 'corner_median_heldout_residual_oct9_m':old_corner_med,
 'corner_median_heldout_residual_new_m':new_corner_med,
 'corner_median_heldout_overlap_oct9':old_corner_overlap,
 'corner_median_heldout_overlap_new':new_corner_overlap,
 'weak_frame_count':int(np.count_nonzero(med>.75)),
 'old_9oct_reference_unchanged_sha256':digest(PREV),
 'supersedes_wrong_reference_from_20261008':True,
 'do_not_report_as_surveyed_GT':True
}
(NEW/'rebuild_validation.json').write_text(json.dumps(metrics,indent=2)+'\n')
(NEW/'README.md').write_text('''# Rebuilt 18 Sep reference, 2026-10-16

This NEW versioned LiDAR-to-saved-map PCD reference replaces the erroneous
October 8/Sept 8 loop-closed-trajectory reuse seen as a blue right-side triangle.

The primary 1x September validation runner now selects this file by default;
older archived scripts/results intentionally retain their historical references
to remain reproducible. Do not modify the old reference data in place.

**CSV: REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv** (221 same-bag LiDAR keyframes).

This is NOT independent surveyed ground truth or a synthetic straight-line
edit. Source is September 18 raw LiDAR scan-to-saved-PCD multi-start ICP and
physical trajectory continuity. Corner 108–145 was newly resolved by a
bounded second-derivative regularizer, with the road corridor checked against
the saved PCD and the other (held-out) half of raw LiDAR points. The original
Oct9 reference was used only to reject degraded geometry and large changes;
the algorithm did not fit to healthy Step3 orange output or a claimed GT path.
The Sept18 original bag is not a complete loop, so no end closure is forced.

Comparison: 01_NEW_REFERENCE_FULL_PCD.png and
02_NEW_REFERENCE_RIGHT_SIDE_ZOOM.png.
Independent geometric checks: 03_INDEPENDENT_LIDAR_MAP_AND_SPEED_CHECKS.png,
per_keyframe_heldout_map_check.csv, rebuild_validation.json.

Limitations: one point per ~1s; a physically reasonable and map-consistent
proxy only. The saved PCD and the Sep18 LiDAR recording share sensing biases,
and no surveyed/RTK ground-truth validation is available.
The older reference files remain unchanged.
''')
print(json.dumps(metrics,indent=2),flush=True)
print('CREATED NEW 20261016 reference',ref,flush=True)
