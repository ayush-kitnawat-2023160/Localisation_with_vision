#!/usr/bin/env python3
"""Experimental physically continuous September 18 corner reference repair.

Select timestamp-matched raw-LiDAR->map ICP hypotheses using global
motion/curvature consistency; validate on withheld scan points. Does not
change the previously published reference until diagnostics pass.
"""
import csv,json,math
from pathlib import Path
from collections import defaultdict
import numpy as np
from scipy.linalg import solve
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build_18sep_heldout_map_reference import read_pcd_xyz,vox

root=Path('results/18_sep_2026_combined/HELDOUT_LIDAR_MAP_REFERENCE_20261009')
out=root/'corner_graph_candidate_v3';out.mkdir(exist_ok=True)
R=list(csv.DictReader(open(root/'REFERENCE_18SEP_LIDAR_TO_LOOPCLOSED_PCD_20261009.csv')))
P=np.array([[float(r[k]) for k in 'xyz'] for r in R]);quats=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in R]);ts=np.array([float(r['timestamp']) for r in R]);
cands=defaultdict(list)
for row in csv.DictReader(open(root/'corner_multistart_icp_candidates.csv')):
 i=int(row['frame']); x=np.array([float(row[k]) for k in 'xyz']);yaw=float(row['yaw']);med=float(row['median']);ov=float(row['overlap']);
 if np.linalg.norm(x[:2]-P[i,:2])<4.5 and med<.65 and ov>.72:
  cands[i].append((x,yaw,med,ov,row['seed']))
start,end=117,134
for i in range(start+1,end):
 if not cands[i]:raise RuntimeError(f'No candidates at {i}')
 # Within 0.1 m, same pose from different seeds: keep better map alignment.
 cands[i].sort(key=lambda a:a[2]);uniq=[]
 for c in cands[i]:
  if any(np.linalg.norm(c[0][:2]-b[0][:2])<.20 and abs(c[1]-b[1])<.08 for b in uniq):continue
  uniq.append(c)
 cands[i]=uniq[:13]
yaws=np.unwrap(Rotation.from_quat(quats).as_euler('ZYX')[:,0]);
for i in (start,end):cands[i]=[(P[i],yaws[i],.29,.97,'anchor')]

# SE2 kinematics: cart advances roughly 1.9 m/s through this corner.
def unary(q):
 x,y,med,ov,seed=q
 return 5.0*max(0,med-.29)**2+2.0*max(0,.99-ov)**2

def edge(i,q,j,p):
 a,y1,*_=q;b,y2,*_=p
 d=b[:2]-a[:2];distance=np.linalg.norm(d);dt=ts[j]-ts[i]
 # Avoid stationary aliasing and artificial leaps.
 speed_target=1.88*dt
 val=2.2*(distance-speed_target)**2
 if distance>2.65*dt:val+=15*(distance-2.65*dt)**2
 if distance<.95*dt:val+=5*(.95*dt-distance)**2
 # Tangent should be roughly aligned with vehicle heading but allow approx
 # 10-degree body-vs-trajectory offset from map extrinsics and slight slips.
 tan=math.atan2(d[1],d[0]);orient=(y1+y2)/2+math.radians(9.0)
 angle=(tan-orient+math.pi)%(2*math.pi)-math.pi
 val+=1.5*(angle/.30)**2
 da=(y2-y1+math.pi)%(2*math.pi)-math.pi
 val+=0.28*(da/.085)**2
 return val

def bend(q,p,r):
 a=q[0][:2];b=p[0][:2];c=r[0][:2]
 v1=(b-a)/(ts[p_idx]-ts[q_idx]) if False else b-a
 v2=c-b
 return 2.0*np.sum((v2-v1)**2)

# Second-order Viterbi over per-frame candidate pose hypotheses.
c1=cands[start];c2=cands[start+1];dp={}
for a_idx,a in enumerate(c1):
 for b_idx,b in enumerate(c2):dp[(a_idx,b_idx)]=(unary(a)+unary(b)+edge(start,a,start+1,b), [a_idx,b_idx])
for i in range(start+2,end+1):
 nxt={}
 for (ip,ic),(cost,seq) in dp.items():
  a=cands[i-2][ip];b=cands[i-1][ic]
  for j,c in enumerate(cands[i]):
   value=cost+unary(c)+edge(i-1,b,i,c)+bend(a,b,c)
   key=(ic,j)
   if key not in nxt or value<nxt[key][0]:nxt[key]=(value,seq+[j])
 dp=nxt
best_cost,choice=min(dp.values(),key=lambda z:z[0]);idx=np.arange(start,end+1)
selected=[cands[i][k] for i,k in zip(idx,choice)]
print('DP total cost',round(best_cost,2),'candidate counts',[len(cands[i]) for i in idx],flush=True)
for i,c in zip(idx,selected):print('choice',i,'t',round(ts[i]-ts[0],2),'xy',np.round(c[0][:2],2),'yaw',round(math.degrees(c[1]),1),'median',round(c[2],3),'ov',round(c[3],2),'source',c[4],flush=True)

# Regularized global small-region smoothing, only 112..139, anchors outside
# transition remain fixed. Penalize curvature, not the absolute coordinate.
Q=P.copy();SY=np.unwrap(yaws.copy())
for i,c in zip(idx,selected):Q[i]=c[0];SY[i]=c[1]+2*math.pi*round((SY[i]-c[1])/(2*math.pi))
a,b=112,139;n=b-a+1
w=np.array([20.0 if (i<=117 or i>=134) else 4.0 for i in range(a,b+1)])
# Candidate global map matches carry uncertainty, never hard overwrite.
T=Q[a:b+1].copy();D2=np.zeros((n-2,n))
for i in range(n-2):D2[i,i:i+3]=[1,-2,1]
A=np.diag(w)+2.1*(D2.T@D2)
for col in range(3):Q[a:b+1,col]=solve(A,w*T[:,col],assume_a='pos')
# preserve map yaw interpolation; small smooth correction only
Ay=np.diag(w)+1.6*(D2.T@D2);SY[a:b+1]=solve(Ay,w*SY[a:b+1],assume_a='pos')
# Fully preserve validated reference outside the corner region
newquats=quats.copy()
for i in range(a,b+1):
 old_R=Rotation.from_quat(quats[i]);old_y=old_R.as_euler('ZYX')[0]
 tilt=Rotation.from_euler('z',-old_y)*old_R
 newquats[i]=(Rotation.from_euler('z',SY[i])*tilt).as_quat()

# Truly held-out scan subset (other half of each LiDAR keyframe).
mapxyz=read_pcd_xyz('step3_global/map_db_sweeps_fast/cache/global_map.pcd').astype(np.float32)
mapxyz=mapxyz[np.isfinite(mapxyz).all(1)];mapxyz=mapxyz[(mapxyz[:,0]>50)&(mapxyz[:,0]<125)&(mapxyz[:,1]>-58)&(mapxyz[:,1]<25)&(mapxyz[:,2]>-7)&(mapxyz[:,2]<20)];mapxyz=vox(mapxyz,.55)
tree=cKDTree(mapxyz.astype(float))
def geometry(x,quats):
 metrics=[]
 for i in range(112,140):
  cloud=np.load(root/'scans'/f'{i:04d}.npy')[1::2]
  trans=cloud@Rotation.from_quat(quats[i]).as_matrix().T+x[i]
  dist,_=tree.query(trans,k=1,workers=2)
  metrics.append((float(np.median(dist)),float(np.mean(dist<1))))
 return np.array(metrics)
base=geometry(P,quats);cand=geometry(Q,newquats)

def kin(x):
 d=np.diff(x[:,:2],axis=0); speed=np.linalg.norm(d,axis=1)/np.diff(ts)
 heading=np.unwrap(np.arctan2(d[:,1],d[:,0]));turn=np.diff(heading)
 region=slice(112,139)
 return speed[region],np.degrees(turn[region])
old_v,old_dyaw=kin(P);new_v,new_dyaw=kin(Q)
print('REGION heldout map median OLD/NEW',round(np.median(base[:,0]),4),round(np.median(cand[:,0]),4),'overlap OLD/NEW',round(np.median(base[:,1]),4),round(np.median(cand[:,1]),4))
print('REGION speed min max OLD/NEW',old_v.min(),old_v.max(),new_v.min(),new_v.max(),'turn max abs OLD/NEW',np.max(np.abs(old_dyaw)),np.max(np.abs(new_dyaw)),flush=True)

with (out/'reference_candidate_v3.csv').open('w',newline='') as f:
 wri=csv.writer(f);wri.writerow(['timestamp','x','y','z','qx','qy','qz','qw','reference_quality'])
 for i,r in enumerate(R):
  label='experimental_multistart_posegraph' if a<=i<=b else r['reference_quality']
  wri.writerow([f'{ts[i]:.9f}',*[f'{n:.7f}' for n in Q[i]],*[f'{n:.9f}' for n in newquats[i]],label])
with (out/'validation.json').open('w') as f:json.dump({'method':'multistart LiDAR scan-map candidates, motion and curvature constrained DP, regularized positions; experimental','section':[a,b],'old_heldout_median_m':float(np.median(base[:,0])),'new_heldout_median_m':float(np.median(cand[:,0])),'old_heldout_overlap_1m':float(np.median(base[:,1])),'new_heldout_overlap_1m':float(np.median(cand[:,1])),'old_max_angle_change_deg':float(np.max(np.abs(old_dyaw))),'new_max_angle_change_deg':float(np.max(np.abs(new_dyaw))),'old_min_speed_mps':float(old_v.min()),'new_min_speed_mps':float(new_v.min()),'old_max_speed_mps':float(old_v.max()),'new_max_speed_mps':float(new_v.max()),'is_validated_gt':False},f,indent=2)
fig,ax=plt.subplots(figsize=(9,10),dpi=180)
mapplot=mapxyz[(mapxyz[:,0]>74)&(mapxyz[:,0]<115)&(mapxyz[:,1]>-45)&(mapxyz[:,1]<20)][::3]
ax.scatter(mapplot[:,0],mapplot[:,1],s=.5,c='.7',alpha=.35,rasterized=True,label='Saved 3D PCD XY points')
ax.plot(P[105:146,0],P[105:146,1],lw=1.6,color='tab:red',ls='--',label='Previous reference: kink')
ax.plot(Q[105:146,0],Q[105:146,1],lw=2.6,color='tab:orange',label='Experimental map+motion candidate')
ch=np.array([c[0][:2] for c in selected]);ax.scatter(ch[:,0],ch[:,1],s=25,color='tab:blue',alpha=.85,label='Raw LiDAR multi-start map hypotheses')
ax.scatter(Q[112:140,0],Q[112:140,1],s=7,c=np.arange(112,140),cmap='viridis',zorder=5)
ax.set_xlim(76,112);ax.set_ylim(-44,18);ax.set_aspect('equal','box');ax.grid(alpha=.14);ax.set_xlabel('PCD X (m)');ax.set_ylabel('PCD Y (m)');ax.set_title('18 Sep top-right bend: independent LiDAR map match + physical continuity\nExperimental candidate; not independently validated GT')
ax.legend(fontsize=8);fig.tight_layout();fig.savefig(out/'04_corner_zoom_v3.png');plt.close(fig)
