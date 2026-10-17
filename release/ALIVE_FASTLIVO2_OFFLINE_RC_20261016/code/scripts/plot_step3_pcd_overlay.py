#!/usr/bin/env python3
import argparse,csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

ROOT=Path('/home/devil/Desktop/car/alive-repo/FAST-LIVO2-failsafe')
ap=argparse.ArgumentParser()
ap.add_argument('run_dir')
ap.add_argument('--window',type=float,default=45.0)
ap.add_argument('--pcd',default=str(ROOT/'step3_global/map_db_sweeps_fast/cache/global_map.pcd'))
ap.add_argument('--reference',required=True,
                help='Explicit same-bag reference for diagnostic plots; NOT assumed ground truth')
ap.add_argument('--sensor',default='none',choices=['none','camera','lidar','imu'])
ap.add_argument('--drop-start',type=float,default=15.0)
ap.add_argument('--drop-duration',type=float,default=20.0)
a=ap.parse_args()
RUN=Path(a.run_dir)
PCD=Path(a.pcd)
REF=Path(a.reference)
OUT=RUN/'plots'; OUT.mkdir(parents=True,exist_ok=True)

def rospose(p):
    a=[]
    with p.open(newline='') as f:
        for r in csv.DictReader(f):
            try:a.append([int(r['%time'])*1e-9,float(r['field.pose.position.x']),float(r['field.pose.position.y']),float(r['field.pose.position.z'])])
            except:pass
    return np.asarray(a,float)

def xyz(p):
    a=[]
    with p.open(newline='') as f:
        for r in csv.DictReader(f):
            try:a.append([float(r['timestamp']),float(r['x']),float(r['y']),float(r['z'])])
            except:pass
    return np.asarray(a,float)

def pcd_xy(p,max_points=450000):
    h={}; off=0
    with p.open('rb') as f:
        while True:
            line=f.readline()
            if not line: raise RuntimeError('bad PCD header')
            off=f.tell(); s=line.decode('latin1').strip()
            if not s or s.startswith('#'): continue
            k,*v=s.split(); h[k.upper()]=v
            if k.upper()=='DATA': break
    if h['DATA'][0].lower()!='binary': raise RuntimeError('expected binary PCD')
    fields=h['FIELDS']; n=int(h.get('POINTS',h['WIDTH'])[0]); nf=len(fields)
    mm=np.memmap(p,dtype='<f4',mode='r',offset=off,shape=(n,nf))
    stride=max(1,int(np.ceil(n/max_points)))
    xy=np.asarray(mm[::stride][:,[fields.index('x'),fields.index('y')]],dtype=np.float32)
    return xy[np.isfinite(xy).all(axis=1)]

g=rospose(RUN/'global_pose.csv'); fixes=rospose(RUN/'gicp_map_pose.csv'); ref=xyz(REF)
if len(g)<2 or len(ref)<2:
    raise ValueError('Empty reference or global trajectory')
if ref[0,0]>g[0,0]+0.15 or ref[-1,0]<min(g[-1,0],g[0,0]+a.window)-0.15:
    raise ValueError('Reference does not cover this bag/time window; old loop-closed CSV is invalid')
t0=ref[0,0]; t1=t0+a.window
clip=lambda a:a[(a[:,0]>=t0)&(a[:,0]<=t1)]
g=clip(g); fixes=clip(fixes); ref=clip(ref)
mapxy=pcd_xy(PCD)
# The clean PCD comparison must not depend on the extent or existence of the
# unvalidated 'loop closed reference', which may come from another recording.
allxy=g[:,1:3]
lo=allxy.min(axis=0)-8; hi=allxy.max(axis=0)+8
m=(mapxy[:,0]>=lo[0])&(mapxy[:,0]<=hi[0])&(mapxy[:,1]>=lo[1])&(mapxy[:,1]<=hi[1])
pc=mapxy[m]

fig,ax=plt.subplots(figsize=(14,9),dpi=170)
ax.scatter(pc[:,0],pc[:,1],s=.18,c='0.55',alpha=.28,label='Saved PCD map',rasterized=True)
ax.plot(ref[:,1],ref[:,2],'--',lw=2,label='Unvalidated trajectory (diagnostic only)')
ax.plot(g[:,1],g[:,2],lw=2.5,label='Published Step-3 global pose')
if a.sensor!='none':
    elapsed=g[:,0]-t0;fm=(elapsed>=a.drop_start)&(elapsed<=a.drop_start+a.drop_duration)
    if np.any(fm):ax.plot(g[fm,1],g[fm,2],lw=5,alpha=.82,label=f'{a.sensor.upper()} FAILED {a.drop_start:.0f}-{a.drop_start+a.drop_duration:.0f}s')
if len(fixes):ax.scatter(fixes[:,1],fixes[:,2],s=32,marker='x',label='Accepted GICP map fixes',zorder=4)
ax.scatter(g[0,1],g[0,2],s=65,marker='o',label='Start',zorder=5);ax.scatter(g[-1,1],g[-1,2],s=70,marker='s',label='End',zorder=5)
ax.set_xlim(*lo[[0]] if False else (lo[0],hi[0])); ax.set_ylim(lo[1],hi[1]); ax.set_aspect('equal',adjustable='box')
ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_title(f'{RUN.name} — global trajectory over saved PCD map');ax.grid(alpha=.15);ax.legend(markerscale=2)
fig.tight_layout();fig.savefig(OUT/'05_global_pose_on_pcd_xy.png',bbox_inches='tight');plt.close(fig)

# Clean planner-facing visualization: saved PCD + published global trajectory only.
# No GICP crosses, no reference line, and no start/end markers.
fig,ax=plt.subplots(figsize=(14,9),dpi=170)
ax.scatter(pc[:,0],pc[:,1],s=.18,c='0.55',alpha=.28,label='Saved PCD map',rasterized=True)
ax.plot(g[:,1],g[:,2],lw=2.8,c='tab:orange',label='Published Step-3 global pose')
ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_aspect('equal',adjustable='box')
ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_title(f'{RUN.name} — trajectory over saved PCD map')
ax.grid(alpha=.15);ax.legend(markerscale=2)
fig.tight_layout();fig.savefig(OUT/'07_orange_trajectory_on_pcd_clean.png',bbox_inches='tight');plt.close(fig)

tail=g[int(len(g)*0.55):,1:3]; zlo=tail.min(axis=0)-5; zhi=tail.max(axis=0)+5
zm=(mapxy[:,0]>=zlo[0])&(mapxy[:,0]<=zhi[0])&(mapxy[:,1]>=zlo[1])&(mapxy[:,1]<=zhi[1]); zpc=mapxy[zm]
fig,ax=plt.subplots(figsize=(10,9),dpi=170)
ax.scatter(zpc[:,0],zpc[:,1],s=.22,c='0.55',alpha=.30,label='Saved PCD map',rasterized=True)
ax.plot(ref[:,1],ref[:,2],'--',lw=2,label='Unvalidated trajectory (diagnostic only)');ax.plot(g[:,1],g[:,2],lw=2.6,label='Published Step-3 global pose')
if a.sensor!='none':
    elapsed=g[:,0]-t0;fm=(elapsed>=a.drop_start)&(elapsed<=a.drop_start+a.drop_duration)
    if np.any(fm):ax.plot(g[fm,1],g[fm,2],lw=5,alpha=.82,label=f'{a.sensor.upper()} FAILED')
if len(fixes):ax.scatter(fixes[:,1],fixes[:,2],s=36,marker='x',label='Accepted GICP map fixes',zorder=4)
ax.set_xlim(zlo[0],zhi[0]);ax.set_ylim(zlo[1],zhi[1]);ax.set_aspect('equal',adjustable='box');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_title(f'{RUN.name} — PCD overlay, later-route region');ax.grid(alpha=.15);ax.legend(markerscale=2)
fig.tight_layout();fig.savefig(OUT/'06_global_pose_on_pcd_xy_zoom_loop.png',bbox_inches='tight');plt.close(fig)
print(OUT/'05_global_pose_on_pcd_xy.png')
print(OUT/'06_global_pose_on_pcd_xy_zoom_loop.png')
print(OUT/'07_orange_trajectory_on_pcd_clean.png')
print('pcd_points_plotted_full',len(pc),'zoom',len(zpc))
