#!/usr/bin/env python3
"""No trajectory editing: saved PCD and raw before/after publisher samples."""
import argparse,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ap=argparse.ArgumentParser()
ap.add_argument('--baseline',type=Path,required=True)
ap.add_argument('--candidate',type=Path,required=True)
ap.add_argument('--reference',type=Path,required=True)
ap.add_argument('--pcd',type=Path,required=True)
ap.add_argument('--out',type=Path,required=True)
a=ap.parse_args()

def positions(p):
 r=list(csv.DictReader(p.open())); is_ros='field.header.stamp' in r[0];t=np.array([float(z['field.header.stamp'])*1e-9 if is_ros else float(z['timestamp']) for z in r]);x=np.array([[float(z['field.pose.position.'+k] if is_ros else z[k]) for k in 'xy'] for z in r]);return t,x
rt,rx=positions(a.reference);t0=rt[0]

def pcd_xy(path):
 with path.open('rb') as f:
  h={}
  while True:
   ln=f.readline().decode('latin1').strip();off=f.tell()
   v=ln.split()
   if v:h[v[0].upper()]=v[1:]
   if v and v[0]=='DATA':break
 assert h['DATA'][0]=='binary'
 fields=h['FIELDS'];n=int(h['POINTS'][0]);mm=np.memmap(path,dtype='<f4',mode='r',offset=off,shape=(n,len(fields)));return mm[::8][:,[fields.index('x'),fields.index('y')]]
xy=pcd_xy(a.pcd);mask=(xy[:,0]>45)&(xy[:,0]<100)&(xy[:,1]>-18)&(xy[:,1]<26)
fig,axes=plt.subplots(1,2,figsize=(15,6.8),sharex=True,sharey=True)
for ax,p,name in [(axes[0],a.baseline,'Earlier healthy: baseline'),(axes[1],a.candidate,'New healthy: lateral prior (experiment)')]:
 t,x=positions(p/'global_pose.csv');g,gx=positions(p/'gicp_map_pose.csv')
 m=(t-t0>=15)&(t-t0<=40);gm=(g-t0>=15)&(g-t0<=40);rr=(rt-t0>=15)&(rt-t0<=40)
 ax.scatter(xy[mask,0],xy[mask,1],s=.3,c='.64',alpha=.31,rasterized=True,label='saved PCD')
 ax.plot(rx[rr,0],rx[rr,1],'--',color='#2c86b8',lw=2.5,label='loop-closed proxy (not ground truth)')
 ax.plot(x[m,0],x[m,1],color='#ff831c',lw=2.5,label='published pose')
 ax.scatter(gx[gm,0],gx[gm,1],color='#7256ad',s=17,alpha=.75,marker='x',label='raw GICP')
 ax.set(xlim=(46,95),ylim=(-16,25),xlabel='X (m)',ylabel='Y (m)',title=name)
 ax.grid(alpha=.15);ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper left')
fig.suptitle('Top-right bend / map overlap — 15–40 s, identical map and bag',fontsize=14)
fig.tight_layout();a.out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.out,dpi=180);plt.close(fig);print(a.out)
