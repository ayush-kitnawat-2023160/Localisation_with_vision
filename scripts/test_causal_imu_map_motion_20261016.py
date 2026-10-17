#!/usr/bin/env python3
"""Causal source-free experiment only. Reference used for final scoring, not pose inference.

During 40s IMU outage, reconstruct global motion from accepted map GICP
poses only, with bounded speed/acceleration and a fixed 0.20s simulated
measurement arrival latency. This is NOT the live published ROS trajectory.
Run on main first, then untouched Sep18 to reject non-generalizing models.
"""
import csv,json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1]
DIR=R/'results/LOCALIZATION_ONLY_COMPLETION_20261016'
DIR.mkdir(parents=True,exist_ok=True)
CASES=[('main',125,165,R/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv'),
('sep18',90,130,R/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv')]
def ld(p):
 a=list(csv.DictReader(open(p)));return np.array([float(z['field.header.stamp'])*1e-9 for z in a]),np.array([[float(z['field.pose.position.'+k]) for k in 'xyz'] for z in a])
def ref(p):
 a=list(csv.DictReader(open(p)));return np.array([float(z['timestamp']) for z in a]),np.array([[float(z[k]) for k in 'xyz'] for z in a])
def filt(ot,op,gt,gp,begin,end,latency=.20,alpha=.60,correction=3.,speed=5.,extra=0.,guard=1.0):
 output=op.copy();last=np.zeros(3);seen=None;velocity=np.zeros(3);previous=None
 k=0
 for i,t in enumerate(ot):
  if not (begin<=t<end+extra):continue
  while k<len(gt) and gt[k]+latency<=t:
   if previous is not None:
    dt=gt[k]-previous[0]
    if .25<dt<3:
     v=(gp[k]-previous[1])/dt
     n=np.linalg.norm(v)
     if n>speed:v*=speed/n
     if n<speed*1.5:velocity=(1-alpha)*velocity+alpha*v
   previous=(gt[k],gp[k]);k+=1
  if previous is None:continue
  age=t-previous[0]
  if age<0 or age>1.6:continue
  target=previous[1]+min(age,1.25)*velocity
  # Drift-gate conditional: bad ESIKF local output vs verified map is a
  # useful anomaly indicator but not proof GICP is correct.
  if np.linalg.norm(target-op[i])<guard:continue
  if i and begin<=ot[i-1]<end+extra:
   dt=t-ot[i-1]
   delta=target-output[i-1]
   norm=np.linalg.norm(delta)
   if norm>correction*dt:delta*=correction*dt/norm
   output[i]=output[i-1]+delta
  else:output[i]=op[i] # maintain continuity at failure edge
 return output
def score(ot,op,r,x,a,b):
 ok=(ot>=r[0]+a)&(ot<r[0]+b)
 ref=np.column_stack([np.interp(ot,r,x[:,j]) for j in range(3)])
 diff=op-ref
 e=np.linalg.norm(diff,axis=1)
 return {'all':round(float(np.sqrt(np.mean(e**2))),3),
 'outage':round(float(np.sqrt(np.mean(e[ok]**2))),3),
 'outage_max_step':round(float(np.max(np.linalg.norm(np.diff(op[ok],axis=0),axis=1))),3)}
tests=[(.2,.60,3.,5.,.4),(.2,.65,5.,6.,.6),(.2,.60,8.,5.,1.),(.2,.45,4.,4.,.3)]
summary={}
for case,a,b,rp in CASES:
 d=R/'results/FINAL_SINGLE_IMAGE_RC_20261016'/('sept18' if case=='sep18' else case)/'IMU_FAIL_SINGLE_IMAGE_1X'
 ot,op=ld(d/'global_pose.csv');gt,gp=ld(d/'gicp_map_pose.csv')
 r,x=ref(rp)
 dres=[]
 for latency,alpha,corr,speed,guard in tests:
  f=filt(ot,op,gt,gp,r[0]+a,r[0]+b,latency,alpha,corr,speed,extra=0,guard=guard)
  st=score(ot,f,r,x,a,b);st['config']=[latency,alpha,corr,speed,guard];dres.append(st)
 summary[case]={'baseline':score(ot,op,r,x,a,b),'variants':dres}
 print(case,summary[case],flush=True)
(DIR/'IMU_CAUSAL_GICP_PREDICTOR_TRIALS.json').write_text(json.dumps(summary,indent=2)+'\n')
