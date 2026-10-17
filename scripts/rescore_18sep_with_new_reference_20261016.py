#!/usr/bin/env python3
"""Rescore SAVED Sep18 case CSVs with NEW raw-LiDAR/PCD reference.
 No re-running algorithm, no trajectory edits and no independent-GT claims.
"""
import csv,json,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build_18sep_heldout_map_reference import read_pcd_xyz,vox

ROOT=Path(__file__).resolve().parents[1]
NEW=ROOT/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016'
REFERENCE=NEW/'REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv'
PREVIOUS=ROOT/'results/18_sep_2026_combined/HELDOUT_LIDAR_MAP_REFERENCE_20261009/CORNER_FIXED_REFERENCE_20261009/REFERENCE_18SEP_LIDAR_PCD_CORNER_FIXED_20261009.csv'
CASEBASE=ROOT/'results/BEST_SO_FAR_20261015/18_sep_2026_combined'
CASES=['HEALTHY','CAMERA_FAILED','LIDAR_FAILED','IMU_FAILED']
def load_ref(path):
 with path.open() as f:a=list(csv.DictReader(f))
 t=np.array([float(x['timestamp']) for x in a])
 xyz=np.array([[float(x[k]) for k in 'xyz'] for x in a])
 return t,xyz
nt,npz=load_ref(REFERENCE);ot,opz=load_ref(PREVIOUS)
assert np.allclose(nt,ot,atol=.001)
f=NEW/'RE_SCORED_18SEP_FOUR_CASES_20261016.csv'
stats=[]
curves={}
for case in CASES:
 path=CASEBASE/case/'global_pose.csv'
 with path.open() as file:a=list(csv.DictReader(file))
 t=np.array([float(x['field.header.stamp'])*1.e-9 for x in a])
 pos=np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in a])
 keep=(t>=nt[0])&(t<=nt[-1])
 t,pos=t[keep],pos[keep]
 assert len(t)>1000 and np.isfinite(pos).all()
 now=np.column_stack([np.interp(t,nt,npz[:,j]) for j in range(3)])
 old=np.column_stack([np.interp(t,ot,opz[:,j]) for j in range(3)])
 d=np.linalg.norm(pos-now,axis=1)
 old_d=np.linalg.norm(pos-old,axis=1)
 delta=np.diff(t);step=np.linalg.norm(np.diff(pos,axis=0),axis=1)
 time=t-nt[0]
 def rmse(arr,mask):return float(np.sqrt(np.mean(arr[mask]**2)))
 maskall=np.ones(len(t),bool)
 masks={'full':maskall,'pre':time<90,'outage':(time>=90)&(time<130),'recovery':(time>=130)&(time<150)}
 st={'case':case,'pose_samples':int(len(t)),
     'reference':'REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv',
     'overall_new_rmse_m':round(rmse(d,maskall),4),
     'overall_old_oct9_rmse_m':round(rmse(old_d,maskall),4),
     'pre_new_rmse_m':round(rmse(d,masks['pre']),4),
     'outage_90_to_130_new_rmse_m':round(rmse(d,masks['outage']),4),
     'recovery_130_to_150_new_rmse_m':round(rmse(d,masks['recovery']),4),
     'largest_step_m':round(float(step.max()),4),'max_timestamp_gap_s':round(float(delta.max()),4),
     'reference_is_independent_ground_truth':False}
 stats.append(st)
 curves[case]=(t,pos,d)
 print(case,'overall RMSE new',st['overall_new_rmse_m'],'oldOct9',st['overall_old_oct9_rmse_m'],
       'outage',st['outage_90_to_130_new_rmse_m'],'recovery',st['recovery_130_to_150_new_rmse_m'])
with f.open('w',newline='') as file:
 wr=csv.DictWriter(file,fieldnames=list(stats[0]));wr.writeheader();wr.writerows(stats)

# Replot all four against correct reference; no fit/translation or GT
fig,axs=plt.subplots(2,2,figsize=(13,16),dpi=140,constrained_layout=True)
for ax,case in zip(axs.flat,CASES):
 t,pos,error=curves[case]
 ax.plot(npz[:,0],npz[:,1],linestyle='--',color='#2178bb',lw=2,label='NEW Oct16 LiDAR/PCD reference proxy')
 ax.plot(pos[:,0],pos[:,1],color='#eb7321',lw=1.65,alpha=.9,label='Actual published Step-3 global')
 ax.plot(opz[:,0],opz[:,1],color='#247a62',lw=1,alpha=.45,label='Oct9 LiDAR/PCD proxy')
 ax.set_title(f'{case} | new ref 3D RMSE {next(s for s in stats if s["case"]==case)["overall_new_rmse_m"]:.3f}m')
 ax.set_xlim(-15,140);ax.set_ylim(-185,27);ax.set_aspect('equal',adjustable='box')
 ax.set_xlabel('Saved PCD X (m)');ax.set_ylabel('Saved PCD Y (m)');ax.grid(alpha=.18);ax.legend(fontsize=8,loc='lower left')
fig.suptitle('18 Sep four archived modes re-evaluated against NEW Oct16 LiDAR/PCD reference\nReference is NOT independent ground truth',fontsize=17)
fig.savefig(NEW/'04_NEW_REFERENCE_VS_ALL_FOUR_PUBLISHED.png',dpi=145,bbox_inches='tight')
plt.close(fig)
fig,axs=plt.subplots(2,2,figsize=(13,8),dpi=140,constrained_layout=True)
for ax,case in zip(axs.flat,CASES):
 t,pos,error=curves[case]
 ax.plot(t-nt[0],error,color='#eb7321',lw=.9)
 ax.axvspan(90,130,color='#eed7bc',alpha=.35,label='Failure window in failure-mode runs')
 ax.set_title(case);ax.set_xlabel('18 Sep relative time (s)');ax.set_ylabel('3D disagreement (m)');ax.grid(alpha=.2)
fig.suptitle('Deviation from newly rebuilt held-out LiDAR/PCD reference — 20261016 (NOT GT)',fontsize=16)
fig.savefig(NEW/'05_NEW_REFERENCE_ERROR_VS_TIME_FOUR_CASES.png',dpi=145,bbox_inches='tight')
plt.close(fig)

summary={'reference':str(REFERENCE.relative_to(ROOT)),
         'reference_sha256':hashlib.sha256(REFERENCE.read_bytes()).hexdigest(),
         'previous_reference':str(PREVIOUS.relative_to(ROOT)),
         'reference_gt_independent':False,
         'all_four_runs_reused_without_any_pose_changes':True,
         'source_bag':'18_sep_2026_combined.bag',
         'diagnostic_previously_reported_rmse_outdated':True,
         'cases':stats}
(NEW/'re_evaluation_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('SAVED',f)
