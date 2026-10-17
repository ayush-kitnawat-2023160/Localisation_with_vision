#!/usr/bin/env python3
"""Read actual 1x replayed ROS PoseStamped. References only for scoring."""
from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/LOCALIZATION_ONLY_COMPLETION_20261016/imu_map_quarantine_dev'
ORIGINAL=ROOT/'results/FINAL_SINGLE_IMAGE_RC_20261016'
PAIRS=[
('main',ORIGINAL/'main/IMU_FAIL_SINGLE_IMAGE_1X/global_pose.csv',
 OUT/'main/IMU_MAP_GUARD_DEV_190S/global_pose.csv',
 ROOT/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv',190,125,165),
('sept18',ORIGINAL/'sept18/IMU_FAIL_SINGLE_IMAGE_1X/global_pose.csv',
 OUT/'sept18/IMU_MAP_GUARD_DEV_160S/global_pose.csv',
 ROOT/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv',160,90,130)]
def poses(p):
 with p.open() as f:rr=list(csv.DictReader(f))
 t=np.array([float(a['field.header.stamp'])*1e-9 for a in rr])
 return t,np.array([[float(a['field.pose.position.'+k]) for k in 'xyz'] for a in rr])
def ref(p):
 with p.open() as f:rr=list(csv.DictReader(f))
 return np.array([float(a['timestamp']) for a in rr]),np.array([[float(a[k]) for k in 'xyz'] for a in rr])
def score(t,pos,ref_t,ref_p,limit,a,b):
 mask=(t>=ref_t[0])&(t<=min(ref_t[-1],ref_t[0]+limit))
 t,pos=t[mask],pos[mask]
 target=np.column_stack([np.interp(t,ref_t,ref_p[:,j]) for j in range(3)])
 e=np.linalg.norm(pos-target,axis=1)
 steps=np.linalg.norm(np.diff(pos,axis=0),axis=1);dt=np.diff(t);time=t-ref_t[0]
 tests={'full':np.ones(len(t),bool),'pre_failure':time<a,
 'failure':(time>=a)&(time<b),
 'guard_outage_start':(time>=a+6)&(time<a+12),
 'recovery20':(time>=b)&(time<min(limit,b+20))}
 return t,pos,e,{'pose_count':len(t),'max_step_m':round(float(steps.max()),3),
 'steps_gt_0p85':int(np.sum(steps>.85)),'max_dt_s':round(float(dt.max()),3),
 **{name+'_3d_rmse_m':round(float(np.sqrt(np.mean(e[m]**2))),3) if m.sum() else None for name,m in tests.items()}}
summary={}
for name,before,after,reference,limit,a,b in PAIRS:
 assert before.is_file() and after.is_file(),(before,after)
 r,refpos=ref(reference)
 ot,op=poses(before);nt,np_=poses(after)
 old_t,old_p,old_err,old=score(ot,op,r,refpos,limit,a,b)
 new_t,new_p,new_err,new=score(nt,np_,r,refpos,limit,a,b)
 print(name,'ORIGINAL',old,'NEW',new,flush=True)
 summary[name]={'before':old,'after':new}
 fig,axes=plt.subplots(1,3,figsize=(18,5.4),dpi=140)
 ax=axes[0]
 ax.plot(refpos[:,0],refpos[:,1],ls='--',lw=2,color='#a9b6cc',label='Unfit reference proxy')
 ax.plot(old_p[:,0],old_p[:,1],lw=1.2,color='#477ab3',label='Original single-image')
 ax.plot(new_p[:,0],new_p[:,1],lw=1.3,color='#e47427',label='Experimental IMU guard')
 ax.axis('equal');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)');ax.legend(fontsize=8);ax.grid(alpha=.18)
 ax.set_title('Actual published XY trajectories')
 for t,v,lab,c in [(old_t,old_err,'Old','#477ab3'),(new_t,new_err,'Trial','#e47427')]:
  axes[1].plot(t-r[0],v,lw=.9,color=c,label=lab)
 axes[1].set_title('3D map-proxy disagreement');axes[1].set_ylabel('Metres');axes[1].set_xlabel('Bag time (s)')
 for t,p,lab,c in [(old_t,old_p,'Old','#477ab3'),(new_t,new_p,'Trial','#e47427')]:
  axes[2].plot(t[1:]-r[0],np.linalg.norm(np.diff(p,axis=0),axis=1),lw=.9,color=c,label=lab)
 axes[2].set_title('Published 3D pose steps');axes[2].axhline(.85,color='red',ls='--',lw=1)
 axes[2].set_ylabel('Metres');axes[2].set_xlabel('Bag time (s)')
 for ax in axes[1:]:ax.axvspan(a,b,alpha=.1,color='#a0a9b9');ax.grid(alpha=.2);ax.legend(fontsize=8)
 fig.suptitle(f'Actual 1x ROS validation {name} — no GT-based correction / no cart control')
 fig.tight_layout();fig.savefig(OUT/(name+'_BEFORE_AFTER_REAL_ROS.png'),bbox_inches='tight')
 plt.close(fig)
(OUT/'BOTH_BAGS_ACTUAL_IMU_GUARD_COMPARISON.json').write_text(json.dumps(summary,indent=2)+'\n')
print('WROTE REAL-ROS TWO-BAG VALIDATION',flush=True)
