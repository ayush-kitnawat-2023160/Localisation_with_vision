#!/usr/bin/env python3
import argparse,csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def rospose(p):
    a=[]
    if not p.exists(): return np.empty((0,4))
    with p.open(newline='') as f:
        for r in csv.DictReader(f):
            try:a.append([int(r['%time'])*1e-9,float(r['field.pose.position.x']),float(r['field.pose.position.y']),float(r['field.pose.position.z'])])
            except (KeyError,ValueError,TypeError):pass
    return np.asarray(a,float).reshape((-1,4))

def xyz(p):
    a=[]
    if not p.exists(): return np.empty((0,4))
    with p.open(newline='') as f:
        for r in csv.DictReader(f):
            try:a.append([float(r['timestamp']),float(r['x']),float(r['y']),float(r['z'])])
            except (KeyError,ValueError,TypeError):pass
    return np.asarray(a,float).reshape((-1,4))

def interp(ref,t): return np.column_stack([np.interp(t,ref[:,0],ref[:,i]) for i in (1,2,3)])
def clip(a,t0,w): return a[(a[:,0]>=t0)&(a[:,0]<=t0+w)] if a.size else a

def main():
    ap=argparse.ArgumentParser();ap.add_argument('run_dir');ap.add_argument('--reference',required=True);ap.add_argument('--sensor',default='none',choices=['none','camera','lidar','imu']);ap.add_argument('--window',type=float,default=45.0);ap.add_argument('--drop-start',type=float,default=15.0);ap.add_argument('--drop-duration',type=float,default=20.0);a=ap.parse_args()
    run=Path(a.run_dir);out=run/'plots';out.mkdir(parents=True,exist_ok=True)
    g=rospose(run/'global_pose.csv');fixes=rospose(run/'gicp_map_pose.csv');ref=xyz(Path(a.reference));raw=xyz(run/'trajectory.csv')
    if not g.size or not ref.size:raise RuntimeError('empty global pose/reference')
    # Injected drop times are relative to rosbag playback, not the first
    # *published* global pose (which can lag start by several seconds).
    t0=ref[0,0];g=clip(g,t0,a.window);fixes=clip(fixes,t0,a.window);rr=clip(ref,t0,a.window);raw=clip(raw,t0,a.window)
    if ref[0,0]>g[0,0]+0.15 or ref[-1,0]<g[-1,0]-0.15:
        raise ValueError('Reference timestamps do not cover this recording; old-map reference is invalid')
    elapsed=g[:,0]-t0;err=np.linalg.norm(g[:,1:4]-interp(ref,g[:,0]),axis=1);steps=np.linalg.norm(np.diff(g[:,1:4],axis=0),axis=1)
    fig,ax=plt.subplots(figsize=(11,9),dpi=150);ax.plot(rr[:,1],rr[:,2],'--',lw=2,label='Unvalidated comparison trajectory');ax.plot(g[:,1],g[:,2],lw=2.2,label='Published Step-3 global pose')
    if a.sensor!='none':
        fm=(elapsed>=a.drop_start)&(elapsed<=a.drop_start+a.drop_duration)
        if np.any(fm):ax.plot(g[fm,1],g[fm,2],lw=5,alpha=.78,label=f'{a.sensor.upper()} FAILED {a.drop_start:.0f}-{a.drop_start+a.drop_duration:.0f}s')
    if fixes.size:ax.scatter(fixes[:,1],fixes[:,2],s=28,marker='x',label='Accepted GICP map fixes',zorder=4)
    ax.scatter(g[0,1],g[0,2],s=55,marker='o',label='Start');ax.scatter(g[-1,1],g[-1,2],s=60,marker='s',label='End');ax.set_aspect('equal',adjustable='box');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_title(f'{run.name} — unvalidated trajectory comparison');ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(out/'01_global_vs_loopclosed_xy_equal.png',bbox_inches='tight');plt.close(fig)
    fig,ax=plt.subplots(figsize=(11,6),dpi=150)
    if a.sensor!='none':ax.axvspan(a.drop_start,a.drop_start+a.drop_duration,alpha=.16,label=f'{a.sensor.upper()} FAILED {a.drop_start:.0f}-{a.drop_start+a.drop_duration:.0f}s')
    ax.plot(elapsed,err,lw=2,label='3D disagreement (not localization accuracy)')
    if fixes.size:
        fe=np.linalg.norm(fixes[:,1:4]-interp(ref,fixes[:,0]),axis=1);ax.scatter(fixes[:,0]-t0,fe,s=24,marker='x',label='GICP-to-reference disagreement')
    ax.set_xlabel('Elapsed time (s)');ax.set_ylabel('3D trajectory disagreement (m)');ax.set_title('Trajectory disagreement — NOT ground truth');ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(out/'02_error_vs_time.png',bbox_inches='tight');plt.close(fig)
    fig,ax=plt.subplots(figsize=(11,6),dpi=150)
    if a.sensor!='none':ax.axvspan(a.drop_start,a.drop_start+a.drop_duration,alpha=.16,label=f'{a.sensor.upper()} FAILED {a.drop_start:.0f}-{a.drop_start+a.drop_duration:.0f}s')
    if steps.size:ax.plot(elapsed[1:],steps,lw=1.5,label='Consecutive global-pose step');ax.axhline(1.0,ls='--',lw=1.2,label='1 m diagnostic line')
    ax.set_xlabel('Elapsed time (s)');ax.set_ylabel('Consecutive pose displacement (m)');ax.set_title('Published trajectory continuity / jump check');ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(out/'03_pose_step_continuity.png',bbox_inches='tight');plt.close(fig)
    if raw.size:
        fig,ax=plt.subplots(figsize=(10,8),dpi=150);ax.plot(raw[:,1],raw[:,2],lw=2,label='Raw FAST-LIVO2 local ESIKF')
        if a.sensor!='none':
            re=raw[:,0]-t0;rm=(re>=a.drop_start)&(re<=a.drop_start+a.drop_duration)
            if np.any(rm):ax.plot(raw[rm,1],raw[rm,2],lw=5,alpha=.78,label=f'{a.sensor.upper()} FAILED')
        ax.set_aspect('equal',adjustable='box');ax.set_xlabel('Local X (m)');ax.set_ylabel('Local Y (m)');ax.set_title('Raw FAST-LIVO2 trajectory (no PCD transform)');ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(out/'04_raw_fastlivo2_local_xy_equal.png',bbox_inches='tight');plt.close(fig)
    (out/'PLOT_INFO.txt').write_text(f'run={run.name}\nsensor={a.sensor}\ntimeline_origin=first_same_bag_reference_scan\nreference_is_validated_gt=false\nfailure_start_s={a.drop_start if a.sensor!="none" else "NA"}\nfailure_end_s={a.drop_start+a.drop_duration if a.sensor!="none" else "NA"}\nglobal_samples={len(g)}\ngicp_fixes={len(fixes)}\nmax_global_step_m={steps.max() if steps.size else float("nan"):.6f}\nend_reference_disagreement_m={err[-1]:.6f}\n')
    print(out);[print(p.name) for p in sorted(out.iterdir())]
if __name__=='__main__':main()
