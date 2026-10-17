#!/usr/bin/env python3
"""Repair scan-map aliasing and export a SAME-BAG LiDAR/map reference proxy.

The source is raw held-out LiDAR scans from the September 18 recording, and a
loop-closed PCD map from a different recording. The reference is *not* surveyed
ground truth; individual LiDAR registration outliers are filled from short
relative-odometry segments constrained at both verified map endpoints.
"""
import argparse,csv,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator
from scipy.signal import savgol_filter
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from build_18sep_heldout_map_reference import read_pcd_xyz,vox,qposes,interp_seed,local_qposes


def wrap(a):return (a+np.pi)%(2*np.pi)-np.pi


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--registrations',required=True)
    ap.add_argument('--scans',required=True)
    ap.add_argument('--map',required=True)
    ap.add_argument('--local-odometry',required=True)
    ap.add_argument('--initial-attitude',required=True)
    ap.add_argument('--output',required=True)
    args=ap.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    rec=list(csv.DictReader(open(args.registrations)))
    times=np.array([float(r['timestamp']) for r in rec]);raw=np.array([[float(r[k]) for k in ('x','y','z')] for r in rec]);
    yaw=np.unwrap(np.array([float(r['yaw']) for r in rec]))
    quality=np.array([float(r['median_residual_m']) for r in rec]);overlap=np.array([float(r['overlap']) for r in rec]);
    lt,lp,lq=local_qposes(args.local_odometry)
    local=np.column_stack([np.interp(times,lt,lp[:,i]) for i in range(3)])
    local_yaw=np.unwrap(np.array([interp_seed(lt,lp,lq,t)[1].as_euler('ZYX')[0] for t in times]))
    steps=np.linalg.norm(np.diff(raw[:,:2],axis=0),axis=1)
    odom_steps=np.linalg.norm(np.diff(local[:,:2],axis=0),axis=1)
    ratio=steps/np.maximum(odom_steps,.10)
    invalid=(odom_steps>.70)&((ratio<.35)|(ratio>1.8))
    invalid|=(steps/np.diff(times)>4.2)
    bad_indices=np.where(invalid)[0]+1
    groups=[]
    for i in bad_indices:
        if not groups or i-groups[-1][-1]>4:groups.append([int(i)])
        else:groups[-1].append(int(i))
    # Each temporary road alias must be corrected as a segment bounded by
    # good poses, not treated as a good map fix despite a low ICP score.
    intervals=[(max(1,g[0]),min(len(times)-2,g[-1])) for g in groups]
    corrected=raw.copy();new_yaw=yaw.copy();filled=np.zeros(len(times),bool)
    for a,b in intervals:
        left=a-1;right=b+1
        A=raw[left,:2];B=raw[right,:2]
        loc_a=local[left,:2];loc_b=local[right,:2]
        old=loc_b-loc_a;target=B-A
        if np.linalg.norm(old)<.25 or np.linalg.norm(target)<.25:
            # No reliable relative-motion baseline: use bounded interpolation.
            interp=np.column_stack([np.interp(times[a:b+1],times[[left,right]],
                                         raw[[left,right],i]) for i in range(3)])
            corrected[a:b+1]=interp
        else:
            alpha=math.atan2(target[1],target[0])-math.atan2(old[1],old[0])
            scale=np.clip(np.linalg.norm(target)/np.linalg.norm(old),.70,1.3)
            c,s=math.cos(alpha),math.sin(alpha)
            R=np.array([[c,-s],[s,c]])
            corrected[a:b+1,:2]=A+scale*((local[a:b+1,:2]-loc_a)@R.T)
            corrected[a:b+1,2]=np.interp(times[a:b+1],times[[left,right]],
                                         raw[[left,right],2])
        # Correct the relative yaw integration to match both PCD-map anchors.
        for i in range(a,b+1):
            frac=(times[i]-times[left])/(times[right]-times[left])
            predicted=yaw[left]+local_yaw[i]-local_yaw[left]
            target_end=yaw[left]+local_yaw[right]-local_yaw[left]
            new_yaw[i]=predicted+frac*wrap(yaw[right]-target_end)
        filled[a:b+1]=True
    # A short symmetric filter removes one-frame map jitter while preserving
    # the genuine roundabout curvature and not imposing any false endpoint loop.
    smooth=corrected.copy()
    for ax in range(3):smooth[:,ax]=savgol_filter(corrected[:,ax],5,2,mode='interp')
    smooth_yaw=savgol_filter(np.unwrap(new_yaw),5,2,mode='interp')
    # Compute true pose roll/pitch from the source recording; only the yaw is
    # replaced by the archived-map-registered LiDAR heading.
    t_seed,p_seed,q_seed=qposes(args.initial_attitude)
    quats=[]
    for t,yy in zip(times,smooth_yaw):
        _,att=interp_seed(t_seed,p_seed,q_seed,t)
        tilt=Rotation.from_euler('z',-att.as_euler('ZYX')[0])*att
        quats.append((Rotation.from_euler('z',yy)*tilt).as_quat())
    quats=np.array(quats)
    target=out/'REFERENCE_18SEP_LIDAR_TO_LOOPCLOSED_PCD_20261009.csv'
    with target.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['timestamp','x','y','z','qx','qy','qz','qw','reference_quality'])
        for i in range(len(times)):
            conf='motion_interpolated' if filled[i] else rec[i]['confidence']
            w.writerow([f'{times[i]:.9f}',*[f'{z:.8f}' for z in smooth[i]],
                        *[f'{z:.9f}' for z in quats[i]],conf])
    # Validate using the other half of each raw LiDAR point cloud. These scan
    # points were NOT included in per-frame scan-to-map ICP registration.
    m=read_pcd_xyz(args.map).astype(np.float32)
    m=m[np.isfinite(m).all(axis=1)]
    m=m[(m[:,0]>-65)&(m[:,0]<155)&(m[:,1]>-205)&(m[:,1]<45)&(m[:,2]>-7)&(m[:,2]<20)]
    m=vox(m,.55)
    tree=cKDTree(m.astype(float))
    check=[]
    for i,(t,pos,quat) in enumerate(zip(times,smooth,quats)):
        cloud=np.load(Path(args.scans)/f'{i:04d}.npy')[1::2]
        world=cloud@Rotation.from_quat(quat).as_matrix().T+pos
        dist,_=tree.query(world,k=1,workers=2)
        check.append([float(np.median(dist)),float(np.quantile(dist,.75)),
                      float((dist<1.0).mean())])
    check=np.array(check)
    after_speed=np.linalg.norm(np.diff(smooth[:,:2],axis=0),axis=1)/np.diff(times)
    # Nearby physical locations from different times can indicate a partial
    # revisit, but a vehicle parked for 20 seconds is not a SLAM loop closure.
    # Require substantial *travelled distance*, not just elapsed time.
    cumulative=np.r_[0,np.cumsum(np.linalg.norm(np.diff(smooth[:,:2],axis=0),axis=1))]
    revisits=[]
    for i in range(len(times)):
        for j in range(i+20,len(times)):
            ds=np.linalg.norm(smooth[j,:2]-smooth[i,:2])
            if ds<1.5 and cumulative[j]-cumulative[i]>20:
                revisits.append([i,j,round(float(times[j]-times[i]),1),round(float(ds),2)])
    # Flag independent held-out-scan map disagreement; this does not certify
    # absolute accuracy, but identifies where a map proxy may be unreliable.
    quality_labels=[('motion_interpolated' if filled[i] else
                     'poor_heldout_map_fit' if check[i,0]>.75 else
                     rec[i]['confidence']) for i in range(len(times))]
    with target.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['timestamp','x','y','z','qx','qy','qz','qw','reference_quality'])
        for i in range(len(times)):
            w.writerow([f'{times[i]:.9f}',*[f'{z:.8f}' for z in smooth[i]],
                        *[f'{z:.9f}' for z in quats[i]],quality_labels[i]])
    quality_csv=out/'registration_quality.csv'
    with quality_csv.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['timestamp','heldout_median_pcd_m','heldout_p75_pcd_m',
                                   'heldout_overlap_1m','reference_quality','filled_motion_segment'])
        for i,t in enumerate(times):w.writerow([t,*check[i],
                    quality_labels[i],int(filled[i])])
    fig,ax=plt.subplots(figsize=(10.5,11),dpi=145)
    mplot=m[::4,:2]
    lo=smooth[:,:2].min(0)-10;hi=smooth[:,:2].max(0)+10
    use=(mplot[:,0]>lo[0])&(mplot[:,0]<hi[0])&(mplot[:,1]>lo[1])&(mplot[:,1]<hi[1])
    ax.scatter(mplot[use,0],mplot[use,1],s=.18,c='.65',alpha=.38,rasterized=True,label='Previously loop-closed PCD map')
    ax.plot(raw[:,0],raw[:,1],lw=1,alpha=.55,label='Raw per-scan PCD match (contains aliases)')
    ax.plot(smooth[:,0],smooth[:,1],color='tab:orange',lw=2.4,label='18 Sep held-out LiDAR/map reference proxy')
    ax.scatter(smooth[filled,0],smooth[filled,1],s=9,c='red',alpha=.8,label='Motion-constrained repair')
    ax.scatter(*smooth[0,:2],s=55,c='green',label='Start')
    ax.scatter(*smooth[-1,:2],s=55,c='black',label='End')
    ax.set_aspect('equal','box');ax.set_xlim(*lo[[0]] if False else (lo[0],hi[0]));ax.set_ylim(lo[1],hi[1]);ax.legend(loc='upper left',fontsize=8)
    ax.set_xlabel('PCD-map X (m)');ax.set_ylabel('PCD-map Y (m)')
    ax.set_title('18 Sep same-bag held-out LiDAR reference (NOT surveyed ground truth)')
    fig.tight_layout();fig.savefig(out/'01_18sep_heldout_lidar_on_pcd.png');plt.close(fig)
    fig,ax=plt.subplots(figsize=(11,5),dpi=145)
    ax.plot(times-times[0],check[:,0],label='Held-out scan median nearest-map residual')
    ax.axhline(.75,color='darkred',ls='--',label='0.75 m diagnostic')
    for aa,bb in intervals:ax.axvspan(times[aa]-times[0],times[bb]-times[0],color='orange',alpha=.20)
    ax.set_ylabel('Median residual (m)');ax.set_xlabel('18 Sep bag relative time (s)');ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(out/'02_reference_heldout_lidar_residual.png');plt.close(fig)
    meta={'reference_type':'held-out LiDAR to previously loop-closed saved PCD; NOT independent ground truth',
          'bag':'18_sep_2026_combined.bag',
          'source_lidar_topic':'/lidar103/velodyne_points',
          'reference_samples':len(times),'first_timestamp':float(times[0]),'last_timestamp':float(times[-1]),
          'map':str(args.map),'local_odometry_used_for_short_motion_bridges':str(args.local_odometry),
          'initial_attitude_source':str(args.initial_attitude),
          'registration_alias_intervals':[[float(times[a]-times[0]),float(times[b]-times[0])] for a,b in intervals],
          'interpolated_count':int(filled.sum()),
          'poor_heldout_fit_count':int(sum(q=='poor_heldout_map_fit' for q in quality_labels)),
          'heldout_median_residual_m':float(np.median(check[:,0])),
          'heldout_p90_residual_m':float(np.quantile(check[:,0],.9)),
          'heldout_overlap_1m_median':float(np.median(check[:,2])),
          'peak_speed_mps':float(after_speed.max()),
          'p99_speed_mps':float(np.quantile(after_speed,.99)),
          'start_end_planar_displacement_m':float(np.linalg.norm(smooth[-1,:2]-smooth[0,:2])),
          'revisit_pairs_greater_than_20s_within_1p5m':revisits[:30],
          'full_start_end_loop_closure':False,
          'reliability_warning':'LiDAR used to make this proxy; do not use as independent sensor-fusion ground truth or for evaluating healthy LiDAR autonomy. Shared saved map and relative FAST-LIVO2 lidar-IMU odometry can bias proxy.'}
    (out/'provenance_and_validation.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps({k:v for k,v in meta.items() if k not in ('revisit_pairs_greater_than_20s_within_1p5m',)},indent=2))
    print('SAVED',target,flush=True)

if __name__=='__main__':main()
