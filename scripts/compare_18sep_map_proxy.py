#!/usr/bin/env python3
"""Review older runs against the September-18 held-out LiDAR map proxy.

No accuracy/GT claims: the proxy incorporates LiDAR and lidar-IMU odometry,
uses a saved map and has weak intervals explicitly marked in its CSV.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build_18sep_heldout_map_reference import read_pcd_xyz,vox


def load_run(path):
    a=[]
    for r in csv.DictReader(open(path)):
        try:a.append([int(r['%time'])*1e-9]+[float(r['field.pose.position.'+k]) for k in 'xyz'])
        except (ValueError,KeyError):pass
    return np.array(a,float)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--reference',required=True)
    ap.add_argument('--map',required=True)
    ap.add_argument('--runs-base',required=True)
    ap.add_argument('--output',required=True)
    args=ap.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    rr=list(csv.DictReader(open(args.reference)))
    ref=np.array([[float(x[k]) for k in ('timestamp','x','y','z')] for x in rr]);
    valid=np.array([x['reference_quality'] in (
        'high','medium','corner_multistart_map_verified',
        'corner_transition_smoothing') for x in rr]);
    names=['STEP4_PHYSICAL_HEALTHY_20261008','STEP4_PHYSICAL_CAMERA_FAIL_20261008',
           'FINAL_LIDAR_FAIL_220S_20261007','STEP4_PHYSICAL_LIDAR_FAIL_20261008',
           'STEP4_DEPTH_RELOCK_FINAL_220S_20261008','STEP4_PHYSICAL_IMU_FAIL_20261008']
    stats=[];curves={}
    for name in names:
        path=Path(args.runs_base)/name/'global_pose.csv'
        if not path.exists():continue
        g=load_run(path)
        g=g[(g[:,0]>=ref[0,0])&(g[:,0]<=ref[-1,0])]
        interp=np.column_stack([np.interp(g[:,0],ref[:,0],ref[:,k]) for k in (1,2,3)])
        err=np.linalg.norm(g[:,1:]-interp,axis=1)
        rel=g[:,0]-ref[0,0]
        good_mask=(np.interp(g[:,0],ref[:,0],valid.astype(float))>.99)
        middle=(rel>=90)&(rel<130)
        tmask=middle&good_mask
        stats.append({'run':name,'diagnostic_full_reference_RMSE_m':round(float(np.mean(err**2)**.5),4),
                      'diagnostic_90to130_all_RMSE_m':round(float(np.mean(err[middle]**2)**.5),4),
                      'diagnostic_90to130_noninterpolated_RMSE_m':round(float(np.mean(err[tmask]**2)**.5),4) if tmask.any() else None,
                      'noninterpolated_window_samples':int(tmask.sum()),
                      'reference_is_ground_truth':False})
        curves[name]=g
    with (out/'comparison_to_heldout_map_proxy.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(stats[0]));w.writeheader();w.writerows(stats)
    m=read_pcd_xyz(args.map).astype(np.float32)
    m=m[np.isfinite(m).all(1)]
    m=m[(m[:,0]>-10)&(m[:,0]<140)&(m[:,1]>-190)&(m[:,1]<20)]
    m=vox(m,.7)[::3]
    for which,plot_names in [('healthy',['STEP4_PHYSICAL_HEALTHY_20261008']),
                             ('lidar',['FINAL_LIDAR_FAIL_220S_20261007',
                                       'STEP4_PHYSICAL_LIDAR_FAIL_20261008'])]:
        fig,ax=plt.subplots(figsize=(9,11),dpi=150)
        ax.scatter(m[:,0],m[:,1],s=.16,c='.65',alpha=.35,rasterized=True,label='Previously loop-closed saved PCD')
        ax.plot(ref[:,1],ref[:,2],color='tab:orange',lw=2.3,label='18 Sep held-out LiDAR/map proxy')
        for name in plot_names:
            if name not in curves:continue
            a=curves[name];label=('Healthy Step 4' if 'HEALTHY' in name else
                                  'Oct 7 LiDAR fail' if 'FINAL_LIDAR' in name else
                                  'Oct 8 LiDAR fail')
            ax.plot(a[:,1],a[:,2],lw=1.5,alpha=.84,label=label)
        low=np.array([x['reference_quality']=='motion_interpolated' for x in rr]);
        ax.scatter(ref[low,1],ref[low,2],color='red',s=9,label='Proxy uncertainty: motion bridge')
        ax.set_xlim(-5,135);ax.set_ylim(-190,18);ax.set_aspect('equal','box')
        ax.set_xlabel('PCD frame X (m)');ax.set_ylabel('PCD frame Y (m)')
        ax.set_title('18 Sep trajectories vs LiDAR-derived proxy — not ground truth')
        ax.legend(loc='lower left',fontsize=8);fig.tight_layout();fig.savefig(out/f'03_{which}_against_18sep_reference.png');plt.close(fig)
    print(json.dumps(stats,indent=2))


if __name__=='__main__':main()
