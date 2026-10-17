#!/usr/bin/env python3
"""OFFLINE scan-to-archived-PCD reference for the 18 September recording.

This trajectory is a held-out-LiDAR/map-registration **proxy**, not surveyed GT
or a claim that the route starts and ends at one closed-loop point. The pose
track supplied as --seed is only a coarse registration proposal, never treated
as an independent reference; scan/map fitness and continuity decide confidence.
"""
import argparse,csv,json,math,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import lsqr


def read_pcd_xyz(path):
    with open(path,'rb') as f:
        m={}
        while True:
            s=f.readline().decode('ascii').strip()
            if not s:raise RuntimeError('Unterminated PCD header')
            k,*v=s.split()
            m[k]=v
            if k=='DATA':break
        offset=f.tell()
    assert m['DATA'][0]=='binary'
    fields=m['FIELDS'];types=m['TYPE'];sizes=list(map(int,m['SIZE']));cnt=list(map(int,m.get('COUNT',['1']*len(fields))))
    fmts=[];offs=[];ofs=0
    formats={('F',4):'<f4',('F',8):'<f8',('I',4):'<i4',('U',4):'<u4',('U',1):'u1'}
    for f,typ,sz,c in zip(fields,types,sizes,cnt):
        assert c==1
        offs.append(ofs);fmts.append(formats[(typ,sz)]);ofs+=sz
    dtype=np.dtype({'names':fields,'formats':fmts,'offsets':offs,'itemsize':ofs})
    data=np.memmap(path,mode='r',dtype=dtype,offset=offset,shape=(int(m['POINTS'][0]),))
    return np.stack([data[k] for k in 'xyz'],axis=1)


def vox(points, leaf):
    keys=np.floor(points/leaf).astype(np.int32)
    _,idx=np.unique(keys,axis=0,return_index=True)
    return points[np.sort(idx)]


def qposes(path):
    rows=list(csv.DictReader(open(path)))
    t=np.array([float(r['%time'])*1e-9 for r in rows])
    xyz=np.array([[float(r['field.pose.position.'+c]) for c in 'xyz'] for r in rows])
    quat=np.array([[float(r['field.pose.orientation.'+c]) for c in 'xyzw'] for r in rows])
    return t,xyz,quat


def local_qposes(path):
    rows=list(csv.DictReader(open(path)))
    return (np.array([float(r['timestamp']) for r in rows]),
            np.array([[float(r[c]) for c in 'xyz'] for r in rows]),
            np.array([[float(r[c]) for c in ('qx','qy','qz','qw')] for r in rows]))


def angle(a):return (a+np.pi)%(2*np.pi)-np.pi


def interp_seed(times,pos,quat,t):
    idx=np.searchsorted(times,t)
    idx=max(1,min(len(times)-1,idx))
    f=np.clip((t-times[idx-1])/(times[idx]-times[idx-1]),0,1)
    xyz=pos[idx-1]*(1-f)+pos[idx]*f
    from scipy.spatial.transform import Slerp
    q=Rotation.from_quat([quat[idx-1],quat[idx]])
    orient=Slerp([0,1],q)(f)
    return xyz,orient


def icp(scan,seed,rot,tree,trim=0.75,max_iters=13):
    """Trimmed rigid xy+yaw and z matching in archived PCD frame."""
    # Preserve measured IMU roll/pitch; solve yaw, XYZ with raw held-out LiDAR.
    y0=rot.as_euler('ZYX')[0]
    Rtilt=Rotation.from_euler('z',-y0).as_matrix()@rot.as_matrix()
    local=scan@Rtilt.T
    pose=seed.copy()
    yy=y0
    for it in range(max_iters):
        c,s=math.cos(yy),math.sin(yy)
        Rxy=np.array([[c,-s],[s,c]])
        predxy=local[:,:2]@Rxy.T+pose[:2]
        world=np.column_stack((predxy,local[:,2]+pose[2]))
        d,inds=tree.query(world,k=1,distance_upper_bound=3.0,workers=2)
        ok=np.isfinite(d)
        if ok.sum()<140:return None
        dist=d[ok]
        lim=min(1.6,max(0.65,float(np.quantile(dist,trim))*1.08))
        ok=ok&(d<=lim)
        if ok.sum()<120:return None
        a=local[ok,:2].astype(np.float64)
        b=tree.data[inds[ok],:2]
        am=a.mean(0);bm=b.mean(0)
        u,_,vt=np.linalg.svd((a-am).T@(b-bm))
        rot_row=u@np.diag([1.0,np.linalg.det(u@vt)])@vt
        newyaw=math.atan2(rot_row[0,1],rot_row[0,0]);nt=bm-am@rot_row
        nz=np.median(tree.data[inds[ok],2]-local[ok,2])
        turn=np.clip(angle(newyaw-yy),-math.radians(5),math.radians(5))
        nt=pose[:2]+np.clip(nt-pose[:2],-1.2,1.2)
        translation_step=np.linalg.norm(nt-pose[:2])
        vertical_step=abs(np.clip(nz-pose[2],-.45,.45))
        yy=angle(yy+turn)
        pose[:2]=nt
        pose[2]+=np.clip(nz-pose[2],-.45,.45)
        if translation_step<.01 and vertical_step<.01 and abs(turn)<.0005 and it>4:break
    c,s=math.cos(yy),math.sin(yy)
    world=np.column_stack((local[:,:2]@np.array([[c,s],[-s,c]])+pose[:2],local[:,2]+pose[2]))
    d,ix=tree.query(world,k=1,distance_upper_bound=3.0,workers=2)
    valid=np.isfinite(d)
    residual=d[valid]
    if len(residual)<200:return None
    med=float(np.median(residual));p75=float(np.quantile(residual,.75))
    overlap=float(np.mean(d<1.0));score=float(np.sqrt(np.mean(np.minimum(residual,1.8)**2)))
    return pose,yy,med,p75,overlap,score


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scans',required=True)
    p.add_argument('--map',required=True)
    p.add_argument('--seed',required=True)
    p.add_argument('--local-odometry',required=True,
                   help='Same-bag healthy LiDAR+IMU *relative* motion, never an absolute GT')
    p.add_argument('--output',required=True)
    p.add_argument('--max-frames',type=int,default=0)
    args=p.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    t0=time.time()
    m=read_pcd_xyz(args.map).astype(np.float32)
    m=m[np.isfinite(m).all(axis=1)]
    m=m[(m[:,0]>-65)&(m[:,0]<155)&(m[:,1]>-205)&(m[:,1]<45)&(m[:,2]>-7)&(m[:,2]<20)]
    m=vox(m,.55)
    print('MAP DOWNSAMPLED',len(m),'time',round(time.time()-t0,1),'s',flush=True)
    tree=cKDTree(m.astype(np.float64),leafsize=32)
    obs=list(csv.DictReader(open(Path(args.scans)/'keyframes.csv')))
    if args.max_frames>0:obs=obs[:args.max_frames]
    times,pos,quats=qposes(args.seed)
    lt,lp,lq=local_qposes(args.local_odometry)
    prev=None;records=[]
    for idx,o in enumerate(obs):
        t=float(o['timestamp']);scan=np.load(Path(args.scans)/f'{int(o["seq"]):04d}.npy')
        scan=scan[::2] # hold deterministic independent points out for fitness checks
        coarse,r=interp_seed(times,pos,quats,t)
        local_p,local_R=interp_seed(lt,lp,lq,t)
        if prev is not None:
            old_t,old_xyz,old_R,old_local_p,old_local_R=prev
            # T_map_body(new) = T_map_body(prev) * inv(T_local_body(prev))
            #                   * T_local_body(new).  This is the correct SE(3)
            #                   *relative* motion; subtracting map/local XY
            #                   directly created frozen poses and 9 m jumps.
            delta_body=old_local_R.inv().apply(local_p-old_local_p)
            pred=old_xyz+old_R.apply(delta_body)
            r=old_R*old_local_R.inv()*local_R
        else:
            pred=coarse.copy()
        seeds=[('propagated',pred),('coarse',coarse)]
        if prev is not None:
            seeds.append(('previous',old_xyz))
        best=None
        for source,seed in seeds:
            result=icp(scan,seed.copy(),r,tree)
            if result is None:continue
            xyz,yaw,med,p75,overlap,score=result
            jump=np.linalg.norm(xyz[:2]-pred[:2])
            # Suppress road alias minima far from locally propagated motion.
            merit=score+.012*jump
            if best is None or merit<best[0]:best=(merit,source,xyz,yaw,med,p75,overlap,score,jump)
        if best is None:
            records.append({'timestamp':t,'x':np.nan,'y':np.nan,'z':np.nan,'yaw':np.nan,'fitness':np.nan,'overlap':0,'confidence':'unmatched'})
            print('NO_MATCH',idx,round(t-times[0],1),flush=True)
            continue
        merit,source,xyz,yaw,med,p75,overlap,score,jump=best
        # ICP can have an excellent score on visually/geometrically repetitive
        # roads yet propose a physically impossible jump. Do not let that
        # overwrite our same-bag relative-motion continuity constraint.
        inconsistent=prev is not None and (jump>3.0 or
                        np.linalg.norm(xyz[:2]-old_xyz[:2])>
                        5.0*max(.2,t-old_t))
        if inconsistent:
            print('REJECT_MAP_ALIAS',idx,'t',round(t-times[0],1),
                  'icp',xyz[:2].round(2),'pred',pred[:2].round(2),
                  'innovation',round(jump,2),flush=True)
            xyz=pred.copy()
            yaw=r.as_euler('ZYX')[0]
            source='motion_only_untrusted_map_fix'
        confidence=('propagated' if inconsistent else
                    'high' if med<.42 and overlap>.55 and jump<1.6 else
                    'medium' if med<.8 and overlap>.3 and jump<3.0 else 'low')
        records.append({'timestamp':t,'x':float(xyz[0]),'y':float(xyz[1]),'z':float(xyz[2]),'yaw':yaw,
                        'fitness':score,'median_residual_m':med,'p75_residual_m':p75,
                        'overlap':overlap,'seed_delta_m':jump,'confidence':confidence,'seed_origin':source})
        # Carry forward the corrected map-frame rotation, retaining the
        # current IMU tilt rather than interpreting yaw alone as attitude.
        tilt=Rotation.from_euler('z',-r.as_euler('ZYX')[0])*r
        new_R=Rotation.from_euler('z',yaw)*tilt
        prev=(t,xyz,new_R,local_p,local_R)
        if idx%10==0 or confidence=='low':
            print('scan',idx,'t',round(t-times[0],1),'xy',xyz[:2].round(2),'med',round(med,3),'overlap',round(overlap,2),'jump',round(jump,2),'confidence',confidence,'seed',source,flush=True)
    with (out/'registration_diagnostics.csv').open('w',newline='') as f:
        fields=['timestamp','x','y','z','yaw','fitness','median_residual_m','p75_residual_m','overlap','seed_delta_m','confidence','seed_origin']
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(records)
    print('REGISTRATION_DONE',len(records),'total seconds',round(time.time()-t0,1),flush=True)

if __name__=='__main__':main()
