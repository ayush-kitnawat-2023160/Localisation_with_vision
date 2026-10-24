#!/usr/bin/env python3
"""Independent held-out ZED X image -> visual map global retrieval + ORB 3D PnP.
No GPS, LiDAR, IMU, prior-map-pose, query depth, or pre-existing query trajectory
is supplied to the localization algorithm. Map is built with mapping bag poses.
"""
import argparse,csv,json,math,time
from pathlib import Path
import cv2,numpy as np,rosbag,yaml

def img_bgr(m):
    a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)
    if m.encoding=='bgra8':return cv2.cvtColor(a[:,:m.width*4].reshape(m.height,m.width,4),cv2.COLOR_BGRA2BGR)
    if m.encoding=='bgr8':return a[:,:m.width*3].reshape(m.height,m.width,3)
    if m.encoding=='rgb8':return cv2.cvtColor(a[:,:m.width*3].reshape(m.height,m.width,3),cv2.COLOR_RGB2BGR)
    raise RuntimeError('bad encoding: '+str(m.encoding))
def quat(T):
    from tf.transformations import quaternion_from_matrix
    return quaternion_from_matrix(T)
def match_one(qkp,qdesc,order,features,entries,K,matcher,max_candidates):
    # Exact geometric verification, unlike the appearance-only retrieval rank.
    successes=[];summaries=[]
    for idx in order[:max_candidates]:
        feat=features[idx];ref=feat['desc'];xyz=feat['xyz'];
        if len(ref)<18:continue
        raw=matcher.knnMatch(ref,qdesc,k=2)
        choices=[x[0] for x in raw if len(x)==2 and x[0].distance<0.78*x[1].distance and x[0].distance<74]
        if len(choices)<16:continue
        # Prevent multiple reference descriptors voting for same query keypoint.
        byquery={}
        for x in sorted(choices,key=lambda x:x.distance):
            if x.trainIdx not in byquery:byquery[x.trainIdx]=x
        matches=list(byquery.values())
        if len(matches)<16:continue
        obj=np.asarray([xyz[x.queryIdx] for x in matches],np.float32)
        pix=np.asarray([qkp[x.trainIdx].pt for x in matches],np.float32)
        ok,rv,tv,inliers=cv2.solvePnPRansac(obj,pix,K,None,iterationsCount=130,reprojectionError=3.5,confidence=.995,flags=cv2.SOLVEPNP_EPNP)
        if not ok or inliers is None or len(inliers)<14:continue
        I=inliers.reshape(-1)
        if len(I)<18 or np.ptp(pix[I,0])<85 or np.ptp(pix[I,1])<65:continue
        ok,rv,tv=cv2.solvePnP(obj[I],pix[I],K,None,rv,tv,True,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:continue
        proj,_=cv2.projectPoints(obj[I],rv,tv,K,None)
        errs=np.linalg.norm(proj.reshape(-1,2)-pix[I],axis=1);e=float(np.median(errs))
        if e>3.5:continue
        R,_=cv2.Rodrigues(rv);T_c_m=np.eye(4);T_c_m[:3,:3]=R;T_c_m[:3,3]=tv[:,0];T_m_c=np.linalg.inv(T_c_m)
        mappos=np.asarray([float(entries[idx][k]) for k in ('x','y','z')]);dist=float(np.linalg.norm(T_m_c[:3,3]-mappos))
        if dist>28:continue
        record={'keyframe_id':int(entries[idx]['id']),'inliers':len(I),'reproj_median_px':e,'mappos_m':T_m_c[:3,3].tolist(),'rotation_xyzw':quat(T_m_c).tolist(),'map_keyframe_distance_m':dist,'coverage_u_px':float(np.ptp(pix[I,0])),'coverage_v_px':float(np.ptp(pix[I,1]))}
        successes.append(record)
    successes.sort(key=lambda r:(r['inliers'],-r['reproj_median_px']),reverse=True)
    winner=successes[0] if successes else None
    ambiguity=False
    if winner is not None:
        for alt in successes[1:]:
            if alt['inliers']<0.7*winner['inliers']:continue
            xy=np.linalg.norm(np.asarray(alt['mappos_m'][:2])-np.asarray(winner['mappos_m'][:2]))
            if xy>8:ambiguity=True;break
    return winner,ambiguity,successes[:4]

def main():
    p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--calibration',required=True);p.add_argument('--bags',nargs='+',required=True);p.add_argument('--out',required=True);p.add_argument('--period',type=float,default=2.0);p.add_argument('--max-queries',type=int,default=0);p.add_argument('--max-candidates',type=int,default=14);p.add_argument('--retrieval-stride',type=int,default=2);p.add_argument('--index-stride',type=int,default=3);args=p.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True);maproot=Path(args.map)/'visual';entries=list(csv.DictReader(open(maproot/'keyframes.csv')))
    features=[];flann_train=[]
    print('LOADING MAP',len(entries),'visual keyframes',flush=True)
    for e in entries:
        f=np.load(maproot/e['landmarks']);feat={k:f[k] for k in ['xyz','desc']};features.append(feat)
        # Appearance descriptors include features without reliable depth, so
        # retrieval remains possible when depth coverage is sparse.
        raw=f['all_desc'];flann_train.append(np.ascontiguousarray(raw[::args.index_stride]))
    flann=cv2.FlannBasedMatcher(dict(algorithm=6,table_number=10,key_size=14,multi_probe_level=2),dict(checks=48))
    tindex=time.perf_counter();flann.add(flann_train);flann.train();index_sec=time.perf_counter()-tindex
    print('FLANN_ORB_LSH_INDEX_READY training_sec=%.2f descriptors=%d'%(index_sec,sum(map(len,flann_train))),flush=True)
    config=yaml.safe_load(open(args.calibration))['laserMapping'];fx,fy,cx,cy=[float(config[k]) for k in ('cam_fx','cam_fy','cam_cx','cam_cy')]
    K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype=np.float64)
    orb=cv2.ORB_create(nfeatures=2200,fastThreshold=8);bf=cv2.BFMatcher(cv2.NORM_HAMMING)
    report=[];allrows=[]
    for bagpath in args.bags:
        bagname=Path(bagpath).stem;first=None;last=-1e15;seen=0;rows=[];times=[];start=time.perf_counter()
        print('START HELDOUT',bagname,flush=True)
        with rosbag.Bag(bagpath) as bag:
            for topic,m,rs in bag.read_messages(topics=['/zedx/zed_node/left/image_rect_color']):
                seen+=1;t=m.header.stamp.to_sec()
                if first is None:first=t
                if t-last<args.period:continue
                last=t
                qstart=time.perf_counter()
                try:bgr=img_bgr(m)
                except:continue
                gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
                kps,desc=orb.detectAndCompute(gray,None);tfeat=time.perf_counter()-qstart
                if desc is None or len(kps)<40:
                    rows.append({'bag':bagname,'query_sec':t-first,'raw_query_orb':0,'verified':False,'reason':'no_ORB'})
                    continue
                t0=time.perf_counter()
                # Genuine global map search: no GPS/odom spatial region supplied.
                lookup=flann.knnMatch(np.ascontiguousarray(desc[::args.retrieval_stride]),k=2)
                votes=np.zeros(len(entries),np.float64)
                for pair in lookup:
                    if len(pair)!=2:continue
                    x,y=pair
                    if x.distance<0.84*y.distance and x.distance<85:
                        votes[x.imgIdx]+=max(0.05,(85-x.distance)/85.0)
                top=np.argsort(-votes)[:args.max_candidates]
                tretr=time.perf_counter()-t0
                t0=time.perf_counter()
                w,amb,alts=match_one(kps,desc,top,features,entries,K,bf,args.max_candidates)
                tgeom=time.perf_counter()-t0
                result={'bag':bagname,'query_sec':float(t-first),'raw_query_orb':len(desc),'visual_retrieval_top_votes':float(votes[top[0]]),'retrieved_keyframe':int(entries[top[0]]['id']),'pnp_verified':w is not None,'ambiguous':amb,'winner':w,'alternate_verified':alts[1:] if w else [],'feature_ms':round(tfeat*1000,1),'retrieval_ms':round(tretr*1000,1),'geometry_ms':round(tgeom*1000,1)}
                rows.append(result)
                if len(rows)%8==0:print('  bag',bagname,'query',len(rows),'t',round(t-first,1),'PnP_verified',sum(x.get('pnp_verified',False) for x in rows),'latest_votes',round(result['visual_retrieval_top_votes'],1),flush=True)
                if args.max_queries and len(rows)>=args.max_queries:break
        good=[r for r in rows if r.get('pnp_verified') and not r.get('ambiguous')];usable=[r for r in rows if r.get('pnp_verified')];
        def pct(x,pct):return float(np.percentile(x,pct)) if x else None
        measurements={'bag':bagname,'rgb_messages_processed':seen,'queries_attempted':len(rows),'pnp_verified':len(usable),'unambiguous_verified':len(good),'ambiguous_candidates':sum(x.get('ambiguous',False) for x in rows),'pnp_verified_fraction':len(usable)/max(1,len(rows)),'unambiguous_fraction':len(good)/max(1,len(rows)),'median_inliers':pct([r['winner']['inliers'] for r in usable],50),'median_reprojection_px':pct([r['winner']['reproj_median_px'] for r in usable],50),'p50_total_match_ms':pct([r['retrieval_ms']+r['geometry_ms'] for r in rows if 'retrieval_ms' in r],50),'p95_total_match_ms':pct([r['retrieval_ms']+r['geometry_ms'] for r in rows if 'retrieval_ms' in r],95),'wall_s':time.perf_counter()-start}
        print('COMPLETED',json.dumps(measurements),flush=True)
        report.append(measurements);allrows.extend(rows)
        (out/(bagname+'_visual_matches.json')).write_text(json.dumps(rows,indent=2)+'\n')
        (out/'summary.json').write_text(json.dumps({'flann_index_build_sec':index_sec,'map_keyframes':len(entries),'bags':report,'cold_search':True,'retrieval_stride':args.retrieval_stride,'index_stride':args.index_stride,'gps_used':False,'lidar_used_for_query':False,'ground_truth_available':False},indent=2)+'\n')
    print('DONE',flush=True)
if __name__=='__main__':main()
