#!/usr/bin/env python3
"""Independent GPS-free visual global relocalization via learned features.
Retrieval: same fast ORB-LSH place candidate index as Oct-20 baseline.
Verification: (SuperPoint+LightGlue) OR (XFeat+MNN/LighterGlue) + PnP.
Outputs diagnostics only; never publishes ROS poses or imports GPS/odom.
"""
import argparse,csv,json,time,math
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation
from learned_visual_backends import load_backend,extract,match_features

def main():
 p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--orb-map',required=True);p.add_argument('--queries',required=True);p.add_argument('--calibration',required=True);p.add_argument('--vendor',required=True);p.add_argument('--out',required=True);p.add_argument('--backend',choices=['xfeat_mnn','xfeat_lighterglue','superpoint_lightglue'],required=True);p.add_argument('--bags',nargs='+',default=['localization','localization_from2','localization_3']);p.add_argument('--candidate-budget',type=int,default=7);p.add_argument('--max-per-bag',type=int,default=0);p.add_argument('--sample-stride',type=int,default=1);p.add_argument('--device',default='cpu');p.add_argument('--max-kpts',type=int,default=1100);p.add_argument('--retrieval-mode',choices=['orb_lsh','learned_float_flann'],default='orb_lsh');args=p.parse_args()
 out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
 k=yaml.safe_load(open(args.calibration))['laserMapping'];fx,fy,cx,cy=[float(k[t]) for t in ('cam_fx','cam_fy','cam_cx','cam_cy')];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],np.float64)
 rows=list(csv.DictReader(open(Path(args.map)/'visual/keyframes.csv')));ref=[]
 for r in rows:
  d=np.load(Path(args.map)/'visual'/r['landmarks']);ref.append({'uv':d['uv'].astype(np.float32),'desc':d['desc'].astype(np.float32),'xyz':d['xyz'].astype(np.float32),'shape':tuple(d['image_size'].tolist())})
 if args.retrieval_mode=='orb_lsh':
  orows=list(csv.DictReader(open(Path(args.orb_map)/'visual/keyframes.csv')));assert len(orows)==len(ref)
  train=[np.load(Path(args.orb_map)/'visual'/r['landmarks'])['all_desc'][::6].copy() for r in orows]
  flann=cv2.FlannBasedMatcher(dict(algorithm=6,table_number=10,key_size=14,multi_probe_level=2),dict(checks=48))
 else:
  train=[np.load(Path(args.map)/'visual'/r['landmarks'])['all_desc'][::6].astype(np.float32).copy() for r in rows]
  flann=cv2.FlannBasedMatcher(dict(algorithm=1,trees=4),dict(checks=80))
 flann.add(train);flann.train()
 torch_backend=load_backend(args.backend,args.vendor,device=args.device,max_kpts=args.max_kpts)
 orb=cv2.ORB_create(nfeatures=2200,fastThreshold=8)
 imgs=list(csv.DictReader(open(Path(args.queries)/'queries.csv')))
 summary={'backend':args.backend,'device':args.device,'gps_used':False,'query_depth_used':False,'lidar_or_query_odom_used':False,'retrieval_backend':args.retrieval_mode,'candidate_budget':args.candidate_budget,'map_keyframes':len(ref),'bags':[],'reference_is_unsurveyed':True};allresults=[]
 def posesolve(matches,kp,xyz):
  if len(matches)<18:return None
  obj=xyz[matches[:,0]];pix=kp[matches[:,1]]
  # Filter duplicate query pixels caused by learned matcher outputs, if any.
  ok,rv,tv,I=cv2.solvePnPRansac(obj,pix,K,None,iterationsCount=110,reprojectionError=3.5,confidence=.995,flags=cv2.SOLVEPNP_EPNP)
  if not ok or I is None or len(I)<18:return None
  I=I.ravel();spreadu=np.ptp(pix[I,0]);spreadv=np.ptp(pix[I,1]);
  if spreadu<85 or spreadv<65:return None
  refined,rv,tv=cv2.solvePnP(obj[I],pix[I],K,None,rv,tv,True,flags=cv2.SOLVEPNP_ITERATIVE)
  if not refined:return None
  proj,_=cv2.projectPoints(obj[I],rv,tv,K,None);err=np.linalg.norm(proj.reshape(-1,2)-pix[I],axis=1)
  if float(np.median(err))>3.5:return None
  mat,_=cv2.Rodrigues(rv);x=-mat.T@tv.ravel()
  return {'inliers':len(I),'reprojection_px':float(np.median(err)),'map_xyz_m':x.tolist(),'rotation_xyzw':Rotation.from_matrix(mat.T).as_quat().tolist()}
 for name in args.bags:
  records=[r for r in imgs if r['bag']==name][::args.sample_stride]
  if args.max_per_bag:records=records[:args.max_per_bag]
  outrows=[];t0=time.perf_counter()
  print('START',name,len(records),'queries, backend',args.backend,flush=True)
  for i,record in enumerate(records):
   img=cv2.imread(str(Path(args.queries)/record['image']));
   tm=time.perf_counter();query=extract(torch_backend,img);qfeat_ms=(time.perf_counter()-tm)*1000
   tm=time.perf_counter();votes=np.zeros(len(ref),float)
   if args.retrieval_mode=='orb_lsh':
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY);okp,odesc=orb.detectAndCompute(gray,None)
    if odesc is not None:
     for pair in flann.knnMatch(np.ascontiguousarray(odesc[::4]),k=2):
      if len(pair)!=2:continue
      m0,m1=pair
      if m0.distance<.84*m1.distance and m0.distance<85:votes[m0.imgIdx]+=(85-m0.distance)/85
   else:
    for pair in flann.knnMatch(np.ascontiguousarray(query['desc'][::4],np.float32),k=2):
     if len(pair)!=2:continue
     m0,m1=pair
     if m0.distance<.93*m1.distance and m0.distance<1.25:votes[m0.imgIdx]+=(1.25-m0.distance)/1.25
   ranked=np.argsort(-votes)[:max(args.candidate_budget,1)]
   retrieve_ms=(time.perf_counter()-tm)*1000
   verified=[];match_ms=0;pnp_ms=0;paircounts=[]
   for idx in ranked:
    r=ref[idx]
    if len(r['desc'])<18:continue
    tm=time.perf_counter();m,ms=match_features(torch_backend,r,query);match_ms+=ms
    paircounts.append(int(len(m)))
    tm=time.perf_counter();res=posesolve(m,query['uv'],r['xyz']);pnp_ms+=(time.perf_counter()-tm)*1000
    if res is None:continue
    center=np.asarray([float(rows[idx][x]) for x in ('x','y','z')]);distance=float(np.linalg.norm(np.asarray(res['map_xyz_m'])-center))
    if distance>28:continue
    res.update(map_keyframe=int(rows[idx]['id']),map_camera_keyframe_distance_m=distance,raw_matched_pairs=int(len(m)))
    verified.append(res)
   verified.sort(key=lambda r:(r['inliers'],-r['reprojection_px']),reverse=True)
   win=verified[0] if verified else None;ambiguous=False
   if win:
    for alt in verified[1:]:
     if alt['inliers']<win['inliers']*.7:continue
     if np.linalg.norm(np.asarray(alt['map_xyz_m'][:2])-np.asarray(win['map_xyz_m'][:2]))>8:ambiguous=True;break
   x={'bag':name,'t_sec':float(record['query_sec']),'matched':win is not None,'ambiguous':ambiguous,'best':win,'num_verified_candidates':len(verified),'reference_ids':[int(rows[x]['id']) for x in ranked],'num_pair_matches':paircounts,'raw_query_learned_features':len(query['uv']),'feature_ms':round(qfeat_ms,2),'retrieval_ms':round(retrieve_ms,2),'learned_matching_ms':round(match_ms,2),'pnp_ms':round(pnp_ms,2),'total_ms':round(qfeat_ms+retrieve_ms+match_ms+pnp_ms,2)}
   outrows.append(x)
   if (i+1)%10==0:print(' bag',name,'i',i+1,'verified',sum(z['matched'] for z in outrows),'latest latency',round(x['total_ms']),flush=True)
  arr=[x['total_ms'] for x in outrows];passed=[x for x in outrows if x['matched']];S={'bag':name,'queries':len(outrows),'pnp_verified':len(passed),'pass_fraction':len(passed)/max(1,len(outrows)),'ambiguous':sum(x['ambiguous'] for x in outrows),'median_inliers':float(np.median([x['best']['inliers'] for x in passed])) if passed else None,'latency_p50_ms':float(np.median(arr)) if arr else None,'latency_p95_ms':float(np.percentile(arr,95)) if arr else None,'median_extraction_ms':float(np.median([x['feature_ms'] for x in outrows])) if arr else None,'median_retrieval_ms':float(np.median([x['retrieval_ms'] for x in outrows])) if arr else None,'median_match_ms':float(np.median([x['learned_matching_ms'] for x in outrows])) if arr else None,'wall_s':time.perf_counter()-t0}
  summary['bags'].append(S);allresults.extend(outrows);(out/(name+'_matches.json')).write_text(json.dumps(outrows,indent=2)+'\n');(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print('RESULT',json.dumps(S),flush=True)
 print('FINISHED',flush=True)
if __name__=='__main__':main()
