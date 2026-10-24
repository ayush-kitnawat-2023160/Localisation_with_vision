#!/usr/bin/env python3
"""Conservative cross-keyframe XFeat 3D observation tracks, NOT bundle adjustment.
Reject desc matches that disagree in map 3D beyond a small threshold; export
multi-observed static-candidate landmarks for future map optimization.
"""
import argparse,json,csv,time
from pathlib import Path
import cv2,numpy as np
p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--out',required=True);p.add_argument('--max-3d-error-m',type=float,default=.45);p.add_argument('--max-frame-gap',type=int,default=2);args=p.parse_args()
out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
root=Path(args.map)/'visual';rows=list(csv.DictReader(open(root/'keyframes.csv')))
feats=[];offsets=[0];des=[];scores=[];ids=[];pts=[]
for r in rows:
 z=np.load(root/r['landmarks']);d=z['desc'].astype(np.float32);xyz=z['xyz'].astype(np.float32)
 feats.append({'desc':d,'xyz':xyz});offsets.append(offsets[-1]+len(d))
n=offsets[-1];parent=np.arange(n,dtype=np.int32);size=np.ones(n,dtype=np.int32)
def find(x):
 while parent[x]!=x:parent[x]=parent[parent[x]];x=parent[x]
 return x
def join(x,y):
 x=find(x);y=find(y)
 if x==y:return
 if size[x]<size[y]:x,y=y,x
 parent[y]=x;size[x]+=size[y]
m=cv2.BFMatcher(cv2.NORM_L2);pairs_attempt=0;matched_pairs=0;inconsistent=0
for i in range(len(feats)):
 for j in range(i+1,min(len(feats),i+1+args.max_frame_gap)):
  d0,d1=feats[i]['desc'],feats[j]['desc']
  if not len(d0) or not len(d1):continue
  raw=m.knnMatch(d0,d1,k=2)
  ms=[r[0] for r in raw if len(r)==2 and r[0].distance<.83*r[1].distance and r[0].distance<.9]
  pairs_attempt+=len(ms)
  for q in ms:
   p0=feats[i]['xyz'][q.queryIdx];p1=feats[j]['xyz'][q.trainIdx]
   if np.linalg.norm(p0-p1)>args.max_3d_error_m:inconsistent+=1;continue
   join(offsets[i]+q.queryIdx,offsets[j]+q.trainIdx);matched_pairs+=1
 if i%35==0:print('TRACK',i,'/',len(feats),'geometry_consistent_pairs',matched_pairs,flush=True)
xyz=np.vstack([x['xyz'] for x in feats]);rep=np.asarray([find(i) for i in range(n)],dtype=np.int32)
keys,inv,count=np.unique(rep,return_inverse=True,return_counts=True)
# Only tracks seen from >=2 distinct reference frames are considered anchors.
frameidx=np.concatenate([np.full(offsets[i+1]-offsets[i],i,np.int32) for i in range(len(feats))])
valid=np.flatnonzero(count>=2);centers=np.zeros((len(valid),3),np.float32);obs_count=count[valid].astype(np.int32);spread=np.zeros(len(valid),np.float32);first=np.zeros(len(valid),np.int32);last=np.zeros(len(valid),np.int32)
for k,idx in enumerate(valid):
 sel=np.flatnonzero(inv==idx);x=xyz[sel];centers[k]=np.median(x,axis=0);spread[k]=float(np.median(np.linalg.norm(x-centers[k],axis=1)));first[k]=frameidx[sel].min();last[k]=frameidx[sel].max()
np.savez_compressed(out/'visual_multiview_track_candidates.npz',xyz=centers,observation_count=obs_count,median_xyz_spread_m=spread,first_map_keyframe=first,last_map_keyframe=last)
report={'input_keyframes':len(rows),'input_3d_observations':int(n),'geometrically_consistent_crossframe_descriptor_matches':matched_pairs,'descriptor_matches_rejected_by_3d_inconsistency':inconsistent,'descriptor_matches_before_geometry':pairs_attempt,'distinct_track_groups_including_singletons':len(keys),'tracks_with_at_least_two_corresponding_3d_observations':len(valid),'observations_belonging_to_multiview_tracks':int(obs_count.sum()),'median_repeated_landmark_spread_m':float(np.median(spread)) if len(spread) else None,'max_correspondence_3d_error_m':args.max_3d_error_m,'reference_map_pose_is_raw_unoptimized':True,'moving_objects_permanently_filtered':False,'bundle_adjustment_done':False,'false_association_rate_not_measured':True}
(out/'track_qa.json').write_text(json.dumps(report,indent=2)+'\n');print('TRACK_QA_COMPLETE',json.dumps(report,indent=2),flush=True)
