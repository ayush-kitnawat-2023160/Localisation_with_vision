#!/usr/bin/env python3
"""Local-only, GPS-free image->visual 3D map candidate service (no ROS pose).
Worker runs in Torch container; ROS Noetic bridge can stay in its native image.

SECURITY: binds 127.0.0.1 only, max 2MB image requests, max one inference
at a time (429 if busy). For offline cart development only.
"""
import argparse,csv,json,threading,time,traceback,hashlib
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import cv2,numpy as np,yaml,torch
from scipy.spatial.transform import Rotation
from learned_visual_backends import load_backend,extract,match_features

p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--orb-map',required=True);p.add_argument('--vendor',required=True);p.add_argument('--calibration',required=True);p.add_argument('--backend',default='xfeat_mnn');p.add_argument('--device',default='cpu');p.add_argument('--port',type=int,default=18090);p.add_argument('--candidates',type=int,default=7);p.add_argument('--retrieval-mode',choices=['orb_lsh','learned_float_flann'],default='learned_float_flann');a=p.parse_args()
assert a.port>1024
r=list(csv.DictReader(open(Path(a.map)/'visual/keyframes.csv')))
def file_sha256(path):
 h=hashlib.sha256()
 with open(path,'rb') as f:
  while True:
   b=f.read(1024*1024)
   if not b:break
   h.update(b)
 return h.hexdigest()
learned_map_hash=file_sha256(Path(a.map)/'visual/keyframes.csv')
orb_map_hash=file_sha256(Path(a.orb_map)/'visual/keyframes.csv')
calibration_hash=file_sha256(a.calibration)
frames=[]
for row in r:
 d=np.load(Path(a.map)/'visual'/row['landmarks']);frames.append({'uv':d['uv'].astype(np.float32),'desc':d['desc'].astype(np.float32),'xyz':d['xyz'].astype(np.float32),'shape':tuple(d['image_size'].tolist())})
if a.retrieval_mode=='orb_lsh':
 orbrows=list(csv.DictReader(open(Path(a.orb_map)/'visual/keyframes.csv')));assert len(r)==len(orbrows)
 train=[np.load(Path(a.orb_map)/'visual'/row['landmarks'])['all_desc'][::6].copy() for row in orbrows]
 index=cv2.FlannBasedMatcher(dict(algorithm=6,table_number=10,key_size=14,multi_probe_level=2),dict(checks=48))
else:
 train=[np.load(Path(a.map)/'visual'/row['landmarks'])['all_desc'][::6].astype(np.float32).copy() for row in r]
 index=cv2.FlannBasedMatcher(dict(algorithm=1,trees=4),dict(checks=80))
index.add(train);index.train();orb=cv2.ORB_create(nfeatures=2200,fastThreshold=8)
model=load_backend(a.backend,a.vendor,a.device,max_kpts=1100,resize_width=640)
cal=yaml.safe_load(open(a.calibration))['laserMapping'];fx,fy,cx,cy=[float(cal[x]) for x in ('cam_fx','cam_fy','cam_cx','cam_cy')];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],np.float64)
expected_width=int(cal['cam_width']);expected_height=int(cal['cam_height'])
if expected_width<32 or expected_height<32 or not frames:
 raise ValueError('Invalid empty learned map or camera calibration dimensions')
if any(f['shape']!=(expected_width,expected_height) for f in frames):
 raise ValueError('Learned map keyframe pixel dimensions differ from calibration; do not resize secretly')
sema=threading.Semaphore(1)

def localize(image):
 t0=time.perf_counter();query=extract(model,image);feat_ms=(time.perf_counter()-t0)*1000
 t0=time.perf_counter();votes=np.zeros(len(frames),float)
 if a.retrieval_mode=='orb_lsh':
  kp,desc=orb.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),None)
  if desc is not None:
   for pair in index.knnMatch(np.ascontiguousarray(desc[::4]),k=2):
    if len(pair)!=2:continue
    x,y=pair
    if x.distance<.84*y.distance and x.distance<85:votes[x.imgIdx]+=(85-x.distance)/85
 else:
  for pair in index.knnMatch(np.ascontiguousarray(query['desc'][::4],np.float32),k=2):
   if len(pair)!=2:continue
   x,y=pair
   if x.distance<.93*y.distance and x.distance<1.25:votes[x.imgIdx]+=(1.25-x.distance)/1.25
 top=np.argsort(-votes)[:a.candidates]
 retrieve_ms=(time.perf_counter()-t0)*1000
 accepted=[];matching_ms=0
 for id in top:
  frame=frames[id]
  if len(frame['xyz'])<18:continue
  try:mi,ms=match_features(model,frame,query)
  except Exception:continue
  matching_ms+=ms
  if len(mi)<18:continue
  pts=frame['xyz'][mi[:,0]];pix=query['uv'][mi[:,1]]
  ok,rv,tv,I=cv2.solvePnPRansac(pts,pix,K,None,iterationsCount=110,reprojectionError=3.5,confidence=.995,flags=cv2.SOLVEPNP_EPNP)
  if not ok or I is None or len(I)<18:continue
  I=I.ravel()
  if np.ptp(pix[I,0])<85 or np.ptp(pix[I,1])<65:continue
  refined,rv,tv=cv2.solvePnP(pts[I],pix[I],K,None,rv,tv,True,flags=cv2.SOLVEPNP_ITERATIVE)
  if not refined:continue
  projected,_=cv2.projectPoints(pts[I],rv,tv,K,None)
  err=float(np.median(np.linalg.norm(projected.reshape(-1,2)-pix[I],axis=1)))
  if err>3.5:continue
  R,_=cv2.Rodrigues(rv);pos=-R.T@tv.ravel();anchor=np.array([float(r[id][key]) for key in ['x','y','z']])
  if np.linalg.norm(pos-anchor)>28:continue
  accepted.append({'id':int(id),'inliers':int(len(I)),'reprojection_px':err,'T_map_camera_xyz':pos.tolist(),'T_map_camera_qxyzw':Rotation.from_matrix(R.T).as_quat().tolist()})
 accepted.sort(key=lambda x:(x['inliers'],-x['reprojection_px']),reverse=True)
 ambiguous=False
 if accepted:
  for alt in accepted[1:]:
   if alt['inliers']>=.7*accepted[0]['inliers'] and np.linalg.norm(np.asarray(accepted[0]['T_map_camera_xyz'][:2])-np.asarray(alt['T_map_camera_xyz'][:2]))>8:
    ambiguous=True;break
 return {'matched':bool(accepted),'ambiguous':ambiguous,'best':accepted[0] if accepted else None,'top_alternates':accepted[1:3],'retrieval_mode':a.retrieval_mode,'image_feature_count':len(query['uv']),'candidate_ids':top.astype(int).tolist(),'feature_ms':feat_ms,'retrieval_ms':retrieve_ms,'match_ms':matching_ms,'total_ms':(time.perf_counter()-t0)*1000+feat_ms}
class Handler(BaseHTTPRequestHandler):
 def do_GET(self):
  if self.path!='/health':self.send_error(404);return
  self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(json.dumps({'ready':True,'backend':a.backend,'gps_used':False,'map_size':len(frames),'retrieval_mode':a.retrieval_mode,'map_sha256':learned_map_hash,'orb_map_sha256':orb_map_hash,'calibration_sha256':calibration_hash,'expected_image_width':expected_width,'expected_image_height':expected_height}).encode())
 def do_POST(self):
  if self.path!='/localize':self.send_error(404);return
  try:n=int(self.headers.get('Content-Length','-1'))
  except: n=-1
  if n<=0 or n>2_000_000:self.send_error(413);return
  if not sema.acquire(blocking=False):self.send_error(429,'BUSY; retry with a newer image');return
  try:
   jpg=self.rfile.read(n);im=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
   if im is None or im.shape[:2]!=(expected_height,expected_width):
    self.send_error(400,'image pixel dimensions do not match learned map and calibrated intrinsics');return
   data=localize(im);data['stamp_s']=float(self.headers.get('X-Sensor-Stamp','0'))
   buf=json.dumps(data,allow_nan=False).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(buf)));self.end_headers();self.wfile.write(buf)
  except Exception as e: traceback.print_exc();self.send_error(500,str(e)[:120])
  finally:sema.release()
 def log_message(self,fmt,*args):pass
print('VISUAL_WORKER_READY backend=%s keyframes=%d retrieval=%s url=http://127.0.0.1:%d'%(a.backend,len(frames),a.retrieval_mode,a.port),flush=True)
ThreadingHTTPServer(('127.0.0.1',a.port),Handler).serve_forever()
