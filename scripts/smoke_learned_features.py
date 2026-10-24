import argparse,time,json
from pathlib import Path
import cv2,numpy as np,torch
from learned_visual_backends import load_backend,extract,match_features
p=argparse.ArgumentParser();p.add_argument('--vendor',required=True);p.add_argument('--map',required=True);p.add_argument('--backend',required=True);p.add_argument('--device',default='cpu');a=p.parse_args()
r=Path(a.map)/'visual';img0=cv2.imread(str(r/'images/00000.jpg'));img1=cv2.imread(str(r/'images/00001.jpg'))
t0=time.time();b=load_backend(a.backend,a.vendor,device=a.device,max_kpts=800)
print('LOADED',a.backend,'sec',round(time.time()-t0,2),flush=True)
x=extract(b,img0);y=extract(b,img1);print('EXTRACTED',len(x['uv']),len(y['uv']),'times',round(x['extract_ms'],1),round(y['extract_ms'],1),'descriptor dim',x['desc'].shape,flush=True)
z,ms=match_features(b,x,y);print('MATCHED',z.shape,'time_ms',round(ms,1),'sample',z[:2].tolist(),flush=True)
assert len(z)>15 and (z[:,0]<len(x['uv'])).all() and (z[:,1]<len(y['uv'])).all()
