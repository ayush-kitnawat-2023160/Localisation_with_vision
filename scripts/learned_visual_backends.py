#!/usr/bin/env python3
"""Thin, pinned wrappers for official SuperPoint+LightGlue and XFeat.
Extracted keypoints always use original 960x600 reference pixel coordinates.
"""
import sys,time
from pathlib import Path
import cv2,numpy as np,torch

def load_backend(name,vendor,device='cpu',max_kpts=1024,resize_width=640,lightglue_layers=5):
    torch.set_num_threads(min(4,torch.get_num_threads()))
    vendor=Path(vendor)
    if name=='superpoint_lightglue':
        sys.path.insert(0,str(vendor/'LightGlue'))
        from lightglue import SuperPoint,LightGlue
        ex=SuperPoint(max_num_keypoints=max_kpts).eval().to(device)
        match=LightGlue(features='superpoint',flash=False,n_layers=lightglue_layers,depth_confidence=0.90,width_confidence=0.95).eval().to(device)
        return {'name':name,'device':device,'extractor':ex,'matcher':match,'resize_width':resize_width,'max_kpts':max_kpts}
    if name in ('xfeat_mnn','xfeat_lighterglue'):
        sys.path.insert(0,str(vendor/'accelerated_features'))
        from modules.xfeat import XFeat
        ex=XFeat(weights=str(vendor/'accelerated_features/weights/xfeat.pt'),top_k=max_kpts).eval().to(device)
        if name=='xfeat_lighterglue':
            from modules.lighterglue import LighterGlue
            glue=LighterGlue().eval().to(device)
        else:glue=None
        return {'name':name,'device':device,'extractor':ex,'matcher':glue,'resize_width':resize_width,'max_kpts':max_kpts}
    raise ValueError(name)

def extract(backend,bgr):
    h,w=bgr.shape[:2];width=backend['resize_width'];height=int(round(h*width/w))
    image=cv2.resize(bgr,(width,height),interpolation=cv2.INTER_AREA)
    name=backend['name'];device=backend['device']
    t0=time.perf_counter()
    if name=='superpoint_lightglue':
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        x=torch.from_numpy(gray.copy()).to(device).float().div_(255.0)[None,None]
        with torch.inference_mode():f=backend['extractor'].extract(x,resize=None)
        k=f['keypoints'][0].cpu().numpy();desc=f['descriptors'][0].cpu().numpy();scores=f['keypoint_scores'][0].cpu().numpy() if 'keypoint_scores' in f else np.ones(len(k),np.float32)
    else:
        rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        x=torch.from_numpy(rgb.copy()).permute(2,0,1)[None].float().to(device)
        with torch.inference_mode():f=backend['extractor'].detectAndCompute(x,top_k=backend['max_kpts'])[0]
        k=f['keypoints'].cpu().numpy();desc=f['descriptors'].cpu().numpy();scores=f['scores'].cpu().numpy()
    if device.startswith('cuda'):torch.cuda.synchronize()
    duration=time.perf_counter()-t0
    k=np.asarray(k,np.float32);k[:,0]*=w/width;k[:,1]*=h/height
    return {'uv':k.astype(np.float32),'desc':np.asarray(desc,np.float32),'scores':np.asarray(scores,np.float32),'shape':(w,h),'extract_ms':duration*1000}

def match_features(backend,a,b):
    device=backend['device'];name=backend['name'];t0=time.perf_counter()
    if name=='superpoint_lightglue':
        def tf(f):
            return {'keypoints':torch.from_numpy(f['uv']).to(device)[None],
                    'descriptors':torch.from_numpy(f['desc']).to(device)[None],
                    'image_size':torch.tensor(f['shape'],device=device)[None]}
        with torch.inference_mode():out=backend['matcher']({'image0':tf(a),'image1':tf(b)})
        idx=out['matches'][0].cpu().numpy().astype(np.int32)
    else:
        d0=torch.from_numpy(a['desc']).float().to(device);d1=torch.from_numpy(b['desc']).float().to(device)
        if name=='xfeat_mnn':
            with torch.inference_mode():i0,i1=backend['extractor'].match(d0,d1,min_cossim=0.77)
            idx=torch.stack((i0,i1),1).cpu().numpy().astype(np.int32)
        else:
            inp={
              'keypoints0':torch.from_numpy(a['uv']).float().to(device)[None],
              'keypoints1':torch.from_numpy(b['uv']).float().to(device)[None],
              'descriptors0':d0[None], 'descriptors1':d1[None],
              'image_size0':torch.tensor(a['shape'],device=device)[None],
              'image_size1':torch.tensor(b['shape'],device=device)[None],
            }
            with torch.inference_mode():result=backend['matcher'](inp,min_conf=.1)
            idx=result['matches'][0].cpu().numpy().astype(np.int32)
    if device.startswith('cuda'):torch.cuda.synchronize()
    return idx,(time.perf_counter()-t0)*1000
