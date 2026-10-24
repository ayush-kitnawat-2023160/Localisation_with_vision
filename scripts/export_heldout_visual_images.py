#!/usr/bin/env python3
"""Select held-out ZED X RGB images for repeatable learned-feature experiments."""
import argparse,csv,json
from pathlib import Path
import cv2,numpy as np,rosbag
p=argparse.ArgumentParser();p.add_argument('--bags',nargs='+',required=True);p.add_argument('--out',required=True);p.add_argument('--period',type=float,default=2.0);args=p.parse_args()
out=Path(args.out);assert not out.exists() or not list(out.iterdir()),'Refuse overwrite queries'
out.mkdir(parents=True,exist_ok=True);rows=[]
for bagfile in args.bags:
 name=Path(bagfile).stem;dest=out/name;dest.mkdir();last=-1e20;first=None;count=0
 with rosbag.Bag(bagfile) as bag:
  for _,m,_ in bag.read_messages(topics=['/zedx/zed_node/left/image_rect_color']):
   ts=m.header.stamp.to_sec()
   if first is None:first=ts
   if ts-last<args.period:continue
   last=ts;a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)
   if m.encoding=='bgra8':im=cv2.cvtColor(a[:,:m.width*4].reshape(m.height,m.width,4),cv2.COLOR_BGRA2BGR)
   elif m.encoding=='bgr8':im=a[:,:m.width*3].reshape(m.height,m.width,3)
   else:continue
   path=f'{name}/{count:05d}.jpg';cv2.imwrite(str(out/path),im,[cv2.IMWRITE_JPEG_QUALITY,91]);rows.append({'bag':name,'timestamp':ts,'query_sec':ts-first,'image':path});count+=1
 print('EXPORTED',name,count,flush=True)
with (out/'queries.csv').open('w',newline='') as f:
 wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
(out/'metadata.json').write_text(json.dumps({'heldout_bags':args.bags,'query_period_s':args.period,'sampled_images':len(rows),'gps_used':False,'heldout_mapping_overlap':False},indent=2)+'\n')
