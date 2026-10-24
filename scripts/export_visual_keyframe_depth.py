#!/usr/bin/env python3
"""Extract ONLY selected ZED X registered-depth frames, never interpolate time."""
import argparse,csv,json
from pathlib import Path
import numpy as np,rosbag
p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--visual-map',required=True);p.add_argument('--output',required=True);args=p.parse_args()
root=Path(args.output);assert not root.exists() or not list(root.iterdir()),'Refuse overwrite'
root.mkdir(parents=True,exist_ok=True)
rows=list(csv.DictReader(open(Path(args.visual_map)/'visual/keyframes.csv')))
wanted={round(float(row['timestamp']),5):int(row['id']) for row in rows}
found=set();tot=0
with rosbag.Bag(args.bag) as bag:
 for _,m,_ in bag.read_messages(topics=['/zedx/zed_node/depth/depth_registered']):
  tot+=1;idx=wanted.get(round(m.header.stamp.to_sec(),5))
  if idx is None or idx in found:continue
  assert m.encoding=='32FC1',m.encoding
  dep=np.frombuffer(m.data,np.float32).reshape(m.height,m.step//4)[:,:m.width]
  np.savez_compressed(str(root/f'{idx:05d}.npz'),depth=dep)
  found.add(idx)
  if len(found)%30==0:print('depth exported',len(found),'of',len(rows),flush=True)
report={'total_depth_messages_read':tot,'selected_reference_frames':len(rows),'exact_matched':len(found),'missing_ids':sorted(set(range(len(rows)))-found),'forbidden_interpolation':True}
(root/'extraction.json').write_text(json.dumps(report,indent=2)+'\n');print('DONE',json.dumps(report))
