#!/usr/bin/env python3
"""Extract GPS timestamps/altitude for a weak, non-surveyed vertical sanity proxy."""
import argparse,csv
from pathlib import Path
import rosbag
p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--output',required=True);a=p.parse_args()
Path(a.output).parent.mkdir(parents=True,exist_ok=True)
with rosbag.Bag(a.bag) as b, open(a.output,'w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=['timestamp','latitude','longitude','altitude','fix_status','covariance_xx','covariance_yy']);w.writeheader()
 n=0
 for topic,msg,_ in b.read_messages(topics=['/mavros/global_position/raw/fix']):
  w.writerow(dict(timestamp=msg.header.stamp.to_sec(),latitude=msg.latitude,longitude=msg.longitude,altitude=msg.altitude,fix_status=msg.status.status,covariance_xx=msg.position_covariance[0],covariance_yy=msg.position_covariance[4]));n+=1
print(Path(a.bag).name,n)
