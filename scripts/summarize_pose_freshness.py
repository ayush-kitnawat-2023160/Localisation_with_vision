#!/usr/bin/env python3
"""Offline percentile, dropout and stale-pose analysis of probe CSV."""
import argparse,csv,json,math,statistics
from collections import defaultdict
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('csv',type=Path);p.add_argument('--out',type=Path)
a=p.parse_args()
by=defaultdict(list);invalid=defaultdict(int)
with a.csv.open() as f:
 for row in csv.DictReader(f):
  source=row['source']
  if row['age_ms']:
   by[source].append((float(row['receiver_ros_time_sec']),float(row['age_ms'])))
  else:invalid[source]+=1

def quantile(nums,q):
 v=sorted(nums)
 if not v:return None
 k=(len(v)-1)*q;lo=int(k);hi=min(len(v)-1,lo+1)
 return round(v[lo]+(v[hi]-v[lo])*(k-lo),3)
report={'method':'ros_receiver_now_minus_header_stamp','rosbag_age_is_sim_clock_measurement_not_physical_end_to_end_latency':True,
        'sources':{}}
for s,records in sorted(by.items()):
 ages=[age for _,age in records];t=[rt for rt,_ in records]
 gap=[b-a for a,b in zip(t,t[1:]) if b>=a]
 report['sources'][s]={'samples':len(ages),'p50_age_ms':quantile(ages,.5),'p95_age_ms':quantile(ages,.95),
   'p99_age_ms':quantile(ages,.99),'maximum_age_ms':round(max(ages),3),
   'fraction_older_than_150ms':round(sum(x>150 for x in ages)/len(ages),4),
   'fraction_older_than_300ms':round(sum(x>300 for x in ages)/len(ages),4),
   'maximum_receiver_gap_s':round(max(gap),4) if gap else None,
   'invalid_or_clock_mismatched_samples':invalid[s]}
report['invalid_only']={k:v for k,v in invalid.items() if k not in by}
print(json.dumps(report,indent=2))
if a.out:a.out.write_text(json.dumps(report,indent=2)+'\n')
