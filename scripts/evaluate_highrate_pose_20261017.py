#!/usr/bin/env python3
"""Read-only diagnosis of 1x FAST-LIVO2 high-rate prediction: no GT used."""
import argparse,csv,json,re,statistics
from collections import defaultdict,Counter
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--out',type=Path);args=p.parse_args()
D=args.run
hi=list(csv.DictReader((D/'highrate_pose_data.csv').open()));mo=list(csv.DictReader((D/'pose_freshness.csv').open()))
base=list(csv.DictReader((D/'global_pose.csv').open()))

def calc(row,key):return float(row[key])
def percent(x,n):return round(float(np.percentile(x,n)),3) if len(x) else None

def ages(s):
 a=[float(r['age_ms']) for r in mo if r['source']==s and r['age_ms']]
 t=np.array([float(r['header_stamp_ros_sec']) for r in mo if r['source']==s and r['age_ms']])
 return {'samples':len(a),'age_p50_ms':percent(a,50),'age_p95_ms':percent(a,95),
         'age_p99_ms':percent(a,99),'max_timestamp_gap_s':round(float(np.max(np.diff(t))),4) if len(t)>2 else None,
         'samples_older_than_300ms':sum(v>300 for v in a)}

t=np.array([calc(r,'timestamp') for r in hi]);pos=np.array([[calc(r,k) for k in ['x','y','z']] for r in hi]);dg=np.diff(t);step=np.linalg.norm(np.diff(pos,axis=0),axis=1)
bt=np.array([float(r['field.header.stamp'])*1.e-9 for r in base]);bpos=np.array([[float(r['field.pose.position.'+k]) for k in 'xyz'] for r in base])
bt_mask=(t>=bt[0])&(t<=bt[-1])
interp=np.column_stack([np.interp(t[bt_mask],bt,bpos[:,i]) for i in range(3)])
proxy=np.linalg.norm(pos[bt_mask]-interp,axis=1)
proc=defaultdict(list)
if (D/'process_cpu.csv').exists():
 for r in csv.DictReader((D/'process_cpu.csv').open()):
  try:
   if 12<=float(r['wall_elapsed_s'])<=130:proc[r['process']].append(float(r['cpu_percent']))
  except(ValueError,KeyError):pass
log=(D/'launch.log').read_text(errors='ignore')
mode_counts=dict(Counter(r['mode'] for r in hi))
msgs=[]
for line in log.splitlines():
 if any(x in line for x in ('FULL_LIVO -> LV_IMU_FAILED','LV_IMU_FAILED -> FULL_LIVO','FULL_LIVO -> LIO_CAMERA_FAILED','LIO_CAMERA_FAILED -> FULL_LIVO')) and '[FAILSAFE]' in line:
  m=re.search(r'\[\d+\.\d+,\s*(\d+\.\d+)\]',line)
  if m:msgs.append({'ros_stamp':float(m.group(1)),'event':line.split('[FAILSAFE]')[-1].strip().split('(')[0].strip()})
summary_line=next((l.rsplit('summary ',1)[-1] for l in log.splitlines() if '[HIGH-RATE POSE DEV] summary' in l),'')
report={'run':str(D),'actual_play_rate':1.0,'has_independent_ground_truth':False,
        'highrate':{'n':len(hi),'mean_published_rate_hz':round(len(hi)/(t[-1]-t[0]),2) if len(t)>2 else None,
                    'median_interval_ms':round(1000*float(np.median(dg)),2),
                    'max_gap_s':round(float(dg.max()),4),'max_step_m':round(float(step.max()),4),
                    'steps_gt_0p85m':int((step>.85).sum()),'nonmonotonic_stamps':int((dg<=0).sum()),
                    'mode_counts':mode_counts,'dev_node_final_stats_line':summary_line},
        'freshness_original_global':ages('global_pose'),'freshness_highrate':ages('highrate_global'),
        'freshness_propagated_imu':ages('imu_propagated'),
        'difference_to_interpolated_existing_pose_m':{
           'p50':percent(proxy,50),'p95':percent(proxy,95),'max':round(float(proxy.max()),3),
           'interpretation':'same-run Stage3 global pose interpolated in time, NOT ground truth'},
        'process_cpu_pct_one_core':{k:round(float(statistics.mean(v)),2) for k,v in proc.items() if v},
        'mode_transitions':msgs,
        'deployment_approved':False,
        'critical_no_imu_failure_mode_publishing':not any(k in mode_counts for k in ('LV_IMU_FAILED','IMU_ONLY_BRIDGE'))}
if msgs:
 for i,m in enumerate(msgs):
  if '-> LV_IMU_FAILED' in m['event']:
   t1=next((x['ros_stamp'] for x in msgs[i+1:] if 'LV_IMU_FAILED -> FULL_LIVO' in x['event']),None)
   if t1 is not None:
    bad=int(((t>=m['ros_stamp'])&(t<t1)).sum())
    report['imu_failed_window_direct_published_count']=bad
    report['critical_no_imu_failure_mode_publishing']=bad==0 and report['critical_no_imu_failure_mode_publishing']
            
print(json.dumps(report,indent=2))
(args.out or D/'highrate_latency_audit.json').write_text(json.dumps(report,indent=2)+'\n')
