#!/usr/bin/env python3
"""Low-overhead per-PID CPU breakdown for an isolated local Step3 Docker replay.

CPU percentages are HOST process CPU time / wall time, summed across cores.
This tool never controls the container. Samples every 4s by default and stops
when the named container exits. Excludes start/stop samples in its summary.
"""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import time
from collections import defaultdict

p=argparse.ArgumentParser()
p.add_argument('--container',required=True)
p.add_argument('--out',required=True,type=Path)
p.add_argument('--interval',type=float,default=4.0)
p.add_argument('--timeout',type=int,default=220)
args=p.parse_args()
assert args.interval>=1
args.out.parent.mkdir(parents=True,exist_ok=True)
hz=os.sysconf('SC_CLK_TCK')

def pids():
    result=subprocess.run(['docker','top',args.container,'-eo','pid,comm'],capture_output=True,text=True)
    if result.returncode:return None
    out=[]
    for line in result.stdout.splitlines()[1:]:
        fields=line.strip().split(None,1)
        if len(fields)==2 and fields[0].isdigit():out.append((int(fields[0]),fields[1]))
    return out

def cpu_time(pid):
    s=Path(f'/proc/{pid}/stat').read_text()
    z=s[s.rfind(')')+2:].split()
    return (int(z[11])+int(z[12]))/hz

def proc_label(pid,name):
    try:
        cmd=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except OSError:cmd=''
    for alias in ('step3_global_acquirer','fast_livo2_mapping','failsafe_map_localizer',
                  'rgbd_failsafe_vo.py','visual_place_hint.py','fast_livo2_sensor_adapter.py',
                  'lidar_relative_failsafe_frontend','rosbag','roslaunch','roscore','rosmaster'):
        if alias in cmd:return alias
    return name

beg=time.monotonic(); last={}; appeared=False; missing_after_start=0
rows=[]
with args.out.open('w',newline='') as f:
    writer=csv.writer(f)
    writer.writerow(['wall_elapsed_s','pid','process','cpu_percent','raw_command'])
    while time.monotonic()-beg<args.timeout:
        t=time.monotonic()
        found=pids()
        if found is None:
            if appeared:
                missing_after_start+=1
                if missing_after_start>=2:break
            time.sleep(args.interval)
            continue
        appeared=True;missing_after_start=0
        now={}
        for pid,comm in found:
            try: ticks=cpu_time(pid)
            except (OSError,ValueError,IndexError):continue
            label=proc_label(pid,comm)
            now[pid]=(t,ticks,label)
            if pid not in last:continue
            earlier,old_ticks,_=last[pid]
            delta=t-earlier
            if delta<=0:continue
            cpu=max(0,(ticks-old_ticks)/delta*100)
            elapsed=round(t-beg,2)
            writer.writerow([elapsed,pid,label,round(cpu,2),comm])
            rows.append((elapsed,label,cpu))
        last=now
        f.flush()
        time.sleep(args.interval)

# The first 12 seconds and last two timestamps often reflect setup/teardown.
if not rows:raise SystemExit('No process CPU samples (container never started or /proc unavailable)')
last_t=max(row[0] for row in rows)
group=defaultdict(lambda:[0.,0])
for elapsed,label,cpu in rows:
    if elapsed>=12 and elapsed<=last_t-8:
        group[label][0]+=cpu;group[label][1]+=1
print(f'Process CPU breakdown (% of one logical core), active window ending at t={last_t:.1f}s')
for name,(total,count) in sorted(group.items(),key=lambda x:x[1][0]/max(x[1][1],1),reverse=True):
    print(f'{name:35s} {total/count:7.1f}% ({count} samples)')
print(f'Saved {args.out}')
