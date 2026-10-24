#!/usr/bin/env python3
"""Per-process RSS and CPU sampling for ROS VIO benchmark.

This measures the estimator PID (and optionally its children), not roscore,
rosbag playback, ZED stereo depth generation, or other cart software.
One CPU core at full utilization equals 100% of one-core CPU occupancy.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import psutil


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--out',required=True)
    p.add_argument('--interval',type=float,default=.20)
    p.add_argument('command',nargs=argparse.REMAINDER)
    a=p.parse_args()
    cmd=a.command
    if cmd and cmd[0]=='--':cmd=cmd[1:]
    if not cmd:raise RuntimeError('Missing estimator command')
    proc=subprocess.Popen(cmd)
    target=psutil.Process(proc.pid)
    start=time.monotonic()
    samples=[]
    def forward(signum,frame):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT if signum==signal.SIGINT else signal.SIGTERM)
    signal.signal(signal.SIGINT,forward)
    signal.signal(signal.SIGTERM,forward)
    while proc.poll() is None:
        try:
            family=[target]+target.children(recursive=True)
            cpu_time=0.
            rss=0
            threads=0
            for child in family:
                try:
                    ct=child.cpu_times()
                    cpu_time+=ct.user+ct.system
                    rss+=child.memory_info().rss
                    threads+=child.num_threads()
                except (psutil.NoSuchProcess,psutil.AccessDenied):pass
            samples.append({'elapsed_s':time.monotonic()-start,
                            'process_cpu_seconds':cpu_time,
                            'rss_bytes':rss,'threads':threads})
        except (psutil.NoSuchProcess,psutil.AccessDenied):pass
        time.sleep(a.interval)
    elapsed=time.monotonic()-start
    rc=proc.wait()
    cpu_end=samples[-1]['process_cpu_seconds'] if samples else None
    summary={'command':cmd,'pid':proc.pid,'exit_code':rc,
             'wall_duration_sec':elapsed,'sample_count':len(samples),
             'process_cpu_seconds_last_sample':cpu_end,
             'mean_cpu_cores':cpu_end/elapsed if cpu_end is not None else None,
             'mean_cpu_percent_of_one_core':100*cpu_end/elapsed if cpu_end is not None else None,
             'peak_rss_mib':max((s['rss_bytes'] for s in samples),default=0)/(1024*1024),
             'peak_threads':max((s['threads'] for s in samples),default=0),
             'cpu_percent_one_core_definition':'100 means full use of one core; 200 means two CPU cores',
             'note':'Tracks this estimator process and spawned children only; excludes ROS master, bag decoding, GPU ZED depth engine'}
    with open(a.out,'w') as f:json.dump({'summary':summary,'samples':samples},f,indent=2)
    print('PROFILE_ESTIMATOR',json.dumps(summary,indent=2),flush=True)
    # Terminated under SIGINT by the experiment harness is an expected stop.
    return 0 if rc in (0, -signal.SIGINT, -signal.SIGTERM, 130, 143) else rc


if __name__=='__main__':sys.exit(main())
