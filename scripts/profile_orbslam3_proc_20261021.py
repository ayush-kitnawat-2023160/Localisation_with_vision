#!/usr/bin/env python3
"""CPU ticks/RSS sampler for an isolated estimator process using /proc only."""
import argparse
import json
import os
import signal
import subprocess
import sys
import time


def sample(pid):
    with open('/proc/%d/stat'%pid) as f:part=f.read().split(') ',1)[1].split()
    # split starts with Linux stat field 3; utime field14-> index11, stime15->index12
    cpu=(int(part[11])+int(part[12]))/os.sysconf('SC_CLK_TCK')
    status=open('/proc/%d/status'%pid).read()
    def kb(name):
        for line in status.splitlines():
            if line.startswith(name+':'):return int(line.split()[1])
        return 0
    return cpu,kb('VmRSS'),kb('VmHWM'),kb('Threads')


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',required=True)
    ap.add_argument('command',nargs=argparse.REMAINDER)
    a=ap.parse_args()
    cmd=a.command[1:] if a.command and a.command[0]=='--' else a.command
    if not cmd:raise ValueError('Missing command')
    child=subprocess.Popen(cmd)
    beginning=time.monotonic()
    rows=[]
    stopped_at=None
    def forward(*_):
        nonlocal stopped_at
        if stopped_at is None:stopped_at=time.monotonic()
        if child.poll() is None:child.send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT,forward)
    signal.signal(signal.SIGTERM,forward)
    while child.poll() is None:
        now=time.monotonic()
        try:
            cpu,rss,peak,threads=sample(child.pid)
            rows.append({'t':now-beginning,'epoch_s':time.time(),'cpu_seconds':cpu,
                         'rss_mib':rss/1024,'max_rss_mib':peak/1024,
                         'threads':threads})
        except (FileNotFoundError,ProcessLookupError):pass
        if stopped_at is not None and now-stopped_at>22:
            child.kill()
        time.sleep(.25)
    rc=child.wait()
    elapsed=time.monotonic()-beginning
    cpu=rows[-1]['cpu_seconds'] if rows else 0.
    result={'process_command':cmd,'exit_code':rc,
            'wall_sec':elapsed,'cpu_seconds':cpu,
            'mean_cpu_cores':cpu/elapsed if elapsed else None,
            'peak_rss_mib':max((r['max_rss_mib'] for r in rows),default=0),
            'peak_threads':max((r['threads'] for r in rows),default=0),
            'includes_rosbag_play':False,'includes_ZED_depth_generation_GPU':False,
            'samples':rows}
    with open(a.output,'w') as f:json.dump(result,f,indent=2)
    print('ORB_PROCESS_PROFILE',json.dumps({k:v for k,v in result.items()
           if k!='samples'},indent=2),flush=True)
    return 0 if rc in (0,-signal.SIGINT,-signal.SIGTERM,130,143) else rc


if __name__=='__main__':sys.exit(main())
