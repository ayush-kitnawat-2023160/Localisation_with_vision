#!/usr/bin/env python3
"""Offline ROS1 end-to-end replay postconditions; fail closed.

A completed rosbag play process does not establish that ROS estimator nodes
actually emitted data. This check also catches any forbidden navigation pose,
legacy fused pose or actuation publication on the isolated ROS master.
"""
import argparse
import json
from pathlib import Path
import sys


FORBIDDEN=('forbidden_pose','forbidden_cmd_vel','forbidden_initialpose',
           'unexpected_legacy_fused','forbidden_canonical_fast',
           'forbidden_canonical_path','forbidden_canonical_tf',
           'forbidden_step3_lock','forbidden_map_correction')
REQUIRED=('fast_odom','klt_odom','step3_status','health_status',
          'klt_continuation_status')


def check(monitor,visual=False,fault=False):
    errors=[]
    try:
        data=json.loads(Path(monitor).read_text())
        counts=data['received']
        if not isinstance(counts,dict):raise TypeError('received must be a dict')
    except (OSError,ValueError,KeyError,TypeError) as e:
        return {'passed':False,'errors':['MISSING_OR_INVALID_MONITOR: '+str(e)],'counts':{}}
    for name in FORBIDDEN:
        if counts.get(name,0)!=0:errors.append('FORBIDDEN_PUBLICATION:'+name)
    required=list(REQUIRED)
    if visual:required.append('xfeat_global')
    if fault:required.append('offline_fault_events')
    for name in required:
        if not isinstance(counts.get(name,0),int) or counts[name]<1:
            errors.append('REQUIRED_EVIDENCE_MISSING:'+name)
    if fault:
        # For this named fault experiment, camera continuation is expected
        # to generate candidates, and failure to do so must NOT be silently
        # reported as a successful fallback test.
        if not isinstance(counts.get('klt_continuation',0),int) or counts['klt_continuation']<1:
            errors.append('NO_CAMERA_CONTINUATION_CANDIDATES_IN_FAULT_RUN')
        if not any(x.startswith('health_STOP_REQUIRED_REARM_LATCHED') for x in
                   data.get('status_counts',{})):
            errors.append('LIDAR_LOSS_DID_NOT_LATCH_REARM')
    return {'passed':not errors,'errors':errors,
            'counts':{k:counts.get(k,0) for k in (*required,*FORBIDDEN,'klt_continuation')},
            'visual_enabled':visual,'fault_injected':fault,
            'note':'Passing this gate proves message flow and safety isolation only, not map accuracy or deployment readiness.'}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--monitor',required=True)
    ap.add_argument('--visual',choices=('true','false'),required=True)
    ap.add_argument('--fault',choices=('true','false'),required=True)
    ap.add_argument('--output',default='')
    args=ap.parse_args()
    result=check(args.monitor,args.visual=='true',args.fault=='true')
    if args.output:
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 12


if __name__=='__main__':sys.exit(main())
