#!/usr/bin/env python3
"""Audit saved real ROS1 replays against conservative handoff requirements.

No simulated brake signals or fake operator approvals are introduced. This
therefore cannot grant recovery authority. Source events are left untouched.
"""
from __future__ import annotations
import collections
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'outputs/learned_visual_20261020'
RUNS={
 'difficult_localization_3':'ros1_final_mh_review_stop_interlock_local3_20261020',
 'stable_localization':'ros1_fresh_mh_candidate_handoff_review_localization_20261020',
 'stable_localization_from2':'ros1_fresh_mh_candidate_handoff_review_from2_20261020',
}


def audit(path):
    data=json.loads((path/'visual_events.json').read_text())['events']
    lidar=json.loads((path/'lidar_events.json').read_text())
    visual=[e for e in data if e['type']=='pose_candidate']
    recovery=[e for e in data if e['type']=='lidar_recovery_candidate']
    statuses=collections.Counter(e.get('message','') for e in data
                                 if e['type']=='handoff_review_status')
    paired=[]
    used=set()
    for li in recovery:
        candidates=[(abs(li['stamp_s']-v['stamp_s']),i,v)
                    for i,v in enumerate(visual)
                    if i not in used and abs(li['stamp_s']-v['stamp_s'])<=1.0]
        if not candidates:continue
        delta,i,vi=min(candidates)
        used.add(i)
        paired.append({'lidar_stamp':li['stamp_s'],'visual_stamp':vi['stamp_s'],
                       'sensor_offset_s':delta,
                       'xy_disagreement_m':math.dist(li['xyz'][:2],vi['xyz'][:2])})
    close=[p for p in paired if p['xy_disagreement_m']<=2.0]
    forbidden=[e for e in data if e['type']=='forbidden_pose_published']
    # There are no brake, controller, operator-approval, epoch-consumer or
    # independent hardware stop attestations among these ROS bag records.
    return dict(replay_complete='complete' in (path/'replay.status').read_text(),
                final_global_status=lidar['last_status'],
                independent_visual_candidates=len(visual),
                lidar_recovery_candidates=len(recovery),
                independent_timestamp_pairs=len(paired),
                close_xy_pairs_under_2m=len(close),
                largest_close_xy_disagreement_m=max((p['xy_disagreement_m'] for p in close),default=None),
                handoff_review_reasons=dict(statuses),
                independent_stop_interlock_proof=False,
                confirmed_brake_engaged_ack=False,
                controller_motion_inhibited_ack=False,
                stopped_planner_ack=False,
                tf_epoch_ack=False,
                authenticated_operator_approval=False,
                dryrun_transaction_ready=False,
                authorized_autonomous_reset=False,
                unexpected_pose_publications=len(forbidden))


def main():
    reports={}
    for name,d in RUNS.items():
        p=BASE/d
        if not (p/'replay.status').exists() or 'complete' not in (p/'replay.status').read_text():
            reports[name]={'status':'INCOMPLETE_OR_UNAVAILABLE'}
            continue
        reports[name]=audit(p)
    out=BASE/'SUPERVISED_HANDOFF_REPLAY_AUDIT_20261020.json'
    out.write_text(json.dumps(reports,indent=2)+'\n')
    for k,v in reports.items():
        if 'status' in v: print(k,v['status']);continue
        print(k,'lidar_candidates',v['lidar_recovery_candidates'],
              'XFeat',v['independent_visual_candidates'],
              'time_paired',v['independent_timestamp_pairs'],
              'close_xy',v['close_xy_pairs_under_2m'],
              'manual_handoff_ready',v['dryrun_transaction_ready'],
              'unexpected_/pose',v['unexpected_pose_publications'])
    assert all(not v.get('authorized_autonomous_reset',False) for v in reports.values())
    assert all(not v.get('unexpected_pose_publications',0) for v in reports.values())
    print('OFFLINE_AUDIT_PASS: No recorded evidence can authorize autonomous reset')


if __name__=='__main__':main()
