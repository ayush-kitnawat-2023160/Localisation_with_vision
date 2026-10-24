#!/usr/bin/env python3
"""Demonstrate a completely synthetic supervised handoff, with NO ROS topics.

Not a vehicle test. The 'operator', stop proof and component acknowledgments
are constructed solely to exercise protocol logic; they establish no real
physical safety property or authenticated human identity.
"""
import json
from dataclasses import replace
from pathlib import Path
from supervised_handoff_transaction import (DryRunHandoff, Evidence,
                                            Acknowledgment, EXPECTED_ACKS)


def run():
    now=1000.0
    evidence=Evidence(candidate_stamp=999.8,visual_stamp=999.7,
        review_stamp=999.75,stop_stamp=999.85,map_frame='map',
        map_version='campus_full_lio_226s_20261019',current_epoch=7,
        current_session='SYNTHETIC_SESSION_NOT_ON_VEHICLE',
        candidate_xyz=(20.5,-5.9,1.6),candidate_yaw=0.5,
        review_status='ELIGIBLE_FOR_OFFLINE_MANUAL_REVIEW',
        independent_stop_verified=True,lidar_visual_confirmations=4,
        median_lidar_visual_xy_error_m=0.23)
    protocol=DryRunHandoff()
    prepared=protocol.prepare(evidence,now)
    for component,value in EXPECTED_ACKS.items():
        protocol.acknowledge(Acknowledgment(component,value,1000.2,
            prepared['proposed_epoch'],prepared['candidate_fingerprint']),1000.25)
    protocol.approve('SYNTHETIC_TEST_OPERATOR_NO_REAL_APPROVAL',
        prepared['transaction_id'],prepared['candidate_fingerprint'],1000.3,
        explicitly_confirmed=True)
    latest=replace(evidence,candidate_stamp=1000.4,visual_stamp=1000.4,
                   review_stamp=1000.4,stop_stamp=1000.4,
                   candidate_xyz=(20.55,-5.88,1.6))
    manifest=protocol.finalize_dryrun(latest,1000.5)
    manifest['SYNTHETIC_DEMO_ONLY']=True
    manifest['NEVER_USE_AS_PHYSICAL_APPROVAL']=True
    assert manifest['actions_performed']==[]
    return manifest


if __name__=='__main__':
    result=run()
    root=Path(__file__).resolve().parents[1]
    p=root/'outputs/learned_visual_20261020/synthetic_supervised_handoff_manifest.json'
    p.write_text(json.dumps(result,indent=2)+'\n')
    print('SYNTHETIC_DRYRUN_PASS; NO VEHICLE COMMANDS OR POSE RESET; manifest:',p)
