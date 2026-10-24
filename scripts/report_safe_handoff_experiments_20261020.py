#!/usr/bin/env python3
"""Generate ground-truth-limited engineering handoff report from real replays."""
import json,re,collections,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'outputs/learned_visual_20261020'
CASES={
 'localization_3.bag':BASE/'ros1_final_mh_review_stop_interlock_local3_20261020',
 'localization_from2.bag':BASE/'ros1_fresh_mh_candidate_handoff_review_from2_20261020',
 'localization.bag':BASE/'ros1_fresh_mh_candidate_handoff_review_localization_20261020',
}
METRICS={}
for name,p in CASES.items():
 if not (p/'replay.status').exists() or 'complete' not in (p/'replay.status').read_text():continue
 events=json.load(open(p/'visual_events.json'))['events']
 lidar=json.load(open(p/'lidar_events.json'))
 log=(p/'step3.log').read_text(errors='replace')
 trace=np.array(lidar['map_pose_trace'],float)
 c=collections.Counter(x['type'] for x in events)
 statuses=collections.Counter(x.get('message') for x in events if x['type']=='handoff_review_status')
 counter=collections.Counter(int(v) for v in re.findall(r'hypotheses=(\d+)',log))
 rec=[x for x in events if x['type']=='lidar_recovery_candidate']
 tc=[x for x in rec if name=='localization_3.bag' and x['stamp_s']>1792221804]
 evidence={'bag':name,
           'completed':True,
           'replay_exit_zero':'bag_exit=0' in (p/'replay.status').read_text(),
           'final_state':lidar['last_status'],
           'first_lock_wall_s_after_first_lidar':lidar.get('first_lock_wall_s_after_first_lidar'),
           'visual_temporally_confirmed_candidates':c.get('pose_candidate',0),
           'lidar_fresh_multi_scan_recovery_candidates':len(rec),
           'late_lidar_recovery_candidates':len(tc),
           'recovery_map_candidates_last_2':[{k:x[k] for k in ('stamp_s','xyz')} for x in rec[-2:]],
           'manual_handoff_review_statuses':dict(statuses),
           'manual_review_eligible_events':sum('ELIGIBLE' in (x.get('message') or '') for x in events if x['type']=='handoff_review_status'),
           'forbidden_pose_publications':c.get('forbidden_pose_published',0),
           'trusted_global_map_published_count':len(trace),
           'last_published_map_bag_sec':float(trace[-1,0]) if len(trace) else None,
           'largest_map_pose_step_m':float(np.linalg.norm(np.diff(trace[:,1:4],axis=0),axis=1).max()) if len(trace)>1 else None,
           'multi_hypothesis_bank_size_occurrences':dict(counter),
           'freshness_gate_rejections':log.count('NO_FRESH_LEADER_EVIDENCE'),
           'unsafe_anchor_rejections':log.count('UNSAFE_REANCHOR'),
           'handoff_required_rejections':log.count('HANDOFF_REQUIRED after'),
           'visual_guided_gicp_passes':len(re.findall(r'VISUAL_SEED_GICP.*pass=1',log)),
           'gps_param':(p/'gps_enabled.txt').read_text().strip() if (p/'gps_enabled.txt').exists() else 'unknown'}
 METRICS[name]=evidence
path=BASE/'SAFE_HANDOFF_REVIEW_METRICS_20261020.json'
path.write_text(json.dumps(METRICS,indent=2)+'\n')
lines=['# GPS-free XFeat + LiDAR MH: safe recovery handoff, October 20, 2026','',
 '## Scope and safety boundaries','',
 '- Changes are offline-only and opt-in. **No physical cart drive test was performed**.',
 '- The known-good raw campus map is unchanged; GPS not supplied to matching; production `alive_dev` and `/pose` unchanged.',
 '- A LiDAR MH candidate now requires a genuinely fresh successful GICP update at the current scan timestamp (a cached 20-hit hypothesis is NOT a fresh measurement).',
 '- After a long untrusted outage, even a small internal map-anchor correction cannot restart the published global pose. The reviewer receives a separate, timestamped LiDAR recovery candidate, not an authoritative position.',
 '- The review-only node requires status `MH_HANDOFF_REQUIRED`, three distinct LiDAR/visual matches, recent measurements, at least 2.5 sec continuous stationary odometry, and a **fresh independently verified external STOP interlock**. No such interlock exists in these bag recordings. Even if the gate returned ELIGIBLE, a surveyed/certified pose and operator-authorized reset would still need a separate implementation.',
 '', '## Real ROS1 held-out replays','',
 '| Replay | Complete | Final Step3 state | Verified visual candidates | Fresh LiDAR recovery candidates | Largest published map step | Unexpected `/pose` |',
 '|---|---|---|---:|---:|---:|---:|']
for name,r in METRICS.items():
 lines.append(f"| {name} | {r['completed'] and r['replay_exit_zero']} | {r['final_state']} | {r['visual_temporally_confirmed_candidates']} | {r['lidar_fresh_multi_scan_recovery_candidates']} | {r['largest_map_pose_step_m']:.3f}m | {r['forbidden_pose_publications']} |")
if 'localization_3.bag' in METRICS:
 x=METRICS['localization_3.bag']
 lines+=['','## Difficult third bag','',
 f"- The confirmed alternative map branch near keyframes 292–295 produced **{x['late_lidar_recovery_candidates']}** late fresh multi-scan candidate publications after the visual proposal/GICP process, but **no automatic pose authority recovery**.",
 f"- Trusted Step3 map pose output ended at **{x['last_published_map_bag_sec']:.2f}s** with max observed published step **{x['largest_map_pose_step_m']:.3f}m**. It did not silently resume after the ~33-second gap.",
 f"- The matcher rejected **{x['freshness_gate_rejections']}** stale/unsupported leader confirmations and requested a supervised handoff **{x['handoff_required_rejections']}** times. These counts include repeated offline matcher log attempts; they are not independent failure incidents.",
 f"- Review-only gate statuses: `{x['manual_handoff_review_statuses']}`. There were **{x['manual_review_eligible_events']}** cases deemed eligible for manual review.",
 '- Geometric consistency with a raw map and camera does not establish surveyed ground-truth accuracy; actual physical LiDAR–IMU mounting, synchronized clocks, and map loop geometry remain unverified.']
lines+=['','## Compile, tests, and replay controls','',
 '- C++ Step3 candidate publisher rebuilt in isolated image `fast_livo2:mh_recovery_review_20261020`.',
 '- ROS Noetic catkin registration check passed for `fresh_safe_handoff_review.py`, `fresh_multimodal_candidate_observer.py`, `fresh_visual_global_learned_bridge.py`.',
 '- Python pure handoff rules have 12 direct tests for stationary/manual review, stale data, NaN poses/velocities, wrong branch, motion, duplicate LiDAR observations, odometry gaps and missing independent external stop.',
 '- Separate isolated ROS1 node test `scripts/test_pose_authority_guard_ros1.py` PASS: permitted one initial trusted fused-map output, suppressed output after MH_HANDOFF_REQUIRED, disallowed automatic restart on returned GLOBAL_LOCKED, and emitted zero `/pose` messages. The MH pose-authority guard is enabled by `lidar_mh_enable` in `fresh_multimodal_global_rosbag.launch`.',
 '- Replay helper: `IMAGE=fast_livo2:mh_recovery_review_20261020 MH_VISUAL_SEED=true MH_REACQUIRE_HZ=1.0 HANDOFF_REVIEW=true RUN=<new_name> ./scripts/run_live_lidar_xfeat_mh_ros1.sh localization_3` (requires the previously started localhost learned XFeat service).',
 '- Main files: `src/step3_global_acquirer.cpp`; `scripts/safe_handoff_rules.py`; `scripts/fresh_safe_handoff_review.py`; `scripts/test_safe_handoff_rules.py`; `launch/fresh_multimodal_global_rosbag.launch`.',
 '', '## What remains before autonomous pose recovery','',
 '1. Implement a **stopped-vehicle externally interlocked pose reset transaction** with explicit manual approval, vehicle brake/velocity verification, complete map->odom TF reset, fresh epoch/session ID, and consumers acknowledging the discontinuity. None of this can be inferred from the bags.',
 '2. Verify actual LiDAR/IMU/base extrinsics on the cart, calibrate camera timestamps, optimize the full campus map in 3D and check against a surveyed global reference.',
 '3. Prove that all downstream trajectory/planning controllers stop on `MH_HANDOFF_REQUIRED` and never act on cached last-good `/pose` or `fused_map_pose` messages. The **experimental MH pose authority** now suppresses output and latches re-arm-required, with a single isolated ROS1 test passed; actual planner integration, stale-latched-topic behavior and braking interlock remain untested.',
 '4. Demonstrate real-time performance and cold/failed-sensor recovery on the target Jetson Orin + NUC architecture rather than this college-PC bag playback.',
 '', '**NO-GO for autonomous `/pose` release.** All new pose output and manual review diagnostics remain experimental.']
(BASE/'SAFE_HANDOFF_REVIEW_REPORT_20261020.md').write_text('\n'.join(lines)+'\n')
print('Wrote',path,'and report; replays included',list(METRICS))
