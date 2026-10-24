#!/usr/bin/env python3
"""Aggregate offline ROS1 GPS-free learned-feature/LiDAR MH experiment results.
Never equate candidate count with globally surveyed accuracy or safe driving.
"""
import json,re,collections,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1]
O=R/'outputs/learned_visual_20261020'
CASES={
 'MH diagnostic, no visual seed':'ros1_live_mh_xfeat_localization_3_timestampfixed_20261020',
 'MH + visual-guided GICP':'ros1_live_mh_xfeat_visual_seed_localization_3_20261020',
 'MH + visual seed, LiDAR+odom outage (before publisher guard fix)':'ros1_fault_lidarodom_xfeat_20261020',
 'MH + visual seed, LiDAR+odom outage (fail-closed retest)':'ros1_fault_lidarodom_failclosed_retry_20261020',
 'MH + visual seed, 1Hz global reacquisition (unsafe publish, before guard)':'ros1_visual_seed_reacquire1hz_localization3_20261020',
 'MH + visual seed, 1Hz reacquisition (handoff guard)':'ros1_handoffguard_reacquire1hz_localization3_20261020',
 'MH + visual seed, stable from2':'ros1_regression_xfeat_from2_retry_20261020',
 'MH + visual seed, stable localization':'ros1_regression_xfeat_localization_20261020',
}
summary={}
for label,dirname in CASES.items():
 d=O/dirname
 if not (d/'replay.status').exists() or 'complete' not in (d/'replay.status').read_text():continue
 ev=json.load(open(d/'visual_events.json'))['events']
 lidar=json.load(open(d/'lidar_events.json'))
 log=(d/'step3.log').read_text(errors='replace')
 c=collections.Counter(x['type'] for x in ev)
 branch=collections.Counter(x['message'].split(' ')[1] for x in ev if x['type']=='branch_decision' and len(x['message'].split(' '))>1)
 candidate=[x for x in ev if x['type']=='pose_candidate']
 bagfirst=1792221743.896889 if 'localization_3' in dirname or 'fault_' in dirname else (1792221635.666433 if 'from2' in dirname else 1792221311.775334)
 during_fault=sum(26.0<=x['stamp_s']-bagfirst<39.0 for x in candidate)
 pose_trace=np.array(lidar.get('map_pose_trace',[]),float)
 bank=collections.Counter(int(x) for x in re.findall(r'hypotheses=(\d+)',log))
 report={
   'run':dirname,
   'complete':True,'replay_exit_zero':'bag_exit=0' in (d/'replay.status').read_text(),
   'lidar_first_lock_wall_sec':lidar.get('first_lock_wall_s_after_first_lidar'),
   'lidar_final_status':lidar.get('last_status'),
   'lidar_published_map_pose_count':len(pose_trace),
   'max_published_map_pose_step_m':float(np.linalg.norm(np.diff(pose_trace[:,1:4],axis=0),axis=1).max()) if len(pose_trace)>1 else None,
   'lidar_last_published_bag_sec':float(pose_trace[-1,0]) if len(pose_trace) else None,
   'lidar_poses_during_26_39s_blackout':int(np.count_nonzero((pose_trace[:,0]>=26)&(pose_trace[:,0]<39))) if len(pose_trace) else 0,
   'lidar_poses_after_39s_resume':int(np.count_nonzero(pose_trace[:,0]>=39)) if len(pose_trace) else 0,
   'visual_temporally_confirmed_candidate_messages':len(candidate),
   'visual_candidate_messages_during_26_to_39s_dropout':during_fault,
   'live_bank_size_distribution':dict(bank),
   'visual_gicp_pass':len(re.findall('VISUAL_SEED_GICP.*pass=1',log)),
   'visual_gicp_fail':len(re.findall('VISUAL_SEED_GICP.*pass=0',log)),
   'visual_inference_latency_median_ms':float(np.median([float(z) for z in re.findall(r'latency=(\d+)ms', ' '.join(x.get('message','') for x in ev if x['type']=='status'))])) if any('latency=' in x.get('message','') for x in ev if x['type']=='status') else None,
   'visual_seed_near_late_294':bool(re.search('VISUAL_SEED_GICP keyframe=294.*pass=1',log)),
   'unsafe_reanchor_rejections':log.count('UNSAFE_REANCHOR'),
   'handoff_required_events':log.count('HANDOFF_REQUIRED after'),
   'global_reacquired_smooth_events':log.count('GLOBAL_REACQUIRED_SMOOTH'),
   'fast_odom_gap_quarantine':bool('FAST pose gap' in log),
   'diagnostic_branch_decisions':dict(branch),
   'forbidden_pose_messages':c.get('forbidden_pose_published',0),
   'gps_flag':(d/'gps_enabled.txt').read_text().strip() if (d/'gps_enabled.txt').exists() else 'unchecked',
 }
 summary[label]=report
 # A global pose must NEVER be emitted from these diagnostic scripts when
 # recovery is untrusted; map stream can legitimately exist during lock.
 if report['forbidden_pose_messages']:
  print('SAFETY TEST FAILURE: /pose was published',dirname)
(O/'ROS1_INTEGRATION_METRICS.json').write_text(json.dumps(summary,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(9,6))
for label,dirname in list(CASES.items())[:2]:
 d=O/dirname
 if not (d/'lidar_events.json').exists():continue
 p=np.array(json.load(open(d/'lidar_events.json'))['map_pose_trace'],float)
 if len(p):ax.plot(p[:,1],p[:,2],alpha=.6,label=label+' LiDAR published',linewidth=1.5)
 c=[x for x in json.load(open(d/'visual_events.json'))['events'] if x['type']=='pose_candidate']
 if c:
  x=np.array([q['xyz'][:2] for q in c]);ax.scatter(x[:,0],x[:,1],s=13,label=label+' learned visual',alpha=.7)
ax.set(xlabel='Raw-map X (m)',ylabel='Raw-map Y (m)',title='Full held-out localization_3.bag: live XFeat candidate vs LiDAR MH (diagnostic)');ax.axis('equal');ax.grid(alpha=.25);ax.legend(fontsize=7);fig.tight_layout();fig.savefig(O/'ros1_live_xfeat_lidar_hypotheses.png',dpi=150);plt.close(fig)
meta={}
for k,v in [('SuperPoint+LightGlue','campus_full_visual_superpoint_226s_20261020'),('XFeat','campus_full_visual_xfeat_226s_20261020')]:
 meta[k]=json.load(open(R/'maps'/v/'metadata.json'))
learned=json.load(open(O/'LEARNED_TEMPORAL_COMPARISON.json'))
track=json.load(open(O/'xfeat_multiview_tracks/track_qa.json'))
lines=['# ROS1 XFeat + LiDAR multi-hypothesis: offline integration results', '', '## Safety conditions', '', '- GPS disabled for Step3 and XFeat retrieval. XFeat global retrieval is local-descriptor FLANN, not MegaLoc/AnyLoc global embeddings.', '- ROS1 visual bridge only publishes `/fast_livo2/visual_global_candidate` after 3 visual observations. It never publishes `/pose`.', '- Experimental Step3 visual-guided GICP is **opt-in**, and a camera candidate only proposes a map region to try. GICP verification and repeated independent LiDAR scans remain necessary for an anchor.', '- Main LiDAR MH retains `mh_max_reanchor_m=4.0`; all test scenarios are offline and rely on an unsurveyed raw map with inherited, unverified physical LiDAR–IMU calibration.', '', '## Learned offline map quality', '', '| Property | SuperPoint / LightGlue map | XFeat map |', '|---|---:|---:|']
for field in ('frames','raw_keypoint_observations','depth_supported_observations','median_depth_support_per_frame','median_extract_ms'):
 lines.append(f"| {field} | {meta['SuperPoint+LightGlue'].get(field)} | {meta['XFeat'].get(field)} |")
lines+=['',f"- Conservative 3D multiview track candidates: **{track['tracks_with_at_least_two_corresponding_3d_observations']}** repeat-observed track groups from {track['input_3d_observations']:,} raw 3D XFeat observations. Only a candidate track export; no bundle adjustment, moving-object segmentation, or loop closure.", '', '## Held-out learned global relocalization (offline)', '', '| Method | localization | from2 | localization_3 | Median `localization_3` inference |', '|---|---:|---:|---:|---:|']
for method,v in learned.items():
 lines.append(f"| {method} | {v['localization']['PnP_passed']}/{v['localization']['queries']} | {v['localization_from2']['PnP_passed']}/{v['localization_from2']['queries']} | {v['localization_3']['PnP_passed']}/{v['localization_3']['queries']} | {v['localization_3']['p50_latency_ms']:.0f}ms |")
lines+=['', '## Combined ROS1 replays', '', '| Configuration | Complete | Visual candidate poses | LiDAR MH states | Visual-guided GICP passes | Unsafe re-anchors rejected | Final Step3 status |', '|---|---|---:|---|---:|---:|---|']
for k,v in summary.items():
 bank=','.join(str(x)+'-hyp:'+str(n) for x,n in sorted(v['live_bank_size_distribution'].items()))
 lines.append(f"| {k} | {v['replay_exit_zero']} | {v['visual_temporally_confirmed_candidate_messages']} | {bank or 'none'} | {v['visual_gicp_pass']} | {v['unsafe_reanchor_rejections']} | {v['lidar_final_status']} |")
lines+=['', '## Interpreting the difficult bag and GPS-free branch choice', '', '- In the no-seed full combined run, LiDAR and XFeat are alive at the same time but LiDAR MH produces at most one high-confidence branch at a time. The observer cannot truthfully claim visual arbitration between *two* branches; it reports insufficient/stale LiDAR bank support.', '- With opt-in visual-guided geometric proposals, Step3 reverified several visual-preferred map regions with LiDAR GICP, including around late keyframe 294. This did not satisfy the full LiDAR temporal confirmation or 4m anchor-change safety gate before bag termination. No authoritative pose promotion.', '- Earlier offline and ROS1 fixture tests (using artificially replayed saved LiDAR alternatives) supported the late 293-branch, but that fixture is NOT an observed live two-branch LiDAR bank and must not be conflated with this result.', '', '## Sensor loss and safety', '', '- The fault replay specifically drops **Step3 LiDAR input and its pre-recorded local odometry topic** for bag seconds 26–39, while preserving camera images. This tests inter-component failure response; the prerecorded odometry is NOT a realistic internally recomputed LiDAR-fallback estimator.',
 '- **Publisher fail-closed bug found and fixed:** before the guard fix, the first resumed stale odom timestamp at ~39.03s produced one held-position map-pose message even though tracking was quarantined. After the code fix, the full fault replay emitted zero map poses during the outage AND zero map poses after resumption without re-lock. Experimental `/pose` remained absent in both.',
 '- **Second safety issue in 1Hz global recovery:** after ~33s without real map-pose publication, the alternative keyframe 292 received three LiDAR confirmations and its internal map anchor differed by only 0.72m from a propagated prediction. However auto-resuming the old published trajectory caused a **11.94m per-sample step**. The new `mh_max_auto_pose_resume_gap_sec` gate (2.5s default, MH only) now refuses to reactivate published map poses after a long trust gap. An explicit vehicle-stopped pose reset/authority handoff is not yet implemented.', '- Check `fast_odom_gap_quarantine`, `visual_candidate_messages_during_26_to_39s_dropout`, `forbidden_pose_messages` and final map status in `ROS1_INTEGRATION_METRICS.json`.', '- Camera-only visual candidates can continue, but map authority must remain untrusted while geometry/pose consistency is unresolved.', '', '## Next implementation milestones', '', '1. Improve the GICP multi-hypothesis bank to retain alternative pose branches for >1 scan instead of collapsing to one. The visual-GICP seed shows the late branch can be proposed and geometrically corroborated.', '2. Integrate visual and LiDAR timestamps through a bounded odometry buffer and state history for any real correction, not directly into the FAST-LIVO2 ESIKF. The present safety observer has sensor-stamp guards; the image->scan seed uses nearest odom pose within 150ms.', '3. Optimize actual physical LiDAR–IMU/base extrinsics, and add at least one mapped/surveyed route reference; the current repeated landmark candidates are not full BA or map geometry verification.', '4. Run the combined pipeline on the Jetson Orin/NUC network with concurrent LiDAR and camera data and profile CPU/GPU, enqueue time, and backend latency; these benchmarks use the college PC CPU.', '5. Keep the new visual seed flags and learned candidate launch disabled by default. Never publish navigation `/pose` based on these results.', '', '## Reproduction', '', '- `scripts/run_live_lidar_xfeat_mh_ros1.sh` runs an isolated ROS Noetic bag playback with the saved LiDAR odometry, Step3, XFeat worker, and read-only observer.', '- Enable `MH_VISUAL_SEED=true` with image `fast_livo2:mh_visual_seed_20261020`. Set `FAULT_LIDAR=true` to simulate 26–39s LiDAR/odom topic interruption. Do not overwrite existing run results.', '- `scripts/learned_visual_backends.py`, `scripts/fresh_visual_global_learned_bridge.py`, `scripts/fresh_multimodal_candidate_observer.py` and `src/step3_global_acquirer.cpp` implement the experimental stack.', '', '**NO-GO for autonomous localization:** successful geometric candidates and cross-sensor agreement do not establish surveyed global accuracy, correct live physical transforms, or continuous safe authority.']
(O/'ROS1_XFEAT_MH_FINAL_REPORT.md').write_text('\n'.join(lines)+'\n')
print('REPORT',O/'ROS1_XFEAT_MH_FINAL_REPORT.md');print('\n'.join(lines[27:43]))
