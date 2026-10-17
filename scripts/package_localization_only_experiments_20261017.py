#!/usr/bin/env python3
"""Package empirical evidence for REJECTED opt-in localisation experiments only.

No planner/DBW/bridge artifacts. No map/rosbag duplicates. Acceptance status
remains the frozen one-image baseline, not a cherry-picked run.
"""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import json, hashlib, subprocess
R=Path(__file__).resolve().parents[1]
ROOT=R/'results/LOCALIZATION_ONLY_COMPLETION_20261017'
b=ROOT/'recovery_voxel_budget'
l=ROOT/'recovery_voxel_budget_lidar'
d=ROOT/'depth_source_clock_gate'
evidence=[
 'reports/LOCALIZATION_CURRENT_STATUS_AND_NEXT_SOURCE_FIX_20261017.md',
 'reports/LOCALIZATION_RECOVERY_BUDGET_AUDIT_20261017.md',
 'reports/DEPTH_SOURCE_CLOCK_GATE_AUDIT_20261017.md',
 'reports/LOCALIZATION_COMPLETION_AUDIT_20261017.md',
 'reports/ACTUAL_INTEGRATION_LOCALIZATION_SCOPE_20261016.md',
 'scripts/build_recovery_voxel_budget_dev_20261017.py',
 'scripts/build_depth_source_clock_gate_dev_20261017.py',
 'scripts/validate_recovery_voxel_budget_dev_20261017.sh',
 'scripts/validate_lidar_recovery_voxel_budget_dev_20261017.sh',
 'scripts/validate_lidar_depth_source_clock_dev_20261017.sh',
 'scripts/evaluate_recovery_voxel_budget_dev_20261017.py',
 'scripts/evaluate_lidar_recovery_voxel_budget_dev_20261017.py',
 'scripts/evaluate_depth_source_clock_gate_dev_20261017.py',
 'scripts/test_depth_source_clock_gate_pure_20261017.py',
 'config/failsafe_recovery_voxel_budget_dev_20261017.yaml',
 'config/map_failsafe_depth_source_clock_dev_20261017.yaml',
 'launch/fast_livo2_recovery_voxel_budget_dev_20261017.launch',
 'step3_global/recovery_voxel_budget_localization_dev_20261017/LIVMapper.cpp',
 'step3_global/recovery_voxel_budget_localization_dev_20261017/LIVMapper.h',
 'step3_global/recovery_voxel_budget_localization_dev_20261017/LIVMapper.diff',
 'step3_global/depth_source_clock_localization_dev_20261017/failsafe_map_localizer.cpp',
 'step3_global/depth_source_clock_localization_dev_20261017/change.diff']
for droot in (b,l,d):
 if not droot.exists():raise RuntimeError('Missing test evidence '+str(droot))
 for f in sorted(droot.rglob('*')):
  if not f.is_file():continue
  # Actual pose CSVs, metric tables, images, and small metadata; full launch
  # logs stay in original host results for forensic inspection.
  if f.name.endswith(('global_pose.csv','.json','.png','run_metadata.txt','.log','source_image.txt')):
   if f.suffix=='.log' and f.name!='run_status.log':continue
   if f.stat().st_size<9_000_000:
    evidence.append(str(f.relative_to(R)))
out=ROOT/'LOCALIZATION_EXPERIMENTS_REJECTED_WITH_REAL_1X_EVIDENCE_20261017.zip'
known=list(dict.fromkeys(Path(x) for x in evidence))
for file in known:
 if not (R/file).is_file():raise RuntimeError('Missing evidence '+str(file))
manifest={'date':'2026-10-17','scope':'LOCALISATION_ONLY_NO_VEHICLE_CONTROL',
 'status':'REJECTED_EXPERIMENTS_NOT_PROMOTED',
 'original_frozen_release_unchanged':True,
 'latest_repo_alive_dev_unchanged':True,
 'independent_surveyed_ground_truth':False,
 'accepted_release_image':'fast_livo2_failsafe:single-build-rc-20261016',
 'experimental_images':[
    'fast_livo2_failsafe:recovery-voxel-budget-localization-dev-20261017',
    'fast_livo2_failsafe:depth-source-clock-gate-localization-dev-20261017'],
 'files':[{'name':str(p),'bytes':(R/p).stat().st_size,
           'sha256':hashlib.sha256((R/p).read_bytes()).hexdigest()}
          for p in known]}
with ZipFile(out,'w',ZIP_DEFLATED,compresslevel=4) as z:
 for p in known:z.write(R/p,str(p))
 z.writestr('MANIFEST.json',json.dumps(manifest,indent=2)+'\n')
with ZipFile(out) as z:
 assert z.testzip() is None
 assert len(z.namelist())==len(known)+1
print('REJECTED LOCALISATION EVIDENCE:',out,'files',len(known)+1,
     'MB',round(out.stat().st_size/1e6,2))
print('CART BRIDGE / PLANNER MODIFICATIONS: NONE')
