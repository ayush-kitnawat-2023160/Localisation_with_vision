#!/usr/bin/env python3
"""Reproducibly curate TWO-bag full-run results and prune diagnostic clutter.

Safety: default dry run. --apply makes verified copies / archive BEFORE any
deletion, within results/ only. Source code, PCD databases outside results,
bag files, and the corrected September reference are NEVER deleted.

The chosen trials are 'best observed PROXIES', NOT deployment validation.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import tarfile
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
RESULTS=ROOT/'results'
LABEL='BEST_SO_FAR_20261015'
BEST=RESULTS/LABEL
ARCHIVES=RESULTS/'_archive'
ARCHIVE_NAME='USEFUL_HISTORICAL_RUNS_20261015.tar.gz'
REF_SEP=Path('18_sep_2026_combined/HELDOUT_LIDAR_MAP_REFERENCE_20261009')

# Every best case is a complete, 1x replay; 290 s original / 220 s holdout.
# Full results are intentionally drawn from DIFFERENT dev builds when better.
SELECT={
 'camera_lidar_imu_gps':{
  'HEALTHY':'HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/HEALTHY_BRIDGE_290S',
  'CAMERA_FAILED':'HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/CAMERA_FAIL_BRIDGE_290S',
  'LIDAR_FAILED':'LATEST_20261013_1X_290S/LIDAR_FAIL',
  'IMU_FAILED':'HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/IMU_FAIL_BRIDGE_290S',
 },
 '18_sep_2026_combined':{
  'HEALTHY':'STEP4_PHYSICAL_HEALTHY_20261008',
  'CAMERA_FAILED':'HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/CAMERA_FAIL_HOLDOUT_220S',
  'LIDAR_FAILED':'FINAL_LIDAR_FAIL_220S_20261007',
  'IMU_FAILED':'HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/IMU_FAIL_HOLDOUT_220S',
 },
}

# Only nonselected history with research value goes in this archive; all other
# old diagnostic results and short smoke trials are intentionally discarded.
HISTORICAL=[
 'camera_lidar_imu_gps/HEALTHY_GAP_HANDOFF_DEV_20261014/HEALTHY_GAP_HANDOFF_FULL_290S',
 'camera_lidar_imu_gps/LATEST_20261013_1X_290S/HEALTHY',
 'camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/LIDAR_FAIL_BRIDGE_290S',
 'camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/COMPARISON_BRIDGE',
 'camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/four_mode_readiness_audit.json',
 'camera_lidar_imu_gps/SLOPE_RECOVERY_VERIFIED_DEV_20261015/LIDAR_FAIL_SLOPE_220S',
 'camera_lidar_imu_gps/SLOPE_RECOVERY_VERIFIED_DEV_20261015/validation_status.txt',
 'camera_lidar_imu_gps/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_FAILCLOSED_195S',
 'camera_lidar_imu_gps/CAMERA_RETURN_HANDOFF_DEV_20261015/CAMERA_FAIL_RETURN_195S',
 'camera_lidar_imu_gps/HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/ab_score.json',
 'camera_lidar_imu_gps/HEALTHY_DESCRIPTOR_NORMCACHE_DEV_20261015/HEALTHY_PROFILE_BASELINE_90S.process_cpu.csv',
 'camera_lidar_imu_gps/final_15cm_map_visual_check',
 '18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/HEALTHY_HOLDOUT_220S',
 '18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/LIDAR_FAIL_HOLDOUT_220S',
 '18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/COMPARISON',
 '18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/reference_aligned_historical_comparisons.json',
 '18_sep_2026_combined/STEP4_DEPTH_RELOCK_FINAL_220S_20261008',
 '18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_FAILCLOSED_V2_160S',
 '18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/LIDAR_FAIL_TEMPORAL_160S',
 '18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/failclosed_v2_vs_previous_lidar_155s.json',
 '18_sep_2026_combined/FAILCLOSED_RECOVERY_DEV_20261015/temporal_vs_failclosed_v2_155s.json',
 'one_full_loop/full_2loop_map',
 'CLEANUP_20260926_deleted_runs.txt',
]

def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for block in iter(lambda:f.read(1<<20),b''):h.update(block)
 return h.hexdigest()

def stat_case(folder,reference,horizon,fail_start):
 with reference.open(newline='') as f:r=list(csv.DictReader(f))
 rt=np.array([float(z['timestamp']) for z in r]);rx=np.array([[float(z[c]) for c in 'xyz'] for z in r]);
 with (folder/'global_pose.csv').open(newline='') as f:a=list(csv.DictReader(f))
 t=np.array([float(z['field.header.stamp'])*1e-9 for z in a]);x=np.array([[float(z['field.pose.position.'+c]) for c in 'xyz'] for z in a]);
 if len(t)<1500 or t[-1]-t[0]<horizon-6 or not np.isfinite(x).all():
  raise ValueError('Incomplete/invalid full replay: '+str(folder))
 if t[0]<rt[0]-0.05 or t[-1]>rt[-1]+1.0:
  raise ValueError('Trajectory and reference misaligned: '+str(folder))
 dx=np.diff(x,axis=0);dt=np.diff(t)
 if (dt<=0).any():raise ValueError('Non-monotone timestamps: '+str(folder))
 y=np.stack([np.interp(t,rt,rx[:,k]) for k in range(3)],axis=1)
 err=np.linalg.norm(x-y,axis=1)
 def span(start,stop):
  mask=(t>=rt[0]+start)&(t<rt[0]+stop)
  return round(float(np.sqrt(np.mean(err[mask]**2))),3) if mask.any() else None
 metric={
   'published_pose_count':len(t),
   'published_pose_duration_s':round(float(t[-1]-t[0]),3),
   'recomputed_proxy_3d_rmse_m':round(float(np.sqrt(np.mean(err**2))),3),
   'max_proxy_3d_difference_m':round(float(err.max()),3),
   'max_published_pose_step_m':round(float(np.linalg.norm(dx,axis=1).max()),3),
   'largest_pose_timestamp_gap_s':round(float(dt.max()),3),
   'failure_window_proxy_3d_rmse_m':span(fail_start,fail_start+40) if fail_start else None,
   'recovery_20s_proxy_3d_rmse_m':span(fail_start+40,fail_start+60) if fail_start else None,
  }
 return metric

def all_files(folder):
 return sorted(x for x in folder.rglob('*') if x.is_file() and not x.is_symlink())

def verify_copy(src,dst):
 a=all_files(src);b=all_files(dst)
 ra=[str(p.relative_to(src)) for p in a];rb=[str(p.relative_to(dst)) for p in b]
 if ra!=rb:raise ValueError('Copied files differ: '+str(src))
 for aa,bb in zip(a,b):
  if aa.stat().st_size!=bb.stat().st_size or digest(aa)!=digest(bb):
   raise ValueError('Hash mismatch: '+str(aa))
 return len(a),sum(p.stat().st_size for p in a)

def main():
 parser=argparse.ArgumentParser()
 parser.add_argument('--apply',action='store_true',help='After verifying artifacts, permanently delete obsolete result directories')
 a=parser.parse_args()
 if not RESULTS.is_dir() or RESULTS.is_symlink():raise ValueError('Unexpected results root')
 roots=[RESULTS/k for k in (*SELECT.keys(),'one_full_loop')]
 for root in roots:
  if not root.is_dir() or root.is_symlink():raise ValueError('Invalid input root: '+str(root))
 sources=[RESULTS/bag/r for bag in SELECT for r in SELECT[bag].values()]
 for source in sources:
  if not source.is_dir() or source.is_symlink():raise ValueError('Missing best source '+str(source))
 hist=[RESULTS/r for r in HISTORICAL]
 for source in hist:
  if not source.exists() or source.is_symlink():raise ValueError('Missing archival evidence '+str(source))
 if BEST.exists() or (RESULTS/('.staging_'+LABEL)).exists():
  raise ValueError('Existing best or staging folder: refusing overwrite')
 if (ARCHIVES/ARCHIVE_NAME).exists():raise ValueError('Archive already exists: refusing overwrite')
 if len(set(sources+hist))!=len(sources+hist):raise ValueError('Duplicate selection')
 print('Selected best runs:')
 for bag,modes in SELECT.items():
  for mode,s in modes.items():print(' ',bag,mode,'<-',s)
 print('Historical paths to archive:',len(hist))
 print('Old result dataset roots to replace:',[str(x.relative_to(RESULTS)) for x in roots])
 if not a.apply:
  print('DRY RUN ONLY; pass --apply to verify copies/archive and delete obsolete results.')
  return

 staging=RESULTS/('.staging_'+LABEL)
 tar_tmp=RESULTS/('.staging_'+ARCHIVE_NAME)
 if tar_tmp.exists():raise ValueError('Staged archive already exists')
 stage_files=[]
 verification={}
 references={
  'camera_lidar_imu_gps':ROOT/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv',
  '18_sep_2026_combined':RESULTS/REF_SEP/'CORNER_FIXED_REFERENCE_20261009/REFERENCE_18SEP_LIDAR_PCD_CORNER_FIXED_20261009.csv',
 }
 for ref in references.values():
  if not ref.is_file():raise ValueError('Missing reference '+str(ref))
 try:
  staging.mkdir()
  for bag,modes in SELECT.items():
   for mode,relative in modes.items():
    src=RESULTS/bag/relative
    dst=staging/bag/mode
    shutil.copytree(src,dst)
    n,size=verify_copy(src,dst)
    stats=stat_case(dst,references[bag],290 if bag=='camera_lidar_imu_gps' else 220,
                    0 if mode=='HEALTHY' else 125 if bag=='camera_lidar_imu_gps' else 90)
    mode_str={'CAMERA_FAILED':'LIO_CAMERA_FAILED','LIDAR_FAILED':'VI_LIDAR_FAILED','IMU_FAILED':'LV_IMU_FAILED'}.get(mode)
    log=(dst/'launch.log').read_text(errors='replace')
    if mode_str and ('-> '+mode_str not in log):raise ValueError('Outage mode not reached: '+str(src))
    if not (dst/'plots/07_orange_trajectory_on_pcd_clean.png').is_file():raise ValueError('Missing PCD overlay: '+str(src))
    status='EXPERIMENTAL, NOT DEPLOYABLE; NOT INDEPENDENT GROUND TRUTH'
    if mode=='LIDAR_FAILED' or stats['max_published_pose_step_m']>.85:
      status='FAILED SAFETY / SMOOTHNESS; BEST AVAILABLE ONLY, DO NOT DEPLOY'
    metadata={'bag':bag,'sensor_case':mode,'original_run':str(src.relative_to(ROOT)),
              'source_files_verified_sha256':n,'source_size_bytes':size,
              'reference_used':str(references[bag].relative_to(ROOT)),
              'evidence':'single full-run 1x replay; reference discrepancy is not independent ground truth',
              'replay_seconds':290 if bag=='camera_lidar_imu_gps' else 220,
              'failure_window':None if mode=='HEALTHY' else
                 {'start_s':125 if bag=='camera_lidar_imu_gps' else 90,'duration_s':40},
              'status':status,'metrics':stats}
    (dst/'CURATION_METADATA.json').write_text(json.dumps(metadata,indent=2)+'\n')
    verification[f'{bag}/{mode}']=metadata
    print('VERIFIED',bag,mode,stats,flush=True)

  # Carry reference descriptions and frozen CSV into the best group. The
  # original September reference source stays intact for existing scripts.
  for bag,ref in references.items():
   target=staging/bag/'REFERENCE'
   target.mkdir()
   shutil.copy2(ref,target/ref.name)
   if digest(ref)!=digest(target/ref.name):raise ValueError('Reference hash failed')
  base_ref=RESULTS/REF_SEP/'CORNER_FIXED_REFERENCE_20261009'
  shutil.copytree(base_ref,staging/'18_sep_2026_combined'/'REFERENCE'/'corner_fixed_evidence',
                  ignore=shutil.ignore_patterns('REFERENCE_18SEP_LIDAR_PCD_CORNER_FIXED_20261009.csv'))

  (staging/'selection_manifest.json').write_text(json.dumps({
    'date':'2026-10-15', 'validated_vehicle_ready':False,
    'independent_ground_truth':False,'accuracy_threshold_0p5m_met':False,
    'WARNING':'These are individually best observed full runs from different experimental branches; not one consistently verified four-mode system.',
    'best':verification,
    'corrected_sep_reference_original_preserved':str((RESULTS/REF_SEP).relative_to(ROOT)),
    'non_selected_useful_archived_as':str((ARCHIVES/ARCHIVE_NAME).relative_to(ROOT)),
  },indent=2)+'\n')
  lines=['# Best saved results so far — 2026-10-15','','**EXPERIMENTAL ONLY — NOT APPROVED FOR GOLF-CART DEPLOYMENT.**',
    'Each folder contains the original replay CSV, logs, settings, metrics and PCD overlays.',
    'All eight selected runs were the complete 290s (main bag) or 220s (September holdout) 1× bag replay.',
    'Different modes can come from different historical experimental builds.',
    'Metrics below are *disagreement with saved PCD/loop-closed trajectory proxies*, not independent ground truth.','',
    '| Dataset | Sensor case | Proxy 3D RMSE (m) | Max published step (m) | Outcome |',
    '| --- | --- | ---: | ---: | --- |']
  for key,v in verification.items():
   lines.append('| '+key.replace('/',' | ')+' | '+str(v['metrics']['recomputed_proxy_3d_rmse_m'])+' | '+str(v['metrics']['max_published_pose_step_m'])+' | '+('FAILED / NOT DEPLOYABLE' if 'FAILED' in v['status'] else 'Experimental')+' |')
  lines+=['','**WARNING: Neither LiDAR-failure run satisfies the no-jump requirement;** no four-mode robust localisation validation exists yet.',
          'Also, none meets independently validated 0.5m accuracy.','',
          'Selection policy: complete consistent bag duration, same frozen reference within a dataset, high overlap / low RMSE, then smoothness and verification provenance.',
          'Use `selection_manifest.json` for original experiment paths and complete metrics.',
          'Historical but useful replay evidence is in `results/_archive/USEFUL_HISTORICAL_RUNS_20261015.tar.gz`.',
          'The corrected September saved-map reference is preserved at its original location to avoid breaking analysis scripts.',
          'Source maps, 15cm map database and all bag files outside `results/` were left untouched.']
  (staging/'README.md').write_text('\n'.join(lines)+'\n')

  print('Building compact evidence archive...',flush=True)
  with tarfile.open(tar_tmp,'w:gz',compresslevel=5) as tar:
   for p in hist:
    tar.add(p,arcname='results/'+p.relative_to(RESULTS).as_posix(),recursive=True)
  # Read every compressed member payload, not merely the headers.
  total_items=0;total_bytes=0
  with tarfile.open(tar_tmp,'r:gz') as tar:
   for member in tar:
    total_items+=1
    if member.isfile():
     with tar.extractfile(member) as reader:
      while True:
       block=reader.read(1<<20)
       if not block:break
       total_bytes+=len(block)
  if total_items<30 or total_bytes<1_000_000:raise ValueError('Archive missing expected content')
  print('VERIFIED archive members',total_items,'decompressed_bytes',total_bytes,flush=True)

  # All archive material + best outputs safe at this point. Commit targets
  # before deletion. Do not allow existing targets to be overwritten.
  ARCHIVES.mkdir(exist_ok=True)
  os.replace(staging,BEST)
  os.replace(tar_tmp,ARCHIVES/ARCHIVE_NAME)
  for name in ('PENDING_FAILSAFE_EVIDENCE_20261015_NOT_DEPLOYABLE.zip',
               'VALIDATION_EVIDENCE_20261015_NOT_DEPLOYABLE.zip'):
   p=RESULTS/name
   if p.is_file() and not (ARCHIVES/name).exists():shutil.move(str(p),str(ARCHIVES/name))

  # ONLY delete the explicitly scoped, enumerated historical result roots;
  # preserve the September reference asset folder at its ORIGINAL path.
  deletions={}
  for root in roots:
   if root.name=='18_sep_2026_combined':
    for child in list(root.iterdir()):
     if child.name==REF_SEP.name:continue
     if not child.is_dir() or child.is_symlink():raise ValueError('Unexpected symlink/file during cleanup: '+str(child))
     deletions[str(child.relative_to(RESULTS))]=True
     shutil.rmtree(child)
   else:
    deletions[str(root.relative_to(RESULTS))]=True
    shutil.rmtree(root)
  text='Curated 8 complete runs. Archived %d useful old paths in %s. Permanently deleted %d obsolete result roots; September reference preserved.\n'%(len(hist),ARCHIVE_NAME,len(deletions))
  (RESULTS/'README.md').write_text('# Curated localisation results\n\n'+text+'\n'
      '- BEST_SO_FAR_20261015/ — eight best available full runs, plots, metrics, metadata\n'
      '- _archive/ — useful historical experiments, failure evidence, and prior reports\n'
      '- 18_sep_2026_combined/HELDOUT_LIDAR_MAP_REFERENCE_20261009/ — protected reference input\n\n'
      '**All variants remain experimental and are not safe for unattended cart use.**\n')
  (ARCHIVES/'cleanup_manifest.json').write_text(json.dumps({
    'date':'2026-10-15','archived_relpaths':HISTORICAL,
    'deleted_result_roots':list(deletions),
    'protected_reference':str(REF_SEP),
    'archive_sha256':digest(ARCHIVES/ARCHIVE_NAME),
    'best_count':len(verification),'archive_members_read':total_items,
    'archive_decompressed_bytes_read':total_bytes,
  },indent=2)+'\n')
  print('DONE: 8 best runs; archived',len(HISTORICAL),'useful paths; deleted',len(deletions),'old result entries. Original reference preserved.')
 except Exception:
  # On error, never touch the source directories; keep staged evidence to
  # inspect manually rather than silently deleting it.
  print('ERROR; source results were not deleted until both artifact checks completed.',flush=True)
  raise

if __name__=='__main__':main()
