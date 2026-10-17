#!/usr/bin/env python3
"""Complete interrupted 2026-10-15 result curation safely.

The eight verified best copies and fully read historical tarball already
exist. First preserve legacy references and human notes; remove only old
result directory children after verifying best, archive, and corrected PCD
reference. No project sources, original rosbags or 15cm map database touched.
"""
from pathlib import Path
import hashlib
import json
import shutil
import tarfile

ROOT=Path(__file__).resolve().parents[1]
RESULTS=ROOT/'results'
BEST=RESULTS/'BEST_SO_FAR_20261015'
ARCHIVE=RESULTS/'_archive/USEFUL_HISTORICAL_RUNS_20261015.tar.gz'
OLD=RESULTS/'18_sep_2026_combined'
REF=OLD/'HELDOUT_LIDAR_MAP_REFERENCE_20261009'
CURRENT=BEST/'18_sep_2026_combined/REFERENCE'

def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for block in iter(lambda:f.read(1<<20),b''):h.update(block)
 return h.hexdigest()

def main():
 manifest=json.loads((BEST/'selection_manifest.json').read_text())
 cases=manifest['best']
 if len(cases)!=8 or not ARCHIVE.is_file() or not REF.is_dir():
  raise RuntimeError('Core copy / archive / reference missing: refusing cleanup')
 for label,data in cases.items():
  folder=BEST/label
  for required in ['global_pose.csv','metrics_exact.txt','launch.log','config.yaml',
                   'plots/07_orange_trajectory_on_pcd_clean.png','CURATION_METADATA.json']:
   if not (folder/required).is_file():
    raise RuntimeError('Curated run is incomplete: '+str(folder/required))
  if json.loads((folder/'CURATION_METADATA.json').read_text())['sensor_case'] != label.split('/')[-1]:
   raise RuntimeError('Metadata mismatch: '+label)
 with tarfile.open(ARCHIVE,'r:gz') as tar:
  entries={entry.name for entry in tar}
  for must in ('results/one_full_loop/full_2loop_map/full_2loop_saved_map.pcd',
               'results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015/LIDAR_FAIL_BRIDGE_290S/metrics_exact.txt',
               'results/18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015/LIDAR_FAIL_HOLDOUT_220S/metrics_exact.txt'):
   if must not in entries:raise RuntimeError('Crucial archive evidence missing: '+must)
 if sha(REF/'CORNER_FIXED_REFERENCE_20261009/REFERENCE_18SEP_LIDAR_PCD_CORNER_FIXED_20261009.csv') != sha(CURRENT/'REFERENCE_18SEP_LIDAR_PCD_CORNER_FIXED_20261009.csv'):
  raise RuntimeError('Protected reference differs from best set')

 # Keep alternative historical reference inputs/documentation alongside
 # curated results before deleting loose, legacy copies. The canonical
 # corrected September reference remains at its original location.
 legacy=CURRENT/'archived_reference_inputs'
 legacy.mkdir(exist_ok=True)
 protected_files=['REFERENCE_HEALTHY_STEP3_20261007.csv',
                  'REFERENCE_HEALTHY_STEP3_PHYSICAL_20261008.csv',
                  'DEPTH_RELOCK_VALIDATION_20261008.md']
 for name in protected_files:
  p=OLD/name
  if p.is_file():
   shutil.copy2(p,legacy/name)
   if sha(p)!=sha(legacy/name):raise RuntimeError('Legacy ref copy mismatch')
 if (OLD/'README.md').is_file():
  shutil.copy2(OLD/'README.md',legacy/'OLD_RESULTS_README.md')
 # Save the old comparison/feedback plots, whose symlink targets will be
 # intentionally removed as superseded short experiments.
 legacyplots=CURRENT/'archived_comparison_plots'
 legacyplots.mkdir(exist_ok=True)
 for link in ('LATEST_18SEP_FOUR_MODE_COMPARISON.png',
              'LATEST_18SEP_MAP_FEEDBACK_COMPARISON.png'):
  p=OLD/link
  if p.is_symlink() and p.exists():
   shutil.copy2(p.resolve(),legacyplots/link)
   if sha(p.resolve())!=sha(legacyplots/link):raise RuntimeError('Old plot copy failed')

 removed=[]
 for child in sorted(OLD.iterdir()):
  if child.name=='HELDOUT_LIDAR_MAP_REFERENCE_20261009':continue
  if child.is_symlink():
   # Keep the two symlink aliases to the protected reference, since scripts
   # and notebooks may use them; delete only expired / invalid aliases.
   if child.name in ('LATEST_18SEP_MAP_REFERENCE.csv','LATEST_18SEP_MAP_REFERENCE_PLOT.png'):
    continue
   child.unlink()
  elif child.is_dir():
   if child.is_symlink():raise RuntimeError('Unexpected directory symlink')
   shutil.rmtree(child)
  elif child.is_file():
   if child.name not in (*protected_files,'README.md'):
    raise RuntimeError('Unexpected loose old-result file: '+str(child))
   child.unlink()
  else:raise RuntimeError('Unexpected filesystem object: '+str(child))
  removed.append(child.name)

 # Old one_full_loop had seven short throwaway smoke tests and the preserved
 # 2-loop PCD mapping baseline (archived above). No retained reference
 # depends on its original results location.
 old_loop=RESULTS/'one_full_loop'
 if old_loop.is_dir() and not old_loop.is_symlink():
  shutil.rmtree(old_loop)
  removed.append('one_full_loop/*')
 if (RESULTS/'camera_lidar_imu_gps').exists():
  raise RuntimeError('Unexpected original main-bag results remain')
 if (RESULTS/'CLEANUP_20260926_deleted_runs.txt').is_file():
  (RESULTS/'CLEANUP_20260926_deleted_runs.txt').unlink()

 (OLD/'README.md').write_text('# Protected September-18 reference files\n\n'
    'The *result runs* have been curated into `../BEST_SO_FAR_20261015/18_sep_2026_combined/`.\n'
    'Only the ORIGINAL corrected heldout PCD reference and two aliases remain here,\n'
    'to preserve old analysis scripts. Legacy auxiliary references and useful\n'
    'plots were retained under the curated group’s `REFERENCE/` directory.\n')
 (RESULTS/'README.md').write_text('# Curated FAST-LIVO2 localisation results\n\n'
    'Primary: `BEST_SO_FAR_20261015/` — eight verified full runs (two bags × four cases),\n'
    'plots, raw pose CSV, original logs, recomputed metrics, and run provenance.\n\n'
    'Archives: `_archive/USEFUL_HISTORICAL_RUNS_20261015.tar.gz`,\n'
    'plus the preexisting 2026-10-15 evidence ZIPs.\n\n'
    'Protected: `18_sep_2026_combined/HELDOUT_LIDAR_MAP_REFERENCE_20261009/`\n'
    'and aliases, retained at original paths for analysis code compatibility.\n\n'
    '**WARNING:** Best available != validated safe; neither LiDAR-failure result\n'
    'meets discontinuity/safety requirements. The reference is not surveyed ground truth.\n')
 audit={'date':'2026-10-15','verified_best_full_runs':8,
        'selected_case_keys':sorted(cases),
        'archived_nonselected_useful_file':str(ARCHIVE.relative_to(ROOT)),
        'archive_sha256':sha(ARCHIVE),
        'deleted_legacy_sep_result_entries':removed,
        'deleted_main_result_tree':True,'deleted_old_one_full_loop_tree':True,
        'historical_reference_preserved_at':str(REF.relative_to(ROOT)),
        'independent_ground_truth':False,'production_approved':False}
 (RESULTS/'_archive/cleanup_manifest.json').write_text(json.dumps(audit,indent=2)+'\n')
 print('SUCCESS: curated 8 full-run folders, verified historical archive, retained corrected reference')
 print('Deleted old September entries:',len(removed))
 print('Results roots now:',[x.name for x in RESULTS.iterdir() if x.is_dir()])

if __name__=='__main__':main()
