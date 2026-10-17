#!/usr/bin/env python3
"""Consolidate eight actual 1x ROS replays. Refuse to certify bad builds."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
from zipfile import ZipFile, ZIP_DEFLATED
import csv, hashlib, json, shutil, subprocess
from datetime import datetime
import numpy as np
R=Path(__file__).resolve().parents[1]
O=R/'results/FINAL_SINGLE_IMAGE_RC_20261016'
RELEASE=R/'release/ALIVE_FASTLIVO2_OFFLINE_RC_20261016'
ROOT=R/'results/BEST_SO_FAR_20261015'
REF={
'main':R/'step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv',
'sept18':R/'results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv'}
CASES=[('main',290,125,165),('sept18',220,90,130)]
MODES=[('HEALTHY','HEALTHY'),('CAMERA_FAIL','CAMERA_FAILED'),('LIDAR_FAIL','LIDAR_FAILED'),('IMU_FAIL','IMU_FAILED')]
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def fnt(i,bold=False):return ImageFont.truetype(BOLD if bold else FONT,i)
def resource_stats(bag,mode):
 path=O/'docker_resource_samples.csv'
 if not path.is_file():return {'samples':0,'tested_on_actual_NUC':False}
 with path.open() as f:rr=list(csv.DictReader(f))
 name='fastlivo_step3_'+mode+'_SINGLE_IMAGE_1X'
 # Both recordings reuse the same container name; disambiguate using the
 # audited completed-run wall-clock boundaries, not the name alone.
 with (O/'RUN_STATUS.csv').open() as f:status=list(csv.DictReader(f))
 finished=[z for z in status if z['recording']==bag and z['mode']==mode and z['state']=='COMPLETE']
 assert len(finished)==1,(bag,mode,finished)
 begin=datetime.fromisoformat(finished[0]['started'])
 end=datetime.fromisoformat(finished[0]['finished'])
 samples=[z for z in rr if z.get('container')==name and begin<=datetime.fromisoformat(z['timestamp'])<=end]
 if not samples:return {'samples':0,'tested_on_actual_NUC':False}
 cpu=[];mem=[]
 for z in samples:
  try:
   cpu.append(float(z['cpu_percent'].replace('%','')))
   value,unit=z['memory_usage'].split(' / ')[0].strip()[:-3],z['memory_usage'].split(' / ')[0].strip()[-3:]
   mul=1.0 if unit=='GiB' else 1./1024. if unit=='MiB' else None
   if mul:mem.append(float(value)*mul)
  except (KeyError,ValueError):continue
 return {'samples':len(samples),'docker_cpu_median_pct':round(float(np.median(cpu)),2) if cpu else None,
         'docker_cpu_p95_pct':round(float(np.percentile(cpu,95)),2) if cpu else None,
         'docker_memory_peak_GiB':round(float(max(mem)),3) if mem else None,
         'tested_on_actual_NUC':False,'interpretation':'Docker 100pct CPU is ~1 core; laptop offline bag replay only'}
def reference(p):
 with p.open() as f: rr=list(csv.DictReader(f))
 return np.array([float(x['timestamp']) for x in rr]),np.array([[float(x[k]) for k in 'xyz'] for x in rr])
def pose_score(p,rt,rx,duration,window):
 with p.open() as f: rr=list(csv.DictReader(f))
 t=np.array([float(x['field.header.stamp'])*1.e-9 for x in rr])
 xyz=np.array([[float(x['field.pose.position.'+k]) for k in 'xyz'] for x in rr])
 mask=(t>=rt[0])&(t<=min(rt[-1],rt[0]+duration))
 t,xyz=t[mask],xyz[mask]
 assert len(t)>=duration*7 and t[-1]>=rt[0]+duration-6,('Short run',p,len(t))
 assert np.all(np.diff(t)>0) and np.isfinite(xyz).all()
 target=np.column_stack([np.interp(t,rt,rx[:,i]) for i in range(3)])
 e=xyz-target;err=np.linalg.norm(e,axis=1)
 dt=np.diff(t);step=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
 def val(m):
  return round(float(np.sqrt(np.mean(err[m]**2))),4) if m.any() else None
 s={'poses':len(t),'duration_s':round(float(t[-1]-t[0]),3),
    'rmse3d_proxy_m':val(np.ones(len(t),bool)),
    'rmse_xy_proxy_m':round(float(np.sqrt(np.mean(np.sum(e[:,:2]**2,axis=1)))),4),
    'max_proxy_error_m':round(float(err.max()),4),
    'max_step_m':round(float(step.max()),4),
    'steps_gt_0p85m':int((step>.85).sum()),
    'max_gap_s':round(float(dt.max()),4),
    'gaps_gt_0p3s':int((dt>.3).sum())}
 if window is not None:
  a,b=window;ts=t-rt[0]
  s.update(failure_rmse3d_proxy_m=val((ts>=a)&(ts<b)),
           recovery20_rmse3d_proxy_m=val((ts>=b)&(ts<b+20)),
           pre_failure_rmse3d_proxy_m=val(ts<a))
 return s
def plot(items,part,filename):
 cw=700;ch=660;gap=16;head=132;foot=96
 w=35*2+4*cw+3*gap;h=head+2*ch+gap+foot
 im=Image.new('RGB',(w,h),'white');d=ImageDraw.Draw(im)
 d.text((35,15),'ALiVE | One immutable experimental image | 8 real ROS replays',font=fnt(30,True),fill='#192c3c')
 d.text((35,60),'Top: main 290s bag   |   Bottom: September 18 220s bag (corrected map reference)',font=fnt(19),fill='#516276')
 d.text((35,92),'Columns: Healthy / Camera failed / LiDAR failed / IMU failed; all at 1x.',font=fnt(18),fill='#516276')
 for i,x in enumerate(items):
  xx=35+(i%4)*(cw+gap);yy=head+(i//4)*(ch+gap)
  d.rounded_rectangle((xx,yy,xx+cw,yy+ch),fill='#f6f8fb',outline='#d4dfe7',radius=12,width=2)
  m=x['measured']
  d.text((xx+13,yy+12),x['bag']+' '+x['mode'],font=fnt(18,True),fill='#203449')
  d.text((xx+13,yy+42),f"RMSE proxy {m['rmse3d_proxy_m']:.2f}m | jump {m['max_step_m']:.2f}m",font=fnt(17),fill='#953745')
  p=R/x['run']/ 'plots'/part
  with Image.open(p) as picture:
   picture=picture.convert('RGB')
   picture.thumbnail((cw-24,ch-100),Image.Resampling.LANCZOS)
   im.paste(picture,(xx+(cw-picture.width)//2,yy+80+(ch-100-picture.height)//2))
 d.text((35,h-foot+18),'PROXY = saved-map-registration reference, not surveyed ground truth. NOT AUTONOMOUS-DRIVE APPROVED.',font=fnt(19,True),fill='#ab3042')
 d.text((35,h-foot+53),'These are not individually cherry-picked best runs; all use the same Docker image and calibration.',font=fnt(17),fill='#526578')
 save=O/filename;im.save(save,optimize=True)
 print('PLOT',save,'size',save.stat().st_size,flush=True)
def main():
 st=list(csv.DictReader((O/'RUN_STATUS.csv').open()))
 assert sum(x['state']=='COMPLETE' for x in st)==8,('Incomplete results. Do not fabricate final files.',st)
 assert (O/'full_suite.log').read_text().strip().endswith('EIGHT_REPLAYS_FINISHED')
 image=(O/'frozen_docker_image_id.txt').read_text().strip()
 assert subprocess.check_output(['docker','image','inspect','--format','{{.Id}}','fast_livo2_failsafe:single-build-rc-20261016'],text=True).strip()==image
 out=[]
 for bag,dur,a,b in CASES:
  rt,rx=reference(REF[bag])
  for mode,histname in MODES:
   run=O/bag/(mode+'_SINGLE_IMAGE_1X')
   for required in ('global_pose.csv','metrics_exact.txt','launch.log','run_metadata.txt','plots/07_orange_trajectory_on_pcd_clean.png','plots/02_error_vs_time.png','plots/03_pose_step_continuity.png'):
    p=run/required
    assert p.is_file() and p.stat().st_size>100,('Missing real replay output',p)
   md=(run/'run_metadata.txt').read_text()
   assert 'play_rate=1' in md and 'failure_duration_s=40' in md
   fields=dict(line.split('=',1) for line in md.splitlines() if '=' in line)
   assert Path(fields['reference']).resolve()==REF[bag].resolve(),(
     'WRONG REFERENCE — refuse finalization',mode,fields['reference'])
   expected_bag='camera_lidar_imu_gps.bag' if bag=='main' else '18_sep_2026_combined.bag'
   assert Path(fields['bag_path']).name==expected_bag
   sensor={'HEALTHY':'none','CAMERA_FAIL':'camera','LIDAR_FAIL':'lidar','IMU_FAIL':'imu'}[mode]
   assert fields['sensor']==sensor
   log=(run/'launch.log').read_text(errors='replace')
   expected={'CAMERA_FAIL':'FULL_LIVO -> LIO_CAMERA_FAILED',
             'LIDAR_FAIL':'FULL_LIVO -> VI_LIDAR_FAILED',
             'IMU_FAIL':'FULL_LIVO -> LV_IMU_FAILED'}
   if mode in expected:
    assert expected[mode] in log,('Injected failure missing from actual ROS mode transitions',mode,run)
    assert '-> FULL_LIVO' in log,('Sensor recovery not observed in actual ROS mode transitions',mode,run)
   win=None if mode=='HEALTHY' else (a,b)
   m=pose_score(run/'global_pose.csv',rt,rx,dur,win)
   historical=ROOT/('camera_lidar_imu_gps' if bag=='main' else '18_sep_2026_combined')/histname/'global_pose.csv'
   old=pose_score(historical,rt,rx,dur,win)
   x={'bag':bag,'mode':mode,'run':str(run.relative_to(R)),
      'measured':m,'historically_selected_best_on_same_reference':old,
      'host_offline_resource_samples':resource_stats(bag,mode),
      'mode_change_verified':mode=='HEALTHY' or expected[mode] in log,
      'pose_csv_sha256':sha(run/'global_pose.csv'),
      'reference':str(REF[bag].relative_to(R)),
      'reference_sha256':sha(REF[bag])}
   out.append(x)
   print(bag,mode,'RMSE new',m['rmse3d_proxy_m'],'historical',old['rmse3d_proxy_m'],
     'max step',m['max_step_m'],'gap',m['max_gap_s'],flush=True)
 gates=[]
 for x in out:
  m=x['measured']
  if m['rmse3d_proxy_m']>.5:gates.append(x['bag']+'/'+x['mode']+':3d_proxy_rmse_above_point_five')
  if m['max_step_m']>.85:gates.append(x['bag']+'/'+x['mode']+':pose_jump_above_0p85m')
  if m['max_gap_s']>.3:gates.append(x['bag']+'/'+x['mode']+':time_gap_above_0p3s')
 data={'date':'2026-10-16','status':'OFFLINE_RELEASE_CANDIDATE_NOT_FOR_CART_DEPLOYMENT',
       'one_image_all_8':True,'docker_image':'fast_livo2_failsafe:single-build-rc-20261016',
       'docker_id':image,'replay_rate_x':1,'main_duration_s':290,'sept18_duration_s':220,
       'failure_window_main_s':[125,165],'failure_window_sept18_s':[90,130],
       'independent_ground_truth':False,'reference_type':'loopclosed map registration proxies, not surveyed GT',
       'stop_to_dbw_braking_tested':False,'real_cart_tested':False,
       'under_0p5m_target_met':False,'approved_for_autonomous_cart':False,
       'accuracy_continuity_gates_failed':gates,
       'cases':out}
 (O/'single_image_final_audit.json').write_text(json.dumps(data,indent=2)+'\n')
 with (O/'EIGHT_FINAL_METRICS.csv').open('w',newline='') as f:
  names=['bag','mode','rmse3d_proxy_m','historical_best_rmse3d_proxy_m','failure_rmse3d_proxy_m','recovery20_rmse3d_proxy_m','max_step_m','steps_gt_0p85m','max_gap_s','gaps_gt_0p3s','poses']
  writer=csv.DictWriter(f,fieldnames=names);writer.writeheader()
  for x in out:
   m=x['measured'];old=x['historically_selected_best_on_same_reference']
   writer.writerow({'bag':x['bag'],'mode':x['mode'],'rmse3d_proxy_m':m['rmse3d_proxy_m'],
    'historical_best_rmse3d_proxy_m':old['rmse3d_proxy_m'],'failure_rmse3d_proxy_m':m.get('failure_rmse3d_proxy_m'),
    'recovery20_rmse3d_proxy_m':m.get('recovery20_rmse3d_proxy_m'),'max_step_m':m['max_step_m'],
    'steps_gt_0p85m':m['steps_gt_0p85m'],'max_gap_s':m['max_gap_s'],
    'gaps_gt_0p3s':m['gaps_gt_0p3s'],'poses':m['poses']})
 plot(out,'07_orange_trajectory_on_pcd_clean.png','01_EIGHT_SINGLE_IMAGE_TRAJECTORIES.png')
 plot(out,'02_error_vs_time.png','02_EIGHT_SINGLE_IMAGE_ERRORS.png')
 plot(out,'03_pose_step_continuity.png','03_EIGHT_SINGLE_IMAGE_JUMPS.png')
 (RELEASE/'manifest').mkdir(parents=True,exist_ok=True)
 (RELEASE/'validation').mkdir(parents=True,exist_ok=True)
 for file in ('single_image_final_audit.json','EIGHT_FINAL_METRICS.csv','reference_map_checksums.sha256'):
  shutil.copy2(O/file,RELEASE/'manifest'/file)
 for file in ('01_EIGHT_SINGLE_IMAGE_TRAJECTORIES.png','02_EIGHT_SINGLE_IMAGE_ERRORS.png','03_EIGHT_SINGLE_IMAGE_JUMPS.png'):
  shutil.copy2(O/file,RELEASE/'validation'/file)
 lines=['# ALiVE FAST-LIVO2 clean code snapshot + offline verification (October 16, 2026)',
 '',
 'Single-image release candidate. NEVER autonomous deployment approved.',
 'Source code/launch/config files copied from the exact frozen Docker image and runtime mounts.',
 'Every mode was replayed in ROS at 1x using the same Docker image and parameter setup.',
 'Both references are saved-map proxies. They are NOT independent surveyed ground truth.',
 '',
 '| Recording | Mode | New proxy 3D RMSE | Hist. cherry-picked best | New outage RMSE | New recovery RMSE | Max step | Max gap |',
 '|---|---|---:|---:|---:|---:|---:|---:|']
 for x in out:
  m=x['measured'];o=x['historically_selected_best_on_same_reference']
  lines.append('| {} | {} | {:.3f}m | {:.3f}m | {} | {} | {:.3f}m | {:.3f}s |'.format(
   x['bag'],x['mode'],m['rmse3d_proxy_m'],o['rmse3d_proxy_m'],
   m.get('failure_rmse3d_proxy_m','—'),m.get('recovery20_rmse3d_proxy_m','—'),
   m['max_step_m'],m['max_gap_s']))
 lines.extend(['','Gate failures: '+str(len(gates))+'.','Original best-so-far 8 runs remain untouched.',
 'This snapshot EXCLUDES external bag files, raw saved map, 800MB visual image keyframes.',
 'Asset paths and Docker hash are in manifest. Full raw runs remain in results/FINAL_SINGLE_IMAGE_RC_20261016.',
 'Do not connect any pose output to DBW before independent braking interlock tests.',
 'Fail-closed localizer trust signalling is NOT yet proven to engage VAT cart brakes.',
 'The corrected September 18 baseline has no artificial right-side triangle.',
 ''])
 (RELEASE/'README_RELEASE.md').write_text('\n'.join(lines))
 target=RELEASE/'ALIVE_FASTLIVO2_OFFLINE_RC_20261016_SOURCE_METRICS.zip'
 with ZipFile(target,'w',ZIP_DEFLATED,compresslevel=4) as z:
  for p in sorted(RELEASE.rglob('*')):
   if p.is_file() and p!=target:z.write(p,str(p.relative_to(RELEASE)))
 with ZipFile(target) as z:assert z.testzip() is None
 print('RELEASE_READY_AS_OFFLINE_RC',target,'MB',round(target.stat().st_size/1e6,2),flush=True)
 # Separate full-evidence package. Do not include 17GB bag or 800MB map assets,
 # but retain exact published poses, local trajectories, map fixes, diagnostic
 # plots and replay metadata from all eight cases in this portable ZIP.
 evidence=O/'FINAL_8_CASE_RAW_RESULTS_AND_PLOTS_20261016.zip'
 with ZipFile(evidence,'w',ZIP_DEFLATED,compresslevel=3) as z:
  for p in [O/'single_image_final_audit.json',O/'EIGHT_FINAL_METRICS.csv',
            O/'reference_map_checksums.sha256',O/'RUN_STATUS.csv',
            O/'docker_resource_samples.csv']:
   if p.is_file():z.write(p,str(p.relative_to(O)))
  for p in REF.values():z.write(p,'REFERENCES/'+p.name)
  for item in out:
   root=R/item['run']
   for name in ['global_pose.csv','trajectory.csv','gicp_map_pose.csv',
                'metrics_exact.txt','run_metadata.txt','launch.log','rosbag.log']:
    p=root/name
    if p.is_file():z.write(p,str(p.relative_to(O)))
   for p in sorted((root/'plots').glob('*.png')):
    z.write(p,str(p.relative_to(O)))
  for name in ('01_EIGHT_SINGLE_IMAGE_TRAJECTORIES.png',
               '02_EIGHT_SINGLE_IMAGE_ERRORS.png',
               '03_EIGHT_SINGLE_IMAGE_JUMPS.png'):
   z.write(O/name,name)
 with ZipFile(evidence) as z:assert z.testzip() is None
 print('FINAL_EIGHT_RAW_RESULTS_EVIDENCE',evidence,'MB',round(evidence.stat().st_size/1e6,2),flush=True)
if __name__=='__main__':main()
