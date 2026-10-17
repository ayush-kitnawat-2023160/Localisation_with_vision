#!/usr/bin/env python3
"""Collect 16 Oct real LiDAR-fail trial plots into a separate, verified folder."""
import json, shutil, hashlib
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/LATEST_LIDAR_TRIAL_PLOTS_20261016'
ITEMS=[
('main_gicp_0p35','Main | GICP seed cap 0.35m','step3_global/depth_bounded_seed_trial_20261016/DEPTH_SEED_BOUNDED_0P35_190S','main',1),
('main_gicp_0p15','Main | GICP seed cap 0.15m','step3_global/depth_bounded_seed_trial_20261016/DEPTH_SEED_BOUNDED_0P15_190S','main',1),
('sep18_gicp_0p35','Sep18 | GICP seed cap 0.35m','step3_global/depth_bounded_seed_trial_20261016/HELDOUT_18SEP_DEPTH_SEED_0P35_220S','sep18',1),
('main_dualmap_veto','Main | dual-map veto-only','step3_global/depth_crossmap_veto_trial_20261016/DEPTH_CROSSMAP_VETO_190S','main',1),
('main_dualmap_blend','Main | dual-map XYZ blend','step3_global/depth_crossmap_trial_20261016/DEPTH_DUAL_MAP_LOADED_190S','main',0),
('main_rgbd_feature','Main | RGB-D feature PnP','step3_global/depth_feature_pnp_trial_20261016/DEPTH_FEATURE_PNP_190S','main',0),
('main_local_tracker','Main | local RGB-D tracker','step3_global/depth_temporal_tracker_trial_20261016/DEPTH_LOCAL_2P2_190S','main',0),
('main_rgbd_hysteresis','Main | RGB-D vote hysteresis','step3_global/rgbd_mode_vote_hysteresis_trial_20261016/LIDAR_FAIL_RGBD_VOTE_HYST_195S','main',0),
('main_rgbd_vote','Main | RGB-D motion vote','step3_global/rgbd_mode_vote_trial_20261016/LIDAR_FAIL_RGBD_MODE_VOTE_195S','main',0),
('main_slow_ramp','Main | slower map correction','step3_global/depth_ramp_rate_trial_20261016/LIDAR_FAIL_SLOW_RAMP_195S','main',0),
]
SOURCE={'trajectory':'07_orange_trajectory_on_pcd_clean.png','error':'02_error_vs_time.png','steps':'03_pose_step_continuity.png'}
F='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FB='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
def font(size,bold=False):return ImageFont.truetype(FB if bold else F,size)
def scores(folder):
 rows=dict(x.split('=',1) for x in (folder/'metrics_exact.txt').read_text().splitlines() if '=' in x)
 return {k:round(float(rows[v]),3) for k,v in {
 'outage':'reference_disagreement_failure_rmse3d_m',
 'recovery':'reference_disagreement_post_failure_20s_rmse3d_m',
 'step':'step_max_m',
 'total':'reference_disagreement_3d_rmse_m'}.items()}
def sheet(items,kind,header,subtitle,cols,width,height,name):
 gap=18; head=150; foot=93; pad=28
 rows=(len(items)+cols-1)//cols
 w=pad*2+cols*width+(cols-1)*gap
 h=head+rows*height+(rows-1)*gap+foot
 im=Image.new('RGB',(w,h),'#ffffff');d=ImageDraw.Draw(im)
 d.text((pad,18),header,font=font(30,True),fill='#172335')
 d.text((pad,66),subtitle,font=font(18),fill='#4e6073')
 d.text((pad,99),'16 Oct 2026 — actual 1x replay plots, not the older curated collage.',font=font(17),fill='#43566a')
 for i,case in enumerate(items):
  x=pad+(i%cols)*(width+gap);y=head+(i//cols)*(height+gap)
  d.rounded_rectangle((x,y,x+width,y+height),fill='#f6f8fb',outline='#d7e1ea',radius=14,width=2)
  d.text((x+16,y+14),case['title'],font=font(20,True),fill='#203047')
  m=case['metrics']
  d.text((x+16,y+46),f"Outage: {m['outage']:.2f} m  |  Recovery: {m['recovery']:.2f} m  |  Max step: {m['step']:.2f} m",font=font(14),fill='#8e3640')
  d.text((x+16,y+71),'REJECTED — development trial, NOT safe to deploy',font=font(14,True),fill='#a12c36')
  with Image.open(OUT/(case['key']+'__'+kind+'.png')) as src:
   src=src.convert('RGB')
   aw=width-26;ah=height-112
   src.thumbnail((aw,ah),Image.Resampling.LANCZOS)
   im.paste(src,(x+(width-src.width)//2,y+101+(ah-src.height)//2))
 d.text((pad,h-foot+16),'Not independent surveyed GT: errors measured against saved loop-closed proxy trajectories.',font=font(17,True),fill='#9d2e37')
 d.text((pad,h-foot+50),'Only LiDAR-failure experiments were replayed on Oct 16; healthy/camera/IMU plots are from Oct 15.',font=font(16),fill='#43566a')
 p=OUT/name
 im.save(p,optimize=True)
 print('CREATED',p,'px',im.size,flush=True)
def main():
 OUT.mkdir(parents=True,exist_ok=True)
 entries=[]
 for key,title,rel,bag,featured in ITEMS:
  run=ROOT/rel
  entry={'key':key,'title':title,'source_run':rel,'bag':bag,'featured':bool(featured),'metrics':scores(run)}
  for suffix,filename in SOURCE.items():
   src=run/'plots'/filename
   dest=OUT/(key+'__'+suffix+'.png')
   shutil.copy2(src,dest)
   if hashlib.sha256(src.read_bytes()).digest()!=hashlib.sha256(dest.read_bytes()).digest():raise RuntimeError('Hash mismatch '+key)
   entry[suffix+'_file']=dest.name
  entries.append(entry)
 first=[x for x in entries if x['featured']]
 others=[x for x in entries if not x['featured']]
 sheet(first,'trajectory','LATEST LiDAR fixes | real PCD trajectory overlays','Main GICP caps, Sep18 holdout, and independent dual-map veto',2,800,850,'01_LATEST_LIDAR_TRAJECTORIES.png')
 sheet(first,'error','LATEST LiDAR fixes | error versus time','Error plots with sensor failure window and recovery',2,1050,690,'02_LATEST_LIDAR_ERRORS.png')
 sheet(first,'steps','LATEST LiDAR fixes | pose-step continuity','Actual published-pose jumps, not hand-smoothed plots',2,1050,690,'03_LATEST_LIDAR_JUMPS.png')
 sheet(others,'trajectory','Other Oct 16 LiDAR-failure fixes','Six additional registered geometric, feature, and motion candidate plots',3,650,760,'04_OTHER_LIDAR_TRAJECTORIES.png')
 (OUT/'manifest.json').write_text(json.dumps({'date':'2026-10-16','trials':entries,'independent_ground_truth':False,'deployment_approved':False},indent=2)+'\n')
 lines=['LATEST LIDAR FAILURE PLOTS — 2026-10-16','','All are newer than the BEST_SO_FAR_20261015 compilation.','Every PNG below is a byte-for-byte copy of the ORIGINAL replay plot.','These candidates were rejected; none is safe for cart deployment.','',
 '01_LATEST_LIDAR_TRAJECTORIES.png — four key trajectory/PCD comparisons',
 '02_LATEST_LIDAR_ERRORS.png — corresponding error versus time',
 '03_LATEST_LIDAR_JUMPS.png — corresponding pose step continuity',
 '04_OTHER_LIDAR_TRAJECTORIES.png — six more actual LiDAR trials','','']
 for e in entries:
  m=e['metrics']
  lines.extend([e['key']+': '+e['title'], '  Failure RMSE '+str(m['outage'])+' m; recovery RMSE '+str(m['recovery'])+' m; max pose step '+str(m['step'])+' m','  Original '+e['source_run'], ''])
 lines.extend(['Only LiDAR-failure replays were run Oct 16; other 3 modes remain in 20261015 curated folder.','Reference comparison is diagnostic proxy only, not independently surveyed GT.',''])
 (OUT/'README.txt').write_text('\n'.join(lines))
 zip_path=OUT/'LATEST_LIDAR_20261016_PLOTS.zip'
 with ZipFile(zip_path,'w',ZIP_DEFLATED,compresslevel=3) as z:
  for p in sorted(OUT.iterdir()):
   if p.is_file() and p!=zip_path:z.write(p,p.name)
 with ZipFile(zip_path) as z: assert z.testzip() is None
 print('COMPLETE',OUT,'zip bytes',zip_path.stat().st_size,'trials',len(entries))
if __name__=='__main__':main()
