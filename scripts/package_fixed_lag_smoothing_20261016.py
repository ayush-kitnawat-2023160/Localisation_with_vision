#!/usr/bin/env python3
"""Actual raw-vs-smoothed trajectory comparisons and verification bundle."""
import json
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
from zipfile import ZipFile,ZIP_DEFLATED
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/FIXED_LAG_SMOOTHING_DEV_20261016'
summary=json.loads((OUT/'summary.json').read_text())
cases=summary['cases']
B='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
F='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def font(size,bold=False):return ImageFont.truetype(B if bold else F,size)
def panel(keys,title,file):
 cw=704;ch=590;gap=16;left=24;top=140;footer=130
 w=left*2+cw*2+gap
 h=top+4*ch+3*gap+footer
 bg=Image.new('RGB',(w,h),'white');d=ImageDraw.Draw(bg)
 d.text((left,18),title,font=font(29,True),fill='#243246')
 d.text((left,65),'New 16 Oct 2026 fixed-lag output | original vs smooth at matching sensor timestamps',font=font(16),fill='#596c7b')
 d.text((left,100),'Blue = published, orange = robust smoothed. Loop-closed reference is diagnostic, NOT ground truth.',font=font(15),fill='#596c7b')
 for index,key in enumerate(keys):
  m=cases[key];y=top+index*(ch+gap)
  d.rounded_rectangle((left,y,left+cw*2+gap,y+ch),radius=12,fill='#f7f9fb',outline='#d6dfe8',width=2)
  d.text((left+18,y+10),key.replace('_',' ').upper(),font=font(18,True),fill='#243246')
  d.text((left+18,y+40),f"3D proxy RMSE {m['positions']['full']['raw_rmse_m']:.3f} -> {m['positions']['full']['smoothed_rmse_m']:.3f} m",font=font(16),fill='#314b65')
  d.text((left+755,y+40),f"max step {m['raw']['max_step_m']:.3f} -> {m['smoothed']['max_step_m']:.3f}m  |  >0.85m steps {m['raw']['steps_above_0p85m']} -> {m['smoothed']['steps_above_0p85m']}",font=font(15),fill='#a03e3c')
  with Image.open(OUT/key/'RAW_VS_SMOOTHED_XY_ERROR_STEPS.png') as im:
   im=im.convert('RGB')
   ow,oh=im.size;third=ow//3
   for j,part in enumerate((im.crop((0,45,third,oh)),im.crop((third*2,45,ow,oh)))):
    part.thumbnail((cw-20,ch-103),Image.Resampling.LANCZOS)
    bg.paste(part,(left+j*(cw+gap)+(cw-part.width)//2,y+90+(ch-103-part.height)//2))
 d.text((left,h-100),'UNTRUSTED FOR AUTONOMOUS DRIVING: numeric smoothness is not independent map accuracy.',font=font(18,True),fill='#9f3542')
 d.text((left,h-67),'5Hz failure streams are processed; untrustworthy pose steps and gaps are flagged, never silently hidden.',font=font(16),fill='#465b71')
 p=OUT/file;bg.save(p,optimize=True)
 print('Created',p,p.stat().st_size,flush=True)
panel(['main_healthy','main_camera','main_lidar','main_imu'],'Main bag | healthy and all three failures','01_MAIN_BAG_RAW_VS_SMOOTHED.png')
panel(['holdout_healthy','holdout_camera','holdout_lidar','holdout_imu'],'September 18 holdout | same fixed filter','02_SEP18_RAW_VS_SMOOTHED.png')
lines=['# Fixed-lag smoothing: eight real trajectories — 16 October 2026','',
 'NEW smoothed pose CSVs and ROS1 shadow preview node. NOT vehicle navigation approved.',
 'Uses 0.23s fixed lag, 1.15s fit window, robust linear regression on actual map-frame poses.',
 'No manual XY translation/rotation, no loop-closed reference used to smooth, no map anchor correction.',
 'All metrics below use diagnostic reference proxies, not independent surveyed ground truth.','',
 '| Case | Raw RMSE | Smoothed RMSE | Raw step | Smooth step | >0.85m steps raw/smooth | Max gap | Numerically unsafe |',
 '|---|---:|---:|---:|---:|---:|---:|---:|']
for key,m in cases.items():
 lines.append(f"| {key} | {m['positions']['full']['raw_rmse_m']:.3f}m | {m['positions']['full']['smoothed_rmse_m']:.3f}m | {m['raw']['max_step_m']:.3f}m | {m['smoothed']['max_step_m']:.3f}m | {m['raw']['steps_above_0p85m']}/{m['smoothed']['steps_above_0p85m']} | {m['smoothed']['max_output_gap_s']:.3f}s | {m['smoothed']['numerically_unsafe_fraction']*100:.1f}% |")
lines.extend(['','## Artifacts',
 '01_MAIN_BAG_RAW_VS_SMOOTHED.png — main bag all 4 modes, raw/filtered XY and step plots',
 '02_SEP18_RAW_VS_SMOOTHED.png — 18 Sep all 4 modes, exact same parameters',
 'Each case has RAW_VS_SMOOTHED_XY_ERROR_STEPS.png, smoothed_pose.csv, metrics.json',
 'summary.json is machine readable; FIXED_LAG_SMOOTHING_8_CASE_EVIDENCE_20261016.zip contains all plots and data.','',
 '## Implementation and live test',
 'Pure filter: scripts/robust_fixed_lag_pose_smoother_20261016.py',
 'ROS preview node: scripts/ros_fixed_lag_pose_smoother_20261016.py',
 'ROS opt-in launch: launch/fast_livo2_fixed_lag_preview_only_dev_20261016.launch',
 'ROS preview output: /fast_livo2/step3_global_pose_smoothed_preview',
 'Numeric discontinuity topic: /fast_livo2/step3_visual_smoother_numeric_unsafe',
 'Image: fast_livo2_failsafe:visual-fixed-lag-shadow-dev-20261016',
 'ROS1 synthetic integration: PASS, 60/63 poses with 5Hz/10Hz input, monotone timestamp and injected 2.6m glitch flagged.','',
 '## Safety and remaining issues',
 'This is a delayed shadow preview, not a validated localisation estimator. Do not feed into planner or DBW.',
 'It does not establish map trust. Large steps remain in LiDAR-fail recordings (>0.85m) and holdout gaps >0.3s.',
 'Current map positioning is still ~1-2m away from the diagnostic proxy; target <=0.5m remains unmet.',
 'No actuator test or localization-to-DBW stop wiring was performed here.',
 'Original eight curated best runs and production image were not changed.'])
(OUT/'README.md').write_text('\n'.join(lines)+'\n')
z=OUT/'FIXED_LAG_SMOOTHING_8_CASE_EVIDENCE_20261016.zip'
with ZipFile(z,'w',ZIP_DEFLATED,compresslevel=4) as f:
 for p in sorted(OUT.rglob('*')):
  if p.is_file() and p!=z:f.write(p,str(p.relative_to(OUT)))
with ZipFile(z) as f:assert f.testzip() is None
print('COMPLETED',z,round(z.stat().st_size/1e6,2),'MB')
