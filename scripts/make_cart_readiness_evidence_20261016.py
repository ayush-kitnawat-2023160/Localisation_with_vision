#!/usr/bin/env python3
"""Actual saved-replay plots and fail-closed readiness score; NEVER invent GT."""
import json,csv,hashlib
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont

root=Path(__file__).resolve().parents[1]
out=root/'results/CART_TEST_READINESS_20261016'
out.mkdir(exist_ok=True)
ref='step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv'
base=root/'results/BEST_SO_FAR_20261015/camera_lidar_imu_gps/LIDAR_FAILED'
a=root/'step3_global/outage_causal_smoother_dev_20261016/MAIN_LIDAR_OUTAGE_SMOOTHER_190S'
b=root/'step3_global/outage_causal_frozen_oct13_dev_20261016/MAIN_FROZEN_OCT13_CAUSAL_190S'
report=json.loads((root/'step3_global/outage_causal_frozen_oct13_dev_20261016/frozen_oct13_smoother_vs_curated_185s.json').read_text())
rows=[
 ('Curated Oct13 baseline (290s original)',base,'historical',report['old']),
 ('Newer-image smoother (190s)',a,'rejected',None),
 ('Frozen-Oct13 causal smoother (190s)',b,'rejected',report['new'])]
F='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FB='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
def font(sz,bold=False):return ImageFont.truetype(FB if bold else F,sz)
w,h=2100,1780
p=Image.new('RGB',(w,h),'white');d=ImageDraw.Draw(p)
d.text((26,14),'PRE-CART CHECK | Original vs causal smoothing replays',font=font(33,True),fill='#172b3c')
d.text((26,67),'Real PCD trajectory images; no manual XY alignment or hand-smoothing. Baseline full 290s, trials only 190s.',font=font(18),fill='#526174')
pw=675;ph=1510
for i,(title,path,status,metrics) in enumerate(rows):
 x=25+i*(pw+13);y=118
 d.rounded_rectangle((x,y,x+pw,y+ph),fill='#f6f8fb',outline='#d5dfe9',radius=13,width=2)
 d.text((x+12,y+13),title,font=font(17,True),fill='#24364c')
 if metrics is not None:
  d.text((x+12,y+45),f"Outage {metrics['outage_proxy_rmse_m']:.2f}m | Recovery {metrics['recovery_proxy_rmse_m']:.2f}m",font=font(15),fill='#9f343e')
  d.text((x+12,y+73),f"Max published step: {metrics['max_published_pose_step_m']:.2f}m",font=font(15),fill='#9f343e')
 else:
  m=dict(x.split('=',1) for x in (path/'metrics_exact.txt').read_text().splitlines() if '=' in x)
  d.text((x+12,y+45),f"Outage {float(m['reference_disagreement_failure_rmse3d_m']):.2f}m | Recovery {float(m['reference_disagreement_post_failure_20s_rmse3d_m']):.2f}m",font=font(15),fill='#9f343e')
  d.text((x+12,y+73),f"Max published step: {float(m['step_max_m']):.2f}m",font=font(15),fill='#9f343e')
 with Image.open(path/'plots/07_orange_trajectory_on_pcd_clean.png') as img:
  img=img.convert('RGB')
  img.thumbnail((pw-22,ph-135),Image.Resampling.LANCZOS)
  p.paste(img,(x+(pw-img.width)//2,y+108+(ph-124-img.height)//2))
d.text((26,1670),'NO-GO: both new smoothing builds reduce some single steps but create/retain severe position drift and pose-stream gaps.',font=font(21,True),fill='#9f3038')
d.text((26,1712),'Proxy RMSE is not surveyed ground truth. Smoothness does NOT certify map-frame localisation accuracy or vehicle safety.',font=font(18),fill='#43576d')
png=out/'ACTUAL_SMOOTHING_VS_BASELINE_PCD_20261016.png'
p.save(png,optimize=True)
readiness={
 'date':'2026-10-16',
 'cart_autonomous_motion_permitted':False,
 'cart_stationary_sensors_and_visualization_only':True,
 'motors_brakes_steering_keep_disabled_during_initial_cart_test':True,
 'reference_only_diagnostic_proxy':True,
 'preferred_deployment_image':'NONE',
 'production_unchanged':True,
 'frozen_best_retained':str(base.relative_to(root)),
 'frozen_baseline_first_185s':report['old'],
 'outage_smoother_newer_image_190s':{
  k:v for k,v in dict((x.split('=',1) for x in (a/'metrics_exact.txt').read_text().splitlines() if '=' in x)).items()
   if k in ['reference_disagreement_3d_rmse_m','reference_disagreement_failure_rmse3d_m','reference_disagreement_post_failure_20s_rmse3d_m','step_max_m']},
 'frozen_baseline_trial_first_185s':report['new'],
 'watchdog_local_unit_tests':'PASS',
 'watchdog_ros_synthetic_untrusted_latch':'PASS, signalling only',
 'safety_flag_patch_compiled':'PASS; no replay of corrected safety flag, and no DBW connection',
 'live_cart_control_arbiter_or_braking_tested':False,
 'two_nuc_host_chrony_and_sensor_time_verified_today':False,
 'physical_extrinsics_validated_against_surveyed_landmarks':False,
 'under_point_five_metres_confirmed':False,
 'all_sensor_failure_modes_validated_in_current_image':False
}
(out/'readiness_manifest.json').write_text(json.dumps(readiness,indent=2)+'\n')
print('WROTE',png,png.stat().st_size,'bytes; readiness',out/'readiness_manifest.json')
