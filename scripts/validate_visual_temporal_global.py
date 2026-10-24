#!/usr/bin/env python3
"""Independent visual-only temporal confirmation on saved cold pose candidates.
This is an offline gate, NOT a running ROS authority, and uses no LiDAR or GPS.
"""
import json,math
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];root=R/'outputs/visual_global_20261020'
summary={}
for variant,outdir in [('baseline',root/'heldout_visual_cold'),('fast',root/'ablation_lsh_stride4_6'),('dense_1hz',root/'dense_1hz_top24')]:
    summary[variant]={}
    for name in ['localization','localization_from2','localization_3']:
        p=outdir/(name+'_visual_matches.json')
        if not p.exists():continue
        rows=json.load(open(p));confirmed=[];streak=0;prev=None;rejections=[];first_at=None
        for r in rows:
            w=r.get('winner')
            if not r.get('pnp_verified',False) or r.get('ambiguous',True) or not w:
                streak=0;prev=None;continue
            xyz=np.array(w['mappos_m'],float);t=float(r['query_sec']);q=np.array(w['rotation_xyzw'],float)
            if prev is not None:
                dt=t-prev[0];distance=float(np.linalg.norm(xyz-prev[1]));dot=abs(float(np.dot(q,prev[2])/max(1e-9,np.linalg.norm(q)*np.linalg.norm(prev[2]))));ang=2*math.degrees(math.acos(max(0,min(1,dot))))
                ok=(0<dt<=5.0 and distance<=1.0+5.0*dt and ang<=15.0+70.0*dt)
                if not ok:
                    rejections.append({'time_s':t,'motion_m':distance,'dt_s':dt,'relative_rotation_deg':ang});streak=0;prev=None
            streak+=1;prev=(t,xyz,q)
            if streak>=3:
                confirmed.append(t)
                if first_at is None:first_at=t
        summary[variant][name]={'queries':len(rows),'independent_pnp':sum(r.get('pnp_verified',False) for r in rows),'temporally_confirmed_positions':len(confirmed),'first_confirmation_bag_sec':first_at,'last_confirmation_bag_sec':confirmed[-1] if confirmed else None,'implausible_motion_rejections':rejections}
(root/'VISUAL_TEMPORAL_CONFIRMATION.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
