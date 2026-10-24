#!/usr/bin/env python3
"""Compare independent GPS-free learned relocalizers and temporal continuity."""
from pathlib import Path
import json,math,collections
import numpy as np,yaml
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1];O=R/'outputs/learned_visual_20261020'
backends={'xfeat_mnn_orb_retrieval':'heldout_xfeat_mnn','xfeat_mnn_learned_retrieval':'heldout_xfeat_float_retrieval','xfeat_lighterglue':'heldout_xfeat_lighterglue','superpoint_lightglue':'heldout_superpoint_lightglue'}
report={}
for backend,path in backends.items():
 folder=O/path;s=json.load(open(folder/'summary.json'));d={}
 for b in ['localization','localization_from2','localization_3']:
  rows=json.load(open(folder/(b+'_matches.json')));track=[];prev=None;confirmed=[];outliers=[];streak=0
  for r in rows:
   win=r.get('best')
   if not r['matched'] or r['ambiguous'] or win is None:prev=None;streak=0;continue
   xyz=np.array(win['map_xyz_m']);q=np.array(win['rotation_xyzw']);t=float(r['t_sec'])
   if prev is not None:
    dt=t-prev[0];dist=float(np.linalg.norm(xyz-prev[1]));ang=2*math.degrees(math.acos(min(1,max(0,abs(float(np.dot(q,prev[2])))))))
    if dt<=0 or dt>5 or dist>1+5*dt or ang>15+70*dt:
     outliers.append({'t_sec':t,'jump_m':dist,'rotation_deg':ang,'dt_sec':dt});prev=None;streak=0
   streak+=1;prev=(t,xyz,q)
   if streak>=3:confirmed.append(t)
  d[b]={'queries':len(rows),'PnP_passed':sum(r['matched'] for r in rows),'three_frame_confirmed':len(confirmed),'first_temporal_confirmation_s':confirmed[0] if confirmed else None,'last_temporal_confirmation_s':confirmed[-1] if confirmed else None,'motion_discontinuity_rejections':outliers,'mean_candidates_verified':float(np.mean([r['num_verified_candidates'] for r in rows])),'p50_latency_ms':float(np.median([r['total_ms'] for r in rows])),'p95_latency_ms':float(np.percentile([r['total_ms'] for r in rows],95))}
 report[backend]=d
(O/'LEARNED_TEMPORAL_COMPARISON.json').write_text(json.dumps(report,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(10,5))
for backend,path in backends.items():
 data=json.load(open(O/path/'localization_3_matches.json'))
 times=[x['t_sec'] for x in data];success=[x['matched'] for x in data];offset={'xfeat_mnn_orb_retrieval':3,'xfeat_mnn_learned_retrieval':2,'xfeat_lighterglue':1,'superpoint_lightglue':0}[backend]
 ax.scatter(times,[offset+(0.4 if x else 0) for x in success],s=24,label=backend)
ax.set(xlabel='localization_3.bag seconds',ylabel='0: failed  0.4: PnP accepted (offset by backend)',title='Held-out global image-only pose verification by learned feature backend');ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(O/'learned_backend_success_timeline.png',dpi=145);plt.close(fig)
print(json.dumps(report,indent=2)[:6000])
