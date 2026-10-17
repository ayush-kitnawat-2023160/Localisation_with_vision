#!/usr/bin/env python3
"""Compare healthy PCD match coverage at a named map segment (proxy, not GT)."""
import argparse,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def read(f):
    rows=list(csv.DictReader(open(f)))
    if not rows:raise ValueError(f'no poses {f}')
    t=np.array([int(r['%time'])*1e-9 for r in rows]);
    xy=np.array([[float(r['field.pose.position.'+a]) for a in ('x','y')] for r in rows]);return t,xy

def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',required=True);p.add_argument('--trial',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();t0=1788852570.782609  # first timestamp of the original same-bag reference
    fig,ax=plt.subplots(1,2,figsize=(14,5.3));ax[0].set_title('Top-right loop: saved PCD matches');ax[1].set_title('Accepted GICP fixes vs time')
    summary=[]
    for name,root,c in [('Original Oct 13',Path(a.baseline),'tab:orange'),('Continuity trial',Path(a.trial),'tab:blue')]:
        tt,xy=read(root/'global_pose.csv');tg,gxy=read(root/'gicp_map_pose.csv')
        segment=(tt-t0>=39)&(tt-t0<=63)
        gt=(tg-t0>=39)&(tg-t0<=63)
        ax[0].plot(xy[segment,0],xy[segment,1],linewidth=2,color=c,label=f'{name} published')
        ax[0].scatter(gxy[gt,0],gxy[gt,1],s=32,marker='x',color=c,label=f'{name} GICP',zorder=3)
        ax[1].scatter(tg[gt]-t0,[1 if name=='Continuity trial' else 0]*gt.sum(),marker='|',s=200,color=c,label=name)
        pairs=sorted(zip(tg[gt],gxy[gt,0],gxy[gt,1]));
        gaps=[(round(pairs[i][0]-t0,1),round(pairs[i+1][0]-t0,1),round(pairs[i+1][0]-pairs[i][0],2)) for i in range(len(pairs)-1)]
        biggest=max(gaps,key=lambda x:x[2]) if gaps else None
        summary.append(f'{name}: {gt.sum()} accepted map fixes within 39–63s, max interval={biggest}')
    ax[0].set(xlabel='Map X (m)',ylabel='Map Y (m)');ax[0].axis('equal');ax[0].grid(alpha=.2);ax[0].legend(fontsize=8,loc='best')
    ax[1].set(xlabel='Time from development bag start (s)',yticks=[0,1],yticklabels=['Baseline','New'],xlim=(39,63),ylim=(-.4,1.4))
    ax[1].grid(alpha=.25);ax[1].axvspan(44,55,color='gray',alpha=.11)
    fig.suptitle('Development bag PCD correspondence continuity (not independent GT)')
    fig.tight_layout()
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(out,dpi=170);plt.close(fig)
    (out.with_suffix('.txt')).write_text('\n'.join(summary)+'\n')
    print('\n'.join(summary));print('wrote',out)
if __name__=='__main__':main()
