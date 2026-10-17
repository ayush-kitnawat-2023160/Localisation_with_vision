#!/usr/bin/env python3
import argparse, bisect, csv, math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


def load(path):
    out=[]
    with open(path,newline='') as f:
        for r in csv.DictReader(f):
            try:
                t=float(r['timestamp']); x=float(r['x']); y=float(r['y']); z=float(r.get('z',0.0))
            except Exception:
                continue
            if all(math.isfinite(v) for v in (t,x,y,z)):
                out.append((t,x,y,z))
    out.sort(key=lambda v:v[0])
    return out


def clip45(points):
    if not points: return []
    t0=points[0][0]
    return [p for p in points if 0.0 <= p[0]-t0 <= 45.0]


def nice_step(span,target=8):
    raw=max(span/target,1e-9); p=10**math.floor(math.log10(raw)); q=raw/p
    return (1 if q<=1 else 2 if q<=2 else 5 if q<=5 else 10)*p


def font(sz,bold=False):
    p='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf' if bold else '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    return ImageFont.truetype(p,sz)

F14,F16,F18,F22,F30=font(14),font(16),font(18),font(22,True),font(30,True)


def xy_plot(path, series, title, subtitle, highlight_failure=False):
    # series: [(label, points, rgb, dashed)]
    W,H=1500,1100; left,top,plot=120,145,820; right,bottom=left+plot,top+plot
    data=[(n,clip45(p),c,dash) for n,p,c,dash in series]
    allpts=[p for _,pts,_,_ in data for p in pts]
    if not allpts: return
    xs=[p[1] for p in allpts]; ys=[p[2] for p in allpts]
    xmin,xmax,ymin,ymax=min(xs),max(xs),min(ys),max(ys)
    span=max(xmax-xmin,ymax-ymin,1.0)*1.10
    cx,cy=(xmin+xmax)/2,(ymin+ymax)/2
    xmin,xmax=cx-span/2,cx+span/2; ymin,ymax=cy-span/2,cy+span/2
    tx=lambda x:left+(x-xmin)/span*plot
    ty=lambda y:bottom-(y-ymin)/span*plot
    img=Image.new('RGB',(W,H),'white'); d=ImageDraw.Draw(img)
    d.text((left,50),title,font=F30,fill=(17,24,39)); d.text((left,92),subtitle,font=F18,fill=(75,85,99))
    step=nice_step(span); x=math.ceil(xmin/step)*step
    while x<=xmax+1e-9:
        px=tx(x); d.line((px,top,px,bottom),fill=(229,231,235),width=1)
        lab=f'{x:g}'; bb=d.textbbox((0,0),lab,font=F14); d.text((px-(bb[2]-bb[0])/2,bottom+14),lab,font=F14,fill=(107,114,128)); x+=step
    y=math.ceil(ymin/step)*step
    while y<=ymax+1e-9:
        py=ty(y); d.line((left,py,right,py),fill=(229,231,235),width=1)
        lab=f'{y:g}'; bb=d.textbbox((0,0),lab,font=F14); d.text((left-15-(bb[2]-bb[0]),py-8),lab,font=F14,fill=(107,114,128)); y+=step
    d.rectangle((left,top,right,bottom),outline=(17,24,39),width=2)
    lab='X (m)'; bb=d.textbbox((0,0),lab,font=F18); d.text((left+plot/2-(bb[2]-bb[0])/2,bottom+55),lab,font=F18,fill=(17,24,39))
    yl=Image.new('RGBA',(200,50),(255,255,255,0)); yd=ImageDraw.Draw(yl); yd.text((0,10),'Y (m)',font=F18,fill=(17,24,39)); yl=yl.rotate(90,expand=1); img.paste(yl,(20,int(top+plot/2-yl.height/2)),yl)
    for label,pts,color,dashed in data:
        xy=[(tx(p[1]),ty(p[2])) for p in pts]
        if dashed:
            for i in range(len(xy)-1):
                if (i//5)%2==0: d.line((xy[i],xy[i+1]),fill=color,width=3)
        elif len(xy)>1:
            d.line(xy,fill=color,width=4,joint='curve')
        if xy:
            x0,y0=xy[0]; xe,ye=xy[-1]
            d.ellipse((x0-5,y0-5,x0+5,y0+5),fill='white',outline=color,width=3)
            d.line((xe-6,ye-6,xe+6,ye+6),fill=color,width=3); d.line((xe-6,ye+6,xe+6,ye-6),fill=color,width=3)
    if highlight_failure and len(data)>=1:
        label,pts,color,_=data[-1]
        if pts:
            t0=pts[0][0]
            seg=[p for p in pts if 15.0 <= p[0]-t0 <= 35.0]
            xy=[(tx(p[1]),ty(p[2])) for p in seg]
            if len(xy)>1: d.line(xy,fill=color,width=9,joint='curve')
    lx,ly=1000,165; d.text((lx,ly-40),'Trajectories',font=F22,fill=(17,24,39))
    for i,(label,_,color,_) in enumerate(data):
        yy=ly+i*52; d.line((lx,yy,lx+55,yy),fill=color,width=5); d.text((lx+72,yy-10),label,font=F16,fill=(31,41,55))
    if highlight_failure:
        yy=ly+len(data)*52+18; d.line((lx,yy,lx+55,yy),fill=data[-1][2],width=10); d.text((lx+72,yy-10),'15–35 s failure interval',font=F16,fill=(31,41,55))
    d.rounded_rectangle((995,850,1370,955),radius=12,fill=(243,244,246)); d.text((1015,875),'Equal metric XY scale',font=F18,fill=(17,24,39)); d.text((1015,912),'1 m X = 1 m Y — no stretching',font=F16,fill=(55,65,81))
    img.save(path)


def interp_ref(ref,times,t):
    if not ref or t<times[0] or t>times[-1]: return None
    j=bisect.bisect_right(times,t)
    if j<=0:return ref[0][1:4]
    if j>=len(ref):return ref[-1][1:4]
    a,b=ref[j-1],ref[j]; u=0.0 if b[0]<=a[0] else (t-a[0])/(b[0]-a[0])
    return tuple(a[k]+u*(b[k]-a[k]) for k in (1,2,3))


def error_plot(path, run, ref, sensor):
    if not run or not ref:return
    rt=[r[0] for r in ref]; t0=run[0][0]; vals=[]
    for r in run:
        tr=r[0]-t0
        if not 0<=tr<=45: continue
        q=interp_ref(ref,rt,r[0])
        if q is None:continue
        e=math.sqrt((r[1]-q[0])**2+(r[2]-q[1])**2+(r[3]-q[2])**2); vals.append((tr,e))
    if not vals:return
    W,H=1500,900; L,T,R,B=100,120,1110,750; pw,ph=R-L,B-T
    img=Image.new('RGB',(W,H),'white'); d=ImageDraw.Draw(img)
    ymax=max(5,math.ceil(max(e for _,e in vals)/5)*5); tx=lambda t:L+t/45*pw; ty=lambda e:B-e/ymax*ph
    d.text((L,50),'3D error vs loop-closed baseline',font=F30,fill=(17,24,39)); d.text((L,92),'Per-run map-frame position error, 0–45 s.',font=F18,fill=(75,85,99))
    if sensor!='none': d.rectangle((tx(15),T,tx(35),B),fill=(243,244,246))
    for t in range(0,46,5):
        x=tx(t); d.line((x,T,x,B),fill=(229,231,235)); d.text((x-8,B+15),str(t),font=F14,fill=(107,114,128))
    ystep=5 if ymax>=15 else 1
    for e in range(0,int(ymax)+1,ystep):
        y=ty(e); d.line((L,y,R,y),fill=(229,231,235)); d.text((L-45,y-8),str(e),font=F14,fill=(107,114,128))
    d.rectangle((L,T,R,B),outline=(17,24,39),width=2)
    pts=[(tx(t),ty(e)) for t,e in vals]; d.line(pts,fill=(37,99,235),width=3,joint='curve')
    d.text((L+pw/2-70,B+60),'Elapsed time (s)',font=F18,fill=(17,24,39))
    img.save(path)


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('run_dir'); ap.add_argument('--reference',required=True); ap.add_argument('--sensor',default='none'); args=ap.parse_args()
    run_dir=Path(args.run_dir); out=run_dir/'plots'; out.mkdir(parents=True,exist_ok=True)
    raw=load(run_dir/'trajectory.csv') if (run_dir/'trajectory.csv').exists() else []
    mapped=load(run_dir/'map_trajectory.csv') if (run_dir/'map_trajectory.csv').exists() else []
    ref=load(args.reference)
    run_name=run_dir.name; sensor=args.sensor
    if raw:
        xy_plot(out/'01_raw_fastlivo2_xy_equal.png',[('FAST-LIVO2 local ESIKF',raw,(37,99,235),False)],f'{run_name} — raw FAST-LIVO2 trajectory','Persistent ESIKF/local-frame trajectory, 0–45 s.')
    if mapped and ref:
        xy_plot(out/'02_map_vs_loopclosed_xy_equal.png',[('Loop-closed baseline',ref,(31,41,55),True),(run_name,mapped,(37,99,235),False)],f'{run_name} vs loop-closed baseline','Global map-frame trajectory, 0–45 s.',highlight_failure=(sensor!='none'))
        error_plot(out/'03_error_vs_loopclosed.png',mapped,ref,sensor)
    info=out/'PLOT_INFO.txt'; info.write_text(f'run={run_name}\nsensor={sensor}\nreference={args.reference}\nraw={run_dir / "trajectory.csv"}\nmap={run_dir / "map_trajectory.csv"}\n')
    print(out)
    for p in sorted(out.glob('*')): print(p.name)

if __name__=='__main__': main()
