#!/usr/bin/env python3
"""Code-grounded FAST-LIVO2 failsafe architecture handout, with ORIGINAL saved plots."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.colors import HexColor
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.utils import simpleSplit, ImageReader
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
RUNS=ROOT/'results/camera_lidar_imu_gps/failsafe_v2_continuous_map'
OUT=ROOT/'reports/FAST_LIVO2_Failsafe_Architecture_Original_PCD_Plots_A4.pdf'
RUN={
 'Healthy':'FINAL290_HEALTHY_BOUNDEDPLACE_20261006',
 'Camera failed':'FINAL290_CAMERA_FAIL_BOUNDEDPLACE_20261006',
 'LiDAR failed':'FINAL290_LIDAR_FAIL_BOUNDEDPLACE_20261006',
 'IMU failed':'FINAL290_IMU_FAIL_BOUNDEDPLACE_20261006',
}
W,H=A4; LM=35; RM=W-35; CW=RM-LM
INK=HexColor('#17263D'); LIGHT=HexColor('#F5F8FB'); MID=HexColor('#516176'); MUTED=HexColor('#77869A'); STROKE=HexColor('#CBD8E3'); TEAL=HexColor('#087E82'); TINT=HexColor('#E8F5F4'); BLUE=HexColor('#2D5B8B'); RED=HexColor('#B84E58'); RED_BG=HexColor('#FFF3F2'); AMBER=HexColor('#F19C40'); WHITE=HexColor('#FFFFFF')
for name,path in [('Reg','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'),('Bold','/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')]: pdfmetrics.registerFont(TTFont(name,path))
C=canvas.Canvas(str(OUT),pagesize=A4)
C.setTitle('FAST-LIVO2 failsafe | architecture and original saved PCD plots')

def txt(x,y,s,size=8.2,color=INK,font='Reg'):
 C.setFont(font,size);C.setFillColor(color);C.drawString(x,y,str(s))
def rt(x,y,s,size=8.2,color=INK,font='Reg'):
 C.setFont(font,size);C.setFillColor(color);C.drawRightString(x,y,str(s))
def ct(x,y,s,size=8.2,color=INK,font='Reg'):
 C.setFont(font,size);C.setFillColor(color);C.drawCentredString(x,y,str(s))
def lin(x1,y1,x2,y2,color=STROKE,width=.8,dash=None):
 C.setLineWidth(width);C.setStrokeColor(color);C.setDash(dash or []);C.line(x1,y1,x2,y2);C.setDash([])
def box(x,y,w,h,bg=WHITE,outline=STROKE,r=5):
 C.setLineWidth(.8);C.setFillColor(bg);C.setStrokeColor(outline);C.roundRect(x,y,w,h,r,fill=1,stroke=1)
def arrow(x1,y1,x2,y2,col=MID,weight=.9):
 import math
 lin(x1,y1,x2,y2,col,weight)
 a=math.atan2(y2-y1,x2-x1); z=4
 p=C.beginPath();p.moveTo(x2,y2);p.lineTo(x2-z*math.cos(a-.56),y2-z*math.sin(a-.56));p.lineTo(x2-z*math.cos(a+.56),y2-z*math.sin(a+.56));p.close();C.setFillColor(col);C.drawPath(p,stroke=0,fill=1)
def wrap(x,y,text,width,size=8,lead=12,max_lines=10,col=INK,font='Reg'):
 parts=simpleSplit(text,font,size,width)
 if len(parts)>max_lines:print('WARN WRAP TOO LONG',len(parts),text[:100])
 for i,s in enumerate(parts[:max_lines]):txt(x,y-i*lead,s,size,col,font)
 return len(parts)
def page(name,n,sub='camera_lidar_imu_gps.bag | accepted ~290 s runs'):
 txt(LM,H-32,'ALiVE  /  ROBUST LOCALIZATION',7.8,TEAL,'Bold');rt(RM,H-32,'FAST-LIVO2 FAILSAFE  •  09 OCT 2026',7.3,MID)
 lin(LM,H-38,RM,H-38)
 txt(LM,H-72,name,19.6,INK,'Bold');txt(LM,H-91,sub,8.5,MID)
 lin(LM,H-103,RM,H-103,INK,.9)
 lin(LM,35,RM,35);txt(LM,22,'ONE persistent FAST-LIVO2 ESIKF • verified map anchor separate from local state',7,MID);rt(RM,22,f'{n:02d} / 05',7.5,MID,'Bold')
def headline(x,y,index,title,tag=None):
 txt(x,y,index+'  /  '+title,12,INK,'Bold')
 if tag:rt(RM,y+1,tag,7.4,TEAL)
 lin(x,y-9,RM,y-9)
def nbox(x,y,w,h,title,lines=[],tint=False,bad=False):
 box(x,y,w,h,RED_BG if bad else TINT if tint else WHITE,RED if bad else STROKE)
 txt(x+9,y+h-17,title,8.5,RED if bad else TEAL if tint else INK,'Bold')
 for i,l in enumerate(lines):txt(x+9,y+h-33-11*i,l,7.3,INK)
def note(x,y,title,explanation,w=CW):
 txt(x,y,title,8.2,TEAL,'Bold');wrap(x,y-15,explanation,w,8.3,12,4)

# PAGE 1 - single page summary, no redundant bar chart
page('System overview & validated replay',1)
headline(LM,H-130,'01','How local motion becomes global position')
# full pipeline top row
unit=(CW-36)/4; ty=H-212
for i,(name,detail) in enumerate([
 ('Sensors','LiDAR  /  RGB-D  /  IMU'),('Health & timing','timestamps + confidence'),('Single ESIKF','continuous local state'),('Map alignment','verified map-to-local anchor')]):
 x=LM+(unit+12)*i;nbox(x,ty,unit,59,name,[detail],tint=i==2)
 if i<3:arrow(x+unit+2,ty+30,x+unit+10,ty+30,TEAL)
wrap(LM,H-237,'Short-term movement is estimated continuously by the ESIKF. Slower geometric matches against the saved map adjust the map-to-local transform, limiting global drift without resetting the filter.',CW,9,13,3)

headline(LM,H-298,'02','Accepted results | original bag','injected outages: 125–165 s')
headers=[('Mode',111),('Full-run RMSE',107),('Outage RMSE',104),('Outage max',103),('Final diff.',CW-425)]
rows=[('Healthy','1.384 m','—','—','0.462 m'),('Camera failed','1.279 m','0.997 m','2.975 m','1.240 m'),('LiDAR failed','1.631 m','2.194 m','3.859 m','0.484 m'),('IMU failed','2.742 m','1.475 m','4.673 m','0.881 m')]
start=H-322;hrow=39; hh=38;table_h=hh+4*hrow
box(LM,start-table_h,CW,table_h)
C.setFillColor(LIGHT);C.rect(LM+1,start-hh,CW-2,hh-1,fill=1,stroke=0)
xpos=[LM]
for _,w in headers:xpos.append(xpos[-1]+w)
for i,(h,w) in enumerate(headers):
 txt(xpos[i]+8,start-23,h,7.8,MID,'Bold')
 if i:lin(xpos[i],start-table_h,xpos[i],start,STROKE,.55)
for r,row in enumerate(rows):
 y=start-hh-(r+1)*hrow
 if r%2:C.setFillColor(HexColor('#FAFCFE'));C.rect(LM+1,y+1,CW-2,hrow-1,fill=1,stroke=0)
 for i,val in enumerate(row):txt(xpos[i]+9,y+15,val,8.45,INK,'Bold' if i else 'Reg')
 lin(LM,y,RM,y,STROKE,.6)

headline(LM,H-558,'03','What changes when a sensor fails')
items=[
 ('Healthy','IMU propagation + LiDAR scan-to-map; camera pose updates gated by confidence.'),
 ('Camera off','Continue LiDAR / IMU odometry; disable visual measurement path.'),
 ('LiDAR off','Use IMU attitude + metric RGB-D motion; saved visual/depth map tries to fix global pose.'),
 ('IMU off','Use RGB-D metric relative pose; LiDAR scan-to-scan relative ICP as fallback.')]
y=H-589
for name,s in items:
 txt(LM+5,y,name,8.7,TEAL,'Bold');wrap(LM+98,y,s,CW-107,8.2,11,2);y-=38
box(LM,57,CW,63,LIGHT)
txt(LM+13,108,'EVALUATION NOTE',8,TEAL,'Bold')
wrap(LM+13,98,'Errors are differences against the original-bag trajectory reference, not independent surveyed ground truth. Later October 8 experiments did not replace the full, accepted four-mode 290 s set; therefore all results and plots below use the matched October 6 runs.',CW-28,7.9,11,4)
C.showPage()

# PAGE 2 - healthy and camera-failure details
page('Healthy operation & camera dropout',2,'Detailed measurement paths, update checks, and saved-map fusion')

def dual_mode(top,title,bad=False):
 headline(LM,top,'01' if not bad else '02',title,'FULL_LIVO' if not bad else 'LIO_CAMERA_FAILED')
 y=top-54; xx=LM
 nbox(xx,y,164,35,'IMU  →  prediction',[],tint=True)
 nbox(xx+179,y,164,35,'LiDAR  →  geometric residual',[],tint=True)
 nbox(xx+358,y,CW-358,35,'Camera  →  confidence gate',[],bad=bad)
 # arrows merge to local
 for cx in [xx+82,xx+261,xx+438]:arrow(cx,y,cx,y-17,RED if bad and cx==xx+438 else MID)
 nbox(LM,y-87,253,67,'Continuous ESIKF state',[
     '1. Predict pose/velocity with IMU',
     '2. De-skew LiDAR; scan-to-voxel map update',
     '3. '+'Reject camera update (failed)' if bad else '3. Accept camera update only if confident',
 ],tint=True)
 nbox(LM+272,y-87,CW-272,67,'Saved LiDAR PCD tracking',[
     '1. Query nearby / descriptor candidate',
     '2. Geometric scan-to-map GICP verifies pose',
     '3. Reject stale, low-fitness or implausible fix',
 ])
 arrow(LM+126,y-104,LM+126,y-123,TEAL)
 arrow(LM+390,y-104,LM+390,y-123,TEAL)
 nbox(LM+67,y-175,CW-134,49,'Global pose output',[
    'P(map) = T(map ← local) × P(ESIKF local)',
    'Slew-limit accepted XY/yaw map-anchor corrections; never reset ESIKF',
 ],tint=True)
 return y-205

b=dual_mode(H-128,'Healthy  |  LiDAR + RGB-D + IMU')
note(LM,b-3,'Interpretation','The LiDAR and IMU form a stable motion backbone. Healthy images may enrich the visual map, but uncertain photometric pose updates are gated so they cannot knock the estimator off course.')
b2=dual_mode(H-471,'Camera failure  |  LiDAR + IMU',bad=True)
note(LM,b2-4,'Interpretation','Camera-frame timeouts switch to LiDAR-inertial operation. Prediction, scan-to-map optimization, saved-PCD registration and smooth global output all continue in the same estimator.')
C.showPage()

# PAGE 3 - LiDAR loss more detailed
page('LiDAR failure | RGB-D + IMU',3,'VI_LIDAR_FAILED / IMU_ONLY_BRIDGE: detailed recovery and confidence gates')
headline(LM,H-128,'03','Motion when LiDAR scans disappear')
nbox(LM,H-225,165,70,'Sensor health',["LiDAR timeout → failed",'RGB-D fresh? confidence gate','Healthy IMU continues'],tint=False)
nbox(LM+180,H-225,164,70,'IMU propagation',['Gyro → angular update','Accel → velocity/position','Carry continuous ESIKF state'],tint=True)
nbox(LM+359,H-225,CW-359,70,'LiDAR map update OFF',['No fresh scan residuals','Do not reuse stale map','Keep predicted state'],bad=True)
arrow(LM+82,H-225,LM+82,H-242);arrow(LM+262,H-225,LM+262,H-242)
headline(LM,H-272,'04','RGB-D relative-motion frontend')
nbox(LM,H-369,164,71,'Synchronized RGB + depth',['Undistort / rectify frames','Track ORB visual features','Associate valid depth'])
nbox(LM+179,H-369,166,71,'Metric motion estimate',['3D from depth and intrinsics','Feature motion / PnP','Robust relative SE(3) step'],tint=True)
nbox(LM+360,H-369,CW-360,71,'ESIKF measurement gate',['Reject stale / implausible delta','IMU still controls attitude','Apply bounded translation'])
arrow(LM+164,H-333,LM+177,H-333,TEAL);arrow(LM+345,H-333,LM+358,H-333,TEAL)
headline(LM,H-405,'05','Global place retrieval and saved visual/depth map')
nbox(LM,H-503,165,72,'Place hypotheses',['Query stored keyframes','RGB appearance / descriptors','Candidate global poses'])
nbox(LM+180,H-503,164,72,'Depth geometry verification',['Live RGB-D → 3D cloud','Register to saved voxel map','Check GICP fitness / overlap'],tint=True)
nbox(LM+359,H-503,CW-359,72,'Safety and continuity gate',['Reject repeated-road aliases','Limit speed / heading jump','Only verified fixes accepted'])
arrow(LM+165,H-467,LM+178,H-467,TEAL);arrow(LM+344,H-467,LM+357,H-467,TEAL)
headline(LM,H-546,'06','Single-state and map-frame correction')
nbox(LM,H-655,CW,83,'Motion stays in ESIKF; absolute correction stays in map frame',[
 'RGB-D relative increments advance the *same* ESIKF (not a second SLAM trajectory).',
 'Verified depth-map fix adjusts smoothed T(map ← local); output = T × P(ESIKF).',
 'If no unique map fix exists, keep continuous odometry but confidence / drift can degrade.',
],tint=True)
wrap(LM,H-682,'Practical limitation: visually similar roads can cause false place recognition. The four-mode October 6 replay is retained, but the LiDAR-failure branch still needs robust cross-date validation.',CW,8.1,12,3)
C.showPage()

# PAGE 4 - IMU loss detailed
page('IMU failure | RGB-D + LiDAR',4,'LV_IMU_FAILED: relative-motion source selection without IMU propagation')
headline(LM,H-128,'07','Motion after IMU messages stop')
nbox(LM,H-228,164,75,'IMU failure detection',['Timestamp timeout; confidence 0','Stop trusting inertial prediction','Preserve last good tilt'],bad=True)
nbox(LM+180,H-228,166,75,'RGB-D primary motion',['Track RGB features + depth','Solve metric translation + yaw','Relative planar SE(2) increment'],tint=True)
nbox(LM+361,H-228,CW-361,75,'LiDAR relative fallback',['Scan-to-scan ICP displacement','Use if RGB-D age > 0.75 s','Geometry is backup, not reset'])
arrow(LM+263,H-228,LM+263,H-259,TEAL);arrow(LM+444,H-228,LM+444,H-259,TEAL)
headline(LM,H-285,'08','Update the original ESIKF state directly')
nbox(LM,H-385,252,70,'Same ESIKF state (no replacement)',[
 'Position += previous rotation × metric Δp',
 'Yaw += measured Δyaw; keep last valid roll/pitch',
 'Velocity from accepted Δp / Δt',
],tint=True)
nbox(LM+268,H-385,CW-268,70,'No-IMU propagation / constraints',[
 'No fresh IMU acceleration / gyro input',
 'Ground vehicle body-Z constrained',
 'Velocity / yaw rates bounded',
])
arrow(LM+126,H-385,LM+126,H-407,TEAL);arrow(LM+385,H-385,LM+385,H-407,TEAL)
headline(LM,H-433,'09','Global position correction and recovery')
nbox(LM,H-532,166,72,'Saved LiDAR geometry',['Use existing PCD keyframes','Generate map pose candidate','Match with scan GICP'])
nbox(LM+181,H-532,163,72,'Verification gate',['Fitness, overlap, continuity','Reject large / inconsistent jumps','Maintain map-frame pose'],tint=True)
nbox(LM+359,H-532,CW-359,72,'Smooth map anchor',['Bound XY, yaw correction rate','Output /pose, /odom, TF','Returning IMU rejoin warmup'])
arrow(LM+166,H-495,LM+179,H-495,TEAL);arrow(LM+344,H-495,LM+357,H-495,TEAL)
box(LM,H-659,CW,80,LIGHT)
txt(LM+13,H-600,'Why the no-IMU branch is not a second estimator',9,TEAL,'Bold')
wrap(LM+13,H-618,'Both RGB-D and LiDAR frontends only provide relative-motion measurements. Their filtered increments update the one surviving FAST-LIVO2 ESIKF state. The saved map supplies an absolute global frame, not an independent parallel trajectory.',CW-26,8.25,12,5)
wrap(LM,H-689,'The original-bag 40 s IMU outage has a 1.475 m reference-relative RMSE; the whole run is 2.742 m because a large earlier disagreement inflates the full-run score.',CW,8,12,3)
C.showPage()

# PAGE 5 - EXACT PNG files, no replotting, no redrawing
page('Original saved PCD trajectory PNGs',5,'Embedded directly from plots/07_orange_trajectory_on_pcd_clean.png for the accepted four runs')
positions=[(LM,410),(LM+CW/2+9,410),(LM,75),(LM+CW/2+9,75)]
panW=CW/2-9; panH=298
for (name,sub),(x,y) in zip(RUN.items(),positions):
 txt(x,y+panH+17,name,9.4,INK,'Bold')
 plot=RUNS/sub/'plots/07_orange_trajectory_on_pcd_clean.png'
 if not plot.exists():raise RuntimeError('MISSING SAVED PNG: '+str(plot))
 im=Image.open(plot)
 scale=min(panW/im.width,panH/im.height)
 drawW=im.width*scale;drawH=im.height*scale
 C.drawImage(ImageReader(im),x+(panW-drawW)/2,y+(panH-drawH)/2,drawW,drawH,mask='auto',preserveAspectRatio=True)
C.showPage(); C.save();print('CREATED',OUT,'BYTES',OUT.stat().st_size)
