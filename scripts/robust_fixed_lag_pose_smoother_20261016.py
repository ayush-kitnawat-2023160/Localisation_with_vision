#!/usr/bin/env python3
"""Bounded-memory FIXED-LAG robust smoother on absolute global poses.

Pure mathematical filter: no saved-reference input; no map/local anchor write.
Delays output by lag (~0.18s), fits short local polynomial, uses robust residual
weights, preserves the absolute map frame, and explicitly warns on discontinuity
or publication backlog. The opt-in ROS node is diagnostics only, NOT a vetted
localizer or command-authority source. Never subscribe planner/DBW to output.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np

@dataclass
class FilterResult:
    stamp:float
    xyz:np.ndarray
    quat:np.ndarray
    # "safe" means only numeric motion continuity, NOT independent map trust!
    continuous:bool
    reason:str
    raw_disagreement_m:float
    last_dt_s:float

class FixedLagPoseSmoother:
    def __init__(self,lag=0.18, window=0.85, sigma=0.30, degree=1,
                 residual_m=0.32, max_speed=5.0, max_gap=0.35,
                 max_raw_disagreement=1.0):
        self.lag=float(lag); self.window=float(window)
        self.sigma=float(sigma); self.residual_m=float(residual_m)
        self.degree=int(degree)
        if self.degree not in (1,2):raise ValueError('degree must be 1 or 2')
        self.max_speed=float(max_speed); self.max_gap=float(max_gap)
        self.max_raw_disagreement=float(max_raw_disagreement)
        self.buffer=deque()
        self.last_input_stamp=None
        self.last_output_stamp=None
        self.last_output_xyz=None
        self.last_output_quat=None
        self.faults=0

    def ingest(self,stamp,xyz,quat):
        t=float(stamp);x=np.asarray(xyz,dtype=float).reshape(3)
        q=np.asarray(quat,dtype=float).reshape(4)
        if not np.isfinite(t) or not np.isfinite(x).all() or not np.isfinite(q).all() or np.linalg.norm(q)<0.5:
            self.faults+=1
            return None
        q=q/np.linalg.norm(q)
        if self.last_input_stamp is not None and t <= self.last_input_stamp+1e-5:
            self.faults+=1
            return None
        gap=0.0 if self.last_input_stamp is None else t-self.last_input_stamp
        self.last_input_stamp=t
        self.buffer.append((t,x,q))
        cutoff=t-2.0
        while len(self.buffer)>3 and self.buffer[1][0] < cutoff:
            self.buffer.popleft()
        target=t-self.lag
        if target < self.buffer[0][0]+.02: return None
        if self.last_output_stamp is not None and target<=self.last_output_stamp+.001:return None
        values=[(tt,xx,qq) for tt,xx,qq in self.buffer if abs(tt-target)<=self.window/2]
        # LiDAR-failure and some recovery windows contain only 5Hz poses,
        # unlike healthy 10Hz. Never drop an entire 40-second failure period
        # just because four poses do not fit a short time window.
        if len(values)<3:
            # No trustworthy polynomial fit possible; do not fill the hole
            # by inventing a position. Publish no pose and force watchdog
            # timeout (this sidecar is not a safety-certified producer).
            self.faults+=1
            return None
        tt=np.array([v[0] for v in values])
        xx=np.array([v[1] for v in values])
        qq=np.array([v[2] for v in values])
        offset=tt-target
        # Window is centred at target and weighted in time; robust IRLS rejects
        # a 1-2 frame PCD jump without moving old or future trajectory points.
        tw=np.exp(-0.5*(offset/self.sigma)**2)
        design=np.stack(([np.ones(len(tt)),offset] if self.degree==1 else [np.ones(len(tt)),offset,offset**2]),axis=1)
        weights=tw.copy()
        pred=None
        for iteration in range(4):
            coefs=np.linalg.lstsq(design*np.sqrt(weights)[:,None],
                                  xx*np.sqrt(weights)[:,None],rcond=None)[0]
            modeled=design@coefs
            residual=np.linalg.norm(xx-modeled,axis=1)
            robust=np.minimum(1.,self.residual_m/np.maximum(residual,1.e-6))
            weights=tw*robust
            pred=coefs[0].copy()
        if not np.isfinite(pred).all(): return None
        # Quaternions sign aligned to nearest observed pose. Orientation has
        # same time/robust weights but no arbitrary yaw offset.
        nearest=int(np.argmin(abs(offset)))
        same=qq.copy()
        same[(same@qq[nearest])<0]*=-1
        quat_out=(same*weights[:,None]).sum(axis=0)
        mag=np.linalg.norm(quat_out)
        if mag<0.2 or not np.isfinite(quat_out).all():return None
        quat_out/=mag
        raw=xx[nearest]
        mismatch=float(np.linalg.norm(pred-raw))
        # Detect a physically invalid jump and mark numerical continuity false;
        # crucially do NOT 'cap and accumulate' it as the previous broken
        # causal filter did. Never claim independent map trust from this flag.
        dt=0.0 if self.last_output_stamp is None else target-self.last_output_stamp
        speed=0 if self.last_output_xyz is None else float(np.linalg.norm(pred-self.last_output_xyz)/max(dt,1e-3))
        ok=gap<=self.max_gap and dt<=self.max_gap and mismatch<=self.max_raw_disagreement and speed<=self.max_speed
        reason='NUMERICALLY_CONTINUOUS' if ok else ('RAW_STREAM_GAP' if gap>self.max_gap or dt>self.max_gap else 'RAW_DISAGREEMENT' if mismatch>self.max_raw_disagreement else 'POSE_SPEED_TOO_LARGE')
        self.last_output_stamp=target
        self.last_output_xyz=pred.copy()
        self.last_output_quat=quat_out.copy()
        if not ok:self.faults+=1
        return FilterResult(target,pred,quat_out,ok,reason,mismatch,dt)

def process_rows(rows, smoother=None):
    if smoother is None:smoother=FixedLagPoseSmoother()
    out=[]
    for row in rows:
        stamp,xyz,quat=row
        res=smoother.ingest(stamp,xyz,quat)
        if res is not None:out.append(res)
    return out
