#!/usr/bin/env python3
"""Deterministic source-time sample-selection unit tests; no ROS/hardware."""
import math

def schedule(frames,callback_jitter,hz):
 last=None;selected=[]
 for index,stamp in enumerate(frames):
  source=float(stamp)
  if callback_jitter is None:
   raise AssertionError('Must provide arrival offsets')
  # Synthetic wall schedule deliberately varies from one replay to another.
  _arrival=source+float(callback_jitter[index])
  # Mirrors opt-in C++ depth_source_time_gate_enable gate.
  if last is not None and (source-last)<=0:
   continue
  if last is not None and source-last < 1./hz:
   continue
  last=source
  selected.append(index)
 return selected

def main():
 frames=[i*.1 for i in range(120)]
 jitter_1=[.003+(i%7)*.021 for i in range(len(frames))]
 jitter_2=[.011+(i%3)*.08 for i in range(len(frames))]
 one=schedule(frames,jitter_1,1.0)
 two=schedule(frames,jitter_2,1.0)
 assert one==two, (one,two)
 # Return out-of-order points: the gate must not let old delayed depth
 # undo the source-clock state.
 scrambled=[0.,1.,2.,1.8,2.1,3.1,3.0,4.1]
 assert schedule(scrambled,[.01]*len(scrambled),1.)==[0,1,2,5],schedule(scrambled,[.01]*len(scrambled),1.)
 print('PASS source-time gate yields stable deterministic selected frame indices')
 print('PASS late out-of-order sensor stamps are rejected, not reapplied')
 print('SELECTED_PER_120_DEPTH_FRAMES:',len(one))
if __name__=='__main__':main()
