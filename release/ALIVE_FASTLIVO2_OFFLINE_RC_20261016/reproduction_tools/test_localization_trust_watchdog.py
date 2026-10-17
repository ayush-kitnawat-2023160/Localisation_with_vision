#!/usr/bin/env python3
"""Run pure no-ROS safety-state tests of the fail-closed watchdog."""
import math
from localization_trust_watchdog import TrustMonitor

def step(m, i, trust=True):
    now=10.0+0.1*i
    m.trust(trust,now)
    m.pose(1789735945.0+0.1*i,(0.1*i,0.,0.),now)
    return m.verdict(now)

m=TrustMonitor()
assert not m.verdict(0)[0]
for i in range(4):assert not step(m,i)[0]
assert step(m,4)[0], 'Must arm after five physically plausible verified frames'
assert not m.verdict(11)[0], 'Must trip after loss of heartbeat even when last state was TRUE'
assert not step(m,11)[0], 'Must trip after a pose timestamp gap over 0.6s'
for i in range(12,17):step(m,i)
assert m.verdict(11.6)[0]
m.trust(False,11.7)
assert not m.verdict(11.7)[0], 'Untrusted map must immediately trip stop'
for i in range(18,24):step(m,i)
assert m.verdict(12.3)[0]
m.trust(True,12.4)
m.pose(1789735945.0+2.4,(1000.,0.,0.),12.4)
assert not m.verdict(12.4)[0], 'Large map jump must trip stop'
for i in range(25,40):
    m.trust(True,10+0.1*i)
    m.pose(1789735945.+0.1*i,(1000.+0.1*(i-24),0.,0.),10+0.1*i)
assert not m.verdict(13.9)[0], 'Huge jump must never auto-rearm on an unverified wrong branch'

# Sustained false trust cannot silently accumulate healthy samples.
n=TrustMonitor()
for i in range(10):step(n,i,trust=False)
assert not n.verdict(10.9)[0]
assert n.healthy_streak==0
for i in range(10,14):assert not step(n,i)[0]
assert step(n,14)[0], 'Five fresh verified samples required after loss of trust'

# Yaw-only teleportation is dangerous even when XY coordinates are smooth.
q=TrustMonitor()
for i in range(5):
    now=20.0+0.1*i
    q.trust(True,now)
    q.pose(1789735945.+i*.1,(i*.1,0.,0.),now,yaw=0.)
assert q.verdict(20.4)[0]
q.trust(True,20.5)
q.pose(1789735945.5,(0.5,0.,0.),20.5,yaw=math.pi)
assert not q.verdict(20.5)[0], 'Yaw jump must trigger hard latch'

# Smoothing does not prove absolute map accuracy. Independent opt-in signal
# must latch stop despite healthy-looking speed and later false messages.
r=TrustMonitor()
for i in range(5):step(r,i)
assert r.verdict(10.4)[0]
r.outage_smoothing_unsafe(True)
assert not r.verdict(10.4)[0]
r.outage_smoothing_unsafe(False)
for i in range(5,16):step(r,i)
assert not r.verdict(11.5)[0], 'Smoother unsafe flag must hard-latch stop'
print('PASS: fail-closed startup, arm, trust-loss, heartbeat timeout, pose gap, jump, and rearm')
