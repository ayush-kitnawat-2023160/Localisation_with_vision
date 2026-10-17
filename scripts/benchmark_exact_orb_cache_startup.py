#!/usr/bin/env python3
"""Exercise the REAL VisualPlaceHint._build_db with/without exact ORB cache.

For an offline test, avoid ROS node initialization and supply identical
identity trajectory interpolation. Map descriptors/ordering are compared
byte-for-byte; metric pose pipeline is not modified.
"""
import argparse
import importlib.util
from pathlib import Path
import time
import cv2
import numpy as np

cv2.setNumThreads(1)
p=argparse.ArgumentParser()
p.add_argument('--images',required=True,type=Path)
p.add_argument('--cache',required=True,type=Path)
p.add_argument('--source',required=True,type=Path)
args=p.parse_args()
spec=importlib.util.spec_from_file_location('visual_place_hint',args.source)
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def run(cache):
    obj=mod.VisualPlaceHint.__new__(mod.VisualPlaceHint)
    obj.image_dir=args.images
    obj.orb_retrieval_cache_path=str(args.cache) if cache else ''
    obj.orb_ret=cv2.ORB_create(nfeatures=700,fastThreshold=12)
    obj._interp_body=lambda ts: np.eye(4,dtype=np.float64)
    start=time.perf_counter()
    db=obj._build_db()
    return db,time.perf_counter()-start

uncached,t_uncached=run(False)
cached,t_cached=run(True)
assert len(cached)==len(uncached), (len(cached),len(uncached))
for a,b in zip(uncached,cached):
    assert a[0]==b[0] and a[1]==b[1]
    assert np.array_equal(a[2],b[2])
    assert np.array_equal(a[3],b[3])
print(f'PASS: identical descriptors/ordering for {len(cached)} map frames',flush=True)
print(f'Startup ORB database build uncached={t_uncached:.3f}s cached={t_cached:.3f}s speedup={t_uncached/max(t_cached,1e-9):.2f}x',flush=True)
