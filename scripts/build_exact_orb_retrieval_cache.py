#!/usr/bin/env python3
"""Build a lossless ORB retrieval cache using the SAME OpenCV as ROS runtime.

Run inside the frozen healthy Docker image, with the static map images mounted
read-only. No runtime query images, poses or trajectory data go in this cache.
The loader refuses the cache if images, ORB config or OpenCV version differ.
"""
import argparse
from pathlib import Path
import cv2
import numpy as np
cv2.setNumThreads(1)

cli=argparse.ArgumentParser()
cli.add_argument('--images',type=Path,required=True)
cli.add_argument('--output',type=Path,required=True)
cli.add_argument('--verify',action='store_true')
args=cli.parse_args()
files=[]
for path in args.images.glob('*.png'):
    try: files.append((float(path.stem),path))
    except ValueError: continue
files.sort()
assert files,'No numeric-timestamp PNG map images'
orb=cv2.ORB_create(nfeatures=700,fastThreshold=12)

if args.verify:
    with np.load(args.output,allow_pickle=False) as z:
        offsets=z['offsets']; descriptors=z['descriptors']
        assert z['opencv_version'].item()==cv2.__version__
        assert np.array_equal(z['orb_config'],[700,12,640,360])
        assert np.array_equal(z['image_stems'],[p.stem for _,p in files])
        assert np.array_equal(z['image_sizes'],[p.stat().st_size for _,p in files])
        assert np.array_equal(z['image_mtimes_ns'],[p.stat().st_mtime_ns for _,p in files])
        assert len(offsets)==len(files)+1 and offsets[-1]==len(descriptors)
        samples=np.unique(np.linspace(0,len(files)-1,min(25,len(files)),dtype=int))
        for i in samples:
            im=cv2.imread(str(files[i][1]),cv2.IMREAD_GRAYSCALE)
            _,d=orb.detectAndCompute(cv2.resize(im,(640,360)),None)
            expected=np.empty((0,32),dtype=np.uint8) if d is None else d
            assert np.array_equal(expected,descriptors[offsets[i]:offsets[i+1]]),i
    print(f'PASS: cache metadata and {len(samples)} exact descriptor samples; OpenCV {cv2.__version__}',flush=True)
else:
    desc=[]; offsets=[0]
    for i,(_ts,p) in enumerate(files):
        im=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
        if im is not None:
            _,d=orb.detectAndCompute(cv2.resize(im,(640,360)),None)
        else:d=None
        if d is not None:desc.append(d)
        offsets.append(offsets[-1]+(0 if d is None else len(d)))
    total=np.concatenate(desc,axis=0) if desc else np.empty((0,32),dtype=np.uint8)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    np.savez(str(args.output),image_stems=np.asarray([p.stem for _,p in files]),
             image_sizes=np.asarray([p.stat().st_size for _,p in files],dtype=np.int64),
             image_mtimes_ns=np.asarray([p.stat().st_mtime_ns for _,p in files],dtype=np.int64),
             offsets=np.asarray(offsets,dtype=np.int64),descriptors=total,
             orb_config=np.asarray([700,12,640,360],dtype=np.int64),
             opencv_version=np.asarray(cv2.__version__))
    args.output.chmod(0o644)
    print(f'Wrote {len(files)} image descriptors ({len(total)} ORB vectors) to {args.output}',flush=True)
