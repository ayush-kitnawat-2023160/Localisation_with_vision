#!/usr/bin/env python3
"""Prepare a reusable, map-only feature-retrieval index for LiDAR outages.

No replay/ground-truth images are used. The saved map's RGB keyframes are
illumination-normalized, indexed with binary AKAZE descriptors and written
to a compact NPZ file with no Python object/pickle payload.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image-dir', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    cv2.setNumThreads(2)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    akaze = cv2.AKAZE_create(threshold=0.00035)
    files = sorted(Path(args.image_dir).glob('*.png'), key=lambda x: float(x.stem))
    if not files:
        raise RuntimeError('No saved image keyframes found')
    stems = []
    offsets = [0]
    desc = []
    dim = None
    for idx, path in enumerate(files):
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        gray = clahe.apply(cv2.resize(gray, (640, 360)))
        _, d = akaze.detectAndCompute(gray, None)
        if d is None:
            d = np.empty((0, 61), dtype=np.uint8)
        if dim is None:
            dim = d.shape[1]
        assert d.dtype == np.uint8 and d.shape[1] == dim
        stems.append(path.stem)
        desc.append(d)
        offsets.append(offsets[-1] + len(d))
        if idx % 100 == 0:
            print('Indexed', idx + 1, '/', len(files), 'features', offsets[-1], flush=True)
    if not desc or offsets[-1] == 0:
        raise RuntimeError('Feature index is empty')
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(str(out),
                        extractor_version=np.array([1], dtype=np.int32),
                        image_stems=np.asarray(stems),
                        offsets=np.asarray(offsets, dtype=np.int64),
                        descriptors=np.concatenate(desc, axis=0))
    print('SAVED', out, 'keyframes', len(stems), 'descriptors', offsets[-1], flush=True)


if __name__ == '__main__':
    main()
