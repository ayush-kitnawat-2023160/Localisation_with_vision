#!/usr/bin/env python3
"""Score one map_trajectory.csv against the loop-closed reference.

The benchmark uses absolute rosbag timestamps, so reference poses are linearly
interpolated at each recorded map-pose timestamp.  In addition to position
error, report consecutive-pose jump/speed statistics so a low RMSE cannot hide
the visible saw-tooth artefacts that matter for Lanelet/planning.
"""

import argparse
import bisect
import csv
import math
from pathlib import Path


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                v = tuple(float(r[k]) for k in ("timestamp", "x", "y", "z"))
            except (KeyError, TypeError, ValueError):
                continue
            if all(math.isfinite(x) for x in v):
                rows.append(v)
    rows.sort(key=lambda x: x[0])
    return rows


def interp(ref, times, t):
    if not ref or t < times[0] or t > times[-1]:
        return None
    j = bisect.bisect_right(times, t)
    if j == 0:
        return ref[0][1:4]
    if j >= len(ref):
        return ref[-1][1:4]
    a, b = ref[j - 1], ref[j]
    if b[0] <= a[0]:
        return a[1:4]
    u = (t - a[0]) / (b[0] - a[0])
    return tuple(a[k] + u * (b[k] - a[k]) for k in (1, 2, 3))


def percentile(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    x = (len(s) - 1) * p
    lo, hi = int(math.floor(x)), int(math.ceil(x))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi - x) + s[hi] * (x - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_csv")
    ap.add_argument("--reference", required=True)
    ap.add_argument("--duration", type=float, default=45.0)
    args = ap.parse_args()

    run = load(args.run_csv)
    ref = load(args.reference)
    if not run or not ref:
        raise SystemExit("empty trajectory/reference")
    ref_times = [r[0] for r in ref]
    t0 = run[0][0]
    run = [r for r in run if 0.0 <= r[0] - t0 <= args.duration]

    samples = []
    for r in run:
        q = interp(ref, ref_times, r[0])
        if q is None:
            continue
        e3 = math.sqrt(sum((r[k] - q[k - 1]) ** 2 for k in (1, 2, 3)))
        e2 = math.hypot(r[1] - q[0], r[2] - q[1])
        samples.append((r[0] - t0, e2, e3))

    def segment(a, b):
        vals = [x for x in samples if a <= x[0] <= b]
        if not vals:
            return "no samples"
        e2 = [x[1] for x in vals]
        e3 = [x[2] for x in vals]
        return (f"n={len(vals)}  2D_RMSE={math.sqrt(sum(x*x for x in e2)/len(e2)):.3f}m "
                f"3D_RMSE={math.sqrt(sum(x*x for x in e3)/len(e3)):.3f}m "
                f"MAE3D={sum(e3)/len(e3):.3f}m MAX3D={max(e3):.3f}m "
                f"END3D={e3[-1]:.3f}m")

    jumps, speeds = [], []
    for a, b in zip(run, run[1:]):
        dt = b[0] - a[0]
        if dt <= 0.0 or dt > 0.5:
            continue
        dxy = math.hypot(b[1] - a[1], b[2] - a[2])
        jumps.append(dxy)
        speeds.append(dxy / dt)

    print(f"run={Path(args.run_csv).name} n={len(run)} t0={t0:.6f}")
    for a, b in ((0, 15), (15, 35), (35, args.duration), (0, args.duration)):
        print(f"{a:>4.0f}-{b:<4.0f}s  {segment(a, b)}")
    if jumps:
        print("smoothness  "
              f"step_max={max(jumps):.3f}m step_p95={percentile(jumps,0.95):.3f}m "
              f"speed_max={max(speeds):.3f}m/s speed_p95={percentile(speeds,0.95):.3f}m/s")


if __name__ == "__main__":
    main()
