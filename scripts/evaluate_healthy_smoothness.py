#!/usr/bin/env python3
"""Evaluate ONLINE healthy map publication; the loop-closed bag track is a proxy, not GT.

Never use the reference to generate or correct runtime poses. Reports explicit
velocity/acceleration so hiding PCD jumps behind a low pose-step number fails.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_pose(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty trajectory {path}")
    if "%time" in rows[0]:
        t = np.array([int(r["field.header.stamp"]) * 1e-9 for r in rows])
        p = np.array([[float(r["field.pose.position." + c]) for c in "xyz"] for r in rows])
    else:
        t = np.array([float(r["timestamp"]) for r in rows])
        p = np.array([[float(r[c]) for c in "xyz"] for r in rows])
    if not np.isfinite(p).all() or np.any(np.diff(t) <= 0):
        raise ValueError(f"out-of-order or invalid poses in {path}")
    return t, p


def metrics(directory, reference, origin):
    t, p = load_pose(directory / "global_pose.csv")
    rt, rp = reference
    if t[-1] - origin < 285:
        raise ValueError(f"run too short: {t[-1] - origin:.2f}s")
    if rt[0] > t[0] + 0.2 or rt[-1] < t[-1] - 0.2:
        raise ValueError("reference does not cover the entire replay")
    r = np.stack([np.interp(t, rt, rp[:, i]) for i in range(3)], 1)
    error = np.linalg.norm(p - r, axis=1)
    dt = np.diff(t)
    delta = np.diff(p, axis=0)
    v = delta / dt[:, None]
    acc = np.diff(v, axis=0) / ((dt[:-1] + dt[1:])[:, None] / 2)
    good_dt = (dt >= 0.045) & (dt <= 0.25)
    good_acc = good_dt[1:] & good_dt[:-1]
    a = np.linalg.norm(acc[good_acc], axis=1)
    if len(a) < 100:
        raise ValueError("too few continuous odometry samples")

    windows = {}
    for lo, hi in ((0, 40), (40, 60), (60, 120), (120, 180), (180, 240), (240, 290)):
        mask = (t >= origin + lo) & (t < origin + hi)
        windows[f"{lo}-{hi}s_rmse_m"] = float(np.sqrt(np.mean(error[mask] ** 2)))
    g, _ = load_pose(directory / "gicp_map_pose.csv")
    with (directory / "trajectory.csv").open(newline="") as stream:
        modes = set(r["failsafe_mode"] for r in csv.DictReader(stream))
    return {
        "whole_rmse_m": float(np.sqrt(np.mean(error**2))),
        "max_error_m": float(np.max(error)),
        "end_error_m": float(error[-1]),
        "max_step_m": float(np.max(np.linalg.norm(delta, axis=1))),
        "speed_p95_mps": float(np.percentile(np.linalg.norm(v[good_dt], axis=1), 95)),
        "speed_max_mps": float(np.max(np.linalg.norm(v[good_dt], axis=1))),
        "accel_p95_mps2": float(np.percentile(a, 95)),
        "accel_p99_mps2": float(np.percentile(a, 99)),
        "accel_max_mps2": float(np.max(a)),
        "gicp_matches": len(g),
        "gicp_max_gap_s": float(np.max(np.diff(g))),
        "observed_modes": sorted(modes),
        "window_rmse": windows,
    }, t, v, acc, good_dt, good_acc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    reference = load_pose(args.reference)
    origin = reference[0][0]
    baseline, bt, bv, ba, bdt, badt = metrics(args.baseline, reference, origin)
    candidate, ct, cv, ca, cdt, cadt = metrics(args.candidate, reference, origin)
    report = {
        "reference_independent_ground_truth": False,
        "target_rmse_m": 0.5,
        "target_achieved": candidate["whole_rmse_m"] <= 0.5,
        "baseline": baseline,
        "candidate": candidate,
        "improved_whole_accuracy": candidate["whole_rmse_m"] < baseline["whole_rmse_m"],
        "improved_small_loop": candidate["window_rmse"]["40-60s_rmse_m"] <
            baseline["window_rmse"]["40-60s_rmse_m"],
        "improved_accel_p95": candidate["accel_p95_mps2"] < baseline["accel_p95_mps2"],
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plots = args.candidate / "plots"
    plots.mkdir(exist_ok=True)
    for label, bdata, cdata, unit, fname in (
        ("Speed", np.linalg.norm(bv, axis=1), np.linalg.norm(cv, axis=1), "m/s", "08_speed_comparison.png"),
        ("Acceleration", np.linalg.norm(ba, axis=1), np.linalg.norm(ca, axis=1), "m/s²", "09_acceleration_comparison.png"),
    ):
        fig, ax = plt.subplots(figsize=(13, 3.7))
        btime = bt[1:] if label == "Speed" else bt[1:-1]
        ctime = ct[1:] if label == "Speed" else ct[1:-1]
        ax.plot(btime - origin, bdata, lw=0.8, alpha=0.7, label="Previous best healthy")
        ax.plot(ctime - origin, cdata, lw=0.8, alpha=0.8, label="Healthy development candidate")
        ax.set(xlabel="Bag time (s)", ylabel=f"{label} ({unit})", xlim=(0, 290))
        ax.grid(alpha=0.2)
        ax.legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(plots / fname, dpi=160)
        plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
