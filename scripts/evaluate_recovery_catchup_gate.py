#!/usr/bin/env python3
"""Reject published-pose correction backlogs after sensor recovery.

The same-bag loop-closed trajectory is a *proxy*, not independent ground truth.
In particular, matching against the reference must not be called absolute accuracy.
"""
import argparse
import csv
import json
from pathlib import Path


CHECKED_METRICS = (
    "reference_disagreement_3d_rmse_m",
    "reference_disagreement_failure_rmse3d_m",
    "reference_disagreement_post_failure_rest_rmse3d_m",
    "reference_disagreement_end3d_m",
    "step_max_m",
)


def read_metrics(path):
    values = {}
    for line in path.read_text().splitlines():
        key, sep, value = line.partition("=")
        if sep:
            try:
                values[key] = float(value)
            except ValueError:
                pass
    missing = set(CHECKED_METRICS).difference(values)
    if missing:
        raise ValueError(f"{path}: missing {sorted(missing)}")
    return values


def verified_outage_and_recovery(case_dir, mode):
    """A passing metric is meaningless if the injected failure never occurred."""
    failed_modes = {
        "lidar": {"VI_LIDAR_FAILED", "IMU_ONLY_BRIDGE"},
        "imu": {"LV_IMU_FAILED"},
        "camera": {"LIO_CAMERA_FAILED"},
    }[mode]
    rows = list(csv.DictReader((case_dir / "trajectory.csv").open(newline="")))
    if len(rows) < 100:
        return False
    failures = [float(r["timestamp"]) for r in rows if r["failsafe_mode"] in failed_modes]
    if len(failures) < 20 or max(failures) - min(failures) < 25.0:
        return False
    # Require the full-sensor mode to resume after the dropout and remain available.
    restored = [float(r["timestamp"]) for r in rows
                if r["failsafe_mode"] == "FULL_LIVO" and float(r["timestamp"]) > max(failures)]
    return len(restored) >= 50 and max(restored) - min(restored) >= 10.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", type=Path, required=True)
    ap.add_argument("--candidate", type=Path, required=True)
    ap.add_argument("--case", choices=("lidar", "imu", "camera"), required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    b = read_metrics(args.baseline / "metrics_exact.txt")
    c = read_metrics(args.candidate / "metrics_exact.txt")
    checks = {
        "actual_failure_and_recovery_seen": verified_outage_and_recovery(args.candidate, args.case),
        # Existing development tolerances, retained rather than weakened.
        "whole_proxy_rmse": c[CHECKED_METRICS[0]] <= b[CHECKED_METRICS[0]] + 0.45,
        "outage_proxy_rmse": c[CHECKED_METRICS[1]] <= b[CHECKED_METRICS[1]] + 0.60,
        "max_pose_step": c["step_max_m"] <= b["step_max_m"] + 0.10,
        # Added: a run must not look good over the first half and drift at the end.
        "post_recovery_rest_rmse": c["reference_disagreement_post_failure_rest_rmse3d_m"]
            <= b["reference_disagreement_post_failure_rest_rmse3d_m"] + 0.50,
        "end_disagreement": c["reference_disagreement_end3d_m"]
            <= min(2.50, b["reference_disagreement_end3d_m"] + 1.00),
    }
    report = {
        "case": args.case,
        "dataset": "camera_lidar_imu_gps.bag",
        "independent_ground_truth": False,
        "measurements": {
            key: {"baseline": b[key], "candidate": c[key]} for key in CHECKED_METRICS
        },
        "checks": checks,
        "pass": all(checks.values()),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["pass"]:
        raise SystemExit("FAIL: post-recovery published-pose regression")


if __name__ == "__main__":
    main()
