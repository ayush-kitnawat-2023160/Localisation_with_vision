#!/usr/bin/env python3
"""Precompute immutable FAST-LOCALIZATION retrieval and registration assets."""

import argparse
import json
import math
import shutil
import struct
from pathlib import Path

import numpy as np


FIELDS = ("x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z", "curvature")


def read_pcd(path):
    with open(path, "rb") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError(f"invalid PCD: {path}")
            text = line.decode("ascii").strip()
            header.append(text)
            if text.startswith("DATA "):
                mode = text.split()[1]
                break
        values = {line.split()[0]: line.split()[1:] for line in header if line and not line.startswith("#")}
        if mode != "binary":
            raise RuntimeError(f"only binary PCD is supported: {path}")
        names = values["FIELDS"]
        sizes = list(map(int, values["SIZE"]))
        types = values["TYPE"]
        counts = list(map(int, values.get("COUNT", ["1"] * len(names))))
        formats, offsets, offset = [], [], 0
        type_map = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 2): "<u2", ("U", 4): "<u4", ("I", 4): "<i4"}
        for size, typ, count in zip(sizes, types, counts):
            if count != 1:
                raise RuntimeError("multi-count PCD fields are unsupported")
            formats.append(type_map[(typ, size)]); offsets.append(offset); offset += size
        dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": offset})
        data = np.frombuffer(stream.read(), dtype=dtype, count=int(values["POINTS"][0]))
    xyz = np.column_stack([data[n] for n in ("x", "y", "z")]).astype(np.float64)
    intensity = np.asarray(data["intensity"], dtype=np.float32) if "intensity" in names else np.zeros(len(data), np.float32)
    good = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
    return xyz[good], intensity[good]


def write_pcd(path, xyz, intensity):
    out = np.zeros((len(xyz), 8), dtype="<f4")
    out[:, :3] = xyz.astype(np.float32); out[:, 3] = intensity.astype(np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS " + " ".join(FIELDS) + "\nSIZE 4 4 4 4 4 4 4 4\n"
        "TYPE F F F F F F F F\nCOUNT 1 1 1 1 1 1 1 1\n"
        f"WIDTH {len(out)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(out)}\nDATA binary\n"
    ).encode("ascii")
    with open(path, "wb") as stream:
        stream.write(header); stream.write(out.tobytes())


def quat_matrix(qw, qx, qy, qz):
    q = np.array([qx, qy, qz, qw], dtype=np.float64); q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def voxel(points, intensity, leaf):
    keys = np.floor(points / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True); idx.sort()
    return points[idx], intensity[idx]


def scan_context(points, rings=20, sectors=60, radius=80.0, lidar_height=2.0):
    # Match the official C++ Scan Context implementation exactly: occupied bins
    # start at a negative sentinel, while truly empty bins become zero only
    # after all points have been accumulated.
    result = np.full((rings, sectors), -1000.0, dtype=np.float64)
    ranges = np.linalg.norm(points[:, :2], axis=1)
    angles = np.mod(np.degrees(np.arctan2(points[:, 1], points[:, 0])), 360.0)
    valid = ranges <= radius
    ring = np.clip(np.ceil(ranges[valid] / radius * rings).astype(np.int64), 1, rings) - 1
    sector = np.clip(np.ceil(angles[valid] / 360.0 * sectors).astype(np.int64), 1, sectors) - 1
    heights = points[valid, 2] + lidar_height
    np.maximum.at(result, (ring, sector), heights)
    result[result == -1000.0] = 0.0
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--radius", type=float, default=4.5)
    ap.add_argument("--voxel", type=float, default=0.20)
    ap.add_argument("--submap-voxel", type=float, default=0.35)
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()
    root = Path(args.map)
    poses = np.loadtxt(root / "pose.json")
    if poses.ndim == 1: poses = poses.reshape(1, -1)
    pcds = [root / "pcd" / f"{i}.pcd" for i in range(len(poses))]
    if not all(p.exists() for p in pcds): raise SystemExit("keyframe PCD sequence is incomplete")
    cache = root / "cache"
    if cache.exists() and args.overwrite: shutil.rmtree(cache)
    if cache.exists(): raise SystemExit(f"cache exists: {cache}; use --overwrite")
    submaps = cache / "local_submaps"; submaps.mkdir(parents=True)

    clouds, descriptors = [], []
    for i, path in enumerate(pcds):
        xyz, intensity = read_pcd(path); clouds.append((xyz, intensity)); descriptors.append(scan_context(xyz))
    descriptors = np.stack(descriptors)
    ring_keys = descriptors.mean(axis=2).astype("<f4")
    with open(cache / "scan_context.bin", "wb") as f:
        f.write(b"SCDBV001"); f.write(struct.pack("<III", len(poses), 20, 60)); f.write(descriptors.astype("<f8").tobytes())
    with open(cache / "ring_keys.bin", "wb") as f:
        f.write(b"RKEYV001"); f.write(struct.pack("<II", len(poses), 20)); f.write(ring_keys.tobytes())
    poses.astype("<f8").tofile(cache / "keyframe_poses.bin")

    rotations = [quat_matrix(*row[3:7]) for row in poses]
    positions = poses[:, :3]
    world_chunks, world_intensity = [], []
    for i, ((xyz, intensity), rot, pos) in enumerate(zip(clouds, rotations, positions)):
        world_chunks.append(xyz @ rot.T + pos); world_intensity.append(intensity)
    global_xyz, global_i = voxel(np.vstack(world_chunks), np.concatenate(world_intensity), args.voxel)
    write_pcd(cache / "global_map.pcd", global_xyz, global_i)

    point_counts = []
    for center in range(len(poses)):
        neighbors = np.flatnonzero(np.linalg.norm(positions - positions[center], axis=1) <= args.radius)
        inv_r = rotations[center].T; center_p = positions[center]
        chunks, intensities = [], []
        for n in neighbors:
            xyz, intensity = clouds[n]
            world = xyz @ rotations[n].T + positions[n]
            chunks.append((world - center_p) @ inv_r.T); intensities.append(intensity)
        xyz, intensity = voxel(np.vstack(chunks), np.concatenate(intensities), args.submap_voxel)
        write_pcd(submaps / f"{center}.pcd", xyz, intensity); point_counts.append(len(xyz))
        if (center + 1) % 25 == 0: print(f"Built {center+1}/{len(poses)} local submaps", flush=True)

    metadata = {"version": 1, "keyframes": len(poses), "rings": 20, "sectors": 60,
                "candidate_radius_m": args.radius, "voxel_leaf_m": args.voxel,
                "submap_voxel_leaf_m": args.submap_voxel,
                "global_points": len(global_xyz), "submap_points_min": min(point_counts),
                "submap_points_max": max(point_counts), "submap_points_total": sum(point_counts)}
    (cache / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__": main()
