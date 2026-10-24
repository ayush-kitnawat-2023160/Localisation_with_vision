#!/usr/bin/env python3
"""Export LiDAR + RGB-D visual maps from a ROS1 bag and a measured LIVO trajectory.

Run inside ROS Noetic with python3-rosbag, cv2, NumPy and SciPy available.
This tool DOES NOT optimize trajectories or claim that its output is loop closed.
Input trajectory: timestamp,x,y,z,qx,qy,qz,qw (FAST-LIVO2 IMU/body poses).
LiDAR PCDs remain in their own instantaneous LiDAR frames and pose.json stores
T_map_lidar. This matches upstream FAST-LOCALIZATION's map/pcd/N.pcd contract.
"""
import argparse
import csv
import json
from pathlib import Path
import math

import cv2
import numpy as np
import rosbag
import yaml
from tf.transformations import quaternion_matrix, quaternion_from_matrix, quaternion_slerp


def homog(rot, xyz):
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = xyz
    return out


def write_pcd(path, xyz, intensity=None, rgb=None):
    xyz = np.asarray(xyz, np.float32)
    if rgb is not None:
        cols = np.asarray(rgb, dtype=np.uint8)
        packed = (cols[:, 2].astype(np.uint32) |
                  (cols[:, 1].astype(np.uint32) << 8) |
                  (cols[:, 0].astype(np.uint32) << 16)).view(np.float32)
        fields = "x y z rgb"
        data = np.column_stack([xyz, packed]).astype("<f4")
    else:
        inten = np.zeros(len(xyz), np.float32) if intensity is None else np.asarray(intensity, np.float32)
        data = np.zeros((len(xyz), 8), dtype="<f4")
        data[:, :3], data[:, 3] = xyz, inten
        fields = "x y z intensity normal_x normal_y normal_z curvature"
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
           f"FIELDS {fields}\nSIZE " + " ".join(["4"] * data.shape[1]) + "\n"
           "TYPE " + " ".join(["F"] * data.shape[1]) + "\n"
           "COUNT " + " ".join(["1"] * data.shape[1]) + "\n"
           f"WIDTH {len(data)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           f"POINTS {len(data)}\nDATA binary\n").encode()
    path.write_bytes(hdr + data.tobytes())


def downsample(xyz, leaf, *additional):
    if not len(xyz):
        return (xyz,) + additional
    _, idx = np.unique(np.floor(xyz / leaf).astype(np.int32), axis=0, return_index=True)
    idx.sort()
    return (xyz[idx],) + tuple(v[idx] for v in additional)


def load_trajectory(path):
    rec = list(csv.DictReader(open(path)))
    if len(rec) < 20:
        raise RuntimeError("Need an actual FAST-LIVO2 trajectory with >=20 odometry poses")
    times = np.array([float(r["timestamp"]) for r in rec])
    xyz = np.array([[float(r[k]) for k in ("x", "y", "z")] for r in rec])
    q = np.array([[float(r[k]) for k in ("qx", "qy", "qz", "qw")] for r in rec])
    valid = np.isfinite(times) & np.isfinite(xyz).all(axis=1) & np.isfinite(q).all(axis=1)
    times, xyz, q = times[valid], xyz[valid], q[valid]
    indices = np.argsort(times)
    times, xyz, q = times[indices], xyz[indices], q[indices]
    unique = np.r_[True, np.diff(times) > 1e-6]
    times, xyz, q = times[unique], xyz[unique], q[unique]
    q /= np.linalg.norm(q, axis=1)[:, None]
    return times, xyz, q


def time_pose(t, traj, max_age=0.22):
    times, xyz, q = traj
    if t < times[0] or t > times[-1]:
        return None
    j = np.searchsorted(times, t)
    near = min(abs(t-times[max(0, j-1)]), abs(t-times[min(j, len(times)-1)]))
    if near > max_age:
        return None
    j = max(1, min(j, len(times)-1))
    alpha = (t-times[j-1]) / (times[j]-times[j-1])
    trans = xyz[j-1] * (1-alpha) + xyz[j] * alpha
    interp_quat = quaternion_slerp(q[j-1], q[j], alpha)
    return homog(quaternion_matrix(interp_quat)[:3, :3], trans)


def nearest_gps(t, gps, max_dt=0.5):
    if not gps:
        return {}
    times = np.asarray([g["timestamp"] for g in gps])
    j = int(np.argmin(abs(times-t)))
    return gps[j] if abs(times[j]-t) <= max_dt else {}


def save_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--trajectory", required=True)
    ap.add_argument("--calibration", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--lidar-topic", default="/lidar103/velodyne_points")
    ap.add_argument("--rgb-topic", default="/zedx/zed_node/left/image_rect_color")
    ap.add_argument("--depth-topic", default="/zedx/zed_node/depth/depth_registered")
    ap.add_argument("--gps-topic", default="/mavros/global_position/raw/fix")
    ap.add_argument("--keyframe-distance", type=float, default=1.5)
    ap.add_argument("--keyframe-yaw-deg", type=float, default=10.)
    ap.add_argument("--max-keyframe-seconds", type=float, default=3.0)
    ap.add_argument("--image-period", type=float, default=1.5)
    ap.add_argument("--lidar-voxel", type=float, default=.25)
    ap.add_argument("--preview-voxel", type=float, default=.35)
    ap.add_argument("--visual-voxel", type=float, default=.18)
    ap.add_argument("--no-deskew", action="store_true")
    ap.add_argument("--lidar-only", action="store_true",
                    help="Export the map-localizer LiDAR PCD/poses/GPS only; do not deserialize camera/depth or require visual landmarks")
    args = ap.parse_args()
    output = Path(args.output)
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"Refusing to overwrite existing map: {output}")
    (output/"pcd").mkdir(parents=True, exist_ok=True)
    (output/"visual"/"images").mkdir(parents=True, exist_ok=True)
    (output/"visual"/"landmarks").mkdir(parents=True, exist_ok=True)
    with open(args.calibration) as f:
        calib = yaml.safe_load(f)
    c = calib["extrin_calib"]
    T_imu_lidar = homog(np.asarray(c["extrinsic_R"]).reshape(3,3), np.asarray(c["extrinsic_T"]))
    T_camera_lidar = homog(np.asarray(c["Rcl"]).reshape(3,3), np.asarray(c["Pcl"]))
    T_lidar_camera = np.linalg.inv(T_camera_lidar)
    traj = load_trajectory(args.trajectory)
    orb = cv2.ORB_create(nfeatures=1800, fastThreshold=8)
    # Read GPS only once, without deserializing GB of image/pointcloud messages.
    gps = []
    with rosbag.Bag(args.bag) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[args.gps_topic]):
            if np.isfinite(msg.latitude) and np.isfinite(msg.longitude):
                gps.append(dict(timestamp=msg.header.stamp.to_sec(), latitude=msg.latitude,
                                longitude=msg.longitude, altitude=msg.altitude,
                                fix_status=int(msg.status.status),
                                covariance_xx=float(msg.position_covariance[0]),
                                covariance_yy=float(msg.position_covariance[4])))
    rgb = None
    prev_lidar_pose = None
    prev_lidar_yaw = None
    last_lidar_t = -1e12
    last_visual_t = -1e12
    lid_rows, vis_rows, poses = [], [], []
    all_lidar, all_i, vis_xyz, vis_colors = [], [], [], []
    depth_buffer = {}
    rgb_buffer = {}
    visual_dropped_for_geometry = 0
    with rosbag.Bag(args.bag) as bag:
        topics = [args.lidar_topic] if args.lidar_only else [args.lidar_topic, args.rgb_topic, args.depth_topic]
        for topic, msg, rec_stamp in bag.read_messages(topics=topics):
            ts = msg.header.stamp.to_sec()
            if topic == args.depth_topic:
                if msg.encoding != "32FC1":
                    raise RuntimeError(f"ZED registered depth must be 32FC1 metres, got {msg.encoding}")
                depth_buffer[round(ts, 5)] = msg
                if len(depth_buffer) > 20:
                    del depth_buffer[next(iter(depth_buffer))]
                # A registered depth frame and RGB frame can be recorded in
                # either callback order.  Never discard a color frame merely
                # because the corresponding depth message arrives later.
                paired_color = rgb_buffer.pop(round(ts, 5), None)
                if paired_color is None:
                    continue
                msg = paired_color
                topic = args.rgb_topic
            if topic == args.rgb_topic:
                if ts-last_visual_t < args.image_period:
                    continue
                p = time_pose(ts, traj)
                dmsg = depth_buffer.get(round(ts, 5))
                if p is None:
                    continue
                if dmsg is None:
                    rgb_buffer[round(ts, 5)] = msg
                    if len(rgb_buffer) > 20:
                        del rgb_buffer[next(iter(rgb_buffer))]
                    continue
                arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
                if msg.encoding == "bgra8":
                    image = cv2.cvtColor(arr[:, :msg.width*4].reshape(msg.height,msg.width,4), cv2.COLOR_BGRA2BGR)
                elif msg.encoding == "bgr8":
                    image = arr[:, :msg.width*3].reshape(msg.height,msg.width,3)
                else:
                    continue
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                kp, desc = orb.detectAndCompute(gray, None)
                if desc is None or len(kp) < 40:
                    continue
                dep = np.frombuffer(dmsg.data, np.float32).reshape(dmsg.height, dmsg.step//4)[:, :dmsg.width]
                uv = np.asarray([k.pt for k in kp])
                ix = np.rint(uv[:,0]).astype(int)
                iy = np.rint(uv[:,1]).astype(int)
                in_bounds = (ix>=1)&(ix<msg.width-1)&(iy>=1)&(iy<msg.height-1)
                ix=np.clip(ix,0,dep.shape[1]-1);iy=np.clip(iy,0,dep.shape[0]-1)
                z = dep[iy,ix]
                good = in_bounds & np.isfinite(z) & (z>=.6) & (z<=25.0)
                # Enforce at least some visible geometry, not just a descriptor match.
                if good.sum() < 18:
                    visual_dropped_for_geometry += 1
                    continue
                K=calib["laserMapping"]; fx,fy,cx,cy=[float(K[v]) for v in ("cam_fx","cam_fy","cam_cx","cam_cy")]
                ptcam=np.column_stack(((uv[good,0]-cx)*z[good]/fx,
                                       (uv[good,1]-cy)*z[good]/fy, z[good]))
                T_map_cam = p @ T_imu_lidar @ T_lidar_camera
                xyzmap = ptcam @ T_map_cam[:3,:3].T + T_map_cam[:3,3]
                j=len(vis_rows)
                jpg=f"images/{j:05d}.jpg"
                npz=f"landmarks/{j:05d}.npz"
                cv2.imwrite(str(output/"visual"/jpg),image,[cv2.IMWRITE_JPEG_QUALITY,83])
                np.savez_compressed(output/"visual"/npz,uv=uv[good].astype(np.float32),
                                    xyz=xyzmap.astype(np.float32),desc=desc[good])
                qwxyz=quaternion_from_matrix(T_map_cam)
                geo=nearest_gps(ts,gps)
                vis_rows.append(dict(id=j,timestamp=ts,x=T_map_cam[0,3],y=T_map_cam[1,3],
                                     z=T_map_cam[2,3],qx=qwxyz[0],qy=qwxyz[1],qz=qwxyz[2],qw=qwxyz[3],
                                     descriptors=int(good.sum()),image=jpg,landmarks=npz,
                                     gps_lat=geo.get("latitude",""),gps_lon=geo.get("longitude",""),
                                     gps_var_x=geo.get("covariance_xx",""),gps_var_y=geo.get("covariance_yy",""),
                                     gps_status=geo.get("fix_status","")))
                # Visualization preview: feature point RGB, subsampled to prevent a dense road-only cloud.
                color=image[iy[good],ix[good]]
                vis_xyz.append(xyzmap.astype(np.float32)); vis_colors.append(color)
                last_visual_t=ts
            elif topic == args.lidar_topic:
                p = time_pose(ts, traj)
                if p is None:
                    continue
                T_map_lidar = p @ T_imu_lidar
                y = math.atan2(T_map_lidar[1,0], T_map_lidar[0,0])
                if prev_lidar_pose is not None:
                    delta=np.linalg.norm(T_map_lidar[:3,3]-prev_lidar_pose)
                    dy=abs(math.atan2(math.sin(y-prev_lidar_yaw),math.cos(y-prev_lidar_yaw)))
                    if delta<args.keyframe_distance and dy<np.deg2rad(args.keyframe_yaw_deg) and ts-last_lidar_t<args.max_keyframe_seconds:
                        continue
                names={f.name: f for f in msg.fields}
                if any(v not in names for v in ("x","y","z")):
                    raise RuntimeError("LiDAR PointCloud2 missing x/y/z")
                formats=[("<f4",names[v].offset) for v in ("x","y","z")]
                dtype=np.dtype(dict(names=["x","y","z","intensity","time"],
                                    formats=["<f4","<f4","<f4","<f4","<f4"],
                                    offsets=[names[v].offset if v in names else 0 for v in ("x","y","z","intensity","time")],
                                    itemsize=msg.point_step))
                arr=np.frombuffer(msg.data,dtype=dtype,count=msg.width*msg.height)
                xyz=np.column_stack((arr["x"],arr["y"],arr["z"])).astype(np.float32)
                inten=arr["intensity"].astype(np.float32)
                valid=np.isfinite(xyz).all(axis=1)
                range_m=np.linalg.norm(xyz,axis=1)
                valid &= (range_m>=1.0)&(range_m<=70.0)
                xyz,inten=xyz[valid],inten[valid]
                timefield=arr["time"][valid].copy()
                # Recorded VLP-16 time field is signed seconds relative to header.
                # Warp observed points to the LiDAR position at the scan stamp.
                if not args.no_deskew and len(xyz):
                    deskewed=np.empty_like(xyz)
                    bins=np.floor(timefield/.010).astype(int)
                    T_scan_inv=np.linalg.inv(T_map_lidar)
                    for bin_id in np.unique(bins):
                        select=bins==bin_id
                        tt=ts+(bin_id+.5)*.010
                        tcur=time_pose(tt,traj)
                        if tcur is None:
                            deskewed[select]=xyz[select]
                            continue
                        M=T_scan_inv @ tcur @ T_imu_lidar
                        deskewed[select]=xyz[select] @ M[:3,:3].T + M[:3,3]
                    xyz=deskewed
                xyz,inten=downsample(xyz,args.lidar_voxel,inten)
                if len(xyz)<300:
                    continue
                j=len(lid_rows)
                write_pcd(output/"pcd"/f"{j}.pcd",xyz,inten)
                pos=T_map_lidar[:3,3]
                quat=quaternion_from_matrix(T_map_lidar)
                poses.append(f"{pos[0]:.9f} {pos[1]:.9f} {pos[2]:.9f} {quat[3]:.12f} {quat[0]:.12f} {quat[1]:.12f} {quat[2]:.12f}")
                geo=nearest_gps(ts,gps)
                lid_rows.append(dict(id=j,timestamp=ts,x=pos[0],y=pos[1],z=pos[2],
                                     qx=quat[0],qy=quat[1],qz=quat[2],qw=quat[3],
                                     points=len(xyz),gps_lat=geo.get("latitude",""),
                                     gps_lon=geo.get("longitude",""),
                                     gps_var_x=geo.get("covariance_xx",""),
                                     gps_var_y=geo.get("covariance_yy",""),
                                     gps_status=geo.get("fix_status","")))
                # Preview uses separate sparse cloud to keep memory bound.
                small,_i=downsample(xyz,.8,inten)
                all_lidar.append(small@T_map_lidar[:3,:3].T+pos)
                all_i.append(_i)
                prev_lidar_pose=pos;prev_lidar_yaw=y;last_lidar_t=ts
                if j%50==0:
                    print(f"LiDAR keyframes={len(lid_rows)} visual={len(vis_rows)} time={ts:.3f}",flush=True)
    if len(lid_rows)<10 or (not args.lidar_only and len(vis_rows)<10):
        raise RuntimeError(f"Insufficient map samples: LiDAR {len(lid_rows)} visual {len(vis_rows)}")
    (output/"pose.json").write_text("\n".join(poses)+"\n")
    save_csv(output/"keyframes.csv",lid_rows)
    save_csv(output/"visual"/"keyframes.csv",vis_rows)
    save_csv(output/"gps_measurements.csv",gps)
    full_xyz, full_int=downsample(np.vstack(all_lidar),args.preview_voxel,np.concatenate(all_i))
    write_pcd(output/"global_map.pcd",full_xyz,full_int)
    if vis_xyz:
        vx, vc=downsample(np.vstack(vis_xyz),args.visual_voxel,np.vstack(vis_colors))
        write_pcd(output/"visual"/"landmarks_preview.pcd",vx,rgb=vc)
    else:
        vx = np.empty((0,3),dtype=np.float32)
    stats=dict(format="FAST-LOCALIZATION per-frame LiDAR PCD + anchored RGB-D visual landmark map" if not args.lidar_only else "FAST-LOCALIZATION per-frame LiDAR PCD and pose database",
               source_bag=Path(args.bag).name,source_trajectory=str(args.trajectory),
               pose_quality="RAW_FAST_LIVO2_ODOMETRY_NOT_LOOP_CLOSED",
               lidar_keyframes=len(lid_rows),visual_keyframes=len(vis_rows),
               gps_messages=len(gps),global_points=len(full_xyz),visual_points=len(vx),
               imu_lidar_transform_source=str(args.calibration),deskew=not args.no_deskew,
               visual_frames_rejected_for_depth_geometry=visual_dropped_for_geometry,
               note="Not surveyed ground truth. Optimize map and check loop consistency before use as authoritative global reference.")
    (output/"metadata.json").write_text(json.dumps(stats,indent=2)+"\n")
    print(json.dumps(stats,indent=2))


if __name__ == "__main__":
    main()
