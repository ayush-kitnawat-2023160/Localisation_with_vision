#!/usr/bin/env python3
"""Structural map QA and visual previews; never mistakes self-consistency for truth.

Produces quality.json, map_overview.png, and visual_keyframes_contact_sheet.jpg.
No other ROS bags are used; this does not test runtime relocalization.
"""
import argparse
import csv
import json
from pathlib import Path
import math
import numpy as np
from build_step3_cache import read_pcd, quat_matrix


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map",required=True)
    ap.add_argument("--sample-neighbors",type=int,default=25)
    args=ap.parse_args()
    root=Path(args.map)
    meta=json.loads((root/"metadata.json").read_text())
    lidar=list(csv.DictReader((root/"keyframes.csv").open()))
    visual=list(csv.DictReader((root/"visual"/"keyframes.csv").open()))
    poses=np.loadtxt(root/"pose.json").reshape(-1,7)
    assert len(poses)==len(lidar)==len(list((root/"pcd").glob("*.pcd"))),"LiDAR keyframe count mismatch"
    assert len(visual)==len(list((root/"visual"/"images").glob("*.jpg"))),"visual images missing"
    assert len(visual)==len(list((root/"visual"/"landmarks").glob("*.npz"))),"visual landmarks missing"
    assert np.isfinite(poses).all() and np.max(abs(np.linalg.norm(poses[:,3:],axis=1)-1))<.01
    p=poses[:,:3]
    dp=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    lidar_times=np.array([float(r["timestamp"]) for r in lidar])
    camera_times=np.array([float(r["timestamp"]) for r in visual])
    print("LiDAR",len(lidar),"Visual",len(visual),"duration",lidar_times[-1]-lidar_times[0])
    print("XY bounds",p[:,:2].min(axis=0),p[:,:2].max(axis=0),
          "Z range",float(np.ptp(p[:,2])))
    from scipy.spatial import cKDTree
    neigh=[]
    # Geometric overlap of nearby physical keyframes; not surveyed accuracy.
    idx=np.unique(np.linspace(0,max(0,len(lidar)-2),args.sample_neighbors).astype(int))
    for i in idx:
        xyz,_=read_pcd(root/"pcd"/f"{i}.pcd")
        j=i+1
        other,_=read_pcd(root/"pcd"/f"{j}.pcd")
        if len(xyz)<100 or len(other)<100:continue
        xyz=xyz[::max(1,len(xyz)//600)]
        other=other[::max(1,len(other)//1800)]
        xyz=xyz@quat_matrix(*poses[i,3:]).T+poses[i,:3]
        other=other@quat_matrix(*poses[j,3:]).T+poses[j,:3]
        d,_=cKDTree(other).query(xyz,k=1,workers=1)
        neigh.append(float(np.mean(d<.8)))
    image_points=[]
    for row in visual:
        data=np.load(root/"visual"/row["landmarks"])
        assert data["desc"].shape[0]==len(data["xyz"])==len(data["uv"])
        assert np.isfinite(data["xyz"]).all()
        image_points.append(len(data["xyz"]))
    quality=dict(
       lidar_keyframes=len(lidar),
       visual_keyframes=len(visual),
       lidar_trajectory_duration_sec=float(lidar_times[-1]-lidar_times[0]),
       visual_trajectory_duration_sec=float(camera_times[-1]-camera_times[0]),
       lidar_preview_points=int(meta["global_points"]),
       visual_preview_points=int(meta["visual_points"]),
       xy_bbox_m=[p[:,:2].min(axis=0).tolist(),p[:,:2].max(axis=0).tolist()],
       z_span_m=float(np.ptp(p[:,2])),
       z_span_warning=bool(np.ptp(p[:,2])>5.0),
       median_neighbor_lidar_overlap_0p8m=float(np.median(neigh)),
       min_neighbor_lidar_overlap_0p8m=float(np.min(neigh)),
       median_depth_supported_landmarks_per_image=float(np.median(image_points)),
       min_depth_supported_landmarks_per_image=int(min(image_points)),
       gps_keyframe_fraction=float(np.mean([bool(r["gps_lat"]) for r in lidar])),
       geometry_validity="STRUCTURAL_AND_ADJACENT_SCAN_CONSISTENCY_ONLY",
       loop_closure_validated=False,
       absolute_accuracy_validated=False,
       external_bags_tested=False,
       map_source_quality=meta["pose_quality"],
       warnings=["Raw odometry map is NOT loop-closed or geodetically aligned to Lanelet.",
                 "Feature landmark geometry reuses FAST-LIVO2 odometry; visual match consistency is not independent accuracy.",
                 "Inherited LiDAR-IMU and base transforms are not validated for final physical mount."])
    (root/"quality.json").write_text(json.dumps(quality,indent=2)+"\n")
    print(json.dumps(quality,indent=2))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(10,9))
    ax.plot(p[:,0],p[:,1],'-',alpha=.8,label="FAST-LIVO2 trajectory (raw)")
    ax.scatter(p[0,0],p[0,1],s=75,marker="o",label="start")
    ax.scatter(p[-1,0],p[-1,1],s=80,marker="x",label="end")
    v=np.array([[float(r["x"]),float(r["y"])] for r in visual])
    ax.scatter(v[:,0],v[:,1],s=9,alpha=.65,label="visual keyframes")
    ax.axis("equal");ax.grid(True,alpha=.3)
    ax.set(xlabel="Raw map X (m)",ylabel="Raw map Y (m)",
           title="Fresh campus map: odometry keyframes (not optimized / not Lanelet aligned)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(root/"map_overview.png",dpi=160)
    plt.close(fig)

    import cv2
    from PIL import Image, ImageDraw
    picks=np.unique(np.linspace(0,len(visual)-1,min(16,len(visual))).astype(int))
    sheet=Image.new("RGB",(4*320,4*212),(30,30,30))
    draw=ImageDraw.Draw(sheet)
    for slot,i in enumerate(picks):
        file=root/"visual"/visual[i]["image"]
        im=Image.open(file).convert("RGB");im.thumbnail((318,187))
        x=(slot%4)*320;y=(slot//4)*212
        sheet.paste(im,(x,y))
        draw.text((x+4,y+189),f'Visual keyframe {i:03d} / {len(visual)}',fill=(255,255,255))
    sheet.save(root/"visual_keyframes_contact_sheet.jpg",quality=84)


if __name__=="__main__":
    main()
