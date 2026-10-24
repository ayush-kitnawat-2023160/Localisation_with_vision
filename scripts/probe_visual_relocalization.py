#!/usr/bin/env python3
"""Offline within-map matching probe; NOT held-out localization accuracy.

Queries one in every N saved keyframes. Tries neighboring saved keyframes,
excluding its own image, to estimate map-frame camera translation via
ORB correspondence + 2D-3D PnP. This is only a feature-map viability check;
the query still comes from the mapping bag and the 3D geometry shares its
input odometry trajectory. Prints success/failure and saves JSON.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import cv2


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--visual",required=True)
    p.add_argument("--stride",type=int,default=3)
    p.add_argument("--neighbors",type=int,default=4)
    p.add_argument("--min-inliers",type=int,default=18)
    p.add_argument("--error-limit-m",type=float,default=3.0)
    a=p.parse_args()
    root=Path(a.visual)
    rows=list(csv.DictReader((root/"keyframes.csv").open()))
    features=[np.load(root/k["landmarks"]) for k in rows]
    K=np.array([[371.6953430175781,0,477.1928405761719],
                [0,371.6953430175781,294.3376159667969],
                [0,0,1]],float)
    matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
    queries=[]
    for i in range(a.neighbors,len(rows)-a.neighbors,a.stride):
        best=(0,None)
        for j in range(i-a.neighbors,i+a.neighbors+1):
            if i==j:continue
            matches=[x[0] for x in matcher.knnMatch(features[j]["desc"],
                      features[i]["desc"],k=2)
                     if len(x)==2 and x[0].distance<.75*x[1].distance]
            if len(matches)<a.min_inliers:continue
            obj=np.asarray([features[j]["xyz"][x.queryIdx] for x in matches],float)
            uv=np.asarray([features[i]["uv"][x.trainIdx] for x in matches],float)
            ok,rv,tv,inliers=cv2.solvePnPRansac(
                obj,uv,K,None,iterationsCount=80,reprojectionError=3.5,
                flags=cv2.SOLVEPNP_EPNP)
            if ok and inliers is not None and len(inliers)>=a.min_inliers:
                R,_=cv2.Rodrigues(rv)
                cam_pos=-R.T@tv[:,0]
                reference=np.array([float(rows[i][k]) for k in ("x","y","z")])
                error=float(np.linalg.norm(cam_pos-reference))
                if len(inliers)>best[0]:best=(len(inliers),error)
        queries.append(dict(query_keyframe=i,inliers=best[0],
                            translation_consistency_error_m=best[1],
                            passed=bool(best[0]>=a.min_inliers and
                                        best[1] is not None and
                                        best[1]<a.error_limit_m)))
    result=dict(map_keyframes=len(rows),queries=len(queries),
                verified_matches=sum(x["passed"] for x in queries),
                success_rate=sum(x["passed"] for x in queries)/max(1,len(queries)),
                target="WITHIN-MAPPING-BAG FEATURE VIABILITY; NOT HELD-OUT GLOBAL LOCALIZATION",
                minimum_validation_rate=0.70,
                ready_for_global_visual_authority=False,
                examples=queries)
    # Do not auto-declare ready just from within-map performance:
    # independent heldout bag validation and a trustworthy global map needed.
    dst=root.parent/"visual_neighbor_probe.json"
    dst.write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k!="examples"},indent=2))


if __name__=="__main__":
    main()
