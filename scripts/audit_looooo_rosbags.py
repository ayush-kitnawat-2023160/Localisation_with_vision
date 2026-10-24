#!/usr/bin/env python3
"""Non-mutating ROS1 full serialized-message + sensor timestamp integrity audit.

Runs in fast_livo2:dev after sourcing /opt/ros/noetic/setup.bash.
PASS establishes readable indexed messages, NOT geometric or SLAM accuracy.
"""
import argparse
import collections
import json
import os
import struct
import sys
import numpy as np
import rosbag

TOPICS = {
    "lidar": "/lidar103/velodyne_points",
    "imu": "/mavros/imu/data",
    "camera": "/zedx/zed_node/left/image_rect_color",
    "depth": "/zedx/zed_node/depth/depth_registered",
    "gps": "/mavros/global_position/raw/fix",
    "zed_imu": "/zedx/zed_node/imu/data",
}

def nearby(x, y):
    if not len(x) or not len(y):
        return {}
    a, b = np.asarray(x), np.sort(np.asarray(y))
    j = np.searchsorted(b, a)
    errors = np.minimum(np.abs(a-b[np.maximum(0,j-1)]),
                        np.abs(a-b[np.minimum(len(b)-1,j)]))
    return {"median_ms": round(float(np.median(errors))*1000, 3),
            "p95_ms": round(float(np.percentile(errors,95))*1000, 3)}

def summary(ts, arrival):
    if not ts: return {"count": 0}
    t=np.asarray(ts);r=np.asarray(arrival);dt=np.diff(t)
    return {
        "count":len(ts),"first_stamp":float(t[0]),"last_stamp":float(t[-1]),
        "max_gap_s":float(dt.max()) if dt.size else None,
        "median_interval_s":float(np.median(dt)) if dt.size else None,
        "nonmonotonic":int((dt < -1.e-4).sum()),
        "gaps_over_0p5_s":int((dt > .5).sum()),
        "arrival_header_median_s":float(np.median(r-t)),
        "arrival_header_p95_s":float(np.percentile(r-t,95)),
    }

def audit(path):
    checks={};counts=collections.Counter();headers=collections.defaultdict(list)
    arrival=collections.defaultdict(list);bad=collections.Counter();errors=[]
    with rosbag.Bag(path) as bag:
        wanted=set(TOPICS.values())
        expected={t:int(x.message_count) for t,x in bag.get_type_and_topic_info().topics.items()}
        duration=bag.get_end_time()-bag.get_start_time()
        for topic,raw,receipt in bag.read_messages(raw=True):
            counts[topic]+=1
            if topic not in wanted:continue
            try:
                data=raw[1]
                if len(data)<16:raise ValueError("short ROS Header")
                _,sec,nsec,flen=struct.unpack_from("<IIII",data,0)
                if nsec>=1_000_000_000 or flen>1024 or flen+16>len(data):
                    raise ValueError("malformed ROS Header")
                headers[topic].append(sec+nsec*1e-9)
                arrival[topic].append(receipt.to_sec())
            except Exception as exc:
                bad[topic]+=1
                if bad[topic]<3:errors.append("%s: %s"%(topic,exc))
        mismatch={t:[int(n),int(counts[t])] for t,n in expected.items() if n!=counts[t]}
        if mismatch:errors.append("topic index vs. serialized-message count mismatch")
        for key,topic in TOPICS.items():
            checks[key]=summary(headers[topic],arrival[topic])
            checks[key]["indexed_count"]=expected.get(topic,0)
            checks[key]["bad_header_count"]=bad[topic]
            if key in ("lidar","imu","camera","depth") and not headers[topic]:
                errors.append("Missing required sensor %s"%(topic))
            if bad[topic]:errors.append("bad headers %s"%(topic))
        return {
            "filename":os.path.basename(path),"duration_s":duration,
            "full_read":True,"indexed_message_total":sum(expected.values()),
            "read_message_total":sum(counts.values()),
            "index_mismatches":mismatch,"sensor_timestamps":checks,
            "nearest_stamp_deltas":{
                "camera_to_depth":nearby(headers[TOPICS["camera"]],headers[TOPICS["depth"]]),
                "lidar_to_imu":nearby(headers[TOPICS["lidar"]],headers[TOPICS["imu"]])},
            "errors":errors,"structural_pass":not errors,
            "limitations":"Readable bag != calibrated sensors or stable estimator",
        }

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--bag",required=True)
    p.add_argument("--out",required=True);a=p.parse_args()
    try:
        result=audit(a.bag)
    except Exception as exc:
        result={"filename":os.path.basename(a.bag),"full_read":False,
                "structural_pass":False,"errors":[str(exc)]}
    os.makedirs(os.path.dirname(a.out) or ".",exist_ok=True)
    with open(a.out,"w") as f:json.dump(result,f,indent=2);f.write("\n")
    print(json.dumps(result,indent=2))
    sys.exit(0 if result["structural_pass"] else 1)
