#!/usr/bin/env python3
"""Isolate publication-ramp rate hypothesis on the Oct 13 LiDAR-best image.

Do not modify the ESIKF, saved map, absolute visual registration, source
trajectory, or production ROS launch. Changes only the published correction
time-constant, rather than inventing a physically translated plot.
"""
from pathlib import Path

root=Path(__file__).resolve().parents[1]
original=root/'launch/fast_livo2_step3_global_rosbag.launch'
copy=root/'launch/fast_livo2_step3_depth_ramp_trial_20261016.launch'
data=original.read_text()
changes={
 '<param name="tracking_publish_smoothing_min_sec" value="0.25"/>':
 '<param name="tracking_publish_smoothing_min_sec" value="0.35"/>',
 '<param name="tracking_publish_smoothing_max_sec" value="1.50"/>':
 '<param name="tracking_publish_smoothing_max_sec" value="3.00"/>',
 '<param name="tracking_publish_max_correction_speed_mps" value="6.0"/>':
 '<param name="tracking_publish_max_correction_speed_mps" value="2.5"/>',
}
for a,b in changes.items():
 assert data.count(a)==1,(a,data.count(a))
 data=data.replace(a,b)
copy.write_text(data)
print(copy)
for a,b in changes.items():print('trial:',a,'->',b)
