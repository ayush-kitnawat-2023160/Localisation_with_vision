#!/usr/bin/env python3
"""Isolated follow-up: geometry is an independent VETO, never a pose source.

Preserves live visual-map GICP pose if the canonical LiDAR map confirms it;
rejects contradictory positions. Never replaces or interpolates measured XYZ
using the second PCD's own locally ambiguous GICP minimum.
"""
from pathlib import Path
import subprocess
import difflib

root=Path(__file__).resolve().parents[1]
source=root/'step3_global/depth_crossmap_trial_20261016/src/failsafe_map_localizer.cpp'
before=source.read_text()
old='''    // Map origins are the same loop-closed PCD frame. Keep camera/IMU yaw,
    // and let the separate geometric map correct *only position*. The 0.75
    // blend reduces one-off 3D minima without inventing measurements.
    verified_measurement=visual_measurement;
    verified_measurement.block<3,1>(0,3)=
        0.25f*visual_measurement.block<3,1>(0,3)+
        0.75f*measured.block<3,1>(0,3);'''
new='''    // VETO-ONLY: the saved LiDAR PCD is independent evidence that the visual
    // measurement is geometrically plausible, NOT a replacement measurement.
    // This leaves the absolute visual-map estimate and its orientation alone;
    // extra ICP against repeated geometric structures could introduce bias.
    verified_measurement=visual_measurement;'''
assert before.count(old)==1,'Unexpected crossmap development source'
after=before.replace(old,new)
out=root/'step3_global/depth_crossmap_veto_trial_20261016'
out.mkdir(parents=True,exist_ok=True)
src=out/'src/failsafe_map_localizer.cpp'
src.parent.mkdir(parents=True,exist_ok=True)
src.write_text(after)
(out/'crossmap_veto_only.patch').write_text('\n'.join(difflib.unified_diff(before.splitlines(),after.splitlines(),lineterm=''))+'\n')
(out/'Dockerfile').write_text('''FROM fast_livo2_failsafe:depth-crossmap-dev-20261016
COPY src/failsafe_map_localizer.cpp /ws/src/Fast_Livo2/src/failsafe_map_localizer.cpp
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 failsafe_map_localizer'
''')
print('Building isolated crossmap veto-only frontend',flush=True)
subprocess.run(['docker','build','-t','fast_livo2_failsafe:depth-crossmap-veto-only-dev-20261016',str(out)],check=True)
