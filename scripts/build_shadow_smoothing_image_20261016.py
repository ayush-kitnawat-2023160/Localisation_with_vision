#!/usr/bin/env python3
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[1]
folder=root/'step3_global/visual_fixed_lag_shadow_dev_20261016'
folder.mkdir(parents=True,exist_ok=True)
(folder/'Dockerfile').write_text('''FROM fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015
COPY scripts/robust_fixed_lag_pose_smoother_20261016.py /ws/src/Fast_Livo2/scripts/robust_fixed_lag_pose_smoother_20261016.py
COPY scripts/ros_fixed_lag_pose_smoother_20261016.py /ws/src/Fast_Livo2/scripts/ros_fixed_lag_pose_smoother_20261016.py
RUN chmod +x /ws/src/Fast_Livo2/scripts/ros_fixed_lag_pose_smoother_20261016.py
''')
# Docker is run from the repository root, not the folder, to COPY scripts.
(folder/'Dockerfile').write_text((folder/'Dockerfile').read_text().replace('scripts/robust_','scripts/robust_'))
subprocess.run(['docker','build','-f',str(folder/'Dockerfile'),
  '-t','fast_livo2_failsafe:visual-fixed-lag-shadow-dev-20261016',str(root)],check=True)
