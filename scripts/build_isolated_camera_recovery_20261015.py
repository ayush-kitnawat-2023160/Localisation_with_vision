#!/usr/bin/env python3
"""Development-only camera-return transition experiment on frozen healthy image.

The existing Step3 recovery correction/verification path activates after
LiDAR/IMU failures, but NOT camera loss. Test whether camera-return qualifies
as a recovery epoch without changing estimator, PCD matcher or healthy modes.
No production changes; accept only with strict no-jump/trajectory gates.
"""
from pathlib import Path
import difflib
import subprocess
root=Path(__file__).resolve().parents[1]
old_image='fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015'
new_image='fast_livo2_failsafe:camera-return-handoff-dev-20261015'
src='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
old=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',old_image,src],text=True)
before='      const bool was_critical_fail=was_fail||failsafe_mode_=="LV_IMU_FAILED";'
after='      const bool was_critical_fail=was_fail||failsafe_mode_=="LV_IMU_FAILED"||failsafe_mode_=="LIO_CAMERA_FAILED";'
assert old.count(before)==1
new=old.replace(before,after)
folder=root/'step3_global/camera_return_handoff_dev_20261015'
folder.mkdir(parents=True,exist_ok=True)
(folder/'step3_global_acquirer.cpp').write_text(new)
(folder/'isolated_change.patch').write_text('\n'.join(difflib.unified_diff(old.splitlines(),new.splitlines(),lineterm=''))+'\n')
(folder/'Dockerfile').write_text(f'''FROM {old_image}
COPY step3_global_acquirer.cpp {src}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
print('Isolated camera-return adaptation only',flush=True)
subprocess.run(['docker','build','-t',new_image,str(folder)],check=True)
