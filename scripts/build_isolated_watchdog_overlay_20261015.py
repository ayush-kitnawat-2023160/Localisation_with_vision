#!/usr/bin/env python3
"""Dev-only ROS health watchdog sidecar; no CAN, DBW or production edits.

FROM the frozen isolated fail-closed Step3 image. The output is a status topic,
NOT a physical stop actuator. It must never be claimed as live-cart safety.
"""
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[1]
base='fast_livo2_failsafe:failclosed-recovery-dev-20261015'
tag='fast_livo2_failsafe:failclosed-watchdog-dev-20261015'
context=root/'step3_global/localization_watchdog_dev_20261015'
context.mkdir(parents=True,exist_ok=True)
src=root/'scripts/localization_trust_watchdog.py'
(context/'localization_trust_watchdog.py').write_bytes(src.read_bytes())
(context/'Dockerfile').write_text(f'''FROM {base}
COPY localization_trust_watchdog.py /ws/src/Fast_Livo2/scripts/localization_trust_watchdog.py
RUN chmod 755 /ws/src/Fast_Livo2/scripts/localization_trust_watchdog.py
''')
original=(root/'launch/fast_livo2_step3_global_failclosed_recovery_dev.launch').read_text()
node='''
  <!-- Development status signal only. The cart has NO DBW actuator connected
       here; a real safety-rated control arbiter must consume stop_required. -->
  <node pkg="fast_livo2" type="localization_trust_watchdog.py"
        name="localization_trust_watchdog" output="screen" launch-prefix="nice -n 10">
    <param name="max_age_sec" value="0.5"/>
    <param name="max_speed_mps" value="5.0"/>
    <param name="max_yaw_rate_deg_s" value="120.0"/>
    <param name="rearm_samples" value="5"/>
    <param name="max_pose_gap_sec" value="0.6"/>
  </node>
'''
assert original.count('</launch>')==1
(root/'launch/fast_livo2_step3_global_failclosed_watchdog_dev.launch').write_text(
    original.replace('</launch>',node+'</launch>'))
print(f'Creating isolated ROS watchdog image {tag}',flush=True)
subprocess.run(['docker','build','-t',tag,str(context)],check=True)
