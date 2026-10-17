#!/usr/bin/env python3
"""Opt-in, physically slope-bounded LiDAR re-acquisition trial.

Construct only one isolated Step3 binary change atop the frozen healthy loop
image. A fixed 3m vertical delta from the LAST MAP FIX wrongly forbids
relocalization on a sloped route after a 40s outage, even when the saved map
has a geometrically sound LiDAR match. This trial permits recovery ONLY
after critical sensor returns, with strong ICP, bounded physical XY travel,
bounded heading and a 10% map-relative grade envelope (never GT lookup).
"""
import difflib
import subprocess
from pathlib import Path

root=Path(__file__).resolve().parents[1]
baseline='fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015'
target='fast_livo2_failsafe:slope-recovery-verified-dev-20261015'
src='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
s=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',baseline,src],text=True)
def repl(old,new):
    global s
    assert s.count(old)==1,(s.count(old),old[:100])
    s=s.replace(old,new)

repl('    pnh_.param("tracking_global_reacquire_physical_gate",tracking_global_reacquire_physical_gate_,true);',
     '    pnh_.param("tracking_global_reacquire_physical_gate",tracking_global_reacquire_physical_gate_,true);\n'
     '    pnh_.param("tracking_global_reacquire_slope_guard_enable",tracking_global_reacquire_slope_guard_enable_,false);')
repl('  bool tracking_global_reacquire_physical_gate_=true;',
     '  bool tracking_global_reacquire_physical_gate_=true;\n'
     '  bool tracking_global_reacquire_slope_guard_enable_=false;')
repl('          branch_ok=map_motion<=map_limit&&map_yaw_deg<=yaw_limit&&vertical<=vertical_limit;\n'
     '          if(valid&&!branch_ok)',
'''          branch_ok=map_motion<=map_limit&&map_yaw_deg<=yaw_limit&&vertical<=vertical_limit;
          // An IMU-only bridge after LiDAR loss may travel >3 vertical metres
          // ON THE ACTUAL SLOPED ROAD. The last map pose is then old; a fixed
          // 3m Z-distance veto can prevent recovery forever. Opt-in only when
          // the critical sensor has returned, geometry is unambiguous, enough
          // time passed for terrain change, and map-relative grade remains
          // compatible with a real road (max 10% plus 1m sampling slack).
          // All original XY / heading / GICP gates and multi-scan temporal
          // consistency still apply. Never use evaluation reference or GPS.
          if(!branch_ok && tracking_global_reacquire_slope_guard_enable_ &&
             publish_critical_sensor_recovered_.load() && age>=20.0 &&
             x.fitness<=0.075 && std::isfinite(map_motion) &&
             map_motion<=map_limit && map_yaw_deg<=yaw_limit &&
             std::isfinite(vertical)){
            const double grade_limit=std::min(12.0,std::max(vertical_limit,1.0+0.10*map_motion));
            if(vertical<=grade_limit){
              branch_ok=true;
              ROS_WARN("[STEP3 SLOPE-RECOVERY] candidate=%d ICP=%.4f dz=%.2f/%.2fm xy=%.2f/%.2fm: grade-bounded recovery hypothesis",
                       id,x.fitness,vertical,grade_limit,map_motion,map_limit);
            }
          }
          if(valid&&!branch_ok)''')
context=root/'step3_global/slope_recovery_verified_dev_20261015'
context.mkdir(parents=True,exist_ok=True)
(context/'step3_global_acquirer.cpp').write_text(s)
(context/'Dockerfile').write_text(f'''FROM {baseline}
COPY step3_global_acquirer.cpp {src}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
orig=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',baseline,src],text=True)
(context/'isolated_change.patch').write_text('\n'.join(difflib.unified_diff(orig.splitlines(),s.splitlines(),lineterm=''))+'\n')
original_launch=(root/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
needle='    <param name="tracking_healthy_loop_motion_bridge_enable" value="true"/>'
assert original_launch.count(needle)==1
(root/'launch/fast_livo2_step3_global_slope_recovery_dev.launch').write_text(original_launch.replace(
    needle,needle+'\n'
    '    <!-- Opt-in map-grade gate after critical sensor recovery, never healthy. -->\n'
    '    <param name="tracking_global_reacquire_slope_guard_enable" value="true"/>'))
print('Building isolated slope trial:',target,flush=True)
subprocess.run(['docker','build','-t',target,str(context)],check=True)
