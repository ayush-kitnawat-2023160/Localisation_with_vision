#!/usr/bin/env python3
"""One-feature-only Step-3 image: verify cross-keyframe GICP with relative motion.

Patch the frozen 2026-10-14 healthy C++ *inside an isolated build context*.
Never recompile all in-flight source changes or modify the production image.
Opt-in via private launch param (defaults false).
"""
import difflib
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
baseline='fast_livo2_failsafe:healthy-gap-handoff-dev-20261014'
tag='fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015'
source_path='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
baseline_code=subprocess.check_output(
    ['docker','run','--rm','--entrypoint','cat',baseline,source_path],text=True)

def replace_once(code,old,new):
    assert code.count(old)==1,(old[:60],code.count(old))
    return code.replace(old,new)

code=baseline_code
code=replace_once(code,
    '    pnh_.param("tracking_healthy_motion_consistency_gate",tracking_healthy_motion_consistency_gate_,false);',
    '    pnh_.param("tracking_healthy_motion_consistency_gate",tracking_healthy_motion_consistency_gate_,false);\n'
    '    pnh_.param("tracking_healthy_loop_motion_bridge_enable",tracking_healthy_loop_motion_bridge_enable_,false);')
code=replace_once(code,
    '  bool tracking_healthy_motion_consistency_gate_=false;',
    '  bool tracking_healthy_motion_consistency_gate_=false;\n'
    '  bool tracking_healthy_loop_motion_bridge_enable_=false;')
anchor='      bool motion_consistent=true;\n'
new_gate='''      // Opt-in for overlapping loops: map keyframe IDs are NOT road distances.
      // Only a very strong registration with small pose innovation can cross
      // the ID window, and it must independently reproduce the SAME RELATIVE
      // vehicle motion as the FAST estimator since the previous PCD fix.
      // Unlike a simple yaw threshold increase this does not admit a wrong
      // nearby branch with incompatible travel / rotation. Never use GT.
      bool loop_motion_bridge=false;
      if(tracking_healthy_loop_motion_bridge_enable_ &&
         publish_full_sensor_mode_.load() &&
         !publish_critical_sensor_recovered_.load() &&
         !reacquire_mode && !fast_local_unstable_.load() &&
         last_tracking_fast_valid_ && last_tracking_map_valid_ &&
         id_jump>id_free_window &&
         x.id>=0 && std::isfinite(x.fitness) && x.fitness<=0.095 &&
         std::isfinite(xi) && xi<=1.25 &&
         std::isfinite(yi) && yi<=14.0){
        const double elapsed=(m->header.stamp-last_tracking_map_stamp_).toSec();
        if(elapsed>0.3 && elapsed<=1.8){
          const Eigen::Isometry3d fast_delta=last_tracking_fast_body_.inverse()*fast_body;
          const Eigen::Isometry3d map_delta=last_tracking_map_body_.inverse()*x_body;
          const double dist_error=(map_delta.translation()-fast_delta.translation()).head<2>().norm();
          const double yaw_error=std::abs(wrap(yawOf(map_delta)-yawOf(fast_delta)))*180.0/M_PI;
          // Both a small relative translation and a consistent turn are
          // necessary; a good ICP fitness alone is not enough at a crossing.
          loop_motion_bridge=std::isfinite(dist_error)&&dist_error<=1.10 &&
                             std::isfinite(yaw_error)&&yaw_error<=8.0;
          ROS_INFO("[STEP3 LOOP-MOTION] candidate=%d fit=%.4f innov=%.2fm/%.1fdeg relative_xy=%.2fm relative_yaw=%.1fdeg accepted=%d",
                   id,x.fitness,xi,yi,dist_error,yaw_error,loop_motion_bridge?1:0);
        }
      }
      bool motion_consistent=true;
'''
code=replace_once(code,anchor,new_gate)
code=replace_once(code,
    '      if(motion_consistent && (normal_gate||relaxed_yaw_gate||route_continuous_geometry_gate)){',
    '      if(motion_consistent && (normal_gate||relaxed_yaw_gate||route_continuous_geometry_gate||loop_motion_bridge)){')
code=replace_once(code,
    '        if(relaxed_yaw_gate&&!normal_gate)',
    '        if(loop_motion_bridge&&!normal_gate&&!relaxed_yaw_gate&&!route_continuous_geometry_gate)\n'
    '          ROS_WARN("[STEP3 LOOP-MOTION] ACCEPT strong geometry across saved-map branch id=%d fitness=%.4f",id,x.fitness);\n'
    '        else if(relaxed_yaw_gate&&!normal_gate)')
assert code.count('loop_motion_bridge')>=5

context=root/'step3_global/healthy_loop_motion_bridge_dev_20261015'
context.mkdir(parents=True,exist_ok=True)
(context/'step3_global_acquirer.cpp').write_text(code)
(context/'Dockerfile').write_text(f'''FROM {baseline}
COPY step3_global_acquirer.cpp {source_path}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
diff=list(difflib.unified_diff(baseline_code.splitlines(),code.splitlines(),
    fromfile='frozen_healthy',tofile='loop_motion_bridge',lineterm=''))
(context/'isolated_change.patch').write_text('\n'.join(diff)+'\n')
launch_src=root/'launch/fast_livo2_step3_global_healthy_gap_handoff_dev.launch'
launch_text=launch_src.read_text()
launch_text=replace_once(launch_text,
    '    <param name="tracking_candidates" value="3"/>',
    '    <param name="tracking_candidates" value="3"/>\n'
    '    <param name="tracking_healthy_loop_motion_bridge_enable" value="true"/>')
(root/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').write_text(launch_text)
print(f'Building {tag}: isolated opt-in GICP gate only',flush=True)
subprocess.run(['docker','build','-t',tag,str(context)],check=True)
