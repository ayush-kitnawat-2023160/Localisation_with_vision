#!/usr/bin/env python3
"""Isolated depth registration rate gate using sensor-time (not ROS callback wall clock).
Never changes map matching thresholds, vehicle control, or reference fitting.
Opt-in, default unchanged. Test independently on main and Sep18 LiDAR loss.
"""
from pathlib import Path
import subprocess,difflib
r=Path(__file__).resolve().parents[1]
base='fast_livo2_failsafe:single-build-rc-20261016'
tag='fast_livo2_failsafe:depth-source-clock-gate-localization-dev-20261017'
src='/ws/src/Fast_Livo2/src/failsafe_map_localizer.cpp'
original=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',base,src],text=True)
now=original
def patch_one(before,after):
 global now
 assert now.count(before)==1,('MISSING ANCHOR',before[:110],now.count(before))
 now=now.replace(before,after)
patch_one('''    pnh_.param<bool>("depth_failover_only", depth_failover_only_, false);''',
'''    pnh_.param<bool>("depth_failover_only", depth_failover_only_, false);
    // In 1x replay, callback scheduling can vary by hundreds of ms with GICP
    // load. A rate limiter based on ros::Time::now() chooses DIFFERENT map
    // observations on every run. Sensor timestamps are the causal clock.
    pnh_.param<bool>("depth_source_time_gate_enable",
                     depth_source_time_gate_enable_, false);''')
patch_one('''    if (!last_depth_localize_.isZero() &&
        (now - last_depth_localize_).toSec() <
            1.0 / std::max(0.5, depth_localization_hz_))
      return;
    last_depth_localize_ = now;''',
'''    // Use normalized source frame time for the depth-GICP observation rate.
    // This changes ONLY which causal depth frames are evaluated; it does not
    // modify a pose, transform, ESIKF state, or registration fitness.
    const ros::Time map_gate_stamp =
        depth_source_time_gate_enable_ ? depth_stamp : now;
    if (!last_depth_localize_.isZero())
    {
      const double dt=(map_gate_stamp-last_depth_localize_).toSec();
      if (dt <= 0.0)
      {
        ROS_WARN_THROTTLE(1.0,
           "[DEPTH SOURCE CLOCK] stale/out-of-order depth stamp rejected");
        return;
      }
      if (dt < 1.0 / std::max(0.5, depth_localization_hz_))
        return;
    }
    last_depth_localize_=map_gate_stamp;
    if (depth_source_time_gate_enable_)
      ROS_INFO_THROTTLE(1.0,
         "[DEPTH SOURCE CLOCK] rate-selected depth source stamp %.6f",depth_stamp.toSec());''')
patch_one('''  bool depth_failover_only_ = false;''',
'''  bool depth_failover_only_ = false;
  bool depth_source_time_gate_enable_ = false;''')
work=r/'step3_global/depth_source_clock_localization_dev_20261017'
work.mkdir(parents=True,exist_ok=True)
(work/'failsafe_map_localizer.cpp').write_text(now)
(work/'change.diff').write_text('\n'.join(difflib.unified_diff(original.splitlines(),now.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {base}
COPY failsafe_map_localizer.cpp {src}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 failsafe_map_localizer'
''')
config=(r/'config/map_failsafe_recorded_cart.yaml').read_text()
assert 'depth_source_time_gate_enable:' not in config
config+='\n# Development-only deterministic source-time depth registration throttle.\ndepth_source_time_gate_enable: true\n'
(r/'config/map_failsafe_depth_source_clock_dev_20261017.yaml').write_text(config)
print('PREPARED',work,'source length',len(original),len(now),flush=True)
subprocess.run(['docker','build','-t',tag,'-f',str(work/'Dockerfile'),str(work)],check=True)
print('BUILT',tag,flush=True)
