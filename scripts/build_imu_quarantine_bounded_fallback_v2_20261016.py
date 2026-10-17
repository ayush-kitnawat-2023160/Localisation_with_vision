#!/usr/bin/env python3
"""Causal IMU-only map-fallback velocity cap after absolute GICP predictor.
No map reference used to build this algorithm. New isolated Docker image."""
from pathlib import Path
import subprocess,difflib
R=Path(__file__).resolve().parents[1]
base='fast_livo2_failsafe:imu-map-quarantine-localization-dev-20261016'
tag='fast_livo2_failsafe:imu-map-quarantine-bounded-v2-dev-20261016'
target='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
code=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',base,target],text=True)
find='''        }else if(fast_local_unstable_.load()&&map_fix_fallback_active_&&map_fix_target_valid_){
          global_pose=mapFixFallbackPoseLocked(m->header.stamp);
          correction_ramp_active_=false;
          publish_recovery_active_=false;
'''
assert code.count(find)==1
replace='''        }else if(fast_local_unstable_.load()&&map_fix_fallback_active_&&map_fix_target_valid_){
          global_pose=mapFixFallbackPoseLocked(m->header.stamp);
          // In the specific IMU-loss quarantine only, constrain the
          // alternative verified-map-fix follower to the SAME physical
          // motion bound as the independent map-motion path above. The
          // old max-speed=12m/s follower created 1.2-1.4m published pose
          // steps at 10Hz when no fresh map-velocity candidate arrived.
          // No change to other modes, map anchor or FAST-LIVO2 ESIKF.
          if(imu_guard_map_motion_follow_enable_ && last_published_global_pose_valid_){
            bool imu_failed=false;
            { std::lock_guard<std::mutex> ml(mode_mtx_);
              imu_failed=(failsafe_mode_=="LV_IMU_FAILED");
            }
            if(imu_failed){
              const double dt=(m->header.stamp-last_published_global_stamp_).toSec();
              if(dt>0.001&&dt<=0.25){
                Eigen::Vector3d step=global_pose.translation()-last_published_global_pose_.translation();
                const double cap=std::max(0.1,imu_guard_map_motion_max_speed_mps_)*dt;
                const double n=step.norm();
                if(n>cap && n>1e-9){
                  step*=cap/n;
                  global_pose.translation()=last_published_global_pose_.translation()+step;
                  ROS_WARN_THROTTLE(0.5,
                    "[STEP3 IMU FALLBACK CAP] map correction limited to %.2fm over %.3fs",
                    cap,dt);
                }
              }
            }
          }
          correction_ramp_active_=false;
          publish_recovery_active_=false;
'''
mod=code.replace(find,replace)
work=R/'step3_global/imu_map_quarantine_bounded_v2_dev_20261016'
work.mkdir(parents=True,exist_ok=True)
(work/'step3_global_acquirer.cpp').write_text(mod)
(work/'change.diff').write_text('\n'.join(difflib.unified_diff(code.splitlines(),mod.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {base}
COPY step3_global_acquirer.cpp {target}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
src=(R/'launch/fast_livo2_imu_map_quarantine_dev_20261016.launch').read_text()
assert '<param name="imu_guard_map_motion_max_age_sec" value="1.80"/>' in src
src=src.replace('<param name="imu_guard_map_motion_max_age_sec" value="1.80"/>',
                '<param name="imu_guard_map_motion_max_age_sec" value="2.50"/>')
(R/'launch/fast_livo2_imu_map_quarantine_bounded_v2_dev_20261016.launch').write_text(src)
subprocess.run(['docker','build','-t',tag,'-f',str(work/'Dockerfile'),str(work)],check=True)
print('BUILT',tag,flush=True)
