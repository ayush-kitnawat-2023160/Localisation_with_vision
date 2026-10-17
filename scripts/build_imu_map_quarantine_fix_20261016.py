#!/usr/bin/env python3
"""One-localization-change experimental image from frozen 8-case baseline.

During a FAST local quarantine triggered by an implausible IMU-loss local
motion increment, keep following fresh REAL LiDAR/PCD map fixes. The old code
froze the published map pose for seconds despite successful 1 Hz PCD matches.
Never use the loopclosed reference for code, settings, seeds, or thresholds.
"""
from pathlib import Path
import difflib,subprocess
R=Path(__file__).resolve().parents[1]
original='fast_livo2_failsafe:single-build-rc-20261016'
image='fast_livo2_failsafe:imu-map-quarantine-localization-dev-20261016'
target='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
code=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',original,target],text=True)
new=code
def once(a,b):
 global new
 n=new.count(a)
 if n!=1:raise ValueError((n,a[:95]))
 new=new.replace(a,b)
once('    pnh_.param("map_fix_fallback_follow_tau_sec",map_fix_fallback_follow_tau_sec_,0.35);',
'''    pnh_.param("map_fix_fallback_follow_tau_sec",map_fix_fallback_follow_tau_sec_,0.35);
    // The IMU-failure FAST guard cannot freeze 10Hz map pose for 3-5s if
    // a completely independent 1Hz LiDAR/PCD GICP pose is still valid.
    pnh_.param("imu_guard_map_motion_follow_enable",imu_guard_map_motion_follow_enable_,false);
    pnh_.param("imu_guard_map_motion_max_age_sec",imu_guard_map_motion_max_age_sec_,1.8);
    pnh_.param("imu_guard_map_motion_max_speed_mps",imu_guard_map_motion_max_speed_mps_,7.5);
    pnh_.param("imu_guard_map_motion_tau_sec",imu_guard_map_motion_tau_sec_,0.12);''')
once('  double map_fix_fallback_follow_tau_sec_=0.35,map_fix_fallback_max_extrapolation_sec_=1.2,map_fix_fallback_max_speed_mps_=12.0,map_fix_fallback_max_yaw_rate_deg_s_=90.0;',
'''  double map_fix_fallback_follow_tau_sec_=0.35,map_fix_fallback_max_extrapolation_sec_=1.2,map_fix_fallback_max_speed_mps_=12.0,map_fix_fallback_max_yaw_rate_deg_s_=90.0;
  bool imu_guard_map_motion_follow_enable_=false;
  double imu_guard_map_motion_max_age_sec_=1.8,imu_guard_map_motion_max_speed_mps_=7.5,imu_guard_map_motion_tau_sec_=0.12;''')
once('''        if(fast_local_unstable_.load()&&map_fix_fallback_active_&&map_fix_target_valid_){
          global_pose=mapFixFallbackPoseLocked(m->header.stamp);''','''        // IMU-LOSS ROOT-CAUSE FIX, localised to FAST's fault quarantine:
        // The original branch held last_published_global_pose for seconds
        // even while fresh independent LiDAR/PCD GICP observations continued.
        // At ~131-134s on the main rosbag this generated a 14m discrepancy
        // without any corresponding GICP registration error.
        //
        // Follow ONLY the independent map-motion predictor built from real
        // accepted PCD matches; never advance on raw FAST odometry while
        // quarantined. Limit map correction in PHYSICAL metres per sample,
        // not by resetting a smoothing window on every new GICP fix.
        bool imu_guard_map_motion_following=false;
        if(imu_guard_map_motion_follow_enable_ && fast_local_unstable_.load() &&
           last_published_global_pose_valid_ && !last_published_global_stamp_.isZero()){
          bool imu_failed=false;
          { std::lock_guard<std::mutex> ml(mode_mtx_);
            imu_failed=(failsafe_mode_=="LV_IMU_FAILED");
          }
          const double publish_dt=(m->header.stamp-last_published_global_stamp_).toSec();
          if(imu_failed && publish_dt>0.001 && publish_dt<=0.25){
            Eigen::Isometry3d independent_map_pose=Eigen::Isometry3d::Identity();
            double independent_age=0.0;
            if(trackingMotionPrediction(m->header.stamp,independent_map_pose,independent_age) &&
               independent_age<=imu_guard_map_motion_max_age_sec_){
              const double weight=std::max(0.0,std::min(1.0,
                1.0-std::exp(-publish_dt/std::max(0.04,imu_guard_map_motion_tau_sec_))));
              Eigen::Vector3d step=weight*(independent_map_pose.translation()-
                    last_published_global_pose_.translation());
              const double max_step=std::max(0.1,imu_guard_map_motion_max_speed_mps_)*publish_dt;
              if(step.norm()>max_step)step*=max_step/step.norm();
              global_pose=last_published_global_pose_;
              global_pose.translation()+=step;
              Eigen::Quaterniond from(last_published_global_pose_.linear());
              Eigen::Quaterniond to(independent_map_pose.linear());
              from.normalize();to.normalize();
              const double yaw_delta=wrap(yawOf(independent_map_pose)-
                                            yawOf(last_published_global_pose_));
              const double yaw_limit=35.0*M_PI/180.0*publish_dt;
              const double yaw_applied=std::max(-yaw_limit,std::min(yaw_limit,weight*yaw_delta));
              global_pose.linear()=Eigen::AngleAxisd(yaw_applied,
                          Eigen::Vector3d::UnitZ()).toRotationMatrix()*
                          last_published_global_pose_.linear();
              correction_ramp_active_=false;
              publish_recovery_active_=false;
              imu_guard_map_motion_following=true;
              ROS_WARN_THROTTLE(0.5,
                "[STEP3 IMU MAP-QUARANTINE] held local ESIKF quarantined; "
                "follow fresh independent PCD pose age %.2fs step %.2fm",
                independent_age,step.norm());
            }
          }
        }
        if(imu_guard_map_motion_following){
          // No FAST pose or artificially held map pose can overwrite the
          // verified GICP-based publication while this short guard is active.
        }else if(fast_local_unstable_.load()&&map_fix_fallback_active_&&map_fix_target_valid_){
          global_pose=mapFixFallbackPoseLocked(m->header.stamp);''')
work=R/'step3_global/imu_map_quarantine_localization_dev_20261016'
work.mkdir(parents=True,exist_ok=True)
(work/'step3_global_acquirer.cpp').write_text(new)
(work/'change.diff').write_text('\n'.join(difflib.unified_diff(code.splitlines(),new.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {original}
COPY step3_global_acquirer.cpp {target}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
launch=(R/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
anchor='    <param name="tracking_publish_healthy_observer_enable" value="true"/>'
assert launch.count(anchor)==1
launch=launch.replace(anchor,anchor+'''
    <!-- Experimental: hold IMU-loss FAST ESIKF quarantine on fresh accepted
         map matches instead of freezing the global pose for several seconds.
         Does not affect healthy, camera fail, or LiDAR fail. -->
    <param name="imu_guard_map_motion_follow_enable" value="true"/>
    <param name="imu_guard_map_motion_max_age_sec" value="1.80"/>
    <param name="imu_guard_map_motion_max_speed_mps" value="7.50"/>
    <param name="imu_guard_map_motion_tau_sec" value="0.12"/>''')
(R/'launch/fast_livo2_imu_map_quarantine_dev_20261016.launch').write_text(launch)
print('Source diff saved',work/'change.diff',flush=True)
subprocess.run(['docker','build','-t',image,'-f',str(work/'Dockerfile'),str(work)],check=True)
print('BUILT',image,flush=True)
