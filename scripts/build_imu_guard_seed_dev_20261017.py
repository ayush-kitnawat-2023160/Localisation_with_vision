#!/usr/bin/env python3
"""Isolated opt-in IMU-loss quarantine seed, no vehicle controllers or GT."""
from pathlib import Path
import subprocess,difflib
R=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:single-build-rc-20261016'
TAG='fast_livo2_failsafe:imu-guard-verified-map-seed-dev-20261017'
TARGET='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
old=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,TARGET],text=True)
new=old
def replace_exact(a,b):
 global new
 n=new.count(a)
 if n!=1: raise ValueError(f'exact match count={n} for {a[:100]}')
 new=new.replace(a,b)
replace_exact('    pnh_.param("map_fix_fallback_follow_tau_sec",map_fix_fallback_follow_tau_sec_,0.35);',
'''    pnh_.param("map_fix_fallback_follow_tau_sec",map_fix_fallback_follow_tau_sec_,0.35);
    // Opt-in: bootstrap IMU-failure quarantine from a *recent actual*
    // LiDAR/PCD registration. NEVER seed from the rejected local ESIKF jump.
    pnh_.param("imu_guard_verified_seed_enable",imu_guard_verified_seed_enable_,false);
    pnh_.param("imu_guard_verified_seed_max_age_sec",imu_guard_verified_seed_max_age_sec_,1.0);''')
replace_exact('  double map_fix_fallback_follow_tau_sec_=0.35,map_fix_fallback_max_extrapolation_sec_=1.2,map_fix_fallback_max_speed_mps_=12.0,map_fix_fallback_max_yaw_rate_deg_s_=90.0;',
'''  double map_fix_fallback_follow_tau_sec_=0.35,map_fix_fallback_max_extrapolation_sec_=1.2,map_fix_fallback_max_speed_mps_=12.0,map_fix_fallback_max_yaw_rate_deg_s_=90.0;
  bool imu_guard_verified_seed_enable_=false;
  double imu_guard_verified_seed_max_age_sec_=1.0;''')
replace_exact('''          }else{
            if(!fast_local_unstable_.load()){
              if(bad_gap)''', '''          }else{
            // FIRST bad FAST increment during an IMU-only outage: the last
            // verified saved-map LiDAR correction is typically <1s old. The
            // old publisher froze its global pose while a full ScanContext
            // search spent multiple seconds finding the SAME road section.
            // Begin quarantine from that prior *independent* map observation
            // with a short constant-velocity physical bound instead. A recent
            // map observation is mandatory; stale fixes cannot start this
            // predictor. No change to the persistent FAST-LIVO2 ESIKF.
            if(imu_guard_verified_seed_enable_ && !fast_local_unstable_.load()){
              bool imu_failed=false;
              {std::lock_guard<std::mutex> ml(mode_mtx_);
               imu_failed=(failsafe_mode_=="LV_IMU_FAILED");}
              Eigen::Isometry3d seed=Eigen::Isometry3d::Identity();
              double age=std::numeric_limits<double>::infinity();
              if(imu_failed && last_tracking_map_valid_ &&
                 trackingMotionPrediction(m->header.stamp,seed,age) &&
                 age>=0.0 && age<=imu_guard_verified_seed_max_age_sec_){
                Eigen::Vector3d v=Eigen::Vector3d::Zero();
                double wr=0.0;
                {std::lock_guard<std::mutex> vm(tracking_motion_mtx_);
                 v=tracking_motion_velocity_;
                 wr=tracking_motion_yaw_rate_;}
                if(v.allFinite() && std::isfinite(wr)){
                  const double speed=v.norm();
                  if(speed>map_fix_fallback_max_speed_mps_ && speed>1e-9)
                    v*=map_fix_fallback_max_speed_mps_/speed;
                  const double wcap=map_fix_fallback_max_yaw_rate_deg_s_*M_PI/180.0;
                  wr=std::max(-wcap,std::min(wcap,wr));
                  {std::lock_guard<std::mutex> a(anchor_mtx_);
                   updateMapFixFallbackTargetLocked(seed,m->header.stamp);
                   map_fix_velocity_=v;
                   map_fix_yaw_rate_=wr;
                   correction_ramp_active_=false;
                   publish_recovery_active_=false;}
                  ROS_WARN("[STEP3 IMU-QUARANTINE-SEED] fresh verified map age=%.3fs velocity=%.2fm/s; local FAST jump rejected",
                      age,v.norm());
                }
              }else if(imu_failed){
                ROS_WARN_THROTTLE(1.0,"[STEP3 IMU-QUARANTINE-SEED] no fresh trusted map seed; hold instead of extrapolating");
              }
            }
            if(!fast_local_unstable_.load()){
              if(bad_gap)''')
work=R/'step3_global/imu_guard_verified_seed_dev_20261017'
work.mkdir(parents=True,exist_ok=True)
(work/'step3_global_acquirer.cpp').write_text(new)
(work/'source.diff').write_text('\n'.join(difflib.unified_diff(old.splitlines(),new.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {BASE}\nCOPY step3_global_acquirer.cpp {TARGET}\nRUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'\n''')
launch=(R/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
anchor='    <param name="tracking_hz" value="1.0"/>'
assert launch.count(anchor)==1
launch=launch.replace(anchor,anchor+'''
    <!-- Opt-in IMU-loss source fix; never applied to healthy/car/brake code. -->
    <param name="imu_guard_verified_seed_enable" value="true"/>
    <param name="imu_guard_verified_seed_max_age_sec" value="1.0"/>
    <param name="map_fix_fallback_max_extrapolation_sec" value="1.8"/>''')
(R/'launch/fast_livo2_imu_guard_verified_seed_dev_20261017.launch').write_text(launch)
print(f'Wrote isolated C++/launch, {len(new)} source chars, {len((work/"source.diff").read_text())} diff chars, no runtime touched.')
