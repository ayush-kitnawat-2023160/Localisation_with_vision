#!/usr/bin/env python3
"""Opt-in, isolated RGB-D odometry confidence vote for LiDAR-loss FSM.

FAST's native camera confidence can briefly dip when visual map features are
LiDAR-supported, even while separate metric RGB-D PnP delivers geometrically
validated SE(3) deltas into the SAME ESIKF. Require three successive bounded
RGB-D deltas with fresh sensor stamps before allowing the VI_LIDAR_FAILED
mode to remain active. Do not invent a pose, override a saved-map match,
modify healthy mode, or relax sensor failure checks. Defaults unchanged.
"""
from pathlib import Path
import subprocess
import difflib

ROOT=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:experimental-continuity-retry-20261013'
IMAGE='fast_livo2_failsafe:rgbd-mode-vote-dev-20261016'
FILES=['include/LIVMapper.h','src/LIVMapper.cpp']
paths={}
for path in FILES:
    paths[path]=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,'/ws/src/Fast_Livo2/'+path],text=True)
new=dict(paths)

def change(path,needle,replacement):
    x=new[path]
    if x.count(needle)!=1:
        raise RuntimeError(f'{path}: expected one anchor, found {x.count(needle)}: {needle[:160]}')
    new[path]=x.replace(needle,replacement)

change('include/LIVMapper.h','  ros::Time last_lidar_relative_stamp_, last_depth_relative_stamp_,\n      last_rgbd_relative_stamp_;',
'''  ros::Time last_lidar_relative_stamp_, last_depth_relative_stamp_,
      last_rgbd_relative_stamp_;
  // Experimental *relative VO health*, NEVER proof of global map position.
  bool lidar_fail_rgbd_mode_vote_enable_ = false;
  int lidar_fail_rgbd_vote_min_valid_deltas_ = 3;
  double lidar_fail_rgbd_vote_max_age_sec_ = 0.55;
  int lidar_fail_rgbd_valid_delta_streak_ = 0;
  ros::Time lidar_fail_rgbd_last_valid_delta_stamp_;
''')
change('src/LIVMapper.cpp','  nh.param<double>("failsafe/imu_timeout", imu_timeout_, 0.2);',
'''  nh.param<double>("failsafe/imu_timeout", imu_timeout_, 0.2);
  nh.param<bool>("failsafe/lidar_fail_rgbd_mode_vote_enable",
                 lidar_fail_rgbd_mode_vote_enable_, false);
  nh.param<int>("failsafe/lidar_fail_rgbd_vote_min_valid_deltas",
                lidar_fail_rgbd_vote_min_valid_deltas_, 3);
  nh.param<double>("failsafe/lidar_fail_rgbd_vote_max_age_sec",
                   lidar_fail_rgbd_vote_max_age_sec_, 0.55);
  lidar_fail_rgbd_vote_min_valid_deltas_ =
      std::max(2, std::min(6, lidar_fail_rgbd_vote_min_valid_deltas_));
  lidar_fail_rgbd_vote_max_age_sec_ =
      std::max(0.30, std::min(1.10, lidar_fail_rgbd_vote_max_age_sec_));''')
change('src/LIVMapper.cpp',
'''    else if (failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE)
      visual_mode_ok = camera_confidence_ >= 0.55;
    setFailsafeMode(visual_mode_ok ? FailsafeMode::VI_LIDAR_FAILED''',
'''    else if (failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE)
      visual_mode_ok = camera_confidence_ >= 0.55;
    // The native camera/photometric score can temporarily degrade when its
    // LiDAR-derived visual geometry becomes stale. When LiDAR is missing,
    // the independent metric RGB-D frontend already supplies accepted,
    // filtered PnP SE(3) deltas directly to this same ESIKF. Accept a
    // SECOND confidence vote only after >=3 distinct valid delta stamps,
    // all recent in sensor time. A stale/invalid stream immediately loses
    // this vote and preserves original IMU_ONLY_BRIDGE behavior.
    if(lidar_fail_rgbd_mode_vote_enable_){
      const double age=lidar_fail_rgbd_last_valid_delta_stamp_.isZero()
        ? 1.0e9 : now-lidar_fail_rgbd_last_valid_delta_stamp_.toSec();
      const bool independent_rgbd_ok=
          lidar_fail_rgbd_valid_delta_streak_ >= lidar_fail_rgbd_vote_min_valid_deltas_ &&
          age>=0.0 && age<=lidar_fail_rgbd_vote_max_age_sec_;
      if(independent_rgbd_ok && !visual_mode_ok)
        ROS_INFO_THROTTLE(1.0,"[FAILSAFE RGBD VOTE] keeping visual/IMU; recent valid delta age %.2fs streak=%d native_conf=%.2f",
                          age,lidar_fail_rgbd_valid_delta_streak_,camera_confidence_);
      visual_mode_ok=visual_mode_ok||independent_rgbd_ok;
    }
    setFailsafeMode(visual_mode_ok ? FailsafeMode::VI_LIDAR_FAILED''')
change('src/LIVMapper.cpp',
'''    if (!delta_t.allFinite() || !delta_rvec.allFinite() ||
        delta_t.norm() > 2.5 || delta_angle > 0.80)
      return;

    M3D delta_R = M3D::Identity();''',
'''    if (!delta_t.allFinite() || !delta_rvec.allFinite() ||
        delta_t.norm() > 2.5 || delta_angle > 0.80)
      return;
    // Count only sequential, bounded, PnP-filtered relative RGB-D frames;
    // the first packet cannot earn a confidence vote. Streak naturally ages
    // out in the mode evaluator even if no more callbacks arrive.
    if(lidar_fail_rgbd_mode_vote_enable_ && rgbd_delta){
      const double dt=lidar_fail_rgbd_last_valid_delta_stamp_.isZero()?
          -1.0 : (msg->header.stamp-lidar_fail_rgbd_last_valid_delta_stamp_).toSec();
      if(dt>0.025 && dt<0.40)
        lidar_fail_rgbd_valid_delta_streak_=
            std::min(8,lidar_fail_rgbd_valid_delta_streak_+1);
      else
        lidar_fail_rgbd_valid_delta_streak_=1;
      if(dt>0.025 || lidar_fail_rgbd_last_valid_delta_stamp_.isZero())
        lidar_fail_rgbd_last_valid_delta_stamp_=msg->header.stamp;
    }

    M3D delta_R = M3D::Identity();''')

folder=ROOT/'step3_global/rgbd_mode_vote_trial_20261016'
folder.mkdir(parents=True,exist_ok=True)
for path in FILES:
 dest=folder/path
 dest.parent.mkdir(parents=True,exist_ok=True)
 dest.write_text(new[path])
 (folder/(Path(path).name+'.patch')).write_text('\n'.join(difflib.unified_diff(
  paths[path].splitlines(),new[path].splitlines(),fromfile='baseline/'+path,
  tofile='candidate/'+path,lineterm=''))+'\n')
(folder/'Dockerfile').write_text('''FROM '''+BASE+'''
COPY include/LIVMapper.h /ws/src/Fast_Livo2/include/LIVMapper.h
COPY src/LIVMapper.cpp /ws/src/Fast_Livo2/src/LIVMapper.cpp
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 fastlivo_mapping'
''')
original=(ROOT/'config/failsafe_rosbag.yaml').read_text()
key='  lidar_fail_covariance_scale: '
assert original.count(key)==1
original=original.replace(key,
'''  # Opt-in, sensor-time-bounded RGB-D confidence vote for LiDAR-loss mode.
  lidar_fail_rgbd_mode_vote_enable: true
  lidar_fail_rgbd_vote_min_valid_deltas: 3
  lidar_fail_rgbd_vote_max_age_sec: 0.55
'''+key)
(ROOT/'config/failsafe_rosbag_rgbd_vote_dev_20261016.yaml').write_text(original)
print('Building isolated RGB-D mode confidence vote',IMAGE,flush=True)
subprocess.run(['docker','build','-t',IMAGE,str(folder)],check=True)
