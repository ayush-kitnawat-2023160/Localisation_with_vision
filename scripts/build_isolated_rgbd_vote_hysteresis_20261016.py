#!/usr/bin/env python3
"""Second isolated confidence-vote trial: bounded enter/exit hysteresis.

Two separate time limits: a newly established RGB-D vote must be very fresh
(<=0.55s), but once confirmed, one loaded callback gap may be tolerated up to
0.85s. Any longer silence *must* return to the original IMU-only fallback.
Only the LiDAR-outage selector changes; map matching and ESIKF are untouched.
"""
from pathlib import Path
import difflib
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:rgbd-mode-vote-dev-20261016'
IMAGE='fast_livo2_failsafe:rgbd-mode-vote-hysteresis-dev-20261016'
paths=['include/LIVMapper.h','src/LIVMapper.cpp']
srcs={k:subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,
                                  '/ws/src/Fast_Livo2/'+k],text=True) for k in paths}
new=dict(srcs)

def change(path,before,after):
    count=new[path].count(before)
    if count!=1:raise RuntimeError(f'{path} expected 1 match, saw {count}: {before[:110]}')
    new[path]=new[path].replace(before,after)

change('include/LIVMapper.h',
'''  double lidar_fail_rgbd_vote_max_age_sec_ = 0.55;
  int lidar_fail_rgbd_valid_delta_streak_ = 0;''',
'''  double lidar_fail_rgbd_vote_max_age_sec_ = 0.55;
  double lidar_fail_rgbd_vote_exit_age_sec_ = 0.85;
  bool lidar_fail_rgbd_vote_latched_ = false;
  int lidar_fail_rgbd_valid_delta_streak_ = 0;''')
change('src/LIVMapper.cpp',
'''  lidar_fail_rgbd_vote_max_age_sec_ =
      std::max(0.30, std::min(1.10, lidar_fail_rgbd_vote_max_age_sec_));''',
'''  lidar_fail_rgbd_vote_max_age_sec_ =
      std::max(0.30, std::min(0.60, lidar_fail_rgbd_vote_max_age_sec_));
  nh.param<double>("failsafe/lidar_fail_rgbd_vote_exit_age_sec",
                   lidar_fail_rgbd_vote_exit_age_sec_, 0.85);
  lidar_fail_rgbd_vote_exit_age_sec_ =
      std::max(lidar_fail_rgbd_vote_max_age_sec_ + 0.10,
               std::min(0.95,lidar_fail_rgbd_vote_exit_age_sec_));''')
change('src/LIVMapper.cpp',
'''      const bool independent_rgbd_ok=
          lidar_fail_rgbd_valid_delta_streak_ >= lidar_fail_rgbd_vote_min_valid_deltas_ &&
          age>=0.0 && age<=lidar_fail_rgbd_vote_max_age_sec_;
      if(independent_rgbd_ok && !visual_mode_ok)''',
'''      const bool enough_valid_deltas =
          lidar_fail_rgbd_valid_delta_streak_ >= lidar_fail_rgbd_vote_min_valid_deltas_;
      if(!lidar_fail_rgbd_vote_latched_){
        if(enough_valid_deltas && age>=0.0 &&
           age<=lidar_fail_rgbd_vote_max_age_sec_)
          lidar_fail_rgbd_vote_latched_=true;
      }else if(age<0.0 || age>lidar_fail_rgbd_vote_exit_age_sec_){
        lidar_fail_rgbd_vote_latched_=false;
        lidar_fail_rgbd_valid_delta_streak_=0;
        ROS_WARN_THROTTLE(1.0,"[FAILSAFE RGBD VOTE] sensor-time stale age %.2fs; IMU fallback allowed",age);
      }
      const bool independent_rgbd_ok=lidar_fail_rgbd_vote_latched_ &&
            enough_valid_deltas && age>=0.0 &&
            age<=lidar_fail_rgbd_vote_exit_age_sec_;
      if(independent_rgbd_ok && !visual_mode_ok)''')

folder=ROOT/'step3_global/rgbd_mode_vote_hysteresis_trial_20261016'
folder.mkdir(parents=True,exist_ok=True)
for path in paths:
    dest=folder/path
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(new[path])
    (folder/(Path(path).name+'.patch')).write_text('\n'.join(difflib.unified_diff(
        srcs[path].splitlines(),new[path].splitlines(),lineterm=''))+'\n')
(folder/'Dockerfile').write_text(f'''FROM {BASE}
COPY include/LIVMapper.h /ws/src/Fast_Livo2/include/LIVMapper.h
COPY src/LIVMapper.cpp /ws/src/Fast_Livo2/src/LIVMapper.cpp
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 fastlivo_mapping'
''')
cfg=(ROOT/'config/failsafe_rosbag_rgbd_vote_dev_20261016.yaml').read_text()
needle='  lidar_fail_rgbd_vote_max_age_sec: 0.55\n'
assert cfg.count(needle)==1
(ROOT/'config/failsafe_rosbag_rgbd_vote_hysteresis_dev_20261016.yaml').write_text(
    cfg.replace(needle,needle+'  lidar_fail_rgbd_vote_exit_age_sec: 0.85\n'))
print('Building isolated RGB-D confidence hysteresis',IMAGE,flush=True)
subprocess.run(['docker','build','-t',IMAGE,str(folder)],check=True)
