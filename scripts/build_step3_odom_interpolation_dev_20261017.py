#!/usr/bin/env python3
"""Isolated, opt-in, strictly time-bracketed SE3 pose interpolation.
No GT in source or config. No image synthesis, ESIKF or cart control changes.
Do NOT compile while another 1x ROSBAG replay is running."""
from pathlib import Path
import subprocess,difflib
R=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:single-build-rc-20261016'
TAG='fast_livo2_failsafe:step3-odom-interpolated-localization-dev-20261017'
target='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
old=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,target],text=True)
new=old
def replace_one(a,b):
 global new
 n=new.count(a)
 assert n==1,(n,a[:90])
 new=new.replace(a,b)
replace_one('    pnh_.param("tracking_hz",tracking_hz_,1.0);',
'''    pnh_.param("tracking_hz",tracking_hz_,1.0);
    pnh_.param("step3_odom_se3_interpolation_enable",
               step3_odom_se3_interpolation_enable_,false);
    pnh_.param("step3_odom_se3_max_bracket_sec",
               step3_odom_se3_max_bracket_sec_,0.30);
    pnh_.param("step3_odom_se3_nearest_grace_sec",
               step3_odom_se3_nearest_grace_sec_,0.045);''')
replace_one('  double tracking_hz_=1.0,tracking_max_fitness_=0.20,tracking_max_translation_innovation_=3.0,tracking_max_yaw_innovation_deg_=10.0;',
'''  double tracking_hz_=1.0,tracking_max_fitness_=0.20,tracking_max_translation_innovation_=3.0,tracking_max_yaw_innovation_deg_=10.0;
  bool step3_odom_se3_interpolation_enable_=false;
  double step3_odom_se3_max_bracket_sec_=0.30,step3_odom_se3_nearest_grace_sec_=0.045;''')
replace_one('  bool odomAt(const ros::Time&t,Eigen::Isometry3d&T){std::lock_guard<std::mutex>l(odom_mtx_);if(odom_.empty())return false;auto it=std::min_element(odom_.begin(),odom_.end(),[&](auto&a,auto&b){return std::abs((a.first-t).toSec())<std::abs((b.first-t).toSec());});if(std::abs((it->first-t).toSec())>.15)return false;T=it->second;return true;}',
'''  bool odomAt(const ros::Time&t,Eigen::Isometry3d&T){
    std::lock_guard<std::mutex>l(odom_mtx_);
    if(odom_.empty())return false;
    if(!step3_odom_se3_interpolation_enable_){
      auto it=std::min_element(odom_.begin(),odom_.end(),[&](auto&a,auto&b){
        return std::abs((a.first-t).toSec())<std::abs((b.first-t).toSec());});
      if(std::abs((it->first-t).toSec())>.15)return false;
      T=it->second;return true;
    }
    // Use physically correct measurement-time motion rather than choosing a
    // LiDAR-to-FAST transform up to 150ms away. At 5m/s that could bias the
    // map association by 0.75m even when GICP itself converges.
    // Only interpolate between already-received ESIKF poses. NEVER infer
    // future poses, extrapolate stale state, or use the loop-closed reference.
    auto it=std::lower_bound(odom_.begin(),odom_.end(),t,
       [](const std::pair<ros::Time,Eigen::Isometry3d>&a,const ros::Time&b){
           return a.first<b;});
    if(it!=odom_.end()&&it->first==t){T=it->second;return T.matrix().allFinite();}
    if(it!=odom_.begin() && it!=odom_.end()){
      const auto prev=std::prev(it);
      const double dt=(it->first-prev->first).toSec();
      const double ahead=(it->first-t).toSec();
      const double behind=(t-prev->first).toSec();
      if(dt>1e-6&&dt<=step3_odom_se3_max_bracket_sec_ &&
         ahead>=0.0&&behind>=0.0){
        const double u=std::max(0.0,std::min(1.0,behind/dt));
        Eigen::Quaterniond q0(prev->second.linear()),q1(it->second.linear());
        q0.normalize();q1.normalize();
        T=Eigen::Isometry3d::Identity();
        T.linear()=q0.slerp(u,q1).normalized().toRotationMatrix();
        T.translation()=(1.0-u)*prev->second.translation()+
                         u*it->second.translation();
        return T.matrix().allFinite();
      }
    }
    // Timing skew can be smaller than a single ROS callback; allow a tiny
    // nearest-neighbour grace only. Never copy a pose 0.15s away at cart
    // driving speed or interpolate across a failed-estimator blackout.
    auto nearest=std::min_element(odom_.begin(),odom_.end(),
       [&](const auto&a,const auto&b){
         return std::abs((a.first-t).toSec())<
                std::abs((b.first-t).toSec());});
    if(std::abs((nearest->first-t).toSec())>
       step3_odom_se3_nearest_grace_sec_)return false;
    T=nearest->second;
    return T.matrix().allFinite();
  }''')
work=R/'step3_global/step3_odom_interpolated_dev_20261017'
work.mkdir(parents=True,exist_ok=True)
(work/'step3_global_acquirer.cpp').write_text(new)
(work/'source.diff').write_text('\n'.join(difflib.unified_diff(old.splitlines(),new.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {BASE}
COPY step3_global_acquirer.cpp {target}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
launch=(R/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
anchor='    <param name="tracking_hz" value="1.0"/>'
assert launch.count(anchor)==1
launch=launch.replace(anchor,anchor+'''
    <!-- Opt-in strictly time-bracketed FAST pose interpolation. -->
    <param name="step3_odom_se3_interpolation_enable" value="true"/>
    <param name="step3_odom_se3_max_bracket_sec" value="0.30"/>
    <param name="step3_odom_se3_nearest_grace_sec" value="0.045"/>''')
(R/'launch/fast_livo2_step3_odom_interpolated_dev_20261017.launch').write_text(launch)
print('Prepared source',work/'source.diff','and isolated launch; compile after 1x replays',flush=True)
