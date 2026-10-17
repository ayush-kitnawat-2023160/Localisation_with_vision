#!/usr/bin/env python3
"""Oct13 frozen baseline + ONLY opt-in causal publication smoother."""
from pathlib import Path
import subprocess,difflib
root=Path(__file__).resolve().parents[1]
base='fast_livo2_failsafe:experimental-continuity-retry-20261013'
image='fast_livo2_failsafe:outage-causal-frozen-oct13-dev-20261016'
src=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',base,
'/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'],text=True)
s=src
def rep(a,b):
 global s
 if s.count(a)!=1:raise ValueError(f'anchor {s.count(a)} {a[:80]}')
 s=s.replace(a,b)
rep('#include <std_msgs/String.h>','#include <std_msgs/String.h>\n#include <std_msgs/Bool.h>')
rep('''    pnh_.param("tracking_publish_max_correction_speed_mps",tracking_publish_max_correction_speed_mps_,6.0);''',
'''    pnh_.param("tracking_publish_max_correction_speed_mps",tracking_publish_max_correction_speed_mps_,6.0);
    pnh_.param("outage_causal_trial_enable",outage_causal_trial_enable_,false);
    pnh_.param("outage_causal_trial_correction_mps",outage_causal_trial_correction_mps_,0.75);
    pnh_.param("outage_causal_trial_yaw_deg_s",outage_causal_trial_yaw_deg_s_,12.0);
    pnh_.param("outage_causal_trial_max_speed_mps",outage_causal_trial_max_speed_mps_,5.0);
    pnh_.param("outage_causal_trial_max_step_m",outage_causal_trial_max_step_m_,0.80);
    pnh_.param("outage_causal_trial_max_backlog_m",outage_causal_trial_max_backlog_m_,0.75);
    outage_causal_trial_max_step_m_=std::max(0.2,std::min(0.85,outage_causal_trial_max_step_m_));''')
rep('''  double tracking_publish_max_correction_speed_mps_=6.0,tracking_publish_max_yaw_correction_deg_s_=35.0;''',
'''  double tracking_publish_max_correction_speed_mps_=6.0,tracking_publish_max_yaw_correction_deg_s_=35.0;
  // Independent safety signal, NOT a control command. Only on trial image.
  bool outage_causal_trial_enable_=false;
  double outage_causal_trial_correction_mps_=0.75;
  double outage_causal_trial_yaw_deg_s_=12.0;
  double outage_causal_trial_max_speed_mps_=5.0;
  double outage_causal_trial_max_step_m_=0.80;
  double outage_causal_trial_max_backlog_m_=0.75;
  bool outage_causal_catchup_active_=false;
  // Once a real LiDAR outage occurred, do not claim navigation trust merely
  // because the smoothed path eventually approached an unverified map pose.
  bool outage_causal_outage_ever_seen_=false;
  int outage_causal_stable_samples_=0;
  bool outage_causal_prev_local_valid_=false;
  Eigen::Isometry3d outage_causal_prev_local_=Eigen::Isometry3d::Identity();
  ros::Time outage_causal_prev_stamp_;
  ros::Publisher outage_causal_untrusted_pub_;''')
rep('''    tracking_pub_=nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>''','''    outage_causal_untrusted_pub_=nh_.advertise<std_msgs::Bool>("/fast_livo2/step3_outage_smoothing_untrusted",1);
    tracking_pub_=nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>''')
rep('''        last_published_global_pose_=global_pose;''',
'''        // Keep world-aligned relative ESIKF motion; apply a rate limit ONLY
        // to asynchronous global map corrections during outage AND recovery.
        // Never change FAST ESIKF, map<-FAST anchor, or map observations.
        if(outage_causal_trial_enable_){
          bool outage=false;
          {
            std::lock_guard<std::mutex> lk(mode_mtx_);
            outage=failsafe_mode_=="VI_LIDAR_FAILED" ||
                   failsafe_mode_=="IMU_ONLY_BRIDGE";
          }
          if(outage){
            outage_causal_catchup_active_=true;
            outage_causal_stable_samples_=0;
            outage_causal_outage_ever_seen_=true;
          }
          // Signal is conservatively latched until explicit safe restart and
          // independent map/vehicle checks. No automatic trust re-arm here.
          bool unsafe=outage_causal_outage_ever_seen_;
          if(outage_causal_catchup_active_){
            if(!last_published_global_pose_valid_||!outage_causal_prev_local_valid_){
              unsafe=true;
            }else{
              const double dt=(m->header.stamp-last_published_global_stamp_).toSec();
              const double ldt=(m->header.stamp-outage_causal_prev_stamp_).toSec();
              if(dt<=0.001||dt>0.35||ldt<=0.001||ldt>0.35){
                // No extrapolation across missing sensor frames. Remain unsafe.
                unsafe=true;
                global_pose=last_published_global_pose_;
              }else{
                Eigen::Isometry3d delta=outage_causal_prev_local_.inverse()*T;
                Eigen::Isometry3d predicted=last_published_global_pose_*delta;
                const double local_speed=delta.translation().norm()/dt;
                if(!std::isfinite(local_speed)||local_speed>outage_causal_trial_max_speed_mps_){
                  unsafe=true;
                  predicted=last_published_global_pose_;
                }
                const Eigen::Vector3d raw_error=global_pose.translation()-predicted.translation();
                const double backlog=raw_error.norm();
                if(!std::isfinite(backlog)||backlog>outage_causal_trial_max_backlog_m_)unsafe=true;
                Eigen::Vector3d step=raw_error;
                const double cap=std::max(0.1,outage_causal_trial_correction_mps_)*dt;
                if(std::isfinite(backlog)&&backlog>cap&&backlog>1e-9)step*=cap/backlog;
                if(!step.allFinite()){step.setZero();unsafe=true;}
                Eigen::Vector3d wanted=predicted.translation()+step;
                Eigen::Vector3d displacement=wanted-last_published_global_pose_.translation();
                const double d=displacement.norm();
                const double vehicle_cap=std::min(outage_causal_trial_max_step_m_,
                     std::max(0.5,outage_causal_trial_max_speed_mps_)*dt);
                if(d>vehicle_cap&&d>1e-9){
                  wanted=last_published_global_pose_.translation()+
                      displacement*vehicle_cap/d;
                  unsafe=true;
                }
                global_pose.translation()=wanted;
                const double yaw_residual=wrap(yawOf(global_pose)-yawOf(predicted));
                const double dyaw=std::max(1.,outage_causal_trial_yaw_deg_s_)*M_PI/180.*dt;
                if(!std::isfinite(yaw_residual))unsafe=true;
                else global_pose.linear()=Eigen::AngleAxisd(
                    std::max(-dyaw,std::min(dyaw,yaw_residual)),
                    Eigen::Vector3d::UnitZ()).toRotationMatrix()*predicted.linear();
                // The correction target is *not* independently trusted here.
                // Permit automatic exit only after >=8 consecutive near-target
                // samples; otherwise continue marking the output untrusted.
                const double remaining=(global_pose.translation()-(map_from_fast_*T).translation()).norm();
                // This check must apply DURING the outage as well: an earlier
                // version missed >15m publisher-to-anchor divergence when a
                // single local increment happened to be small.
                if(!std::isfinite(remaining) || remaining>outage_causal_trial_max_backlog_m_)
                  unsafe=true;
                if(!outage&&remaining<0.35 && !unsafe)
                  ++outage_causal_stable_samples_;
                else outage_causal_stable_samples_=0;
                if(!outage&&outage_causal_stable_samples_>=8){
                  outage_causal_catchup_active_=false;
                  outage_causal_stable_samples_=0;
                }else if(!outage && remaining>0.35)
                  unsafe=true;
                ROS_INFO_THROTTLE(0.6,"[STEP3 FROZEN-CAUSAL] pose_correction=%.2fm max_step=%.2fm backlog=%.2fm untrusted=%d",
                    backlog,vehicle_cap,remaining,unsafe?1:0);
              }
            }
          }
          std_msgs::Bool flag;flag.data=unsafe;
          outage_causal_untrusted_pub_.publish(flag);
          if(unsafe)
            ROS_WARN_THROTTLE(1.0,"[STEP3 FROZEN-CAUSAL] LOC UNSAFE; smooth publication is NOT verified absolute pose; STOP must be connected in DBW before cart movement");
          outage_causal_prev_local_=T;
          outage_causal_prev_stamp_=m->header.stamp;
          outage_causal_prev_local_valid_=true;
        }
        last_published_global_pose_=global_pose;''')
dir=root/'step3_global/outage_causal_frozen_oct13_dev_20261016'
(dir/'src').mkdir(parents=True,exist_ok=True)
(dir/'src/step3_global_acquirer.cpp').write_text(s)
(dir/'trial_change.patch').write_text('\n'.join(difflib.unified_diff(src.splitlines(),s.splitlines(),lineterm=''))+'\n')
(dir/'Dockerfile').write_text(f'''FROM {base}
COPY src/step3_global_acquirer.cpp /ws/src/Fast_Livo2/src/step3_global_acquirer.cpp
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
launch=(root/'launch/fast_livo2_step3_global_rosbag.launch').read_text()
node='<param name="tracking_publish_max_correction_speed_mps" value="6.0"/>'
assert launch.count(node)==1
launch=launch.replace(node,node+'''
    <param name="outage_causal_trial_enable" value="true"/>
    <param name="outage_causal_trial_correction_mps" value="0.75"/>
    <param name="outage_causal_trial_yaw_deg_s" value="12.0"/>
    <param name="outage_causal_trial_max_speed_mps" value="5.0"/>
    <param name="outage_causal_trial_max_step_m" value="0.80"/>
    <param name="outage_causal_trial_max_backlog_m" value="0.75"/>''')
(root/'launch/fast_livo2_step3_frozen_causal_dev_20261016.launch').write_text(launch)
print('Compiling isolated Oct13 frozen baseline with opt-in smoothing',flush=True)
subprocess.run(['docker','build','-t',image,str(dir)],check=True)
