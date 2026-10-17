#!/usr/bin/env python3
"""Build opt-in sensor-outage causal PUBLISHED-pose limiter from frozen Oct13.

FAST ESIKF, original map authority, failure FSM and map registrations remain
untouched. This is a dev trial, never a stop actuator or deployment image.
"""
from pathlib import Path
import subprocess,difflib
root=Path(__file__).resolve().parents[1]
base='fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015'
image='fast_livo2_failsafe:outage-causal-smoother-dev-20261016'
original=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',base,
      '/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'],text=True)
s=original
def change(a,b):
 global s
 if s.count(a)!=1:raise RuntimeError(f'Expected exactly one anchor {a[:140]}, found {s.count(a)}')
 s=s.replace(a,b)

change('#include <std_msgs/String.h>','#include <std_msgs/String.h>\n#include <std_msgs/Bool.h>')
change('''    pnh_.param("tracking_publish_relative_motion_limit_enable",tracking_publish_relative_motion_limit_enable_,false);''','''    pnh_.param("tracking_publish_relative_motion_limit_enable",tracking_publish_relative_motion_limit_enable_,false);
    // DEV-ONLY: apply a causal map-correction limit *also during LiDAR loss*.
    // Unlike blending XY points, extrapolates from consecutive SE(3) local FAST
    // increments, limits extra global corrections, then reports trust loss.
    pnh_.param("tracking_outage_causal_smoother_enable",tracking_outage_causal_smoother_enable_,false);
    pnh_.param("tracking_outage_causal_correction_speed_mps",tracking_outage_causal_correction_speed_mps_,0.75);
    pnh_.param("tracking_outage_causal_yaw_rate_deg_s",tracking_outage_causal_yaw_rate_deg_s_,12.0);
    pnh_.param("tracking_outage_causal_max_vehicle_speed_mps",tracking_outage_causal_max_vehicle_speed_mps_,5.0);
    pnh_.param("tracking_outage_causal_max_step_m",tracking_outage_causal_max_step_m_,0.80);
    pnh_.param("tracking_outage_causal_untrusted_backlog_m",tracking_outage_causal_untrusted_backlog_m_,0.75);
    tracking_outage_causal_correction_speed_mps_=std::max(0.10,std::min(3.0,tracking_outage_causal_correction_speed_mps_));
    tracking_outage_causal_max_vehicle_speed_mps_=std::max(1.0,std::min(8.0,tracking_outage_causal_max_vehicle_speed_mps_));
    tracking_outage_causal_max_step_m_=std::max(0.20,std::min(0.85,tracking_outage_causal_max_step_m_));
    tracking_outage_causal_untrusted_backlog_m_=std::max(0.20,std::min(1.5,tracking_outage_causal_untrusted_backlog_m_));''')
change('''  bool tracking_publish_relative_motion_limit_enable_=false;''','''  bool tracking_publish_relative_motion_limit_enable_=false;
  bool tracking_outage_causal_smoother_enable_=false;
  double tracking_outage_causal_correction_speed_mps_=0.75;
  double tracking_outage_causal_yaw_rate_deg_s_=12.0;
  double tracking_outage_causal_max_vehicle_speed_mps_=5.0;
  double tracking_outage_causal_max_step_m_=0.80;
  double tracking_outage_causal_untrusted_backlog_m_=0.75;
  ros::Publisher outage_smoothing_unsafe_pub_;''')
# pub init use place where pose_pub_ exists
print('Pose publisher context:')
for line in s.splitlines():
 if 'pose_pub_=' in line:print(line[:180])
change('''        last_published_global_pose_=global_pose;''','''        // DEV causal limiter: operate ONLY during simulated/real LiDAR loss.
        // The internal map<-FAST anchor is deliberately never modified here.
        // Corrections beyond physically plausible motion are kept as a
        // *trust violation*, not silently masked by a cosmetically smooth pose.
        if(tracking_outage_causal_smoother_enable_){
          bool outage=false;
          {
            std::lock_guard<std::mutex> mode_lock(mode_mtx_);
            outage=(failsafe_mode_=="VI_LIDAR_FAILED" ||
                    failsafe_mode_=="IMU_ONLY_BRIDGE");
          }
          bool untrusted=false;
          if(outage){
            if(!last_published_global_pose_valid_ || !publish_previous_local_valid_){
              untrusted=true; // do not arm on an unverified first outage pose
            }else{
              const double dt=(m->header.stamp-last_published_global_stamp_).toSec();
              const double local_dt=(m->header.stamp-publish_previous_local_stamp_).toSec();
              if(dt<=0.001 || dt>0.35 || local_dt<=0.001 || local_dt>0.35){
                untrusted=true; // time jump, dropout or out-of-order callback
                // Never turn a long-gap arrival into an unbounded snap.
                global_pose=last_published_global_pose_;
              }else{
                const Eigen::Isometry3d local_step=publish_previous_local_pose_.inverse()*T;
                Eigen::Isometry3d predicted=last_published_global_pose_*local_step;
                const double local_v=local_step.translation().norm()/dt;
                if(!std::isfinite(local_v) || local_v>tracking_outage_causal_max_vehicle_speed_mps_){
                  untrusted=true; // local estimator itself physically implausible
                  predicted=last_published_global_pose_;
                }
                const Eigen::Vector3d raw_error=global_pose.translation()-predicted.translation();
                const double backlog=raw_error.norm();
                if(!std::isfinite(backlog) || backlog>tracking_outage_causal_untrusted_backlog_m_)
                  untrusted=true;
                Eigen::Vector3d limited=raw_error;
                const double cap=tracking_outage_causal_correction_speed_mps_*dt;
                if(std::isfinite(backlog) && backlog>cap && backlog>1.e-9)
                  limited*=cap/backlog;
                if(!limited.allFinite()){ limited.setZero();untrusted=true; }
                Eigen::Vector3d trial=predicted.translation()+limited;
                Eigen::Vector3d from_last=trial-last_published_global_pose_.translation();
                const double step=from_last.norm();
                const double physical_cap=std::min(tracking_outage_causal_max_step_m_,
                    tracking_outage_causal_max_vehicle_speed_mps_*dt);
                if(step>physical_cap && step>1.e-9){
                  trial=last_published_global_pose_.translation()+
                      from_last*(physical_cap/step);
                  untrusted=true;
                }
                global_pose.translation()=trial;
                const double yaw_residual=wrap(yawOf(global_pose)-yawOf(predicted));
                const double max_yaw=tracking_outage_causal_yaw_rate_deg_s_*M_PI/180.0*dt;
                if(!std::isfinite(yaw_residual))untrusted=true;
                else global_pose.linear()=Eigen::AngleAxisd(
                   std::max(-max_yaw,std::min(max_yaw,yaw_residual)),
                   Eigen::Vector3d::UnitZ()).toRotationMatrix()*predicted.linear();
                ROS_INFO_THROTTLE(0.7,"[STEP3 OUTAGE-CAUSAL] raw-correction=%.3fm limited=%.3fm backlog=%.3fm local=%.2fm/s unsafe=%d",
                     backlog,limited.norm(),(global_pose.translation()-(map_from_fast_*T).translation()).norm(),
                     local_v,untrusted?1:0);
              }
            }
          }
          std_msgs::Bool unsafe_msg;
          unsafe_msg.data=untrusted;
          outage_smoothing_unsafe_pub_.publish(unsafe_msg);
          if(untrusted)
            ROS_WARN_THROTTLE(0.7,"[STEP3 OUTAGE-CAUSAL] LOCALIZATION UNSAFE: bounded visual output is NOT proof of map accuracy; stop-required must be implemented upstream of DBW");
        }
        last_published_global_pose_=global_pose;''')
# Add publisher initialisation in constructor at existing publisher line
needle='pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>'
idx=s.find(needle)
if idx<0:raise RuntimeError('Could not find pose publisher')
lineend=s.index('\n',idx)
line=s[idx:lineend]
s=s[:lineend+1]+'    outage_smoothing_unsafe_pub_=nh_.advertise<std_msgs::Bool>("/fast_livo2/step3_outage_smoothing_untrusted",1);\n'+s[lineend+1:]
dir=root/'step3_global/outage_causal_smoother_dev_20261016'
(dir/'src').mkdir(parents=True,exist_ok=True)
(dir/'src/step3_global_acquirer.cpp').write_text(s)
(dir/'isolated_change.patch').write_text('\n'.join(difflib.unified_diff(original.splitlines(),s.splitlines(),fromfile='frozen/src/step3_global_acquirer.cpp',tofile='candidate/src/step3_global_acquirer.cpp',lineterm=''))+'\n')
(dir/'Dockerfile').write_text(f'''FROM {base}
COPY src/step3_global_acquirer.cpp /ws/src/Fast_Livo2/src/step3_global_acquirer.cpp
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
launch=(root/'launch/fast_livo2_step3_global_rosbag.launch').read_text()
node='<param name="tracking_publish_max_correction_speed_mps" value="6.0"/>'
assert launch.count(node)==1
launch=launch.replace(node,node+'''
    <!-- Isolated replay ONLY. Do not wire the smoothed map pose into planning
         until localisation-stop is connected to and tested at the DBW layer. -->
    <param name="tracking_outage_causal_smoother_enable" value="true"/>
    <param name="tracking_outage_causal_correction_speed_mps" value="0.75"/>
    <param name="tracking_outage_causal_yaw_rate_deg_s" value="12.0"/>
    <param name="tracking_outage_causal_max_vehicle_speed_mps" value="5.0"/>
    <param name="tracking_outage_causal_max_step_m" value="0.80"/>
    <param name="tracking_outage_causal_untrusted_backlog_m" value="0.75"/>''')
(root/'launch/fast_livo2_step3_global_outage_causal_smoother_dev_20261016.launch').write_text(launch)
print('building isolated dev image',image,flush=True)
subprocess.run(['docker','build','-t',image,str(dir)],check=True)
