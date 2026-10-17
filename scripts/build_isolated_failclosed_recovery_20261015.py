#!/usr/bin/env python3
"""Build a fail-closed Step3 LiDAR handoff from the frozen accepted healthy image.

This fixes two data-integrity bugs without weakening map thresholds:
* Do not use an UNVERIFIED RGB-D absolute pose as a trusted LiDAR recovery seed.
* Do not apply in-flight LiDAR PCD corrections after sensor-failure transition.
Also expose a conservative, opt-in ROS localization-trust Bool. This is a
status output only; cart planner/DBW integration is NOT changed or certified.
"""
from pathlib import Path
import subprocess
import difflib

root=Path(__file__).resolve().parents[1]
image='fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015'
new_image='fast_livo2_failsafe:failclosed-recovery-dev-20261015'
target='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
old=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',image,target],text=True)
s=old
def replace_once(before,after):
    global s
    n=s.count(before)
    if n!=1:raise RuntimeError(f'replace expected unique anchor, matched {n}: {before[:110]!r}')
    s=s.replace(before,after)

# Instrumented development-only Bool channel: false is NOT a vehicle brake.
replace_once('#include <std_msgs/String.h>', '#include <std_msgs/String.h>\n#include <std_msgs/Bool.h>')
replace_once('status_pub_=nh_.advertise<std_msgs::String>("/fast_livo2/step3_status",1,true);',
    'status_pub_=nh_.advertise<std_msgs::String>("/fast_livo2/step3_status",1,true);\n'
    '    trust_pub_=nh_.advertise<std_msgs::Bool>("/fast_livo2/step3_pose_trusted",1,true);')
replace_once('lock_pub_,pose_pub_,status_pub_,tracking_pub_',
    'lock_pub_,pose_pub_,status_pub_,trust_pub_,tracking_pub_')
replace_once('  bool tracking_healthy_loop_motion_bridge_enable_=false;',
    '  bool tracking_healthy_loop_motion_bridge_enable_=false;\n'
    '  bool failclosed_recovery_enable_=false;\n'
    '  double trust_fresh_lidar_sec_=2.0,trust_fresh_depth_sec_=1.25;')
replace_once('    pnh_.param("tracking_healthy_loop_motion_bridge_enable",tracking_healthy_loop_motion_bridge_enable_,false);',
    '    pnh_.param("tracking_healthy_loop_motion_bridge_enable",tracking_healthy_loop_motion_bridge_enable_,false);\n'
    '    pnh_.param("failclosed_recovery_enable",failclosed_recovery_enable_,false);\n'
    '    pnh_.param("trust_fresh_lidar_sec",trust_fresh_lidar_sec_,2.0);\n'
    '    pnh_.param("trust_fresh_depth_sec",trust_fresh_depth_sec_,1.25);')
replace_once('  void publishStatus(const std::string&s){std_msgs::String m;m.data=s;status_pub_.publish(m);ROS_INFO_STREAM("[STEP3] "<<s);}',
'''  void publishStatus(const std::string&s){std_msgs::String m;m.data=s;status_pub_.publish(m);ROS_INFO_STREAM("[STEP3] "<<s);}
  void publishTrust(const ros::Time& stamp){
    if(!failclosed_recovery_enable_)return;
    bool trust=false;
    if(locked_.load() && !fast_local_unstable_.load() && !stamp.isZero()){
      bool lidar_out=false;
      { std::lock_guard<std::mutex> l(mode_mtx_);
        lidar_out=failsafe_mode_=="VI_LIDAR_FAILED" || failsafe_mode_=="IMU_ONLY_BRIDGE";
      }
      if(lidar_out){
        std::lock_guard<std::mutex> dl(depth_verified_mtx_);
        const double age=(stamp-depth_verified_last_stamp_).toSec();
        trust=depth_verified_pose_valid_ && !depth_verified_last_stamp_.isZero() &&
              age>=0.0 && age<=trust_fresh_depth_sec_;
      }else{
        const double age=stamp.toSec()-last_verified_map_fix_stamp_sec_.load();
        trust=age>=0.0 && age<=trust_fresh_lidar_sec_;
      }
    }
    std_msgs::Bool b; b.data=trust; trust_pub_.publish(b);
    if(!trust)ROS_WARN_THROTTLE(1.0,"[STEP3 TRUST] global pose UNTRUSTED; planner must fail closed; predicted pose is not validated");
  }''')

# Do not mark a camera match "verified" before checking that it aligns in time
# to an actual FAST pose. This previously left a false positive flag behind.
before='''    // A single repeated-geometry lock must not teleport the cart to a different
    // part of the campus map.'''
replace_once(before,
'''    Eigen::Isometry3d fast_body;
    if(!odomAt(m->header.stamp,fast_body)){
      ROS_WARN_THROTTLE(1.0,"[STEP3 DEPTH] no time-aligned FAST pose; not verified");
      return;
    }
    // A single repeated-geometry lock must not teleport the cart to a different
    // part of the campus map.''')
replace_once('    Eigen::Isometry3d fast_body;if(!odomAt(m->header.stamp,fast_body)){ROS_WARN_THROTTLE(1.0,"[STEP3 DEPTH] no time-aligned FAST pose");return;}\n', '')

# Once the sensor is marked absent, in-flight scan callbacks must not rebase
# map<-FAST with delayed LiDAR scans. A withheld sensor cannot restore trust.
replace_once('  void trackLocked(const sensor_msgs::PointCloud2ConstPtr&m){\n',
'''  void trackLocked(const sensor_msgs::PointCloud2ConstPtr&m){
    if(failclosed_recovery_enable_){
      std::lock_guard<std::mutex> l(mode_mtx_);
      if(failsafe_mode_=="VI_LIDAR_FAILED"||failsafe_mode_=="IMU_ONLY_BRIDGE")return;
    }
''')
replace_once('    // best.map_body is T_map_lidar at the source scan time. Convert to body.',
'''    if(failclosed_recovery_enable_){
      std::lock_guard<std::mutex> l(mode_mtx_);
      if(failsafe_mode_=="VI_LIDAR_FAILED"||failsafe_mode_=="IMU_ONLY_BRIDGE"){
        ROS_WARN("[STEP3 TRUST] discarded in-flight GICP match after LiDAR loss");
        return;
      }
    }
    // best.map_body is T_map_lidar at the source scan time. Convert to body.''')

# A global re-acquisition may also have begun before the failure callback.
# Prevent its map-frame reanchor from committing after the mode changed.
replace_once('    last_tracking_accept_stamp_=stamp;\n    last_verified_map_fix_stamp_sec_.store(stamp.toSec());',
'''    if(failclosed_recovery_enable_){
      std::lock_guard<std::mutex> l(mode_mtx_);
      if(failsafe_mode_=="VI_LIDAR_FAILED"||failsafe_mode_=="IMU_ONLY_BRIDGE"){
        ROS_WARN("[STEP3 TRUST] discarded in-flight global reacquisition after LiDAR loss");
        return;
      }
    }
    last_tracking_accept_stamp_=stamp;
    last_verified_map_fix_stamp_sec_.store(stamp.toSec());''')

# Guard the recovery bridge. Unless an independently verified map-depth fix is
# recent, do NOT reset the last LiDAR map ID, lie about a fresh map fix, or seed
# local GICP from a potentially drifted prediction. Preserve the old map ID
# and require global search with temporal verification.
start=s.index('    if(exiting_lidar_fail){\n')
end=s.index('\n  void depthMapCb(',start)
block=s[start:end]
assert block.count('    if(exiting_lidar_fail){')==1
new_block='''    if(exiting_lidar_fail && failclosed_recovery_enable_){
      const ros::Time now=ros::Time::now();
      Eigen::Isometry3d verified_depth=Eigen::Isometry3d::Identity();
      ros::Time verified_stamp;
      bool fresh_verified_depth=false;
      {
        std::lock_guard<std::mutex> dl(depth_verified_mtx_);
        const double age=(now-depth_verified_last_stamp_).toSec();
        fresh_verified_depth=depth_verified_pose_valid_ && !depth_verified_last_stamp_.isZero() &&
                             age>=0.0 && age<=trust_fresh_depth_sec_;
        if(fresh_verified_depth){verified_depth=depth_verified_last_pose_;verified_stamp=depth_verified_last_stamp_;}
      }
      {
        std::lock_guard<std::mutex> rl(recovery_bridge_mtx_);
        lidar_recovery_bridge_active_=fresh_verified_depth;
        lidar_recovery_bridge_until_=fresh_verified_depth?
          now+ros::Duration(std::max(0.5,lidar_recovery_bridge_sec_)):ros::Time(0);
      }
      if(fresh_verified_depth){
        last_tracking_accept_stamp_=now;
        last_tracking_id_=-1;
        tracking_misses_=0;
        global_reacquire_consistent_=0;
        global_reacquire_last_map_valid_=false;
        last_global_reacquire_query_=ros::Time();
        fast_local_unstable_.store(false);
        fast_local_recovery_ready_.store(false);
        fast_local_stable_samples_=0;
        updateTrackingMotionModel(verified_depth,verified_stamp);
        last_tracking_map_body_=verified_depth;
        last_tracking_map_stamp_=verified_stamp;
        last_tracking_map_valid_=true;
        resetTrajectoryFit();
        ROS_WARN("[STEP3 TRUST] LiDAR returned with recent independently verified depth map pose; local bridge enabled");
      }else{
        // Crucially: no unverified rebase and no fake map timestamp. Force
        // map-search/temporal verification, while the planner receives false.
        tracking_misses_=std::max(tracking_misses_,tracking_global_reacquire_after_misses_);
        global_reacquire_consistent_=0;
        global_reacquire_last_map_valid_=false;
        last_global_reacquire_query_=ros::Time();
        fast_local_unstable_.store(true);
        fast_local_recovery_ready_.store(false);
        fast_local_stable_samples_=0;
        resetTrajectoryFit();
        ROS_ERROR("[STEP3 TRUST] LiDAR returned WITHOUT recent verified depth pose: refusing false local bridge; requiring verified global reacquisition");
      }
    }else if(exiting_lidar_fail){'''
assert block.startswith('    if(exiting_lidar_fail){')
block=block.replace('    if(exiting_lidar_fail){',new_block,1)
s=s[:start]+block+s[end:]

# Latched status must turn false on the very failure transition, rather than
# waiting for the next odometry callback. The mode mutex is released here.
replace_once('''    }
  }
  void depthMapCb(const geometry_msgs::PoseStamped::ConstPtr& m){''',
'''    }
    publishTrust(ros::Time::now());
  }
  void depthMapCb(const geometry_msgs::PoseStamped::ConstPtr& m){''')

# Publish trust on EVERY pose callback, including degraded odometry. Must be
# monitored by planner fail-closed; this component never commands actuators.
replace_once('pose_pub_.publish(p);\n    }', 'pose_pub_.publish(p);\n      publishTrust(m->header.stamp);\n    }')

context=root/'step3_global/failclosed_recovery_dev_20261015'
context.mkdir(parents=True,exist_ok=True)
(context/'step3_global_acquirer.cpp').write_text(s)
(context/'isolated_change.patch').write_text('\n'.join(difflib.unified_diff(old.splitlines(),s.splitlines(),lineterm=''))+'\n')
(context/'Dockerfile').write_text(f'''FROM {image}\nCOPY step3_global_acquirer.cpp {target}\nRUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'\n''')
old_launch=(root/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
anchor='    <param name="tracking_healthy_loop_motion_bridge_enable" value="true"/>'
assert old_launch.count(anchor)==1
(root/'launch/fast_livo2_step3_global_failclosed_recovery_dev.launch').write_text(old_launch.replace(anchor,
    anchor+'\n    <param name="failclosed_recovery_enable" value="true"/>\n'
    '    <param name="trust_fresh_lidar_sec" value="2.0"/>\n'
    '    <param name="trust_fresh_depth_sec" value="1.25"/>'))
print('Source diff',context/'isolated_change.patch','size',len(s)-len(old),flush=True)
subprocess.run(['docker','build','-t',new_image,str(context)],check=True)
