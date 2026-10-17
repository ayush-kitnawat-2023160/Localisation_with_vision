#!/usr/bin/env python3
"""Read-only diagnostics for LiDAR-outage absolute registration/anchor latency.

Uses exactly the Oct 13 baseline sources from the frozen image. This image
adds timestamped diagnostics ONLY, not estimator changes, thresholds, or fixes.
"""
from pathlib import Path
import difflib
import subprocess

root=Path(__file__).resolve().parents[1]
image='fast_livo2_failsafe:experimental-continuity-retry-20261013'
out=root/'step3_global/lidar_fail_depth_timing_diag_20261016'
out.mkdir(parents=True,exist_ok=True)
path='src/failsafe_map_localizer.cpp'
s=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',image,'/ws/src/Fast_Livo2/'+path],text=True)
new=s
def replace(old,new_text):
 global new
 count=new.count(old)
 if count!=1:raise RuntimeError(f'expected one match; found {count}: {old[:120]}')
 new=new.replace(old,new_text)

replace('''    const Eigen::Matrix4f current_local_lidar = currentWorldLidar(Rwi, twi);
    const Eigen::Matrix4f candidate_anchor =
        predicted_map_lidar * current_local_lidar.inverse();
    queueAnchorTarget(candidate_anchor,''',
'''    const Eigen::Matrix4f current_local_lidar = currentWorldLidar(Rwi, twi);
    const Eigen::Matrix4f candidate_anchor =
        predicted_map_lidar * current_local_lidar.inverse();
    if(lidar_fail){
      Eigen::Matrix4f old_anchor=Eigen::Matrix4f::Identity();
      bool valid=false;
      {
        std::lock_guard<std::mutex> lk(anchor_mutex_);
        old_anchor=anchor_T_map_current_;
        valid=anchor_valid_;
      }
      if(valid){
        const Eigen::Matrix4f old_map_lidar=old_anchor*current_local_lidar;
        const Eigen::Matrix4f delta=old_anchor.inverse()*candidate_anchor;
        ROS_INFO_THROTTLE(0.6,
          "[DEPTH TIMING DIAG] predictor stamp=%.3f map_pose_diff=%.3fm anchor_trans_diff=%.3fm anchor_rot_diff=%.2fdeg map_target_xyz=%.2f,%.2f,%.2f old_map_xyz=%.2f,%.2f,%.2f",
          stamp.toSec(),(predicted_map_lidar.block<3,1>(0,3)-old_map_lidar.block<3,1>(0,3)).norm(),
          delta.block<3,1>(0,3).norm(),rotationAngleDeg(delta.block<3,3>(0,0).cast<double>()),
          predicted_map_lidar(0,3),predicted_map_lidar(1,3),predicted_map_lidar(2,3),
          old_map_lidar(0,3),old_map_lidar(1,3),old_map_lidar(2,3));
      }
    }
    queueAnchorTarget(candidate_anchor,''')

replace('''      depth_failover_pose_pub_.publish(out);
      ROS_INFO_THROTTLE(0.5,
          "[STEP3 DEPTH MAP] verified map-body pose fitness=%.3f xyz=[%.2f %.2f %.2f]",''',
'''      depth_failover_pose_pub_.publish(out);
      ROS_INFO("[DEPTH TIMING DIAG] depth_src=%.3f depth_corrected=%.3f ros_now=%.3f local_latest=%.3f compensated=%d delay=%.3fs xyz=%.2f,%.2f,%.2f fitness=%.4f",
          stamp.toSec(),out.header.stamp.toSec(),ros::Time::now().toSec(),
          latest_stamp.toSec(),latency_compensated?1:0,
          ros::Time::now().toSec()-out.header.stamp.toSec(),
          out.pose.position.x,out.pose.position.y,out.pose.position.z,fitness);
      ROS_INFO_THROTTLE(0.5,
          "[STEP3 DEPTH MAP] verified map-body pose fitness=%.3f xyz=[%.2f %.2f %.2f]",''')

dest=out/path;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(new)
(out/'diff.patch').write_text('\n'.join(difflib.unified_diff(s.splitlines(),new.splitlines(),lineterm=''))+'\n')
(out/'Dockerfile').write_text(f'''FROM {image}
COPY src/failsafe_map_localizer.cpp /ws/src/Fast_Livo2/src/failsafe_map_localizer.cpp
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 failsafe_map_localizer'
''')
print('Building timing/anchor diagnostic only',flush=True)
subprocess.run(['docker','build','-t','fast_livo2_failsafe:depth-timing-diagnostic-20261016',str(out)],check=True)
