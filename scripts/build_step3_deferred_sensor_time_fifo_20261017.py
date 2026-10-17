#!/usr/bin/env python3
"""Opt-in timestamp-FIFO selection for deferred LiDAR scans; source-only.
Do not change ESIKF, saved map, estimator velocity, ROS bridge or /pose.
"""
from pathlib import Path
import subprocess,difflib
R=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:single-build-rc-20261016'
TAG='fast_livo2_failsafe:step3-deferred-fifo-localization-dev-20261017'
src='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
old=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,src],text=True)
new=old
def rep(a,b):
 global new
 n=new.count(a)
 assert n==1,(n,a[:94])
 new=new.replace(a,b)
rep('    pnh_.param("tracking_deferred_scan_enable",tracking_deferred_scan_enable_,false);',
'''    pnh_.param("tracking_deferred_scan_enable",tracking_deferred_scan_enable_,false);
    // Experimental: process earliest odometry-aligned scan at each 1Hz
    // sampling boundary, rather than newest available when callback arrives.
    // A delayed callback cannot silently skip an earlier sensor timestamp.
    pnh_.param("tracking_deferred_sensor_time_fifo_enable",
                tracking_deferred_sensor_time_fifo_enable_,false);''')
rep('  bool tracking_deferred_scan_enable_=false,tracking_continuity_retry_enable_=false;',
'''  bool tracking_deferred_scan_enable_=false,tracking_continuity_retry_enable_=false;
  bool tracking_deferred_sensor_time_fifo_enable_=false;''')
rep('''      int ready_index=-1;
      for(int i=(int)deferred_lidar_.size()-1;i>=0;--i){
        const auto& scan=deferred_lidar_[i];
        if(!last_tracking_query_.isZero() &&
           (scan->header.stamp-last_tracking_query_).toSec()<min_period)continue;
        Eigen::Isometry3d aligned;
        if(odomAt(scan->header.stamp,aligned)){ready_index=i;break;}
      }
      if(ready_index>=0){''',
'''      int ready_index=-1;
      // This choice is deterministic in recorded sensor time. The original
      // always selected the newest scan that happened to be aligned when
      // this callback acquired the costly GICP mutex. A small CPU delay
      // changed the chosen map match and made recovery errors nonrepeatable.
      const int count=static_cast<int>(deferred_lidar_.size());
      for(int k=0;k<count;++k){
        const int i=tracking_deferred_sensor_time_fifo_enable_?
           k:(count-1-k);
        const auto& scan=deferred_lidar_[i];
        if(!last_tracking_query_.isZero() &&
           (scan->header.stamp-last_tracking_query_).toSec()<min_period)continue;
        Eigen::Isometry3d aligned;
        if(odomAt(scan->header.stamp,aligned)){ready_index=i;break;}
      }
      if(ready_index>=0){
        if(tracking_deferred_sensor_time_fifo_enable_ && deferred_lidar_.size()>1)
          ROS_INFO_THROTTLE(0.5,
             "[STEP3 SENSOR-FIFO] ready=%d/%zu stamp=%.6f last_query=%.6f",
             ready_index,deferred_lidar_.size(),
             deferred_lidar_[ready_index]->header.stamp.toSec(),
             last_tracking_query_.toSec());''')
work=R/'step3_global/step3_deferred_sensor_fifo_dev_20261017'
work.mkdir(parents=True,exist_ok=True)
(work/'step3_global_acquirer.cpp').write_text(new)
(work/'change.diff').write_text('\n'.join(difflib.unified_diff(old.splitlines(),new.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {BASE}
COPY step3_global_acquirer.cpp {src}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
launch=(R/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
anchor='    <param name="tracking_deferred_scan_enable" value="$(arg tracking_deferred_scan_enable)"/>'
assert launch.count(anchor)==1
launch=launch.replace(anchor,anchor+'''
    <param name="tracking_deferred_sensor_time_fifo_enable" value="true"/>''')
(R/'launch/fast_livo2_step3_sensor_time_fifo_dev_20261017.launch').write_text(launch)
subprocess.run(['docker','build','-t',TAG,'-f',str(work/'Dockerfile'),str(work)],check=True)
print('BUILT',TAG,flush=True)
