#!/usr/bin/env python3
"""Opt-in ROS1 freshest-pose transport for Step3 only, frozen base image.

No change to geometry, sensor synchronization, ESIKF or cart interfaces.
ROS1 dynamic poses should not be latched. Bound outgoing + incoming queues;
keep tcpNoDelay to reduce small-message batching on real networks.
"""
from pathlib import Path
import subprocess,difflib
R=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:single-build-rc-20261016'
TARGET='/ws/src/Fast_Livo2/src/step3_global_acquirer.cpp'
old=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,TARGET],text=True)
new=old
def once(a,b):
 global new
 assert new.count(a)==1,('missing/duplicate',a[:100])
 new=new.replace(a,b)
once('    pnh_.param("tracking_hz",tracking_hz_,1.0);', '''    pnh_.param("tracking_hz",tracking_hz_,1.0);
    // Only transport backlog, not filtering, pose semantics or ESIKF.
    pnh_.param("tracking_fresh_pose_transport_enable",
               tracking_fresh_pose_transport_enable_,false);''')
once('  double tracking_hz_=1.0,tracking_max_fitness_=0.20,tracking_max_translation_innovation_=3.0,tracking_max_yaw_innovation_deg_=10.0;', '''  double tracking_hz_=1.0,tracking_max_fitness_=0.20,tracking_max_translation_innovation_=3.0,tracking_max_yaw_innovation_deg_=10.0;
  bool tracking_fresh_pose_transport_enable_=false;''')
once('pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>("/fast_livo2/step3_global_pose",5,true);', '''pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>(
        "/fast_livo2/step3_global_pose",
        tracking_fresh_pose_transport_enable_?1:5,
        !tracking_fresh_pose_transport_enable_);''')
once('    odom_sub_=nh_.subscribe(odom_topic_,500,&Step3GlobalAcquirer::odomCb,this);','''    // 500 incoming odometry callbacks can let a saturated node replay tens
    // of seconds of stale states. When enabled, discard outdated queue data
    // and prioritize fresh source-stamped FAST odometry (up to 10 samples).
    if(tracking_fresh_pose_transport_enable_){
      odom_sub_=nh_.subscribe(odom_topic_,10,&Step3GlobalAcquirer::odomCb,
                              this,ros::TransportHints().tcpNoDelay(true));
      ROS_WARN("[STEP3 FRESH TRANSPORT] pose queue=1 unlatched; FAST odom input=10 tcpNoDelay");
    }else{
      odom_sub_=nh_.subscribe(odom_topic_,500,&Step3GlobalAcquirer::odomCb,this);
    }''')
work=R/'step3_global/low_latency_pose_transport_dev_20261017';work.mkdir(parents=True,exist_ok=True)
(work/'step3_global_acquirer.cpp').write_text(new)
(work/'source.diff').write_text('\n'.join(difflib.unified_diff(old.splitlines(),new.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {BASE}
COPY step3_global_acquirer.cpp {TARGET}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 step3_global_acquirer'
''')
launch=(R/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
a='    <param name="tracking_hz" value="1.0"/>'
assert launch.count(a)==1
launch=launch.replace(a,a+'\n    <param name="tracking_fresh_pose_transport_enable" value="true"/>')
(R/'launch/fast_livo2_step3_low_latency_transport_dev_20261017.launch').write_text(launch)
print('Prepared',work,'diff lines',len((work/'source.diff').read_text().splitlines()))
