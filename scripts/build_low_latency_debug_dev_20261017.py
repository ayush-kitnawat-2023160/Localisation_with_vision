#!/usr/bin/env python3
"""Single isolated mapper optimization: no change to ESIKF, input timing,
registration or motion safety. Only skip debug work when not subscribed.
Launch opt-in; frozen image and runtime untouched.
"""
from pathlib import Path
import subprocess,difflib
R=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:single-build-rc-20261016'
TAG='fast_livo2_failsafe:low-latency-debug-dev-20261017'
SRC='/ws/src/Fast_Livo2/src/LIVMapper.cpp'
HEADER='/ws/src/Fast_Livo2/include/LIVMapper.h'
old=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,SRC],text=True)
oldhead=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,HEADER],text=True)
new=old

def once(a,b):
 global new
 n=new.count(a)
 if n!=1:raise ValueError(f'expected 1 match, got {n}: {a[:130]}')
 new=new.replace(a,b)

once('  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);', '''  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  // Debug-only performance guard. Never skip fused sensor updates or map
  // measurements; /aft_mapped_to_init and /fast_livo2/step3_global_pose keep
  // their original processing and publishing cadence.
  nh.param<bool>("publish/low_latency_debug_guard_enable",
                 low_latency_debug_guard_enable_,false);
  nh.param<int>("publish/low_latency_path_every_n",
                low_latency_path_every_n_,5);
  nh.param<int>("publish/low_latency_path_max_poses",
                low_latency_path_max_poses_,250);
  low_latency_path_every_n_=std::max(1,std::min(100,low_latency_path_every_n_));
  low_latency_path_max_poses_=std::max(20,std::min(5000,low_latency_path_max_poses_));''')
once('''  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;''', '''  // Converting a raw BGR8 frame solely for an absent RViz subscriber wastes
  // bandwidth/CPU in every visual update. Does not affect visual ESIKF input.
  if(low_latency_debug_guard_enable_ && pubImage.getNumSubscribers()==0) return;
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;''')
once('''  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());''', '''  if (pcl_w_wait_pub->empty()) return;
  if(low_latency_debug_guard_enable_ &&
     pubLaserCloudFullRes.getNumSubscribers()==0 && !pcd_save_en && !img_save_en){
    // These buffers feed only visualization/optional disk map output. Clear
    // even if color projection finds zero points to avoid memory growth.
    pcl_wait_pub->clear();
    pcl_w_wait_pub->clear();
    return;
  }
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());''')
once('''  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  // Dedicated full-LiDAR world cloud for persistent-map building and
  // scan-to-map failsafe localization. /cloud_registered in LIVO mode is
  // camera-FOV/colorized and is therefore not a complete geometric map.
  sensor_msgs::PointCloud2 map_cloud_msg;
  pcl::toROSMsg(*laserCloudWorld, map_cloud_msg);
  map_cloud_msg.header.stamp = ros::Time().fromSec(currentUpdateTime());
  map_cloud_msg.header.frame_id = "camera_init";
  pubLaserCloudMap.publish(map_cloud_msg);

  publish_frame_world(pubLaserCloudFullRes, vio_manager);''', '''  // CPU isolation: the live ESIKF / voxel map were updated ABOVE this point.
  // World-cloud conversion below is purely for visualization and optional
  // recording. Do not spend compute on a full cloud nobody subscribed to.
  const bool world_cloud_consumed = !low_latency_debug_guard_enable_ ||
      pubLaserCloudMap.getNumSubscribers()>0 ||
      pubLaserCloudFullRes.getNumSubscribers()>0 || pcd_save_en || img_save_en;
  if(world_cloud_consumed){
    PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    for (int i=0;i<size;++i)
      RGBpointBodyToWorld(&laserCloudFullRes->points[i],&laserCloudWorld->points[i]);
    *pcl_w_wait_pub=*laserCloudWorld;
    if(!low_latency_debug_guard_enable_ || pubLaserCloudMap.getNumSubscribers()>0){
      sensor_msgs::PointCloud2 map_cloud_msg;
      pcl::toROSMsg(*laserCloudWorld,map_cloud_msg);
      map_cloud_msg.header.stamp=ros::Time().fromSec(currentUpdateTime());
      map_cloud_msg.header.frame_id="camera_init";
      pubLaserCloudMap.publish(map_cloud_msg);
    }
    publish_frame_world(pubLaserCloudFullRes,vio_manager);
  }else{
    // No full-cloud consumer: drop temporary visualization-only buffers.
    // The saved map used by Step3 is a separate read-only PCD/database.
    pcl_wait_pub->clear();
    pcl_w_wait_pub->clear();
  }''')
once('''void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}''', '''void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  // /path is debug visualization, NEVER the authoritative driving pose.
  // Without a cap the full accumulated nav_msgs/Path was serialized at every
  // update (O(n^2) bytes overall) and can monopolize the single estimator.
  if(low_latency_debug_guard_enable_){
    static uint64_t debug_index=0;
    ++debug_index;
    if(pubPath.getNumSubscribers()==0){path.poses.clear();return;}
    if(debug_index % static_cast<uint64_t>(low_latency_path_every_n_)!=0)return;
  }
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  if(low_latency_debug_guard_enable_ &&
     path.poses.size()>static_cast<size_t>(low_latency_path_max_poses_)){
    path.poses.erase(path.poses.begin(),
       path.poses.begin()+(path.poses.size()-low_latency_path_max_poses_));
  }
  pubPath.publish(path);
}''')
assert oldhead.count('  bool dense_map_en = false;')==1
newhead=oldhead.replace('  bool dense_map_en = false;', '''  bool dense_map_en = false;
  bool low_latency_debug_guard_enable_ = false;
  int low_latency_path_every_n_ = 5;
  int low_latency_path_max_poses_ = 250;''')
work=R/'step3_global/low_latency_debug_dev_20261017';work.mkdir(parents=True,exist_ok=True)
(work/'LIVMapper.cpp').write_text(new);(work/'LIVMapper.h').write_text(newhead)
(work/'source.diff').write_text('\n'.join(difflib.unified_diff(old.splitlines(),new.splitlines(),lineterm=''))+'\n')
(work/'header.diff').write_text('\n'.join(difflib.unified_diff(oldhead.splitlines(),newhead.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {BASE}
COPY LIVMapper.cpp {SRC}
COPY LIVMapper.h {HEADER}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 fastlivo_mapping'
''')
print('Prepared',work,'source.diff bytes',len((work/'source.diff').read_text()))
