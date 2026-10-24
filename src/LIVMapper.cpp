/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#ifdef __GLIBC__
#include <malloc.h>
#endif
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

LIVMapper::LIVMapper(ros::NodeHandle &nh)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
  bool calibration_ready = false;
  nh.param("calibration/ready", calibration_ready, false);
  if (!calibration_ready)
    throw std::runtime_error("Calibration profile is locked or missing");

  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(nh);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(nh, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  step3_prior_voxelmap_manager.reset(
      new VoxelMapManager(voxel_config, step3_prior_voxel_map_));
  // The prior matcher solves full 6-DoF internally to stay in the correct
  // geometric basin. Only X/Y/yaw are fused into the persistent ESIKF below.
  step3_prior_voxelmap_manager->prior_planar_mode_ = false;
  vio_manager.reset(new VIOManager());

  step3_prior_map_mapframe_.reset(new PointCloudXYZI());
  if (step3_prior_enable_)
  {
    if (step3_prior_map_path_.empty())
      throw std::runtime_error("step3/prior_map_enable=true but step3/prior_map_path is empty");
    PointCloudXYZI::Ptr raw_prior(new PointCloudXYZI());
    if (pcl::io::loadPCDFile(step3_prior_map_path_, *raw_prior) != 0 || raw_prior->empty())
      throw std::runtime_error("failed to load Step-3 prior map: " + step3_prior_map_path_);
    pcl::VoxelGrid<PointType> prior_filter;
    const float leaf = static_cast<float>(std::max(0.05, step3_prior_leaf_));
    prior_filter.setLeafSize(leaf, leaf, leaf);
    prior_filter.setInputCloud(raw_prior);
    prior_filter.filter(*step3_prior_map_mapframe_);
    ROS_INFO("[STEP3] preloaded prior map %s: raw=%zu filtered=%zu leaf=%.2fm",
             step3_prior_map_path_.c_str(), raw_prior->size(),
             step3_prior_map_mapframe_->size(), leaf);
  }
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
  nh.param<int>("common/img_en", img_en, 1);
  nh.param<int>("common/lidar_en", lidar_en, 1);
  nh.param<string>("common/img_topic", img_topic, "/left_camera/image");

  nh.param<bool>("vio/normal_en", normal_en, true);
  nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
  nh.param<int>("vio/max_iterations", max_iterations, 5);
  nh.param<bool>("vio/gated_visual_fastpath_enable", gated_visual_fastpath_enable_, false);
  nh.param<bool>("vio/preserve_lidar_cadence", preserve_lidar_cadence_, false);
  nh.param<bool>("vio/diagnostic_force_lidar_sync_en",
                 diagnostic_force_lidar_sync_en_, false);
  nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 100);
  nh.param<bool>("vio/raycast_en", raycast_en, false);
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  nh.param<bool>("vio/colorize_only_en", colorize_only_en, false);
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
  nh.param<int>("vio/grid_size", grid_size, 5);
  nh.param<int>("vio/grid_n_height", grid_n_height, 17);
  nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  nh.param<int>("vio/patch_size", patch_size, 8);
  nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);
  nh.param<double>("vio/visual_map_radius", visual_map_radius, 60.0);
  nh.param<int>("vio/visual_map_prune_every_n", visual_map_prune_every_n, 20);

  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  nh.param<bool>("uav/gravity_align_en", gravity_align_en, false);

  nh.param<string>("evo/seq_name", seq_name, "01");
  nh.param<bool>("evo/pose_output_en", pose_output_en, false);
  nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
  nh.param<double>("imu/acc_cov", acc_cov, 1.0);
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<bool>("preprocess/hilti_en", hilti_en, false);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/type", pcd_save_type, 0);
  nh.param<bool>("image_save/img_save_en", img_save_en, false);
  nh.param<int>("image_save/interval", img_save_interval, 1);

  nh.param<bool>("pcd_save/colmap_output_en", colmap_output_en, false);
  nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
  nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
  nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());
  nh.param<double>("debug/plot_time", plot_time, -10);
  nh.param<int>("debug/frame_cnt", frame_cnt, 6);

  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
  nh.param<bool>("publish/dense_map_en", dense_map_en, false);

  nh.param<bool>("failsafe/enable", failsafe_enable_, true);
  nh.param<double>("failsafe/camera_timeout", camera_timeout_, 0.5);
  nh.param<double>("failsafe/lidar_timeout", lidar_timeout_, 0.5);
  nh.param<double>("failsafe/imu_timeout", imu_timeout_, 0.2);
  nh.param<double>("failsafe/lidar_fail_rgbd_fresh_window_sec",
                   lidar_fail_rgbd_fresh_window_sec_, 0.40);
  lidar_fail_rgbd_fresh_window_sec_ =
      std::max(0.20, std::min(1.20, lidar_fail_rgbd_fresh_window_sec_));
  nh.param<int>("failsafe/camera_recovery_samples", camera_recovery_samples_, 5);
  nh.param<int>("failsafe/lidar_recovery_samples", lidar_recovery_samples_, 3);
  nh.param<int>("failsafe/imu_recovery_samples", imu_recovery_samples_, 20);
  nh.param<int>("failsafe/min_lidar_points", min_lidar_points_, 50);
  nh.param<int>("failsafe/min_visual_constraints", min_visual_constraints_, 8);
  nh.param<bool>("failsafe/healthy_visual_gate_enable", healthy_visual_gate_enable_, true);
  nh.param<bool>("failsafe/healthy_visual_pose_fusion_enable", healthy_visual_pose_fusion_enable_, false);
  nh.param<bool>("failsafe/healthy_lidar_update_guard_enable", healthy_lidar_update_guard_enable_, false);
  nh.param<double>("failsafe/healthy_lidar_max_update_translation_m", healthy_lidar_max_update_translation_m_, 0.75);
  nh.param<double>("failsafe/healthy_lidar_max_update_rotation_deg", healthy_lidar_max_update_rotation_deg_, 7.0);
  nh.param<double>("failsafe/healthy_lidar_max_update_velocity_mps", healthy_lidar_max_update_velocity_mps_, 3.0);
  nh.param<int>("failsafe/healthy_visual_min_points", healthy_visual_min_points_, 24);
  nh.param<int>("failsafe/healthy_visual_bootstrap_frames", healthy_visual_bootstrap_frames_, 24);
  nh.param<int>("failsafe/healthy_visual_reenable_samples", healthy_visual_reenable_samples_, 8);
  nh.param<double>("failsafe/healthy_visual_min_confidence", healthy_visual_min_confidence_, 0.55);
  nh.param<double>("failsafe/healthy_visual_reenable_confidence", healthy_visual_reenable_confidence_, 0.70);
  nh.param<double>("failsafe/healthy_visual_max_translation_update", healthy_visual_max_translation_update_, 0.20);
  nh.param<double>("failsafe/healthy_visual_max_rotation_deg", healthy_visual_max_rotation_deg_, 2.0);
  nh.param<double>("failsafe/healthy_visual_max_velocity_update", healthy_visual_max_velocity_update_, 1.0);
  nh.param<double>("failsafe/no_imu_covariance_scale", no_imu_covariance_scale_, 5.0);
  nh.param<double>("failsafe/camera_fail_covariance_scale", camera_fail_covariance_scale_, 1.05);
  nh.param<double>("failsafe/lidar_fail_covariance_scale", lidar_fail_covariance_scale_, 1.10);
  nh.param<double>("failsafe/imu_bridge_covariance_scale", imu_bridge_covariance_scale_, 1.50);
  nh.param<double>("failsafe/lidar_rejoin_max_translation_jump", lidar_rejoin_max_translation_jump_, 0.8);
  nh.param<double>("failsafe/lidar_rejoin_max_rotation_deg", lidar_rejoin_max_rotation_deg_, 20.0);
  nh.param<double>("failsafe/lidar_fail_visual_max_translation_update", lidar_fail_visual_max_translation_update_, 0.35);
  nh.param<double>("failsafe/lidar_fail_visual_max_rotation_deg", lidar_fail_visual_max_rotation_deg_, 5.0);
  nh.param<double>("failsafe/lidar_fail_visual_max_velocity_update", lidar_fail_visual_max_velocity_update_, 1.5);
  nh.param<bool>("failsafe/map_correction_enable", map_correction_enable_, true);
  nh.param<double>("failsafe/map_correction_max_age", map_correction_max_age_, 0.5);
  nh.param<double>("failsafe/map_correction_max_translation_jump", map_correction_max_translation_jump_, 2.5);
  nh.param<double>("failsafe/map_correction_max_rotation_deg", map_correction_max_rotation_deg_, 20.0);
  nh.param<double>("failsafe/map_correction_blend", map_correction_blend_, 1.0);
  nh.param<double>("failsafe/smooth_map_max_pos_rate", smooth_map_max_pos_rate_, 1.3);
  nh.param<double>("failsafe/smooth_map_max_rot_rate_deg", smooth_map_max_rot_rate_deg_, 16.0);
  nh.param<double>("failsafe/smooth_map_max_pos_rate_lidar_fail",
                   smooth_map_max_pos_rate_lidar_fail_, 1.3);
  nh.param<double>("failsafe/smooth_map_max_rot_rate_lidar_fail_deg",
                   smooth_map_max_rot_rate_lidar_fail_deg_, 0.0);
  nh.param<double>("failsafe/smooth_map_max_pos_rate_imu_fail",
                   smooth_map_max_pos_rate_imu_fail_, 3.0);
  nh.param<double>("failsafe/smooth_map_max_rot_rate_imu_fail_deg",
                   smooth_map_max_rot_rate_imu_fail_deg_, 30.0);
  nh.param<double>("failsafe/rgbd_displacement_gain", rgbd_displacement_gain_, 0.75);
  nh.param<double>("failsafe/rgbd_displacement_max_step", rgbd_displacement_max_step_, 0.30);
  nh.param<double>("failsafe/rgbd_velocity_weight", rgbd_velocity_weight_, 0.70);
  nh.param<bool>("failsafe/relative_delta_residual_enable",
                 relative_delta_residual_enable_, true);
  nh.param<bool>("failsafe/rgbd_delta_residual_enable",
                 rgbd_delta_residual_enable_, true);
  nh.param<bool>("failsafe/lidar_delta_residual_enable",
                 lidar_delta_residual_enable_, true);
  nh.param<bool>("failsafe/lidar_delta_direct_state_update",
                 lidar_delta_direct_state_update_, true);
  nh.param<bool>("failsafe/rgbd_delta_direct_position_update",
                 rgbd_delta_direct_position_update_, true);
  nh.param<double>("failsafe/rgbd_delta_position_gain",
                   rgbd_delta_position_gain_, 1.0);
  nh.param<double>("failsafe/rgbd_delta_rotation_gain",
                   rgbd_delta_rotation_gain_, 0.20);
  nh.param<double>("failsafe/rgbd_delta_max_position_step",
                   rgbd_delta_max_position_step_, 0.45);
  nh.param<double>("failsafe/rgbd_delta_max_rotation_step_deg",
                   rgbd_delta_max_rotation_step_deg_, 2.0);
  nh.param<double>("failsafe/lidar_delta_position_gain",
                   lidar_delta_position_gain_, 0.90);
  nh.param<double>("failsafe/lidar_delta_rotation_gain",
                   lidar_delta_rotation_gain_, 0.85);
  nh.param<double>("failsafe/lidar_delta_max_position_step",
                   lidar_delta_max_position_step_, 0.80);
  nh.param<double>("failsafe/lidar_delta_max_rotation_step_deg",
                   lidar_delta_max_rotation_step_deg_, 6.0);
  nh.param<bool>("failsafe/map_depth_primary_on_lidar_failure",
                 map_depth_primary_on_lidar_failure_, false);
  nh.param<bool>("failsafe/map_primary_on_imu_failure",
                 map_primary_on_imu_failure_, false);
  nh.param<int>("failsafe/recovery_lio_warmup_scans",
                recovery_lio_warmup_scans_, 5);

  nh.param<bool>("step3/prior_map_enable", step3_prior_enable_, false);
  nh.param<bool>("step3/replace_local_voxel_map",
                 step3_replace_local_voxel_map_, false);
  nh.param<std::string>("step3/prior_map_path", step3_prior_map_path_, std::string());
  nh.param<double>("step3/prior_map_leaf", step3_prior_leaf_, 0.30);
  nh.param<double>("step3/prior_point_variance", step3_prior_point_variance_, 0.01);
  nh.param<bool>("step3/tracking_correction_enable",
                 step3_tracking_correction_enable_, true);
  nh.param<bool>("step3/gyro_bias_feedback_enable",
                 step3_gyro_bias_feedback_enable_, false);
  nh.param<bool>("step3/anchored_local_feedback_enable",
                 step3_anchored_local_feedback_enable_, false);
  nh.param<double>("step3/tracking_max_age", step3_tracking_max_age_, 1.5);
  nh.param<double>("step3/tracking_xy_gain", step3_tracking_xy_gain_, 0.65);
  nh.param<double>("step3/tracking_max_xy_step", step3_tracking_max_xy_step_, 0.30);
  nh.param<double>("step3/tracking_yaw_gain", step3_tracking_yaw_gain_, 0.55);
  nh.param<double>("step3/tracking_max_yaw_step_deg",
                   step3_tracking_max_yaw_step_deg_, 1.0);
  nh.param<double>("step3/tracking_hard_xy_gate",
                   step3_tracking_hard_xy_gate_, 2.0);
  nh.param<double>("step3/tracking_hard_yaw_gate_deg",
                   step3_tracking_hard_yaw_gate_deg_, 10.0);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

const char *LIVMapper::sensorHealthName(SensorHealth health)
{
  switch (health)
  {
    case SensorHealth::UNKNOWN: return "UNKNOWN";
    case SensorHealth::HEALTHY: return "HEALTHY";
    case SensorHealth::DEGRADED: return "DEGRADED";
    case SensorHealth::FAILED: return "FAILED";
    case SensorHealth::RECOVERING: return "RECOVERING";
  }
  return "UNKNOWN";
}

const char *LIVMapper::failsafeModeName(FailsafeMode mode)
{
  switch (mode)
  {
    case FailsafeMode::FULL_LIVO: return "FULL_LIVO";
    case FailsafeMode::LIO_CAMERA_FAILED: return "LIO_CAMERA_FAILED";
    case FailsafeMode::LV_IMU_FAILED: return "LV_IMU_FAILED";
    case FailsafeMode::VI_LIDAR_FAILED: return "VI_LIDAR_FAILED";
    case FailsafeMode::IMU_ONLY_BRIDGE: return "IMU_ONLY_BRIDGE";
    case FailsafeMode::INVALID: return "INVALID";
  }
  return "INVALID";
}

void LIVMapper::markSensorSample(SensorHealth &health, int &recovery_count,
                                 int required_samples, const char *name)
{
  if (health == SensorHealth::UNKNOWN)
  {
    health = SensorHealth::HEALTHY;
    recovery_count = 0;
    ROS_INFO("[FAILSAFE] %s healthy", name);
    return;
  }
  if (health == SensorHealth::DEGRADED)
  {
    // A packet arriving proves availability, not measurement quality.
    // Confidence/innovation checks decide when DEGRADED becomes HEALTHY.
    return;
  }
  if (health == SensorHealth::FAILED)
  {
    health = SensorHealth::RECOVERING;
    recovery_count = 1;
    ROS_WARN("[FAILSAFE] %s recovering", name);
    return;
  }
  if (health == SensorHealth::RECOVERING)
  {
    if (++recovery_count >= std::max(1, required_samples))
    {
      health = SensorHealth::HEALTHY;
      recovery_count = 0;
      ROS_INFO("[FAILSAFE] %s healthy after recovery window", name);
    }
  }
}

double LIVMapper::currentUpdateTime() const
{
  if (LidarMeasures.measures.empty()) return LidarMeasures.last_lio_update_time;
  return LidarMeasures.lio_vio_flg == VIO ? LidarMeasures.measures.back().vio_time
                                          : LidarMeasures.measures.back().lio_time;
}

void LIVMapper::trimStaleBuffers(double estimator_time)
{
  if (!std::isfinite(estimator_time) || estimator_time < 0.0) return;
  std::lock_guard<std::mutex> lock(mtx_buffer);
  while (!lid_header_time_buffer.empty() && lid_header_time_buffer.front() <= estimator_time)
  {
    lid_header_time_buffer.pop_front();
    lid_raw_data_buffer.pop_front();
  }
  while (!img_time_buffer.empty() && img_time_buffer.front() + exposure_time_init <= estimator_time)
  {
    img_time_buffer.pop_front();
    img_buffer.pop_front();
  }
  while (!imu_buffer.empty() && imu_buffer.front()->header.stamp.toSec() <= estimator_time)
    imu_buffer.pop_front();
}

void LIVMapper::inflateFallbackCovariance(double scale)
{
  const double q = std::max(0.0, scale - 1.0);
  if (q <= 0.0) return;
  _state.cov.block<3, 3>(0, 0).diagonal().array() += 1.0e-5 * q;
  _state.cov.block<3, 3>(3, 3).diagonal().array() += 1.0e-4 * q;
  _state.cov.block<3, 3>(7, 7).diagonal().array() += 1.0e-3 * q;
}

void LIVMapper::setFailsafeMode(FailsafeMode mode, const std::string &reason)
{
  if (mode == failsafe_mode_) return;
  const FailsafeMode previous = failsafe_mode_;

  // Do not return to normal fusion until the recovered sensor has been tied
  // back to the immutable saved map.  This prevents rebuilding the local voxel
  // map around an inertially drifted pose.
  if (mode == FailsafeMode::FULL_LIVO &&
      (previous == FailsafeMode::VI_LIDAR_FAILED ||
       previous == FailsafeMode::IMU_ONLY_BRIDGE) &&
      map_depth_primary_on_lidar_failure_ && !lidar_recovery_map_lock_)
  {
    ROS_WARN_THROTTLE(0.5,
        "[FAILSAFE] Waiting for returning-LiDAR saved-map relocalization before FULL_LIVO");
    return;
  }
  if (mode == FailsafeMode::FULL_LIVO &&
      previous == FailsafeMode::LV_IMU_FAILED &&
      map_primary_on_imu_failure_)
  {
    const double age = last_applied_map_correction_stamp_.isZero()
        ? 1.0e9
        : std::abs(ros::Time::now().toSec() -
                   last_applied_map_correction_stamp_.toSec());
    const bool saved_lidar_lock =
        last_applied_map_correction_source_ == "lidar_saved_map_icp" ||
        last_applied_map_correction_source_ == "lidar_saved_map_esikf" ||
        last_applied_map_correction_source_ == "lidar_place_verified_map" ||
        last_applied_map_correction_source_ == "lidar_place_verified_esikf";
    if (!saved_lidar_lock || age > 1.6)
    {
      ROS_WARN_THROTTLE(0.5,
          "[FAILSAFE] Waiting for fresh verified LiDAR saved-map correction before IMU rejoin");
      return;
    }
  }
  double estimator_time = LidarMeasures.last_lio_update_time;
  const bool recovering_to_full =
      mode == FailsafeMode::FULL_LIVO &&
      (previous == FailsafeMode::LV_IMU_FAILED ||
       previous == FailsafeMode::VI_LIDAR_FAILED ||
       previous == FailsafeMode::IMU_ONLY_BRIDGE);
  if (recovering_to_full)
  {
    const double update_time = currentUpdateTime();
    if (std::isfinite(update_time))
      estimator_time = std::max(estimator_time, update_time);
    if (!fallback_increment_stamp_.isZero())
      estimator_time = std::max(estimator_time, fallback_increment_stamp_.toSec());
    if (previous == FailsafeMode::LV_IMU_FAILED &&
        !last_lidar_relative_stamp_.isZero())
      estimator_time = std::max(estimator_time, last_lidar_relative_stamp_.toSec());
    if ((previous == FailsafeMode::VI_LIDAR_FAILED ||
         previous == FailsafeMode::IMU_ONLY_BRIDGE) &&
        !last_rgbd_relative_stamp_.isZero())
      estimator_time = std::max(estimator_time, last_rgbd_relative_stamp_.toSec());
    // Do NOT advance the estimator trim point to ros::Time::now() here.
    // During real-time rosbag playback (and on a loaded vehicle computer),
    // /clock can be noticeably ahead of the sensor callbacks waiting in ROS
    // queues.  Using wall/sim "now" as an estimator timestamp discards valid
    // LiDAR/IMU/image measurements that have arrived late but have not yet
    // been fused.  A brief simultaneous sensor timeout would then recover to
    // FULL_LIVO with empty synchronization buffers ("[LIO]: No point!!!") and
    // the estimator could stop publishing permanently.  Recovery trimming
    // must only use timestamps that the estimator or a degraded measurement
    // source has actually consumed.
    ROS_WARN("[FAILSAFE] recovery handoff trim time %.6f (last_lio %.6f, update %.6f)",
             estimator_time, LidarMeasures.last_lio_update_time, update_time);
  }
  const bool visual_bridge_only_change =
      (previous == FailsafeMode::VI_LIDAR_FAILED && mode == FailsafeMode::IMU_ONLY_BRIDGE) ||
      (previous == FailsafeMode::IMU_ONLY_BRIDGE && mode == FailsafeMode::VI_LIDAR_FAILED);
  if (!visual_bridge_only_change)
  {
    trimStaleBuffers(estimator_time);
    LidarMeasures.measures.clear();
    LidarMeasures.lio_vio_flg = WAIT;
    lidar_pushed = false;
    if (LidarMeasures.pcl_proc_cur) LidarMeasures.pcl_proc_cur->clear();
    if (LidarMeasures.pcl_proc_next) LidarMeasures.pcl_proc_next->clear();
  }

  if (mode == FailsafeMode::FULL_LIVO &&
      (previous == FailsafeMode::LV_IMU_FAILED ||
       previous == FailsafeMode::VI_LIDAR_FAILED ||
       previous == FailsafeMode::IMU_ONLY_BRIDGE))
  {
    // A correction produced while degraded must never survive into a later
    // outage. Recovery starts from the state already corrected by the saved
    // map, then normal sensor updates take over.
    {
      std::lock_guard<std::mutex> lock(mtx_map_correction_);
      map_correction_pending_ = false;
    }
  }

  if (mode == FailsafeMode::FULL_LIVO && previous == FailsafeMode::LV_IMU_FAILED)
  {
    lidar_rejoin_guard_ = true;
    recovery_from_imu_outage_ = true;

    // Re-establish gravity tilt from the fresh IMU recovery window while
    // preserving the LiDAR-relative yaw, position and velocity.  During the
    // no-IMU interval only yaw is observable from scan-to-scan LiDAR, so stale
    // roll/pitch can make gravity look like linear acceleration after rejoin.
    if (imu_recovery_sample_count_ >= 5)
    {
      const V3D acc_body = imu_recovery_acc_sum_ /
                           static_cast<double>(imu_recovery_sample_count_);
      if (acc_body.allFinite() && acc_body.norm() > 1.0 &&
          _state.gravity.allFinite() && _state.gravity.norm() > 1.0)
      {
        const V3D target_world = -_state.gravity.normalized();
        const V3D measured_world = (_state.rot_end * acc_body.normalized()).normalized();
        double c = std::max(-1.0, std::min(1.0, measured_world.dot(target_world)));
        const double raw_angle = std::acos(c);
        V3D axis = measured_world.cross(target_world);
        if (axis.norm() > 1.0e-6 && std::isfinite(raw_angle))
        {
          const double max_corr = std::max(0.0, imu_rejoin_max_tilt_correction_deg_) * M_PI / 180.0;
          const double corr = std::min(raw_angle, max_corr);
          const double yaw_before = RotMtoEuler(_state.rot_end)(2);
          M3D candidate = Eigen::AngleAxisd(corr, axis.normalized()).toRotationMatrix() * _state.rot_end;
          const V3D rpy = RotMtoEuler(candidate);
          const M3D Rz = Eigen::AngleAxisd(yaw_before, V3D::UnitZ()).toRotationMatrix();
          const M3D Ry = Eigen::AngleAxisd(rpy(1), V3D::UnitY()).toRotationMatrix();
          const M3D Rx = Eigen::AngleAxisd(rpy(0), V3D::UnitX()).toRotationMatrix();
          _state.rot_end = Rz * Ry * Rx;
          state_propagat.rot_end = _state.rot_end;
          voxelmap_manager->state_.rot_end = _state.rot_end;
          ROS_WARN("[FAILSAFE] IMU rejoin gravity-tilt correction %.2f deg from %d fresh samples",
                   corr * 180.0 / M_PI, imu_recovery_sample_count_);
        }
      }
    }

    sensor_msgs::Imu::ConstPtr seed;
    if (!imu_buffer.empty())
    {
      sensor_msgs::Imu::Ptr copy(new sensor_msgs::Imu(*imu_buffer.front()));
      copy->header.stamp = ros::Time().fromSec(std::max(0.0, estimator_time));
      seed = copy;
    }
    p_imu->ResyncAfterImuOutage(estimator_time, seed);
    map_correction_prev_valid_ = false;
    // The live/local FAST-LIVO2 voxel map was frozen throughout the IMU
    // outage while the persistent ESIKF state kept moving on LiDAR-relative
    // increments. Rebuild local geometry unconditionally before normal
    // LIO/VIO takes authority again, even when global PCD matching is off.
    rebuild_voxel_map_on_recovery_ = true;
    recovery_lio_warmup_remaining_ = std::max(1, recovery_lio_warmup_scans_);
  }
  if (mode == FailsafeMode::FULL_LIVO &&
      (previous == FailsafeMode::VI_LIDAR_FAILED || previous == FailsafeMode::IMU_ONLY_BRIDGE))
  {
    lidar_rejoin_guard_ = true;
    // LiDAR loss also leaves the live local voxel map at the pre-outage
    // location. Rebuild it around the current ESIKF pose regardless of
    // Step-3/global-map settings. Arm warm-up now so VIO cannot touch stale
    // geometric support before the first returning LiDAR scan seeds the map.
    rebuild_voxel_map_on_recovery_ = true;
    recovery_lio_warmup_remaining_ = std::max(1, recovery_lio_warmup_scans_);
  }
  if (mode == FailsafeMode::LV_IMU_FAILED && previous != FailsafeMode::LV_IMU_FAILED)
  {
    imu_recovery_acc_sum_.setZero();
    imu_recovery_gyr_sum_.setZero();
    imu_recovery_sample_count_ = 0;
    recovery_from_imu_outage_ = false;
    map_correction_prev_valid_ = false;
    degraded_map_alignment_valid_ = false;
    smooth_map_target_valid_ = false;
    smooth_map_last_apply_time_ = ros::Time(0);
    verified_map_motion_valid_ = false;
    last_verified_map_motion_stamp_ = ros::Time(0);
    fallback_increment_anchor_valid_ = true;
    fallback_increment_rot_ = _state.rot_end;
    fallback_increment_pos_ = _state.pos_end;
    fallback_increment_stamp_ = ros::Time(0);
    fallback_increment_source_.clear();
  }
  if ((mode == FailsafeMode::VI_LIDAR_FAILED ||
       mode == FailsafeMode::IMU_ONLY_BRIDGE) &&
      previous == FailsafeMode::FULL_LIVO)
  {
    lidar_recovery_map_lock_ = false;
    degraded_map_alignment_valid_ = false;
    // Preserve the last healthy velocity in the BODY frame. During LiDAR
    // loss the healthy IMU rotates this vector with the vehicle, which enforces
    // the car's non-holonomic motion instead of continuing in a stale world
    // direction through turns.
    lidar_fail_velocity_reference_ =
        _state.rot_end.transpose() * _state.vel_end;
    lidar_fail_velocity_valid_ = true;
    rgbd_displacement_constraint_valid_ = false;
    rgbd_displacement_constraint_stamp_ = ros::Time(0);
    fallback_increment_anchor_valid_ = true;
    fallback_increment_rot_ = _state.rot_end;
    fallback_increment_pos_ = _state.pos_end;
    fallback_increment_stamp_ = ros::Time(0);
    fallback_increment_source_.clear();
  }
  if (mode == FailsafeMode::FULL_LIVO &&
      (previous == FailsafeMode::LV_IMU_FAILED ||
       previous == FailsafeMode::VI_LIDAR_FAILED ||
       previous == FailsafeMode::IMU_ONLY_BRIDGE))
  {
    lidar_fail_velocity_valid_ = false;
    rgbd_displacement_constraint_valid_ = false;
    rgbd_displacement_constraint_stamp_ = ros::Time(0);
    degraded_map_alignment_valid_ = false;
    verified_map_motion_valid_ = false;
    last_verified_map_motion_stamp_ = ros::Time(0);
    fallback_increment_anchor_valid_ = false;
    fallback_increment_stamp_ = ros::Time(0);
    fallback_increment_source_.clear();
  }

  failsafe_mode_ = mode;
  ROS_WARN("[FAILSAFE] %s -> %s (%s)", failsafeModeName(previous),
           failsafeModeName(mode), reason.c_str());
  publishFailsafeStatus(true);
}

void LIVMapper::publishFailsafeStatus(bool force)
{
  if (!pubFailsafeMode || !pubSensorHealth) return;
  const double now = ros::Time::now().toSec();
  if (!force && last_status_publish_time_ > 0.0 && now - last_status_publish_time_ < 0.1) return;
  last_status_publish_time_ = now;
  std_msgs::String mode_msg;
  mode_msg.data = failsafeModeName(failsafe_mode_);
  pubFailsafeMode.publish(mode_msg);
  std_msgs::String health_msg;
  std::ostringstream ss;
  ss << "camera=" << sensorHealthName(camera_health_)
     << "(" << std::fixed << std::setprecision(2) << camera_confidence_ << ")"
     << ",lidar=" << sensorHealthName(lidar_health_)
     << "(" << lidar_confidence_ << ")"
     << ",imu=" << sensorHealthName(imu_health_)
     << "(" << imu_confidence_ << ")";
  health_msg.data = ss.str();
  pubSensorHealth.publish(health_msg);
}

void LIVMapper::updateFailsafeHealthAndMode()
{
  if (!failsafe_enable_) return;
  const double now = ros::Time::now().toSec();
  auto timeout = [now](SensorHealth &health, int &recovery_count,
                       double last_rx, double limit, const char *name)
  {
    if (last_rx <= 0.0 || now <= 0.0 || now - last_rx <= limit) return;
    if (health != SensorHealth::FAILED)
    {
      health = SensorHealth::FAILED;
      recovery_count = 0;
      ROS_ERROR("[FAILSAFE] %s timeout (%.3f s)", name, now - last_rx);
    }
  };
  if (img_en) timeout(camera_health_, camera_recovery_count_, last_camera_rx_time_, camera_timeout_, "Camera");
  if (lidar_en) timeout(lidar_health_, lidar_recovery_count_, last_lidar_rx_time_, lidar_timeout_, "LiDAR");
  if (imu_en) timeout(imu_health_, imu_recovery_count_, last_imu_rx_time_, imu_timeout_, "IMU");

  if (camera_health_ == SensorHealth::FAILED) camera_confidence_ = 0.0;
  if (lidar_health_ == SensorHealth::FAILED) lidar_confidence_ = 0.0;
  if (imu_health_ == SensorHealth::FAILED) imu_confidence_ = 0.0;

  auto usable = [](SensorHealth h) {
    return h == SensorHealth::HEALTHY || h == SensorHealth::DEGRADED;
  };
  // DEGRADED sensors remain available to the same ESIKF, but their
  // measurements are gated/down-weighted. Only FAILED/RECOVERING removes a
  // sensor from the active fusion subset.
  const bool camera_ok = !img_en || usable(camera_health_);
  const bool lidar_ok = !lidar_en || usable(lidar_health_);
  const bool imu_ok = !imu_en || usable(imu_health_);

  if (!failsafe_initialized_)
  {
    if (camera_ok && lidar_ok && imu_ok && lidar_map_inited)
    {
      failsafe_initialized_ = true;
      ROS_INFO("[FAILSAFE] FULL_LIVO armed; persistent LiDAR/visual map is initialized");
      publishFailsafeStatus(true);
    }
    return;
  }

  if (camera_ok && lidar_ok && imu_ok)
    setFailsafeMode(FailsafeMode::FULL_LIVO, "all sensors healthy");
  else if (!camera_ok && lidar_ok && imu_ok)
    setFailsafeMode(FailsafeMode::LIO_CAMERA_FAILED, "camera unavailable");
  else if (camera_ok && lidar_ok && !imu_ok)
    setFailsafeMode(FailsafeMode::LV_IMU_FAILED, "IMU unavailable");
  else if (camera_ok && !lidar_ok && imu_ok)
  {
    // Hysteresis prevents mode chatter when a single visual frame is weak.
    bool visual_mode_ok = camera_confidence_ >= 0.35;
    if (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED)
      visual_mode_ok = camera_confidence_ >= 0.20;
    else if (failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE)
      visual_mode_ok = camera_confidence_ >= 0.55;
    setFailsafeMode(visual_mode_ok ? FailsafeMode::VI_LIDAR_FAILED
                                   : FailsafeMode::IMU_ONLY_BRIDGE,
                    visual_mode_ok ? "LiDAR unavailable; visual confidence usable"
                                   : "LiDAR unavailable; visual confidence weak");
  }
  else
    setFailsafeMode(FailsafeMode::INVALID, "more than one required sensor unavailable");

  publishFailsafeStatus(false);
}

void LIVMapper::initializeComponents() 
{
  if (extrinT.size() != 3 || extrinR.size() != 9 ||
      cameraextrinT.size() != 3 || cameraextrinR.size() != 9)
    throw std::runtime_error("FAST-LIVO2 extrinsic arrays have invalid lengths");

  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);
  M3D camera_rotation;
  camera_rotation << MAT_FROM_ARRAY(cameraextrinR);
  const double imu_rotation_error =
      (extR.transpose() * extR - M3D::Identity()).norm();
  const double camera_rotation_error =
      (camera_rotation.transpose() * camera_rotation - M3D::Identity()).norm();
  if (!extT.allFinite() || !extR.allFinite() ||
      !camera_rotation.allFinite() ||
      std::abs(extR.determinant() - 1.0) > 1.0e-3 ||
      std::abs(camera_rotation.determinant() - 1.0) > 1.0e-3 ||
      imu_rotation_error > 1.0e-3 || camera_rotation_error > 1.0e-3)
    throw std::runtime_error("FAST-LIVO2 extrinsic calibration is malformed");

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);
  step3_prior_voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  step3_prior_voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromRosNs("laserMapping", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->minimum_visual_update_points =
      std::max(1, std::max(min_visual_constraints_, healthy_visual_min_points_));
  vio_manager->gated_visual_fastpath_enable = gated_visual_fastpath_enable_;
  ros::param::param<bool>("/vio/diagnostic_tracking_only_en",
                          vio_manager->diagnostic_tracking_only_en, false);
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colorize_only_en = colorize_only_en;
  vio_manager->max_state_update_translation_m = healthy_visual_max_translation_update_;
  vio_manager->max_state_update_rotation_deg = healthy_visual_max_rotation_deg_;
  vio_manager->max_state_update_velocity_mps = healthy_visual_max_velocity_update_;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it) 
{
  // These used to be 200000. With raw 1280x720 images that allows ROS's
  // callback queue itself to retain gigabytes whenever processing falls even
  // slightly behind the bag/sensors. The algorithm already has explicit,
  // bounded internal synchronization buffers (10 LiDAR, 16 image, 300 IMU),
  // so keep only a small amount of ingress headroom here as well.
  constexpr uint32_t kRosLidarQueue = 20;
  constexpr uint32_t kRosImageQueue = 20;
  constexpr uint32_t kRosImuQueue = 600;
  sub_pcl = p_pre->lidar_type == AVIA ?
            nh.subscribe(lid_topic, kRosLidarQueue, &LIVMapper::livox_pcl_cbk, this):
            nh.subscribe(lid_topic, kRosLidarQueue, &LIVMapper::standard_pcl_cbk, this);
  sub_imu = nh.subscribe(imu_topic, kRosImuQueue, &LIVMapper::imu_cbk, this);
  sub_img = nh.subscribe(img_topic, kRosImageQueue, &LIVMapper::img_cbk, this);
  sub_map_correction = nh.subscribe("/fast_livo2/map_correction", 5,
                                    &LIVMapper::mapCorrectionCallback, this);
  sub_relative_motion_hint = nh.subscribe("/fast_livo2/relative_motion_hint", 20,
                                    &LIVMapper::relativeMotionHintCallback, this);
  sub_step3_global_lock = nh.subscribe("/fast_livo2/step3_global_lock", 2,
                                    &LIVMapper::step3GlobalLockCallback, this);
  sub_step3_tracking_correction = nh.subscribe(
      "/fast_livo2/step3_tracking_correction", 5,
      &LIVMapper::step3TrackingCorrectionCallback, this);
  
  // Visualization/debug cloud: never buffer a long history of full scans.
  // A queue of 100 can consume multiple GiB when RViz or another subscriber
  // renders slower than FAST-LIVO2 publishes. Keep only the newest frames.
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 2);
  pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 5);
  pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 2);
  pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 2);
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 2);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
  plane_pub = nh.advertise<visualization_msgs::Marker>("/planner_normal", 1);
  voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 2);
  pubLaserCloudDynRmed = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_removed", 2);
  pubLaserCloudDynDbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 2);
  mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = nh.advertise<nav_msgs::Odometry>("/LIVO2/imu_propagate", 20);
  pubFailsafeMode = nh.advertise<std_msgs::String>("/fast_livo2/failsafe_mode", 5, true);
  pubSensorHealth = nh.advertise<std_msgs::String>("/fast_livo2/sensor_health", 5, true);
  imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);
  voxelmap_manager->voxel_map_pub_= nh.advertise<visualization_msgs::MarkerArray>("/planes", 5);
  publishFailsafeStatus(true);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  const double now = ros::Time::now().toSec();
  auto age = [now](const ros::Time &stamp) -> double {
    return stamp.isZero() ? 1.0e9 : std::abs(now - stamp.toSec());
  };

  const double lidar_rel_age = age(last_lidar_relative_stamp_);
  const double depth_rel_age = age(last_depth_relative_stamp_);
  const double rgbd_rel_age = age(last_rgbd_relative_stamp_);

  if (failsafe_enable_ && failsafe_initialized_ &&
      failsafe_mode_ == FailsafeMode::LV_IMU_FAILED)
  {
    const double verified_map_age =
        last_verified_map_motion_stamp_.isZero()
            ? 1.0e9
            : std::abs(now - last_verified_map_motion_stamp_.toSec());

    if (verified_map_motion_valid_ && verified_map_age < 1.60)
    {
      if (verified_map_linear_world_.allFinite() &&
          verified_map_linear_world_.norm() < 15.0)
        _state.vel_end =
            0.15 * _state.vel_end + 0.85 * verified_map_linear_world_;
      p_imu->SetRuntimeAngularVelocityHint(verified_map_angular_body_);
    }
    else if (rgbd_rel_age < 0.60)
    {
      // IMU failure: RGB-D is the primary metric motion source.  Keep the
      // between-delta propagation consistent with the direct RGB-D updates;
      // otherwise fresh LiDAR velocity hints pull the state away between every
      // camera/depth observation.
      const V3D v_world = _state.rot_end * rgbd_relative_linear_body_;
      if (v_world.allFinite() && v_world.norm() < 12.0)
        _state.vel_end = 0.20 * _state.vel_end + 0.80 * v_world;
      p_imu->SetRuntimeAngularVelocityHint(rgbd_relative_angular_body_);
    }
    else if (lidar_rel_age < 0.75)
    {
      const V3D v_world = _state.rot_end * lidar_relative_linear_body_;
      if (v_world.allFinite() && v_world.norm() < 15.0)
        _state.vel_end = 0.35 * _state.vel_end + 0.65 * v_world;
      p_imu->SetRuntimeAngularVelocityHint(lidar_relative_angular_body_);
    }
    else
    {
      p_imu->SetRuntimeAngularVelocityHint(V3D::Zero());
    }

    p_imu->ProcessWithoutImuRuntime(
        LidarMeasures, _state, feats_undistort, no_imu_covariance_scale_);
  }
  else
  {
    // Recovery seeding is a special handoff cycle. The returning LiDAR scan
    // must be time-synchronized / undistorted so it can seed a fresh local
    // voxel map, but normal Process2 prediction must not move the persistent
    // ESIKF state before that map exists. Keep the degraded pose continuous,
    // advance only IMU/LiDAR bookkeeping inside ImuProcess, and let handleLIO
    // build the fresh map from feats_undistort on this same cycle.
    const bool imu_rejoin_state_hold =
        failsafe_enable_ && failsafe_initialized_ &&
        failsafe_mode_ == FailsafeMode::FULL_LIVO &&
        recovery_from_imu_outage_ && lidar_rejoin_guard_;
    const bool recovery_seed_cycle =
        failsafe_enable_ && failsafe_initialized_ &&
        failsafe_mode_ == FailsafeMode::FULL_LIVO &&
        rebuild_voxel_map_on_recovery_;
    const bool recovery_state_hold = recovery_seed_cycle || imu_rejoin_state_hold;
    if (recovery_state_hold)
    {
      const StatesGroup preserved_state = _state;
      p_imu->Process2(LidarMeasures, _state, feats_undistort);
      _state = preserved_state;
      ROS_INFO_THROTTLE(0.5,
          "[FAILSAFE] Recovery handoff: preserved ESIKF state while synchronizing returning scan");
    }
    else
    {
      p_imu->Process2(LidarMeasures, _state, feats_undistort);
    }

    if (failsafe_enable_ && failsafe_initialized_ &&
        (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
         failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE))
    {
      // IMU is still trusted for attitude. Use metric RGB-D visual odometry
      // for translation first; raw depth ICP is only a fallback.
      if (lidar_fail_velocity_valid_)
      {
        // Process2 above already integrated the healthy IMU acceleration.
        // Convert that velocity back into the vehicle frame and retain its
        // longitudinal acceleration while suppressing unconstrained lateral/
        // vertical drift. This is a ground-vehicle non-holonomic constraint.
        const V3D imu_body_velocity =
            _state.rot_end.transpose() * _state.vel_end;
        V3D body_ref = lidar_fail_velocity_reference_;

        if (imu_body_velocity.allFinite() &&
            std::abs(imu_body_velocity.x()) < 12.0)
        {
          // Let IMU acceleration change forward speed, but keep the estimate
          // smooth enough that one accelerometer spike cannot dominate.
          body_ref.x() =
              0.55 * body_ref.x() + 0.45 * imu_body_velocity.x();
          // A golf cart should have little sideslip/heave during normal driving.
          // Preserve a small lateral component instead of forcing exact zero.
          body_ref.y() =
              0.85 * body_ref.y() + 0.15 * imu_body_velocity.y();
          body_ref.z() *= 0.90;
        }

        // RGB-D/depth is the metric translation observation while LiDAR is
        // absent. The IMU remains attitude authority, but it must not be
        // allowed to keep a stale forward speed/direction when RGB-D observes
        // a turn or slowdown.
        V3D measured_body = V3D::Zero();
        bool measured_valid = false;
        if (rgbd_rel_age < 0.25)
        {
          measured_body = rgbd_relative_linear_body_;
          measured_valid = true;
        }
        else if (depth_rel_age < 1.0)
        {
          measured_body = depth_relative_linear_body_;
          measured_valid = true;
        }

        if (measured_valid && measured_body.allFinite())
        {
          if (measured_body.norm() < 6.0)
          {
            const double w = std::max(0.0, std::min(1.0, rgbd_velocity_weight_));
            body_ref.x() = (1.0 - w) * body_ref.x() + w * measured_body.x();
            // Lateral/heave are less observable/noisier for the cart, but the
            // measured lateral component is still needed through turns.
            body_ref.y() = 0.70 * body_ref.y() + 0.30 * measured_body.y();
            body_ref.z() = 0.90 * body_ref.z() + 0.10 * measured_body.z();
          }
        }

        // Constrain implausible reverse/lateral velocity for this recorded
        // forward-driving segment.
        body_ref.x() = std::max(-1.0, std::min(10.0, body_ref.x()));
        body_ref.y() = std::max(-1.5, std::min(1.5, body_ref.y()));
        body_ref.z() = std::max(-0.5, std::min(0.5, body_ref.z()));
        lidar_fail_velocity_reference_ = body_ref;

        const V3D v_world = _state.rot_end * body_ref;
        if (v_world.allFinite())
          _state.vel_end = v_world;
      }

      if (_state.vel_end.norm() > 12.0)
        _state.vel_end *= 12.0 / _state.vel_end.norm();
    }
  }

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;
}

void LIVMapper::stateEstimationAndMapping() 
{
  // There is always exactly one estimator: the original FAST-LIVO2 ESIKF
  // state.  Sensor failures change which measurement updates are available;
  // they never switch to a separate map-only estimator or reset the state.
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleMapOnlyFallback()
{
  // During an IMU outage the prebuilt PCD is the geometric authority.
  // ProcessWithoutImuRuntime supplies only a short prediction; the normal
  // voxel-map StateEstimation/UpdateVoxelMap path is intentionally frozen so
  // it cannot fight the accepted saved-map correction.
  inflateFallbackCovariance(no_imu_covariance_scale_);
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  if (imu_prop_enable)
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = currentUpdateTime();
    state_update_flg = true;
  }
}

void LIVMapper::resetLocalVoxelMapForRecovery()
{
  if (step3_prior_active_ && step3_replace_local_voxel_map_)
  {
    rebuild_voxel_map_on_recovery_ = false;
    recovery_lio_warmup_remaining_ = 0;
    last_recovery_warmup_lio_time_ = -1.0;
    lidar_rejoin_guard_ = true;
    ROS_WARN("[STEP3] preserving replacement prior voxel map through sensor recovery");
    return;
  }
  ROS_WARN("[FAILSAFE] Rebuilding local voxel map around corrected recovery pose");
  for (auto &kv : voxel_map)
    delete kv.second;
  voxel_map.clear();
  voxelmap_manager->pv_list_.clear();
  voxelmap_manager->ptpl_list_.clear();
  voxelmap_manager->cross_mat_list_.clear();
  voxelmap_manager->body_cov_list_.clear();
  _pv_list.clear();
  lidar_map_inited = false;
  // Keep the returning-LiDAR innovation guard armed. The first scan-to-map
  // correction after rebuilding must prove that it is consistent with the
  // continuous degraded ESIKF state before normal corrections are accepted.
  lidar_rejoin_guard_ = true;
  recovery_lio_warmup_remaining_ = std::max(0, recovery_lio_warmup_scans_);
  last_recovery_warmup_lio_time_ = -1.0;
  rebuild_voxel_map_on_recovery_ = false;
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub && !pcl_w_wait_pub->empty())
    std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  const bool recovery_visual_hold =
      recovery_lio_warmup_remaining_ > 0 ||
      (failsafe_enable_ && failsafe_initialized_ &&
       failsafe_mode_ == FailsafeMode::FULL_LIVO && lidar_rejoin_guard_);
  if (recovery_visual_hold)
  {
    // After a long degraded interval, rebuild and validate LiDAR geometric
    // support first. Do not let stale pre-outage visual-map associations pull
    // the continuous state before the first returning LiDAR update has passed
    // the explicit rejoin guard.
    euler_cur = RotMtoEuler(_state.rot_end);
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
        euler_cur(0), euler_cur(1), euler_cur(2));
    publish_odometry(pubOdomAftMapped);
    publish_path(pubPath);
    publish_mavros(mavros_pose_publisher);
    return;
  }

  const bool lidar_outage = failsafe_enable_ && failsafe_initialized_ &&
      (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED || failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE);
  if (lidar_outage)
  {
    // Original FAST-LIVO visual updates are tied to fresh LiDAR-supported
    // visual-map geometry. Reusing that geometry for tens of seconds after
    // LiDAR disappears produced multi-metre photometric jumps. In degraded
    // mode the RGB-D front-end now supplies the metric relative translation
    // measurement to this SAME ESIKF, and the saved PCD supplies its absolute
    // correction. The healthy IMU continues to own attitude propagation.
    const double rgbd_age = last_rgbd_relative_stamp_.isZero()
        ? 1.0e9
        : std::abs(ros::Time::now().toSec() - last_rgbd_relative_stamp_.toSec());
    visual_constraints_ok_ = rgbd_age < lidar_fail_rgbd_fresh_window_sec_;
    const double target = visual_constraints_ok_ ? 1.0 : 0.0;
    camera_confidence_ =
        std::max(0.0, std::min(1.0, 0.85 * camera_confidence_ + 0.15 * target));
    euler_cur = RotMtoEuler(_state.rot_end);
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
        euler_cur(0), euler_cur(1), euler_cur(2));
    publish_odometry(pubOdomAftMapped);
    publish_path(pubPath);
    publish_mavros(mavros_pose_publisher);
    return;
  }
  // During LiDAR loss the visual update remains part of the same ESIKF.
  // Saved-map/RGB-D measurements are supplementary corrections only; they
  // must never replace FAST-LIVO2's visual update path.
  // _pv_list is the last LiDAR-derived geometric support. Reuse it only to
  // retrieve existing visual-map voxels when LiDAR is absent; VIOManager is
  // told not to grow new LiDAR-derived visual geometry in that case.
  const StatesGroup visual_prediction = _state;
  bool visual_update_accepted = true;
  ++visual_frames_seen_;
  const bool visual_bootstrap_active =
      visual_frames_seen_ <= std::max(0, healthy_visual_bootstrap_frames_);

  bool allow_visual_state_update = !colorize_only_en;
  if (healthy_visual_gate_enable_ && allow_visual_state_update)
  {
    // Normal healthy motion stays on the stable LiDAR+IMU backbone after the
    // visual map has bootstrapped. Direct visual pose updates are allowed again
    // when LiDAR is unavailable, where camera support is genuinely required.
    const bool mode_allows_visual_pose =
        visual_bootstrap_active || healthy_visual_pose_fusion_enable_ || lidar_outage;
    const bool camera_measurement_healthy =
        camera_health_ == SensorHealth::HEALTHY &&
        (visual_bootstrap_active ||
         camera_confidence_ >= healthy_visual_min_confidence_);
    if (!mode_allows_visual_pose)
      allow_visual_state_update = false;
    if (mode_allows_visual_pose && visual_fusion_enabled_ && !camera_measurement_healthy)
    {
      visual_fusion_enabled_ = false;
      visual_recovery_good_frames_ = 0;
      ROS_WARN("[VIO GATE] visual ESIKF updates disabled: camera=%s confidence=%.2f",
               sensorHealthName(camera_health_), camera_confidence_);
    }
    if (mode_allows_visual_pose)
      allow_visual_state_update = visual_fusion_enabled_ && camera_measurement_healthy;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list,
                            voxelmap_manager->voxel_map_,
                            LidarMeasures.last_lio_update_time - _first_lidar_time,
                            !lidar_outage,
                            allow_visual_state_update);

  if (allow_visual_state_update && vio_manager->last_update_attempted &&
      !vio_manager->last_update_accepted)
  {
    // VIOManager already rolled the ESIKF back atomically.  Latch the visual
    // gate off so one rejected frame cannot be followed by another aggressive
    // photometric correction before visual support has rebuilt.
    visual_update_accepted = false;
    if (healthy_visual_gate_enable_)
    {
      visual_fusion_enabled_ = false;
      visual_recovery_good_frames_ = 0;
      ROS_WARN("[VIO GATE] rejected visual state update %.3fm / %.2fdeg / %.3fmps; falling back to LiDAR+IMU",
               vio_manager->last_update_translation_m,
               vio_manager->last_update_rotation_deg,
               vio_manager->last_update_velocity_mps);
    }
  }

  // A LiDAR outage removes the fresh 3-D support normally used by FAST-LIVO2
  // to validate visual associations.  Never allow one bad photometric update
  // to pull the persistent estimator far away from the IMU prediction.
  if (lidar_outage)
  {
    const double visual_translation_update =
        (_state.pos_end - visual_prediction.pos_end).norm();
    const double visual_velocity_update =
        (_state.vel_end - visual_prediction.vel_end).norm();
    const M3D visual_delta_R =
        visual_prediction.rot_end.transpose() * _state.rot_end;
    double cos_angle = (visual_delta_R.trace() - 1.0) * 0.5;
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    const double visual_rotation_update_deg =
        std::acos(cos_angle) * 180.0 / M_PI;
    const bool finite_update =
        _state.pos_end.allFinite() && _state.vel_end.allFinite() &&
        _state.rot_end.allFinite() && _state.cov.allFinite();
    const bool gated_update =
        !finite_update ||
        visual_translation_update > lidar_fail_visual_max_translation_update_ ||
        visual_rotation_update_deg > lidar_fail_visual_max_rotation_deg_ ||
        visual_velocity_update > lidar_fail_visual_max_velocity_update_;
    if (gated_update)
    {
      ROS_WARN("[FAILSAFE] Rejected visual-only correction: %.3f m / %.2f deg / %.3f mps",
               visual_translation_update, visual_rotation_update_deg,
               visual_velocity_update);
      _state = visual_prediction;
      voxelmap_manager->state_ = _state;
      visual_update_accepted = false;
    }
  }

  const int required_visual_points =
      std::max(min_visual_constraints_, healthy_visual_min_points_);
  const bool visual_support_strong =
      vio_manager->safe_visual_points >= required_visual_points;
  visual_constraints_ok_ = visual_update_accepted && visual_support_strong;
  {
    const double support = std::min(
        1.0, static_cast<double>(std::max(0, vio_manager->safe_visual_points)) /
                 static_cast<double>(std::max(4 * required_visual_points, 1)));
    double target = (visual_update_accepted || !allow_visual_state_update)
                        ? support : 0.0;
    if (visual_bootstrap_active)
      target = std::max(target, healthy_visual_reenable_confidence_);
    camera_confidence_ =
        std::max(0.0, std::min(1.0, 0.80 * camera_confidence_ + 0.20 * target));
    if (camera_health_ == SensorHealth::HEALTHY && camera_confidence_ < 0.25)
      camera_health_ = SensorHealth::DEGRADED;
    else if (camera_health_ == SensorHealth::DEGRADED && camera_confidence_ > 0.55)
      camera_health_ = SensorHealth::HEALTHY;

    if (healthy_visual_gate_enable_ && !visual_fusion_enabled_)
    {
      const bool recovery_frame_good =
          visual_support_strong &&
          camera_health_ != SensorHealth::FAILED &&
          camera_health_ != SensorHealth::RECOVERING &&
          camera_confidence_ >= healthy_visual_reenable_confidence_;
      visual_recovery_good_frames_ = recovery_frame_good
          ? visual_recovery_good_frames_ + 1 : 0;
      if (visual_recovery_good_frames_ >= healthy_visual_reenable_samples_)
      {
        visual_fusion_enabled_ = true;
        visual_recovery_good_frames_ = 0;
        ROS_INFO("[VIO GATE] visual ESIKF updates re-enabled after stable recovery; points=%d confidence=%.2f",
                 vio_manager->total_points, camera_confidence_);
      }
    }
  }
  if (lidar_outage)
  {
    // Mode transitions are handled centrally by updateFailsafeHealthAndMode()
    // using smoothed confidence + hysteresis. A single weak frame must not
    // change the active sensor subset.
    const bool visual_confident =
        visual_constraints_ok_ && camera_confidence_ >= 0.20;
    inflateFallbackCovariance(visual_confident ? lidar_fail_covariance_scale_
                                               : imu_bridge_covariance_scale_);
    euler_cur = RotMtoEuler(_state.rot_end);
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
    publish_odometry(pubOdomAftMapped);
    publish_path(pubPath);
    publish_mavros(mavros_pose_publisher);
  }

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  // The upstream visual sparse map is global and each Feature can retain a
  // cv::Mat reference to an old image. For long vehicle runs that makes RAM
  // grow with travelled distance. Localization needs only a local visual map,
  // so periodically delete buckets that are well behind the vehicle.
  static int visual_prune_counter = 0;
  if (!lidar_outage && visual_map_prune_every_n > 0 && ++visual_prune_counter % visual_map_prune_every_n == 0)
  {
    const size_t removed = vio_manager->pruneVisualMap(vio_manager->new_frame_->pos(), visual_map_radius);
    if (removed > 0)
    {
      ROS_INFO_THROTTLE(10.0,
                        "FAST-LIVO2 visual-map pruning active; removed %zu old voxels, %zu remain",
                        removed, vio_manager->feat_map.size());
    }
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{
  // IMU failure: never let the live local voxel map become a competing
  // geometric authority.  Keep the SAME FAST-LIVO2 ESIKF state, but freeze
  // normal voxel-map scan-to-map updates and let the saved-map verified
  // LiDAR/vision measurements + no-IMU propagation drive that state.
  if (failsafe_enable_ && failsafe_initialized_ &&
      failsafe_mode_ == FailsafeMode::LV_IMU_FAILED)
  {
    handleMapOnlyFallback();
    return;
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  bool seeded_recovery_map = false;
  if (rebuild_voxel_map_on_recovery_)
  {
    resetLocalVoxelMapForRecovery();
    seeded_recovery_map = true;
  }

  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();

    if (seeded_recovery_map)
    {
      // The first returning scan defines the rebuilt local voxel map. Matching
      // it immediately against itself produces an artificially perfect
      // measurement and can corrupt rejoin covariance/state. Keep the
      // saved-map-corrected propagated state and start scan-to-map matching on
      // the next LiDAR scan.
      _state = state_propagat;
      voxelmap_manager->state_ = _state;
      _pv_list.clear();
      if (recovery_lio_warmup_remaining_ > 0)
        --recovery_lio_warmup_remaining_;
      euler_cur = RotMtoEuler(_state.rot_end);
      geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
          euler_cur(0), euler_cur(1), euler_cur(2));
      publish_odometry(pubOdomAftMapped);
      publish_path(pubPath);
      publish_mavros(mavros_pose_publisher);
      ROS_INFO("[FAILSAFE] Seeded recovery voxel map; deferring scan-to-map update to next LiDAR scan");
      return;
    }
  }

  double t1 = omp_get_wtime();

  // During recovery warm-up, keep propagating the ONE ESIKF state but do not
  // allow scan-to-map or visual corrections to act on the newly seeded map.
  // The map needs a few temporally adjacent frames before it is a trustworthy
  // local geometric reference. The first correction after warm-up is still
  // protected by lidar_rejoin_guard_.
  if (recovery_lio_warmup_remaining_ > 0)
  {
    _state = state_propagat;
    voxelmap_manager->state_ = _state;

    // Grow the rebuilt local map with each UNIQUE returning LiDAR frame while
    // keeping the persistent ESIKF prediction-only. Previously these warm-up
    // scans were discarded, so the first enabled scan-to-map update matched
    // against an effectively one-scan map and could jump by metres.
    const double warmup_time = currentUpdateTime();
    if (last_recovery_warmup_lio_time_ < 0.0 ||
        warmup_time > last_recovery_warmup_lio_time_ + 1.0e-6)
    {
      transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
      voxelmap_manager->feats_down_world_ = feats_down_world;
      voxelmap_manager->feats_down_body_ = feats_down_body;
      voxelmap_manager->feats_down_size_ = feats_down_body->size();
      voxelmap_manager->BuildVoxelMap();
      last_recovery_warmup_lio_time_ = warmup_time;
      --recovery_lio_warmup_remaining_;
      ROS_INFO("[FAILSAFE] accumulated recovery LiDAR scan at %.6f; warmup remaining: %d",
               warmup_time, recovery_lio_warmup_remaining_);
    }

    euler_cur = RotMtoEuler(_state.rot_end);
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
        euler_cur(0), euler_cur(1), euler_cur(2));
    publish_odometry(pubOdomAftMapped);
    publish_path(pubPath);
    publish_mavros(mavros_pose_publisher);
    return;
  }

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  // Log the actual measurement update relative to the IMU-propagated prior.
  // ROS/rosbag pose continuity by itself does not detect pathological ESIKF
  // scan updates: the next local voxel-map insertion can then reinforce a
  // geometrically wrong basin. This guard is opt-in until cross-bag verified.
  bool healthy_lidar_update_rejected = false;
  if (failsafe_enable_ && failsafe_initialized_ &&
      failsafe_mode_ == FailsafeMode::FULL_LIVO && !lidar_rejoin_guard_)
  {
    const double delta_xyz = (_state.pos_end - state_propagat.pos_end).norm();
    const double delta_velocity = (_state.vel_end - state_propagat.vel_end).norm();
    const M3D delta_rot = state_propagat.rot_end.transpose() * _state.rot_end;
    const double cos_delta = std::max(-1.0, std::min(1.0,
                               (delta_rot.trace() - 1.0) * 0.5));
    const double delta_deg = std::acos(cos_delta) * 180.0 / M_PI;
    const bool sane = _state.pos_end.allFinite() && _state.vel_end.allFinite() &&
                      _state.rot_end.allFinite() && _state.cov.allFinite() &&
                      std::isfinite(delta_xyz) && std::isfinite(delta_velocity) &&
                      std::isfinite(delta_deg) &&
                      delta_xyz <= healthy_lidar_max_update_translation_m_ &&
                      delta_velocity <= healthy_lidar_max_update_velocity_mps_ &&
                      delta_deg <= healthy_lidar_max_update_rotation_deg_;
    ROS_INFO_THROTTLE(1.0,
         "[ESIKF LIO DIAG] t=%.6f update=%.3fm/%.2fdeg/%.3fmps points=%d residual=%.4f sane=%d gate=%d",
         LidarMeasures.last_lio_update_time, delta_xyz, delta_deg, delta_velocity,
         voxelmap_manager->effct_feat_num_,
         voxelmap_manager->last_average_residual_, sane ? 1 : 0,
         healthy_lidar_update_guard_enable_ ? 1 : 0);
    if (!sane && healthy_lidar_update_guard_enable_)
    {
      ROS_ERROR("[ESIKF LIO GUARD] rejecting implausible scan update at %.6f: %.3fm/%.2fdeg/%.3fmps (%d features)",
                LidarMeasures.last_lio_update_time, delta_xyz, delta_deg,
                delta_velocity, voxelmap_manager->effct_feat_num_);
      _state = state_propagat;
      voxelmap_manager->state_ = _state;
      _pv_list.clear();
      healthy_lidar_update_rejected = true;
    }
  }
  _pv_list = voxelmap_manager->pv_list_;
  if (step3_prior_active_ && !step3_replace_local_voxel_map_ &&
      step3_prior_voxelmap_manager &&
      !step3_prior_voxelmap_manager->voxel_map_.empty())
  {
    // First keep the exact Step-2 local FAST-LIVO2 solution.  The immutable
    // saved PCD is an additional GLOBAL measurement map, not a replacement
    // odometry/map state.
    StatesGroup local_state = _state;
    // Compute the independent saved-map measurement update from the SAME
    // propagated ESIKF prior used by the normal local-map update. Feeding the
    // already-converged local posterior into the global update made its
    // covariance so tight that a valid prior residual had effectively zero
    // authority. This is measurement branching, not a second estimator: only
    // one fused state survives below.
    StatesGroup prior_seed = state_propagat;
    step3_prior_voxelmap_manager->state_ = prior_seed;
    step3_prior_voxelmap_manager->feats_undistort_ = feats_undistort;
    step3_prior_voxelmap_manager->feats_down_body_ = feats_down_body;
    step3_prior_voxelmap_manager->feats_down_size_ = feats_down_size;
    step3_prior_voxelmap_manager->StateEstimation(prior_seed);
    const StatesGroup prior_state = step3_prior_voxelmap_manager->state_;

    // The global matcher is allowed to solve all 6 DoF internally, because
    // pitch/Z are coupled to scan-to-map correspondence quality.  However only
    // X/Y/yaw are allowed to update the vehicle state.  This keeps the good
    // global XY basin from the direct-prior experiment without importing its
    // vertical/tilt failure.
    const double yaw_local = std::atan2(local_state.rot_end(1, 0),
                                        local_state.rot_end(0, 0));
    const double yaw_prior = std::atan2(prior_state.rot_end(1, 0),
                                        prior_state.rot_end(0, 0));
    double dyaw = yaw_prior - yaw_local;
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
    const double candidate_dxy =
        std::hypot(prior_state.pos_end.x() - local_state.pos_end.x(),
                   prior_state.pos_end.y() - local_state.pos_end.y());
    const double candidate_dyaw_deg = std::abs(dyaw) * 180.0 / M_PI;
    const bool prior_candidate_ok =
        std::isfinite(candidate_dxy) && std::isfinite(candidate_dyaw_deg) &&
        candidate_dxy <= 0.35 && candidate_dyaw_deg <= 2.5 &&
        step3_prior_voxelmap_manager->effct_feat_num_ >= 120 &&
        std::isfinite(step3_prior_voxelmap_manager->last_average_residual_) &&
        step3_prior_voxelmap_manager->last_average_residual_ <= 0.35;

    _state = local_state;
    if (prior_candidate_ok)
    {
      _state.pos_end.x() = prior_state.pos_end.x();
      _state.pos_end.y() = prior_state.pos_end.y();
      _state.rot_end =
          Eigen::AngleAxisd(dyaw, V3D::UnitZ()).toRotationMatrix() *
          local_state.rot_end;
    }
    _state.cov = local_state.cov;
    voxelmap_manager->state_ = _state;

    ROS_INFO_THROTTLE(0.5,
        "[STEP3 DUAL PRIOR] %s dxy=%.3fm dyaw=%.2fdeg corr=%d residual=%.3f local_z=%.3f prior_z=%.3f",
        prior_candidate_ok ? "ACCEPT" : "REJECT",
        candidate_dxy, dyaw * 180.0 / M_PI,
        step3_prior_voxelmap_manager->effct_feat_num_,
        step3_prior_voxelmap_manager->last_average_residual_,
        local_state.pos_end.z(), prior_state.pos_end.z());
  }
  else if (step3_prior_active_ && step3_replace_local_voxel_map_)
  {
    ROS_WARN_THROTTLE(2.0,
        "[STEP3] replacement-map regression mode active");
  }
  applyPendingStep3TrackingCorrection();

  bool accept_lidar_update = !healthy_lidar_update_rejected;
  if (lidar_rejoin_guard_)
  {
    const double translation_jump = (_state.pos_end - state_propagat.pos_end).norm();
    const M3D delta_R = state_propagat.rot_end.transpose() * _state.rot_end;
    double cos_angle = (delta_R.trace() - 1.0) * 0.5;
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    const double rotation_jump_deg = std::acos(cos_angle) * 180.0 / M_PI;
    const double rejoin_rotation_limit_deg =
        recovery_from_imu_outage_ ? imu_rejoin_max_rotation_deg_
                                  : lidar_rejoin_max_rotation_deg_;
    if (translation_jump > lidar_rejoin_max_translation_jump_ ||
        rotation_jump_deg > rejoin_rotation_limit_deg)
    {
      ROS_ERROR("[FAILSAFE] Rejected returning LiDAR update: jump %.3f m / %.2f deg",
                translation_jump, rotation_jump_deg);
      _state = state_propagat;
      voxelmap_manager->state_ = _state;
      _pv_list.clear();
      accept_lidar_update = false;
    }
    else
    {
      ROS_INFO("[FAILSAFE] Returning LiDAR update accepted: jump %.3f m / %.2f deg",
               translation_jump, rotation_jump_deg);
      lidar_rejoin_guard_ = false;
      recovery_from_imu_outage_ = false;
    }
  }

  // Scan-to-map acceptance is the strongest online LiDAR confidence cue.
  if (accept_lidar_update)
    lidar_confidence_ = std::min(1.0, 0.90 * lidar_confidence_ + 0.10);
  else
    lidar_confidence_ = std::max(0.0, 0.65 * lidar_confidence_);

  if (failsafe_enable_ && failsafe_initialized_ && failsafe_mode_ == FailsafeMode::LIO_CAMERA_FAILED)
    inflateFallbackCovariance(camera_fail_covariance_scale_);

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  if (!accept_lidar_update)
  {
    publish_path(pubPath);
    publish_mavros(mavros_pose_publisher);
    return;
  }

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  if (!step3_prior_active_ || !step3_replace_local_voxel_map_)
  {
    voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
    std::cout << "[ LIO ] Update Voxel Map" << std::endl;
    if (step3_prior_active_)
      ROS_INFO_THROTTLE(5.0,
          "[STEP3] dual prior active; normal FAST-LIVO2 live map insertion preserved");
  }
  else
  {
    ROS_INFO_THROTTLE(5.0,
        "[STEP3] replacement prior active; live map insertion disabled");
  }
  _pv_list = voxelmap_manager->pv_list_;
  if (recovery_lio_warmup_remaining_ > 0 && accept_lidar_update)
  {
    --recovery_lio_warmup_remaining_;
    ROS_INFO("[FAILSAFE] recovery LiDAR warmup remaining: %d",
             recovery_lio_warmup_remaining_);
  }
  
  double t4 = omp_get_wtime();

  if ((!step3_prior_active_ || !step3_replace_local_voxel_map_) &&
      voxelmap_manager->config_setting_.map_sliding_en)
  {
    const size_t vox_before = voxel_map.size();
    voxelmap_manager->mapSliding();
    const size_t vox_after = voxel_map.size();
    if (vox_after < vox_before)
    {
#ifdef __GLIBC__
      malloc_trim(0);
#endif
      ROS_INFO_THROTTLE(5.0,
                        "FAST-LIVO2 sliding local map: %zu -> %zu root voxels (bounded live map)",
                        vox_before, vox_after);
    }
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
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

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string configured_pcd_path;
    ros::param::param<std::string>("pcd_save/pcd_path", configured_pcd_path, "");
    std::string raw_points_dir = configured_pcd_path.empty()
        ? std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd"
        : configured_pcd_path + ".raw.pcd";
    std::string downsampled_points_dir = configured_pcd_path.empty()
        ? std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd"
        : configured_pcd_path;
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::step3TrackingCorrectionCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (!step3_tracking_correction_enable_ || !msg) return;
  const auto &p = msg->pose.pose.position;
  const auto &qm = msg->pose.pose.orientation;
  Eigen::Quaterniond q(qm.w, qm.x, qm.y, qm.z);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.norm()) || q.norm() < 1.0e-9)
    return;
  q.normalize();
  std::lock_guard<std::mutex> lock(mtx_step3_tracking_);
  step3_tracking_stamp_ = msg->header.stamp;
  step3_tracking_pos_ << p.x, p.y, p.z;
  step3_tracking_rot_ = q.toRotationMatrix();
  step3_tracking_fitness_ = msg->pose.covariance[0];
  step3_tracking_measurement_is_residual_ =
      msg->header.frame_id == "step3_gicp_local_residual";
  step3_tracking_measurement_is_gyro_bias_ =
      msg->header.frame_id == "step3_verified_gyro_bias_error_rate";
  step3_tracking_measurement_is_anchored_local_ =
      msg->header.frame_id == "step3_verified_fixed_anchor_local_delta";
  step3_tracking_pending_ = true;
}

void LIVMapper::applyPendingStep3TrackingCorrection()
{
  if (!step3_tracking_correction_enable_ || !failsafe_initialized_) return;
  // Healthy/LiDAR+IMU operation first. Degraded modes keep their validated
  // Step-2 measurement paths until Step-3 healthy tracking is fully proven.
  if (lidar_health_ == SensorHealth::FAILED ||
      imu_health_ == SensorHealth::FAILED)
    return;

  ros::Time stamp;
  V3D target_pos;
  M3D target_rot;
  double fitness = std::numeric_limits<double>::infinity();
  bool measurement_is_residual = false;
  bool measurement_is_gyro_bias = false;
  bool measurement_is_anchored_local = false;
  {
    std::lock_guard<std::mutex> lock(mtx_step3_tracking_);
    if (!step3_tracking_pending_) return;
    stamp = step3_tracking_stamp_;
    target_pos = step3_tracking_pos_;
    target_rot = step3_tracking_rot_;
    fitness = step3_tracking_fitness_;
    measurement_is_residual = step3_tracking_measurement_is_residual_;
    measurement_is_gyro_bias = step3_tracking_measurement_is_gyro_bias_;
    measurement_is_anchored_local = step3_tracking_measurement_is_anchored_local_;
    step3_tracking_pending_ = false;
  }

  double now = currentUpdateTime();
  if (!std::isfinite(now) || now <= 0.0) now = ros::Time::now().toSec();
  if (!stamp.isZero() && std::abs(now - stamp.toSec()) > step3_tracking_max_age_)
  {
    ROS_WARN("[STEP3 TRACK ESIKF] stale correction age=%.3fs",
             std::abs(now - stamp.toSec()));
    return;
  }

  if (measurement_is_anchored_local)
  {
    // Three-dimensional map measurement (XY and yaw), expressed as an
    // INNOVATION in the unchanged initial FAST local frame. Unlike the
    // smoothly moving map<-FAST output anchor, this frame remains fixed.
    // Fuse only geometrically verified measurements in healthy FAST-LIVO2;
    // keep the local LiDAR voxel map and the one persistent ESIKF alive.
    if (!step3_anchored_local_feedback_enable_ ||
        failsafe_mode_ != FailsafeMode::FULL_LIVO ||
        !std::isfinite(fitness) || fitness > 0.12 ||
        !target_pos.allFinite()) return;
    const double dyaw = std::atan2(target_rot(1,0),target_rot(0,0));
    if (!std::isfinite(dyaw) || std::abs(dyaw)>0.6 ||
        target_pos.head<2>().norm()>30.0) return;
    constexpr double kMaxPosCorrection = 0.28;
    constexpr double kMaxYawCorrection = 1.25*M_PI/180.0;
    // The local map cannot constrain global gauge drift: prevent false
    // overconfidence in X/Y/yaw relative to an immutable surveyed-map frame.
    // Covariance floors represent unmodeled local-map drift process noise.
    constexpr double kXYCovFloor = 0.12*0.12;
    constexpr double kYawCovFloor = (2.0*M_PI/180.0)*(2.0*M_PI/180.0);
    Eigen::Matrix<double,DIM_STATE,DIM_STATE> P=_state.cov;
    for(int k : {2,3,4}){
      const double floor=k==2?kYawCovFloor:kXYCovFloor;
      P(k,k)=std::max(P(k,k),floor);
    }
    Eigen::Matrix<double,3,DIM_STATE> H=Eigen::Matrix<double,3,DIM_STATE>::Zero();
    H(0,3)=1.0; H(1,4)=1.0; H(2,2)=1.0;
    Eigen::Matrix3d R=Eigen::Matrix3d::Zero();
    const double xy_sigma=std::max(0.16,std::min(0.40,2.0*std::sqrt(fitness)));
    R(0,0)=R(1,1)=xy_sigma*xy_sigma;
    R(2,2)=std::pow(std::max(1.0,std::min(4.0,8.0*std::sqrt(fitness)))*M_PI/180.0,2);
    const Eigen::Matrix3d S=(H*P*H.transpose()+R).eval();
    if (!S.allFinite() || S.determinant()<=1e-16) return;
    Eigen::Matrix<double,DIM_STATE,3> K=P*H.transpose()*S.inverse();
    Eigen::Vector3d innovation;
    innovation<<target_pos.x(),target_pos.y(),dyaw;
    Eigen::Matrix<double,DIM_STATE,1> delta=K*innovation;
    double scale=1.0;
    const double dpos=delta.block<2,1>(3,0).norm();
    if(dpos>kMaxPosCorrection)scale=std::min(scale,kMaxPosCorrection/dpos);
    if(std::abs(delta(2))>kMaxYawCorrection)
      scale=std::min(scale,kMaxYawCorrection/std::abs(delta(2)));
    if(delta.block<3,1>(7,0).norm()>0.15)
      scale=std::min(scale,0.15/delta.block<3,1>(7,0).norm());
    if(!std::isfinite(scale) || scale<1e-5) return;
    K*=scale;delta*=scale;
    _state+=delta;
    const auto I=Eigen::Matrix<double,DIM_STATE,DIM_STATE>::Identity();
    const Eigen::Matrix<double,DIM_STATE,DIM_STATE> A=I-K*H;
    _state.cov=(A*P*A.transpose()+K*R*K.transpose()).eval();
    _state.cov=0.5*(_state.cov+_state.cov.transpose()).eval();
    voxelmap_manager->state_=_state;
    state_propagat=_state;
    if (imu_prop_enable) latest_ekf_state=_state;
    ROS_INFO("[STEP3 ANCHORED ESIKF] updated XY=%.3fm yaw=%.3fdeg residual=%.2fm/%.2fdeg fit=%.4f scale=%.3f",
             delta.block<2,1>(3,0).norm(),delta(2)*180/M_PI,
             target_pos.head<2>().norm(),dyaw*180/M_PI,fitness,scale);
    return;
  }

  // Opt-in zero-pose-jump ESIKF gyro-bias observation. Two successive strong
  // PCD/GICP heading measurements provide a relative yaw-rate check. This
  // does not overwrite FAST's position/rotation or the current local voxel map.
  // Use a scalar Joseph-form covariance update in the SAME error-state filter.
  if (measurement_is_gyro_bias)
  {
    if (!step3_gyro_bias_feedback_enable_ ||
        failsafe_mode_ != FailsafeMode::FULL_LIVO ||
        !std::isfinite(target_pos.z()) || std::abs(target_pos.z()) >= 0.06 ||
        !std::isfinite(fitness) || fitness > 0.12)
      return;
    constexpr int kBiasGzIndex = 12; // StatesGroup: gyro bias indices 10..12.
    constexpr double kObservedRateVariance = 0.00005; // (~0.4 deg/s)^2
    constexpr double kMaxBiasDelta = 0.0015; // rad/s per verified match
    const double variance = _state.cov(kBiasGzIndex, kBiasGzIndex);
    if (!std::isfinite(variance) || variance <= 0.0) return;
    Eigen::Matrix<double,DIM_STATE,1> K =
        _state.cov.col(kBiasGzIndex) / (variance+kObservedRateVariance);
    Eigen::Matrix<double,DIM_STATE,1> delta = K * target_pos.z();
    double scale = 1.0;
    if (std::abs(delta(kBiasGzIndex)) > kMaxBiasDelta)
      scale = std::min(scale,kMaxBiasDelta/std::abs(delta(kBiasGzIndex)));
    // Reject control-pose side effects even if the bias is correlated with
    // other elements of the ESIKF covariance.
    if (delta.block<3,1>(3,0).norm() > 0.025)
      scale = std::min(scale,0.025/delta.block<3,1>(3,0).norm());
    if (delta.block<3,1>(0,0).norm() > 0.002)
      scale = std::min(scale,0.002/delta.block<3,1>(0,0).norm());
    if (!std::isfinite(scale) || scale < 1e-5) return;
    K *= scale;
    delta *= scale;
    if (std::abs(_state.bias_g.z()+delta(kBiasGzIndex)) > 0.03)
      return;
    _state += delta;
    Eigen::Matrix<double,1,DIM_STATE> H = Eigen::Matrix<double,1,DIM_STATE>::Zero();
    H(kBiasGzIndex) = 1.0;
    const auto I = Eigen::Matrix<double,DIM_STATE,DIM_STATE>::Identity();
    const Eigen::Matrix<double,DIM_STATE,DIM_STATE> A = I-K*H;
    _state.cov = A*_state.cov*A.transpose() + kObservedRateVariance*K*K.transpose();
    _state.cov = 0.5*(_state.cov+_state.cov.transpose()).eval();
    voxelmap_manager->state_ = _state;
    state_propagat = _state;
    if (imu_prop_enable) latest_ekf_state = _state;
    ROS_INFO("[STEP3 GYRO BIAS ESIKF] accepted relative-rate=%.3fdeg/s delta_bias_gz=%.4fdeg/s bgz=%.4fdeg/s scale=%.3f",
             target_pos.z()*180.0/M_PI,delta(kBiasGzIndex)*180.0/M_PI,
             _state.bias_g.z()*180.0/M_PI,scale);
    return;
  }

  V2D residual_xy;
  double yaw_residual = 0.0;
  if (measurement_is_residual)
  {
    residual_xy << target_pos.x(), target_pos.y();
    yaw_residual = std::atan2(target_rot(1,0), target_rot(0,0));
  }
  else
  {
    residual_xy << target_pos.x() - _state.pos_end.x(),
                   target_pos.y() - _state.pos_end.y();
    const double yaw_now = std::atan2(_state.rot_end(1,0), _state.rot_end(0,0));
    const double yaw_target = std::atan2(target_rot(1,0), target_rot(0,0));
    yaw_residual = yaw_target - yaw_now;
  }
  const double residual_xy_norm = residual_xy.norm();
  while (yaw_residual > M_PI) yaw_residual -= 2.0 * M_PI;
  while (yaw_residual < -M_PI) yaw_residual += 2.0 * M_PI;
  const double yaw_residual_deg = std::abs(yaw_residual) * 180.0 / M_PI;

  if (!std::isfinite(residual_xy_norm) ||
      residual_xy_norm > step3_tracking_hard_xy_gate_ ||
      yaw_residual_deg > step3_tracking_hard_yaw_gate_deg_)
  {
    ROS_WARN("[STEP3 TRACK ESIKF] rejected target residual %.2fm/%.1fdeg fitness=%.4f",
             residual_xy_norm, yaw_residual_deg, fitness);
    return;
  }

  V2D step_xy = step3_tracking_xy_gain_ * residual_xy;
  const double step_xy_norm = step_xy.norm();
  if (step_xy_norm > step3_tracking_max_xy_step_ && step_xy_norm > 1.0e-9)
    step_xy *= step3_tracking_max_xy_step_ / step_xy_norm;

  const double max_yaw = step3_tracking_max_yaw_step_deg_ * M_PI / 180.0;
  double yaw_step = step3_tracking_yaw_gain_ * yaw_residual;
  yaw_step = std::max(-max_yaw, std::min(max_yaw, yaw_step));

  _state.pos_end.x() += step_xy.x();
  _state.pos_end.y() += step_xy.y();
  _state.rot_end = Eigen::AngleAxisd(yaw_step, V3D::UnitZ()).toRotationMatrix() *
                   _state.rot_end;
  voxelmap_manager->state_ = _state;
  state_propagat = _state;
  if (imu_prop_enable)
    latest_ekf_state = _state;

  ROS_INFO("[STEP3 TRACK ESIKF] applied xy=%.3fm yaw=%.2fdeg residual=%.3fm/%.2fdeg fitness=%.4f",
           step_xy.norm(), yaw_step * 180.0 / M_PI,
           residual_xy_norm, yaw_residual * 180.0 / M_PI, fitness);
}

void LIVMapper::step3GlobalLockCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (!step3_prior_enable_ || !msg || step3_prior_active_) return;
  Eigen::Quaterniond q(msg->pose.orientation.w,
                       msg->pose.orientation.x,
                       msg->pose.orientation.y,
                       msg->pose.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1.0e-9) return;
  q.normalize();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = q.toRotationMatrix();
  T(0,3) = msg->pose.position.x;
  T(1,3) = msg->pose.position.y;
  T(2,3) = msg->pose.position.z;
  if (!T.allFinite()) return;

  std::lock_guard<std::mutex> lock(mtx_step3_prior_);
  step3_T_map_fast_ = T;
  step3_prior_lock_pending_ = true;
  ROS_INFO("[STEP3] received global lock map<-camera_init xyz=[%.3f %.3f %.3f]",
           T(0,3), T(1,3), T(2,3));
}

bool LIVMapper::activateStep3PriorMap()
{
  if (!step3_prior_enable_ || step3_prior_active_ ||
      !step3_prior_map_mapframe_ || step3_prior_map_mapframe_->empty())
    return false;

  Eigen::Matrix4d T_map_fast = Eigen::Matrix4d::Identity();
  {
    std::lock_guard<std::mutex> lock(mtx_step3_prior_);
    if (!step3_prior_lock_pending_) return false;
    T_map_fast = step3_T_map_fast_;
  }

  const Eigen::Matrix4f T_fast_map = T_map_fast.inverse().cast<float>();
  PointCloudXYZI::Ptr prior_fast(new PointCloudXYZI());
  prior_fast->reserve(step3_prior_map_mapframe_->size());
  pcl::transformPointCloud(*step3_prior_map_mapframe_, *prior_fast, T_fast_map);

  ROS_WARN("[STEP3] activating immutable prior map in camera_init frame: points=%zu mode=%s",
           prior_fast->size(),
           step3_replace_local_voxel_map_ ? "replace-local-regression"
                                          : "dual-map-same-ESIKF");
  if (step3_replace_local_voxel_map_)
  {
    voxelmap_manager->BuildPriorMap(prior_fast, step3_prior_point_variance_);
    if (voxelmap_manager->voxel_map_.empty())
    {
      ROS_ERROR("[STEP3] replacement prior voxel-map build produced no voxels");
      return false;
    }
    lidar_map_inited = true;
  }
  else
  {
    step3_prior_voxelmap_manager->BuildPriorMap(
        prior_fast, step3_prior_point_variance_);
    if (step3_prior_voxelmap_manager->voxel_map_.empty())
    {
      ROS_ERROR("[STEP3] dual prior voxel-map build produced no voxels");
      return false;
    }
  }

  step3_prior_active_ = true;
  {
    std::lock_guard<std::mutex> lock(mtx_step3_prior_);
    step3_prior_lock_pending_ = false;
  }
  // Only the rejected replacement regression uses the returning-LiDAR guard,
  // because it swaps the primary map under the ESIKF.  Dual-map mode leaves
  // the validated Step-2 local estimator/map untouched.
  if (step3_replace_local_voxel_map_)
    lidar_rejoin_guard_ = true;
  recovery_from_imu_outage_ = false;
  recovery_lio_warmup_remaining_ = 0;
  rebuild_voxel_map_on_recovery_ = false;
  _pv_list.clear();
  step3_prior_map_mapframe_.reset(new PointCloudXYZI());
  if (step3_replace_local_voxel_map_)
    ROS_INFO("[STEP3] immutable prior replacement ACTIVE; dynamic LiDAR-map insertion disabled");
  else
    ROS_INFO("[STEP3] dual-map SAME-ESIKF tracking ACTIVE; local map remains live; prior constrains X/Y/yaw");
  return true;
}

void LIVMapper::run() 
{
  ros::Rate rate(5000);
  while (ros::ok()) 
  {
    ros::spinOnce();
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    if (step3_prior_enable_ && !step3_prior_active_ && step3_prior_lock_pending_)
      activateStep3PriorMap();

    // Absolute map corrections are produced by the saved-map fallback
    // localizer.  They are applied only while a required sensor is unavailable
    // and never reset the ESIKF, biases, gravity, or persistent maps.
    applyPendingMapCorrection();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::relativeMotionHintCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  if (!failsafe_enable_ || !failsafe_initialized_ || !msg) return;

  const V3D v_body(msg->twist.linear.x,
                   msg->twist.linear.y,
                   msg->twist.linear.z);
  const V3D omega_body(msg->twist.angular.x,
                       msg->twist.angular.y,
                       msg->twist.angular.z);

  // The successful standalone LiDAR-fail and IMU-fail implementations advance
  // pose with a FILTERED RELATIVE SE(3), not by replacing it with a velocity
  // heuristic.  Preserve that exact quantity here.  The anchor below is merely
  // the previous value of the ONE FAST-LIVO2 state; no parallel odometry is
  // maintained.
  const bool rgbd_delta = msg->header.frame_id == "rgbd_relative_delta";
  const bool lidar_delta = msg->header.frame_id == "lidar_relative_delta";
  const bool imu_rejoin_window =
      failsafe_mode_ == FailsafeMode::FULL_LIVO &&
      recovery_from_imu_outage_ && lidar_rejoin_guard_;
  const bool imu_failed_relative_mode =
      failsafe_mode_ == FailsafeMode::LV_IMU_FAILED || imu_rejoin_window;
  const bool delta_for_mode =
      (rgbd_delta &&
       (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
        failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE ||
        imu_failed_relative_mode)) ||
      (lidar_delta && imu_failed_relative_mode);
  if (delta_for_mode)
  {
    if (rgbd_delta)
      last_rgbd_relative_stamp_ = msg->header.stamp;
    else
      last_lidar_relative_stamp_ = msg->header.stamp;

    // With IMU unavailable both camera/depth and LiDAR survive.  Validation on
    // this bag shows the metric RGB-D relative frontend preserves motion scale
    // and turn rate substantially better than scan-to-scan LiDAR ICP.  Use
    // RGB-D as the primary relative measurement and keep LiDAR as a true
    // fallback only when RGB-D has gone stale.  This still updates the same
    // persistent FAST-LIVO2 ESIKF state; there is no second odometry state.
    if (lidar_delta && imu_failed_relative_mode &&
        !last_rgbd_relative_stamp_.isZero())
    {
      const double rgbd_age_at_lidar =
          std::abs((msg->header.stamp - last_rgbd_relative_stamp_).toSec());
      if (rgbd_age_at_lidar < 0.75)
        return;
    }

    const bool residual_enabled =
        relative_delta_residual_enable_ &&
        ((rgbd_delta && rgbd_delta_residual_enable_) ||
         (lidar_delta && lidar_delta_residual_enable_));
    if (!residual_enabled)
      return;

    // TwistStamped intentionally carries an SE(3) increment for *_delta:
    // linear = translation [m], angular = Rodrigues rotation vector [rad].
    // Build the measured previous-body <- current-body increment.
    const V3D delta_t = v_body;
    const V3D delta_rvec = omega_body;
    const double delta_angle = delta_rvec.norm();
    if (!delta_t.allFinite() || !delta_rvec.allFinite() ||
        delta_t.norm() > 2.5 || delta_angle > 0.80)
      return;

    M3D delta_R = M3D::Identity();
    if (delta_angle > 1.0e-9)
      delta_R = Eigen::AngleAxisd(delta_angle,
                                  delta_rvec / delta_angle).toRotationMatrix();

    const std::string delta_source = rgbd_delta ? "rgbd" : "lidar";
    const bool need_anchor =
        !fallback_increment_anchor_valid_ ||
        fallback_increment_source_ != delta_source ||
        fallback_increment_stamp_.isZero();

    // The first delta after a mode transition establishes the observation-time
    // anchor.  Applying it immediately would add motion already propagated by
    // the ESIKF before this asynchronous callback arrived.
    if (need_anchor)
    {
      fallback_increment_anchor_valid_ = true;
      fallback_increment_rot_ = _state.rot_end;
      fallback_increment_pos_ = _state.pos_end;
      fallback_increment_stamp_ = msg->header.stamp;
      fallback_increment_source_ = delta_source;
      return;
    }

    const double delta_dt =
        (msg->header.stamp - fallback_increment_stamp_).toSec();
    if (delta_dt <= 0.025 || delta_dt >= 0.40)
    {
      // Event-time discontinuity: re-anchor to the current ESIKF rather than
      // integrating a stale increment across the gap.
      fallback_increment_rot_ = _state.rot_end;
      fallback_increment_pos_ = _state.pos_end;
      fallback_increment_stamp_ = msg->header.stamp;
      return;
    }

    const M3D measured_rot = fallback_increment_rot_ * delta_R;
    const V3D measured_pos = fallback_increment_pos_ +
                             fallback_increment_rot_ * delta_t;

    // LiDAR-failure: the RGB-D front-end already publishes the exact filtered
    // body-frame SE(3) increment used by the successful standalone fallback.
    // Advance the ONE persistent ESIKF position from the previous accepted
    // state with that metric translation, while the healthy IMU remains the
    // sole attitude authority.  This avoids clipping each valid visual step to
    // a small residual and accumulating metres of lag.
    if (rgbd_delta && rgbd_delta_direct_position_update_ &&
        (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
         failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE))
    {
      const V3D previous_pos = fallback_increment_pos_;
      const M3D previous_rot = fallback_increment_rot_;
      _state.pos_end = measured_pos;

      const V3D measured_vel_world = previous_rot * (delta_t / delta_dt);
      if (measured_vel_world.allFinite() && measured_vel_world.norm() < 10.0)
      {
        const double w = std::max(0.0, std::min(1.0, rgbd_velocity_weight_));
        _state.vel_end = (1.0 - w) * _state.vel_end + w * measured_vel_world;
      }

      state_propagat = _state;
      voxelmap_manager->state_ = _state;
      fallback_increment_pos_ = _state.pos_end;
      // IMU owns orientation; seed the next visual translation with the actual
      // current IMU attitude rather than the visual rotation estimate.
      fallback_increment_rot_ = _state.rot_end;
      fallback_increment_stamp_ = msg->header.stamp;
      fallback_increment_source_ = delta_source;

      ROS_INFO_THROTTLE(0.5,
          "[FAILSAFE DELTA DIRECT] rgbd step=%.3fm dt=%.3fs speed=%.2fm/s",
          delta_t.norm(), delta_dt, measured_vel_world.norm());
      return;
    }

    // IMU failure: RGB-D is the primary metric relative-motion source while
    // the camera and depth sensor are healthy.  Integrate its planar SE(2)
    // increment directly into the ONE ESIKF state.  Body-Z is intentionally
    // constrained for the ground vehicle; yaw is observable from RGB-D motion
    // while roll/pitch remain at the continuous pre-failure gravity alignment.
    if (rgbd_delta && imu_failed_relative_mode)
    {
      const V3D previous_pos = fallback_increment_pos_;
      const M3D previous_rot = fallback_increment_rot_;
      V3D ground_delta_t = delta_t;
      ground_delta_t.z() = 0.0;
      _state.pos_end = previous_pos + previous_rot * ground_delta_t;

      const double rgbd_yaw_delta = std::atan2(delta_R(1, 0), delta_R(0, 0));
      const M3D yaw_only_delta =
          Eigen::AngleAxisd(rgbd_yaw_delta, V3D::UnitZ()).toRotationMatrix();
      _state.rot_end = previous_rot * yaw_only_delta;

      const V3D measured_vel_world = previous_rot * (ground_delta_t / delta_dt);
      if (measured_vel_world.allFinite() && measured_vel_world.norm() < 12.0)
        _state.vel_end = measured_vel_world;

      const V3D measured_omega_body = delta_rvec / delta_dt;
      if (measured_omega_body.allFinite() && measured_omega_body.norm() < 3.5)
        p_imu->SetRuntimeAngularVelocityHint(measured_omega_body);

      state_propagat = _state;
      voxelmap_manager->state_ = _state;
      fallback_increment_rot_ = _state.rot_end;
      fallback_increment_pos_ = _state.pos_end;
      fallback_increment_stamp_ = msg->header.stamp;
      fallback_increment_source_ = delta_source;

      ROS_INFO_THROTTLE(0.5,
          "[FAILSAFE DELTA DIRECT] imu-fail rgbd-primary step=%.3fm/%.2fdeg dt=%.3fs speed=%.2fm/s",
          delta_t.norm(), std::abs(rgbd_yaw_delta) * 180.0 / M_PI, delta_dt,
          measured_vel_world.norm());
      return;
    }

    // IMU-failure v2e: reproduce the proven standalone LiDAR-relative process
    // model inside the ONE persistent FAST-LIVO2 ESIKF state.  The no-IMU
    // propagator is useful between scans, but its unconstrained gravity/vertical
    // state must not accumulate on top of a reliable 10 Hz LiDAR SE(3).
    // At each accepted LiDAR increment, advance the actual ESIKF pose from the
    // previous accepted ESIKF pose exactly as standalone `pose = pose @ Df`.
    // Map corrections still act on this same state and covariance; no parallel
    // trajectory/localizer is created.
    if (lidar_delta && lidar_delta_direct_state_update_ &&
        (failsafe_mode_ == FailsafeMode::LV_IMU_FAILED ||
         (failsafe_mode_ == FailsafeMode::FULL_LIVO &&
          recovery_from_imu_outage_ && lidar_rejoin_guard_)))
    {
      const V3D previous_pos = fallback_increment_pos_;
      const M3D previous_rot = fallback_increment_rot_;
      // The cart is ground constrained. With IMU absent, scan-to-scan ICP has
      // weak observability in the body vertical direction; accepting that
      // component caused ~10 m of Z drift in 20 s even after roll/pitch was
      // stabilized. Keep the measured horizontal LiDAR translation and reject
      // only the unobservable body-Z increment.
      V3D ground_delta_t = delta_t;
      ground_delta_t.z() = 0.0;
      _state.pos_end = fallback_increment_pos_ +
                       fallback_increment_rot_ * ground_delta_t;
      // Ground-cart IMU-loss constraint: scan-to-scan ICP has no gravity
      // reference, so its small roll/pitch errors compound into severe vertical
      // drift over a 20 s outage. Preserve the last continuous ESIKF gravity
      // alignment and accept only the LiDAR-observed heading increment. This is
      // still a measurement update on the same ESIKF state, not a pose reset or
      // second localizer. Translation remains the full measured LiDAR SE(3).
      const double lidar_yaw_delta = std::atan2(delta_R(1, 0), delta_R(0, 0));
      const M3D yaw_only_delta =
          Eigen::AngleAxisd(lidar_yaw_delta, V3D::UnitZ()).toRotationMatrix();
      _state.rot_end = fallback_increment_rot_ * yaw_only_delta;

      const V3D measured_vel_world = previous_rot * (ground_delta_t / delta_dt);
      if (measured_vel_world.allFinite() && measured_vel_world.norm() < 12.0)
        _state.vel_end = measured_vel_world;

      const V3D measured_omega_body = delta_rvec / delta_dt;
      if (measured_omega_body.allFinite() && measured_omega_body.norm() < 3.5)
        p_imu->SetRuntimeAngularVelocityHint(measured_omega_body);

      state_propagat = _state;
      voxelmap_manager->state_ = _state;
      fallback_increment_rot_ = _state.rot_end;
      fallback_increment_pos_ = _state.pos_end;
      fallback_increment_stamp_ = msg->header.stamp;
      fallback_increment_source_ = delta_source;

      ROS_INFO_THROTTLE(0.5,
          "[FAILSAFE DELTA DIRECT] lidar step=%.3fm/%.2fdeg dt=%.3fs speed=%.2fm/s",
          delta_t.norm(), delta_angle * 180.0 / M_PI, delta_dt,
          measured_vel_world.norm());
      return;
    }

    V3D position_residual = measured_pos - _state.pos_end;

    const double pos_gain = std::max(0.0, std::min(1.0,
        rgbd_delta ? rgbd_delta_position_gain_ : lidar_delta_position_gain_));
    const double max_pos_step = std::max(0.0,
        rgbd_delta ? rgbd_delta_max_position_step_ : lidar_delta_max_position_step_);
    V3D position_step = pos_gain * position_residual;
    const double position_step_norm = position_step.norm();
    if (max_pos_step > 0.0 && position_step_norm > max_pos_step &&
        position_step_norm > 1.0e-9)
      position_step *= max_pos_step / position_step_norm;
    if (position_step.allFinite())
      _state.pos_end += position_step;

    const M3D rotation_residual = _state.rot_end.transpose() * measured_rot;
    Eigen::AngleAxisd aa(rotation_residual);
    double applied_rot_deg = 0.0;
    if (std::isfinite(aa.angle()) && aa.axis().allFinite())
    {
      const double rot_gain = std::max(0.0, std::min(1.0,
          rgbd_delta ? rgbd_delta_rotation_gain_ : lidar_delta_rotation_gain_));
      const double max_rot_step = std::max(0.0,
          rgbd_delta ? rgbd_delta_max_rotation_step_deg_ :
                       lidar_delta_max_rotation_step_deg_) * M_PI / 180.0;
      double step_angle = rot_gain * aa.angle();
      if (max_rot_step > 0.0 && std::abs(step_angle) > max_rot_step)
        step_angle = (step_angle >= 0.0 ? 1.0 : -1.0) * max_rot_step;
      if (std::abs(step_angle) > 1.0e-7)
      {
        _state.rot_end =
            _state.rot_end * Eigen::AngleAxisd(step_angle, aa.axis()).toRotationMatrix();
        applied_rot_deg = std::abs(step_angle) * 180.0 / M_PI;
      }
    }

    state_propagat = _state;
    voxelmap_manager->state_ = _state;

    // The next relative increment must start from the already-corrected ESIKF
    // state. Saved-map smoothing also synchronizes this anchor in
    // applyPendingMapCorrection().
    fallback_increment_rot_ = _state.rot_end;
    fallback_increment_pos_ = _state.pos_end;
    fallback_increment_stamp_ = msg->header.stamp;
    fallback_increment_source_ = delta_source;

    ROS_INFO_THROTTLE(0.5,
        "[FAILSAFE DELTA] %s residual=%.3fm applied=%.3fm rot=%.2fdeg dt=%.3fs",
        delta_source.c_str(), position_residual.norm(), position_step.norm(),
        applied_rot_deg, delta_dt);
    return;
  }

  if (!v_body.allFinite() || !omega_body.allFinite() ||
      v_body.norm() > 20.0 || omega_body.norm() > 4.0)
    return;

  last_relative_motion_stamp_ = msg->header.stamp;
  last_relative_motion_source_ = msg->header.frame_id;
  relative_linear_body_ = v_body;
  relative_angular_body_ = omega_body;

  if (msg->header.frame_id == "lidar_relative_motion")
  {
    last_lidar_relative_stamp_ = msg->header.stamp;
    lidar_relative_linear_body_ = v_body;
    lidar_relative_angular_body_ = omega_body;
  }
  else if (msg->header.frame_id == "depth_relative_motion")
  {
    last_depth_relative_stamp_ = msg->header.stamp;
    depth_relative_linear_body_ = v_body;
    depth_relative_angular_body_ = omega_body;
  }
  else if (msg->header.frame_id == "rgbd_visual_motion")
  {
    last_rgbd_relative_stamp_ = msg->header.stamp;
    rgbd_relative_linear_body_ = v_body;
    rgbd_relative_angular_body_ = omega_body;
  }

  if (failsafe_mode_ == FailsafeMode::LV_IMU_FAILED)
  {
    const bool rgbd_primary =
        msg->header.frame_id == "rgbd_visual_motion";
    const bool lidar_backup =
        msg->header.frame_id == "lidar_relative_motion" &&
        (last_rgbd_relative_stamp_.isZero() ||
         std::abs(msg->header.stamp.toSec() -
                  last_rgbd_relative_stamp_.toSec()) > 0.75);
    if (rgbd_primary || lidar_backup)
    {
      const double verified_age =
          last_verified_map_motion_stamp_.isZero()
              ? 1.0e9
              : std::abs(msg->header.stamp.toSec() -
                         last_verified_map_motion_stamp_.toSec());
      if (!(verified_map_motion_valid_ && verified_age < 1.60))
      {
        const V3D v_world = _state.rot_end * v_body;
        if (v_world.allFinite() && v_world.norm() < 15.0)
          _state.vel_end = 0.35 * _state.vel_end + 0.65 * v_world;
        p_imu->SetRuntimeAngularVelocityHint(omega_body);
        state_propagat.vel_end = _state.vel_end;
        voxelmap_manager->state_.vel_end = _state.vel_end;
      }
    }
  }
  else if (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
           failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE)
  {
    const bool rgbd_primary =
        msg->header.frame_id == "rgbd_visual_motion";
    const bool depth_backup =
        msg->header.frame_id == "depth_relative_motion" &&
        (last_rgbd_relative_stamp_.isZero() ||
         std::abs(msg->header.stamp.toSec() -
                  last_rgbd_relative_stamp_.toSec()) > 0.25);
    if ((rgbd_primary || depth_backup) && lidar_fail_velocity_valid_)
    {
      const double ms = v_body.norm();
      if (v_body.allFinite() && ms < 6.0)
      {
        const double w = std::max(0.0, std::min(1.0, rgbd_velocity_weight_));
        lidar_fail_velocity_reference_.x() =
            (1.0 - w) * lidar_fail_velocity_reference_.x() + w * v_body.x();
        lidar_fail_velocity_reference_.y() =
            0.70 * lidar_fail_velocity_reference_.y() + 0.30 * v_body.y();
        lidar_fail_velocity_reference_.z() =
            0.90 * lidar_fail_velocity_reference_.z() + 0.10 * v_body.z();
      }

      const V3D v_world =
          _state.rot_end * lidar_fail_velocity_reference_;
      if (v_world.allFinite())
        _state.vel_end = 0.05 * _state.vel_end + 0.95 * v_world;

      // Constrain the displacement accumulated by IMU propagation between two
      // RGB-D observations.  This avoids double-integrating RGB-D: only the
      // difference between the ESIKF-predicted displacement and the measured
      // metric displacement is applied as a bounded ESIKF correction.
      if (rgbd_primary &&
          !(relative_delta_residual_enable_ && rgbd_delta_residual_enable_))
      {
        if (rgbd_displacement_constraint_valid_)
        {
          const double dt =
              (msg->header.stamp - rgbd_displacement_constraint_stamp_).toSec();
          if (dt > 0.025 && dt < 0.40)
          {
            const V3D predicted_disp =
                _state.pos_end - rgbd_displacement_constraint_state_pos_;
            const V3D measured_disp = _state.rot_end * (v_body * dt);
            V3D residual = measured_disp - predicted_disp;
            if (residual.allFinite() && residual.norm() < 1.5)
            {
              residual *= std::max(0.0, std::min(1.0, rgbd_displacement_gain_));
              const double n = residual.norm();
              if (n > rgbd_displacement_max_step_ && n > 1.0e-9)
                residual *= rgbd_displacement_max_step_ / n;
              _state.pos_end += residual;
              ROS_INFO_THROTTLE(0.5,
                  "[FAILSAFE RGBD] ESIKF displacement residual %.3fm dt=%.3fs v=(%.2f %.2f %.2f)",
                  residual.norm(), dt, v_body.x(), v_body.y(), v_body.z());
            }
          }
        }
        rgbd_displacement_constraint_valid_ = true;
        rgbd_displacement_constraint_stamp_ = msg->header.stamp;
        rgbd_displacement_constraint_state_pos_ = _state.pos_end;
      }
      state_propagat.vel_end = _state.vel_end;
      state_propagat.pos_end = _state.pos_end;
      voxelmap_manager->state_.vel_end = _state.vel_end;
      voxelmap_manager->state_.pos_end = _state.pos_end;
    }
  }
}

void LIVMapper::mapCorrectionCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (!map_correction_enable_ || !msg) return;
  const auto &p = msg->pose.pose.position;
  const auto &qmsg = msg->pose.pose.orientation;
  Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.w()) || !std::isfinite(q.x()) ||
      !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
      q.norm() < 1.0e-6)
    return;
  q.normalize();

  std::lock_guard<std::mutex> lock(mtx_map_correction_);
  map_correction_stamp_ = msg->header.stamp;
  map_correction_pos_ << p.x, p.y, p.z;
  map_correction_rot_ = q.toRotationMatrix();
  map_correction_source_ = msg->header.frame_id;
  map_correction_quality_ = msg->pose.covariance[0];
  map_correction_pending_ = true;
}

void LIVMapper::applyPendingMapCorrection()
{
  if (!map_correction_enable_ || !failsafe_initialized_) return;
  if (!(failsafe_mode_ == FailsafeMode::LV_IMU_FAILED ||
        failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
        failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE))
  {
    smooth_map_target_valid_ = false;
    smooth_map_last_apply_time_ = ros::Time(0);
    return;
  }

  // Causal correction release.  This runs every FAST-LIVO2 estimator cycle,
  // independently of the lower-rate saved-map measurement callbacks.
  const ros::Time smooth_now = ros::Time::now();
  if (smooth_map_target_valid_)
  {
    double dt = 0.02;
    if (!smooth_map_last_apply_time_.isZero())
      dt = std::max(0.002, std::min(0.10,
          (smooth_now - smooth_map_last_apply_time_).toSec()));
    smooth_map_last_apply_time_ = smooth_now;

    double active_pos_rate = smooth_map_max_pos_rate_;
    double active_rot_rate_deg = smooth_map_max_rot_rate_deg_;
    if (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
        failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE)
    {
      active_pos_rate = smooth_map_max_pos_rate_lidar_fail_;
      active_rot_rate_deg = smooth_map_max_rot_rate_lidar_fail_deg_;
    }
    else if (failsafe_mode_ == FailsafeMode::LV_IMU_FAILED)
    {
      active_pos_rate = smooth_map_max_pos_rate_imu_fail_;
      active_rot_rate_deg = smooth_map_max_rot_rate_imu_fail_deg_;
    }

    const V3D residual = smooth_map_target_pos_ - _state.pos_end;
    const double residual_norm = residual.norm();
    const double max_step = active_pos_rate * dt;
    if (residual_norm > 1.0e-5)
    {
      const double step =
          std::min(residual_norm, std::max(0.002, max_step));
      _state.pos_end += (step / residual_norm) * residual;
    }

    if (smooth_map_target_rotation_enabled_)
    {
      const M3D dR = _state.rot_end.transpose() * smooth_map_target_rot_;
      Eigen::AngleAxisd aa(dR);
      if (std::isfinite(aa.angle()) && aa.axis().allFinite())
      {
        const double max_angle =
            active_rot_rate_deg * M_PI / 180.0 * dt;
        const double step_angle =
            std::min(std::abs(aa.angle()), std::max(1.0e-5, max_angle));
        if (step_angle > 1.0e-6)
        {
          const double sign = aa.angle() >= 0.0 ? 1.0 : -1.0;
          Eigen::AngleAxisd step(sign * step_angle, aa.axis());
          _state.rot_end = _state.rot_end * step.toRotationMatrix();
        }
      }
    }

    voxelmap_manager->state_ = _state;
    state_propagat = _state;

    // A saved-map correction changes the same degraded-state anchor from which
    // the next relative increment must continue.  Synchronizing it here keeps
    // map correction and relative propagation in one continuous trajectory.
    if (fallback_increment_anchor_valid_)
    {
      fallback_increment_pos_ = _state.pos_end;
      fallback_increment_rot_ = _state.rot_end;
    }

    const double remaining =
        (smooth_map_target_pos_ - _state.pos_end).norm();
    const M3D rrem =
        _state.rot_end.transpose() * smooth_map_target_rot_;
    double c = (rrem.trace() - 1.0) * 0.5;
    c = std::max(-1.0, std::min(1.0, c));
    const double rem_deg = std::acos(c) * 180.0 / M_PI;
    if (remaining < 0.03 &&
        (!smooth_map_target_rotation_enabled_ || rem_deg < 0.35))
      smooth_map_target_valid_ = false;
  }

  ros::Time stamp;
  V3D corrected_pos;
  M3D corrected_rot;
  std::string source;
  double correction_quality = std::numeric_limits<double>::infinity();
  {
    std::lock_guard<std::mutex> lock(mtx_map_correction_);
    if (!map_correction_pending_) return;
    stamp = map_correction_stamp_;
    corrected_pos = map_correction_pos_;
    corrected_rot = map_correction_rot_;
    source = map_correction_source_;
    correction_quality = map_correction_quality_;
    map_correction_pending_ = false;
  }

  // A returning LiDAR saved-map registration can be used purely as a recovery
  // handshake when the global map anchor (not the ESIKF) owns absolute pose.
  // Mark the geometric lock and return immediately: applying the same pose here
  // would double-correct the one persistent FAST-LIVO2 state.
  if (source == "lidar_recovery_lock_only")
  {
    lidar_recovery_map_lock_ = true;
    last_applied_map_correction_stamp_ =
        stamp.isZero() ? ros::Time::now() : stamp;
    last_applied_map_correction_source_ = source;
    ROS_WARN("[FAILSAFE MAP] Returning LiDAR relocalization LOCKED (anchor-only handshake)");
    return;
  }

  const bool place_verified_depth =
      source == "depth_place_verified_map" ||
      source == "depth_place_verified_esikf";
  const bool place_verified_lidar =
      source == "lidar_place_verified_map" ||
      source == "lidar_place_verified_esikf";
  const bool correction_already_esikf =
      source == "depth_place_verified_esikf" ||
      source == "lidar_place_verified_esikf" ||
      source == "depth_saved_map_esikf" ||
      source == "lidar_saved_map_esikf" ||
      source == "lidar_recovery_saved_map_esikf";

  // Compare correction age against the timestamp of the degraded state that
  // the relative-motion front-end most recently anchored. This is an absolute
  // sensor timestamp in both RGB-D and LiDAR fallback modes and avoids mixing
  // rosbag clock time with FAST-LIVO2's internal update-time conventions.
  double now = ros::Time::now().toSec();
  if (fallback_increment_anchor_valid_ && !fallback_increment_stamp_.isZero())
    now = fallback_increment_stamp_.toSec();
  else
  {
    const double estimator_time = currentUpdateTime();
    if (std::isfinite(estimator_time) && estimator_time > 1.0e8)
      now = estimator_time;
  }
  // Place-verified LiDAR registration is more expensive than RGB-D.  The
  // map-localizer already latency-compensates the pose to its latest odometry
  // before publishing, so use a source-specific compute-latency allowance
  // rather than dropping every strong IMU-failure measurement at 0.5 s.
  double max_correction_age = map_correction_max_age_;
  if (place_verified_lidar)
    max_correction_age = std::max(max_correction_age, 3.5);
  else if (place_verified_depth)
    max_correction_age = std::max(max_correction_age, 0.9);
  else if (source == "lidar_saved_map_esikf" ||
           source == "lidar_recovery_saved_map_esikf")
    max_correction_age = std::max(max_correction_age, 1.6);
  else if (source == "depth_saved_map_esikf")
    max_correction_age = std::max(max_correction_age, 1.2);

  if (!stamp.isZero() && now > 0.0 &&
      std::abs(now - stamp.toSec()) > max_correction_age)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[FAILSAFE MAP] Dropped stale %s correction (age %.3f s gate %.2f s)",
        source.c_str(), std::abs(now - stamp.toSec()), max_correction_age);
    return;
  }

  const bool place_verified =
      place_verified_depth || place_verified_lidar;
  // Incoming correction is in the immutable saved-map frame.  The map->ESIKF
  // alignment must NEVER be initialized from blind local ICP: repeated
  // geometry and degraded-state latency can make that first pose wrong by
  // several metres.  Only a visual-place candidate that is independently
  // verified by live depth/LiDAR geometry may establish the outage alignment.
  if (!correction_already_esikf)
  {
    if (!degraded_map_alignment_valid_)
    {
      if (!place_verified)
      {
        ROS_WARN_THROTTLE(
            0.75,
            "[FAILSAFE MAP] Ignoring blind %s until a place-verified map alignment exists",
            source.c_str());
        return;
      }

      degraded_R_odom_map_ = _state.rot_end * corrected_rot.transpose();
      degraded_t_odom_map_ =
          _state.pos_end - degraded_R_odom_map_ * corrected_pos;
      degraded_map_alignment_valid_ = true;
      map_correction_prev_valid_ = false;
      ROS_WARN("[FAILSAFE MAP] Place-verified degraded map alignment LOCKED");
    }

    corrected_pos =
        degraded_R_odom_map_ * corrected_pos + degraded_t_odom_map_;
    corrected_rot = degraded_R_odom_map_ * corrected_rot;
  }
  else
  {
    // The map localizer already used a pre-failure healthy map<-ESIKF anchor.
    // Do not erase that absolute information by re-locking at the drifted state.
    degraded_map_alignment_valid_ = true;
  }

  // Place-verified LiDAR corrections are intentionally published at the
  // source scan time during IMU loss. Propagate them to the present using the
  // PREVIOUS independently verified map-motion estimate, never the degraded
  // no-IMU odometry.
  if (place_verified_lidar &&
      failsafe_mode_ == FailsafeMode::LV_IMU_FAILED &&
      !stamp.isZero())
  {
    const double age =
        std::max(0.0, std::min(1.6, ros::Time::now().toSec() - stamp.toSec()));
    if (verified_map_motion_valid_ && age > 1.0e-3)
    {
      if (verified_map_linear_world_.allFinite() &&
          verified_map_linear_world_.norm() < 15.0)
        corrected_pos += verified_map_linear_world_ * age;
      if (verified_map_angular_body_.allFinite() &&
          verified_map_angular_body_.norm() < 3.5)
        corrected_rot =
            corrected_rot * Exp(verified_map_angular_body_, age);
    }
  }

  // With LiDAR unavailable the IMU is healthy, so it remains the attitude
  // authority. RGB-D/depth map matching corrects translation/velocity only;
  // sparse depth geometry is not allowed to rotate the ESIKF.

  const bool depth_position_only =
      (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
       failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE) &&
      (source == "depth_saved_map_icp" ||
       source == "depth_saved_map_esikf" || place_verified_depth);
  if (depth_position_only)
    corrected_rot = _state.rot_end;

  const double translation_jump = (corrected_pos - _state.pos_end).norm();
  const M3D delta_R = _state.rot_end.transpose() * corrected_rot;
  double cos_angle = (delta_R.trace() - 1.0) * 0.5;
  cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
  const double rotation_jump_deg = std::acos(cos_angle) * 180.0 / M_PI;
  double translation_gate = map_correction_max_translation_jump_;
  double rotation_gate = map_correction_max_rotation_deg_;
  const double now_rel = ros::Time::now().toSec();
  const double lidar_relative_age =
      last_lidar_relative_stamp_.isZero()
          ? 1.0e9
          : std::abs(now_rel - last_lidar_relative_stamp_.toSec());
  const double depth_relative_age =
      last_depth_relative_stamp_.isZero()
          ? 1.0e9
          : std::abs(now_rel - last_depth_relative_stamp_.toSec());
  const double rgbd_relative_age =
      last_rgbd_relative_stamp_.isZero()
          ? 1.0e9
          : std::abs(now_rel - last_rgbd_relative_stamp_.toSec());

  const bool lidar_relative_support =
      failsafe_mode_ == FailsafeMode::LV_IMU_FAILED &&
      (source == "lidar_saved_map_icp" ||
       source == "lidar_saved_map_esikf" || place_verified_lidar) &&
      (lidar_relative_age < 0.50 || rgbd_relative_age < 0.35) &&
      std::isfinite(correction_quality) && correction_quality < 0.18;
  const bool depth_relative_support =
      (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
       failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE) &&
      (source == "depth_saved_map_icp" ||
       source == "depth_saved_map_esikf" || place_verified_depth) &&
      (rgbd_relative_age < lidar_fail_rgbd_fresh_window_sec_ || depth_relative_age < 1.0) &&
      std::isfinite(correction_quality) && correction_quality < 0.18;
  const bool anchored_depth_support =
      (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
       failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE) &&
      source == "depth_saved_map_esikf" &&
      rgbd_relative_age < lidar_fail_rgbd_fresh_window_sec_ &&
      std::isfinite(correction_quality) && correction_quality < 0.08;

  // A visual-place candidate that was independently refined by current
  // LiDAR/depth geometry is much stronger than blind local ICP.  Allow it to
  // recover several metres of drift, but still reject implausible jumps.
  if (place_verified)
  {
    if (place_verified_lidar &&
        failsafe_mode_ == FailsafeMode::LV_IMU_FAILED)
    {
      // Match the standalone IMU-failure safety basin. Visual retrieval +
      // LiDAR GICP may still lock onto repeated campus geometry; never allow a
      // large relocalization pull simply because the local GICP fit is good.
      // Frequent small verified fixes are the intended correction mechanism.
      translation_gate = 4.0;
      rotation_gate = 18.0;
    }
    else
    {
      // LiDAR-loss keeps healthy IMU attitude and already has a stable RGB-D
      // metric trajectory. A place+depth match is only a local translational
      // correction, never permission for an 8 m recovery pull.
      translation_gate = std::max(translation_gate,
                                  place_verified_depth ? 4.0 : 8.0);
      rotation_gate = std::max(rotation_gate,
                               place_verified_lidar ? 25.0 : 12.0);
    }
  }
  else if (lidar_relative_support)
  {
    translation_gate = std::max(translation_gate, 8.0);
    rotation_gate = std::max(rotation_gate, 35.0);
  }
  else if (anchored_depth_support)
  {
    // This pose has already been transformed through the healthy pre-failure
    // map<-ESIKF anchor and passed strict local depth-to-saved-map geometry.
    // The stable standalone RGB-D path safely accepted up to ~5 m of residual
    // and released it gradually.  Keeping the old 2.5 m gate made recovery
    // impossible once the short-term predictor fell a few metres behind.
    translation_gate = std::max(translation_gate, 5.0);
    rotation_gate = std::max(rotation_gate, 12.0);
  }
  else if (depth_relative_support)
  {
    // Sparse RGB-D/depth map matching is useful only as a local correction.
    // Large jumps are more likely repeated-geometry locks than real pose error.
    translation_gate = std::min(std::max(translation_gate, 2.0), 2.5);
    rotation_gate = std::max(rotation_gate, 12.0);
  }
  else if (depth_position_only && std::isfinite(correction_quality) &&
           correction_quality < 0.08)
  {
    translation_gate = std::min(std::max(translation_gate, 2.0), 2.5);
  }

  if (translation_jump > translation_gate ||
      rotation_jump_deg > rotation_gate)
  {
    ROS_WARN_THROTTLE(1.0,
        "[FAILSAFE MAP] Rejected %s correction: %.3f m / %.2f deg "
        "(gate %.1fm/%.1fdeg quality=%.3f rel=%d)",
        source.c_str(), translation_jump, rotation_jump_deg,
        translation_gate, rotation_gate, correction_quality,
        (lidar_relative_support || depth_relative_support) ? 1 : 0);
    return;
  }

  // Saved-map localization is an auxiliary measurement to the SAME ESIKF,
  // never a competing estimator.  Use causal gains validated in standalone
  // tests; quality further down-weights weak registration.
  double quality_weight = 1.0;
  if (std::isfinite(correction_quality))
    quality_weight = std::max(0.15, std::min(1.0, 1.0 - correction_quality / 0.18));

  double a_pos = std::max(0.0, std::min(1.0, map_correction_blend_));
  double a_rot = a_pos;
  if (failsafe_mode_ == FailsafeMode::LV_IMU_FAILED)
  {
    // With no IMU, dual-verified LiDAR+vision map measurements must correct
    // both position and attitude quickly enough to prevent heading runaway.
    a_pos = (place_verified_lidar ? 0.88 : 0.28) * quality_weight;
    a_rot = (place_verified_lidar ? 0.85 : 0.22) * quality_weight;
  }
  else if (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
           failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE)
  {
    a_pos = (place_verified_depth ? 0.62 : 0.30) * quality_weight;
    // Healthy IMU remains attitude authority while LiDAR is absent.
    a_rot = depth_position_only ? 0.0 : 0.18 * quality_weight;
  }

  // Successive accepted global map poses provide a low-frequency motion
  // estimate. Fuse this with the high-rate relative-motion hints so correcting
  // pose never leaves an inconsistent velocity/rate in the same ESIKF state.
  const bool imu_map_motion =
      failsafe_mode_ == FailsafeMode::LV_IMU_FAILED &&
      (source == "lidar_saved_map_icp" ||
       source == "lidar_saved_map_esikf" || place_verified_lidar);
  // Blind depth ICP is too ambiguous to estimate velocity, but consecutive
  // visual-place + depth-geometry verified map poses are suitable as a
  // low-frequency metric velocity observation.
  // Consecutive place-verified RGB-D map poses provide useful metric speed
  // during LiDAR loss, but they are fused conservatively below rather than
  // overwriting most of the ESIKF velocity in one update.
  const bool depth_map_motion =
      (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
       failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE) &&
      place_verified_depth;

  if (imu_map_motion || depth_map_motion)
  {
    if (map_correction_prev_valid_)
    {
      const double dt = (stamp - prev_map_correction_stamp_).toSec();
      if (dt > 0.03 && dt < 2.0)
      {
        const V3D vel_map = (corrected_pos - prev_map_correction_pos_) / dt;
        if (vel_map.allFinite() && vel_map.norm() < 15.0)
        {
          if (imu_map_motion && place_verified_lidar)
          {
            verified_map_linear_world_ = vel_map;
            last_verified_map_motion_stamp_ =
                stamp.isZero() ? ros::Time::now() : stamp;
            verified_map_motion_valid_ = true;
          }

          const double map_vel_weight =
              place_verified ? (depth_map_motion ? 0.35 : 0.50)
                             : (imu_map_motion ? 0.40 : 0.25);
          V3D vel_delta = map_vel_weight * (vel_map - _state.vel_end);
          // Bound the instantaneous velocity correction.  This is the other
          // half of removing visible trajectory jumps: a map pose must not
          // inject a several-m/s speed discontinuity.
          const double max_vel_delta = depth_map_motion ? 0.75 : 1.00;
          const double vel_delta_norm = vel_delta.norm();
          if (vel_delta_norm > max_vel_delta && vel_delta_norm > 1.0e-6)
            vel_delta *= max_vel_delta / vel_delta_norm;
          _state.vel_end += vel_delta;
          if (depth_map_motion)
          {
            const V3D body_from_map =
                _state.rot_end.transpose() * _state.vel_end;
            const double speed = body_from_map.norm();
            if (body_from_map.allFinite() && speed < 12.0)
            {
              const double ref_forward =
                  std::max(0.1, std::abs(lidar_fail_velocity_reference_.x()));
              if (std::abs(body_from_map.x()) > 0.55 * ref_forward &&
                  std::abs(body_from_map.x()) < 1.60 * ref_forward &&
                  body_from_map.x() * lidar_fail_velocity_reference_.x() >= 0.0)
              {
                lidar_fail_velocity_reference_.x() =
                    0.85 * lidar_fail_velocity_reference_.x() +
                    0.15 * body_from_map.x();
              }
              lidar_fail_velocity_valid_ = true;
            }
          }
        }

        if (imu_map_motion)
        {
          const M3D dR = prev_map_correction_rot_.transpose() * corrected_rot;
          Eigen::AngleAxisd aa(dR);
          const V3D omega = aa.axis() * (aa.angle() / dt);
          if (omega.allFinite() && omega.norm() < 3.5)
          {
            p_imu->SetRuntimeAngularVelocityHint(omega);
            if (place_verified_lidar)
            {
              verified_map_angular_body_ = omega;
              last_verified_map_motion_stamp_ =
                  stamp.isZero() ? ros::Time::now() : stamp;
              verified_map_motion_valid_ = true;
            }
          }
        }
      }
    }
    prev_map_correction_stamp_ = stamp;
    prev_map_correction_pos_ = corrected_pos;
    prev_map_correction_rot_ = corrected_rot;
    map_correction_prev_valid_ = true;
  }

  // Convert the accepted measurement into a bounded correction target.
  // Keep the confidence/gain semantics by moving the target only partway
  // toward a weaker measurement, but release that target smoothly afterward.
  smooth_map_target_pos_ =
      (1.0 - a_pos) * _state.pos_end + a_pos * corrected_pos;
  smooth_map_target_rot_ = _state.rot_end;
  smooth_map_target_rotation_enabled_ = a_rot > 1.0e-6;
  if (smooth_map_target_rotation_enabled_)
  {
    Eigen::Quaterniond q_old(_state.rot_end);
    Eigen::Quaterniond q_new(corrected_rot);
    smooth_map_target_rot_ =
        q_old.slerp(a_rot, q_new).normalized().toRotationMatrix();
  }
  smooth_map_target_valid_ = true;
  if (smooth_map_last_apply_time_.isZero())
    smooth_map_last_apply_time_ = ros::Time::now();

  // A trusted global/map measurement reduces pose uncertainty. Gravity and
  // biases remain untouched.
  const double rot_cov_scale = 1.0 - 0.15 * a_rot;
  const double pos_cov_scale = 1.0 - 0.20 * a_pos;
  _state.cov.block<3, 3>(0, 0) *= rot_cov_scale;
  _state.cov.block<3, 3>(3, 3) *= pos_cov_scale;

  last_applied_map_correction_stamp_ = stamp.isZero() ? ros::Time::now() : stamp;
  last_applied_map_correction_source_ = source;
  if (source == "lidar_recovery_saved_map_icp" ||
      source == "lidar_recovery_saved_map_esikf")
  {
    lidar_recovery_map_lock_ = true;
    ROS_WARN("[FAILSAFE MAP] Returning LiDAR relocalization LOCKED");
  }

  ROS_INFO_THROTTLE(0.5,
      "[FAILSAFE MAP] Queued smooth %s correction: %.3f m / %.2f deg",
      source.c_str(), translation_jump, rotation_jump_deg);
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom.publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

static const size_t kMaxBufferedLidarScans = 10;
static const size_t kMaxBufferedImageFrames = 16;
static const size_t kMaxBufferedImuMsgs = 300;

template <typename PrimaryBuf, typename SecondaryBuf>
static void trimBufferPairFront(PrimaryBuf &primary, SecondaryBuf &secondary, size_t max_size)
{
  while (primary.size() > max_size)
  {
    primary.pop_front();
    secondary.pop_front();
  }
}

template <typename Buf>
static void trimBufferFront(Buf &buffer, size_t max_size)
{
  while (buffer.size() > max_size) buffer.pop_front();
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, reset all synchronization buffers");
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    imu_buffer.clear();
    img_buffer.clear();
    img_time_buffer.clear();
    lidar_pushed = false;
    last_timestamp_imu = -1.0;
    last_timestamp_img = -1.0;
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  if (!ptr || ptr->size() < static_cast<size_t>(std::max(1, min_lidar_points_)))
  {
    ROS_WARN("[FAILSAFE] Rejected LiDAR cloud with only %zu usable points", ptr ? ptr->size() : 0UL);
    mtx_buffer.unlock();
    return;
  }
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;
  last_lidar_rx_time_ = ros::Time::now().toSec();
  markSensorSample(lidar_health_, lidar_recovery_count_, lidar_recovery_samples_, "LiDAR");
  // Point count is a cheap online proxy for LiDAR measurement support.
  const double lidar_point_score =
      std::min(1.0, static_cast<double>(ptr->size()) /
                        static_cast<double>(std::max(4 * min_lidar_points_, 1)));
  lidar_confidence_ = 0.85 * lidar_confidence_ + 0.15 * lidar_point_score;
  trimBufferPairFront(lid_raw_data_buffer, lid_header_time_buffer, kMaxBufferedLidarScans);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = msg->header.stamp.toSec();
  ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, reset all synchronization buffers");
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    imu_buffer.clear();
    img_buffer.clear();
    img_time_buffer.clear();
    lidar_pushed = false;
    last_timestamp_imu = -1.0;
    last_timestamp_img = -1.0;
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->size() < static_cast<size_t>(std::max(1, min_lidar_points_))) {
    ROS_WARN("[FAILSAFE] Rejected LiDAR cloud with only %zu usable points", ptr ? ptr->size() : 0UL);
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;
  last_lidar_rx_time_ = ros::Time::now().toSec();
  markSensorSample(lidar_health_, lidar_recovery_count_, lidar_recovery_samples_, "LiDAR");
  // Point count is a cheap online proxy for LiDAR measurement support.
  const double lidar_point_score =
      std::min(1.0, static_cast<double>(ptr->size()) /
                        static_cast<double>(std::max(4 * min_lidar_points_, 1)));
  lidar_confidence_ = 0.85 * lidar_confidence_ + 0.15 * lidar_point_score;
  trimBufferPairFront(lid_raw_data_buffer, lid_header_time_buffer, kMaxBufferedLidarScans);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  if (!imu_en) return;

  if (!std::isfinite(msg_in->linear_acceleration.x) || !std::isfinite(msg_in->linear_acceleration.y) ||
      !std::isfinite(msg_in->linear_acceleration.z) || !std::isfinite(msg_in->angular_velocity.x) ||
      !std::isfinite(msg_in->angular_velocity.y) || !std::isfinite(msg_in->angular_velocity.z))
  {
    ROS_WARN_THROTTLE(1.0, "[FAILSAFE] Rejected IMU packet containing NaN/Inf");
    return;
  }

  if (last_timestamp_lidar < 0.0) return;
  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
  double timestamp = msg->header.stamp.toSec();

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = ros::Time().fromSec(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  last_imu_rx_time_ = ros::Time::now().toSec();
  if (failsafe_enable_ && failsafe_initialized_ &&
      (failsafe_mode_ == FailsafeMode::LV_IMU_FAILED ||
       imu_health_ == SensorHealth::FAILED ||
       imu_health_ == SensorHealth::RECOVERING))
  {
    if (imu_health_ == SensorHealth::FAILED)
    {
      imu_recovery_acc_sum_.setZero();
      imu_recovery_gyr_sum_.setZero();
      imu_recovery_sample_count_ = 0;
    }
    const V3D a(msg->linear_acceleration.x, msg->linear_acceleration.y,
                msg->linear_acceleration.z);
    const V3D g(msg->angular_velocity.x, msg->angular_velocity.y,
                msg->angular_velocity.z);
    if (a.allFinite() && g.allFinite())
    {
      imu_recovery_acc_sum_ += a;
      imu_recovery_gyr_sum_ += g;
      ++imu_recovery_sample_count_;
    }
  }
  markSensorSample(imu_health_, imu_recovery_count_, imu_recovery_samples_, "IMU");
  // Finite packets at the expected callback cadence are trusted.  Outliers
  // are already rejected above; confidence rises gradually on recovery.
  imu_confidence_ = std::min(1.0, 0.92 * imu_confidence_ + 0.08);
  trimBufferFront(imu_buffer, kMaxBufferedImuMsgs);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = msg->header.stamp.toSec() + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  ROS_INFO("Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    ROS_ERROR("image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  if (img_cur.empty() || img_cur.cols <= 0 || img_cur.rows <= 0)
  {
    ROS_WARN_THROTTLE(1.0, "[FAILSAFE] Rejected empty/invalid camera frame");
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);
  last_camera_rx_time_ = ros::Time::now().toSec();
  markSensorSample(camera_health_, camera_recovery_count_, camera_recovery_samples_, "Camera");
  // Packet arrival is sensor health, not visual measurement quality.  Do not
  // inflate fusion confidence merely because a frame arrived; confidence is
  // updated after sparse-map support / visual-update validation above.
  trimBufferPairFront(img_buffer, img_time_buffer, kMaxBufferedImageFrames);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::syncLidarImuFallback(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() || lid_header_time_buffer.empty()) return false;
  if (imu_en && imu_buffer.empty()) return false;

  meas.lidar = lid_raw_data_buffer.front();
  if (!meas.lidar || meas.lidar->points.size() <= 1)
  {
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    return false;
  }
  meas.lidar_frame_beg_time = lid_header_time_buffer.front();
  meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / 1000.0;
  if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = meas.lidar_frame_beg_time;

  if (meas.lidar_frame_end_time <= meas.last_lio_update_time + 1.0e-6)
  {
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    return false;
  }
  if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time) return false;

  MeasureGroup m;
  m.lio_time = meas.lidar_frame_end_time;
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    while (!imu_buffer.empty() && imu_buffer.front()->header.stamp.toSec() <= m.lio_time)
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time)
        m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    // When healthy visual pose fusion is gated out, camera frames remain
    // useful for health monitoring / RGB-D recovery but must not accumulate
    // and later be replayed as stale VIO updates.  Keep the camera stream
    // causal with the LiDAR+IMU backbone by discarding frames already behind
    // this LIO update time.
    while (!img_time_buffer.empty() &&
           img_time_buffer.front() + exposure_time_init <= m.lio_time)
    {
      img_time_buffer.pop_front();
      img_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
  }
  meas.pcl_proc_cur = meas.lidar;
  meas.measures.clear();
  meas.measures.push_back(m);
  meas.lio_vio_flg = LIO;
  lidar_pushed = false;
  return true;
}

bool LIVMapper::syncLidarNoImuMapFallback(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() || lid_header_time_buffer.empty()) return false;

  meas.lidar = lid_raw_data_buffer.front();
  if (!meas.lidar || meas.lidar->points.size() <= 1)
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    return false;
  }

  meas.lidar_frame_beg_time = lid_header_time_buffer.front();
  meas.lidar_frame_end_time =
      meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / 1000.0;
  if (meas.last_lio_update_time < 0.0)
    meas.last_lio_update_time = meas.lidar_frame_beg_time;
  if (meas.lidar_frame_end_time <= meas.last_lio_update_time + 1.0e-6)
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    return false;
  }

  MeasureGroup m;
  m.lio_time = meas.lidar_frame_end_time;
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    // Camera frames are not estimator updates in this mode. Keep only recent
    // frames for health/recovery and prevent an old visual frame from being
    // consumed immediately when normal LIVO resumes.
    while (!img_time_buffer.empty() &&
           img_time_buffer.front() + exposure_time_init <= m.lio_time)
    {
      img_time_buffer.pop_front();
      img_buffer.pop_front();
    }
  }

  meas.pcl_proc_cur = meas.lidar;
  meas.measures.clear();
  meas.measures.push_back(m);
  meas.lio_vio_flg = LIO;
  lidar_pushed = false;
  return true;
}

bool LIVMapper::syncVisionImuFallback(LidarMeasureGroup &meas)
{
  if (img_buffer.empty() || img_time_buffer.empty() || imu_buffer.empty()) return false;
  const double image_time = img_time_buffer.front() + exposure_time_init;
  if (meas.last_lio_update_time < 0.0) return false;

  if (image_time <= meas.last_lio_update_time + 1.0e-6)
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    img_buffer.pop_front();
    img_time_buffer.pop_front();
    return false;
  }
  if (last_timestamp_imu < image_time) return false;

  MeasureGroup m;
  m.vio_time = image_time;
  m.lio_time = meas.last_lio_update_time;
  m.img = img_buffer.front();
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    while (!imu_buffer.empty() && imu_buffer.front()->header.stamp.toSec() <= image_time)
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time)
        m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    img_buffer.pop_front();
    img_time_buffer.pop_front();
  }
  if (m.imu.empty()) return false;
  meas.measures.clear();
  meas.measures.push_back(m);
  meas.lio_vio_flg = VIO;
  return true;
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  updateFailsafeHealthAndMode();
  const bool preserve_lidar_cadence = preserve_lidar_cadence_ && !p_imu->imu_need_init;

  if (diagnostic_force_lidar_sync_en_)
  {
    // Profiling control: camera callback/adapter remain active but do not
    // split LiDAR or IMU into image-timed LIVO propagation cycles.
    // This intentionally disables photometric tracking and corrections;
    // it is a scheduler-only control, not a production sensor mode.
    return syncLidarImuFallback(meas);
  }

  if (failsafe_enable_ && failsafe_initialized_)
  {
    if (failsafe_mode_ == FailsafeMode::LIO_CAMERA_FAILED)
      return syncLidarImuFallback(meas);
    if (failsafe_mode_ == FailsafeMode::LV_IMU_FAILED)
      return syncLidarNoImuMapFallback(meas);
    if (failsafe_mode_ == FailsafeMode::VI_LIDAR_FAILED ||
        failsafe_mode_ == FailsafeMode::IMU_ONLY_BRIDGE)
      return syncVisionImuFallback(meas);
    if (failsafe_mode_ == FailsafeMode::INVALID) return false;
  }

  // Once the short visual bootstrap has completed, a configuration that
  // disables healthy visual pose fusion must also stop scheduling VIO-time
  // propagation events.  Merely skipping the photometric correction while
  // still consuming IMU at image timestamps halves the effective LIO update
  // cadence on this bag and was enough to destabilize Z/attitude.  Run the
  // exact LiDAR+IMU synchronization path instead; camera callbacks continue
  // to update health and the RGB-D failsafe remains available immediately if
  // LiDAR later fails.
  if (failsafe_enable_ && failsafe_initialized_ &&
      failsafe_mode_ == FailsafeMode::FULL_LIVO &&
      healthy_visual_gate_enable_ &&
      !healthy_visual_pose_fusion_enable_ &&
      visual_frames_seen_ >= healthy_visual_bootstrap_frames_)
  {
    return syncLidarImuFallback(meas);
  }

  const bool runtime_imu_available = !(failsafe_enable_ && failsafe_initialized_ &&
                                       failsafe_mode_ == FailsafeMode::LV_IMU_FAILED);
  if (lid_raw_data_buffer.empty() && lidar_en &&
      !(preserve_lidar_cadence && slam_mode_ == LIVO && !meas.pcl_proc_next->empty())) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en && runtime_imu_available) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      const bool have_image = !img_buffer.empty() && !img_time_buffer.empty();
      double img_capture_time = have_image
                                    ? img_time_buffer.front() + exposure_time_init
                                    : std::numeric_limits<double>::infinity();
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double pending_end = meas.last_lio_update_time;
      if (preserve_lidar_cadence)
        for (const auto& point : meas.pcl_proc_next->points)
          pending_end = std::max(pending_end, meas.last_lio_update_time + point.curvature / 1000.0);
      double lid_newest_time = pending_end;
      if (!lid_raw_data_buffer.empty() && !lid_raw_data_buffer.back()->points.empty())
      {
        const auto &newest_scan = lid_raw_data_buffer.back()->points;
        const auto newest_point = std::max_element(
            newest_scan.begin(), newest_scan.end(),
            [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
        lid_newest_time = lid_header_time_buffer.back() + newest_point->curvature / 1000.0;
      }
      lid_newest_time = std::max(lid_newest_time, pending_end);
      double lio_target_time = img_capture_time;
      if (preserve_lidar_cadence) {
        if (!have_image) lio_target_time = std::numeric_limits<double>::infinity();
        if (pending_end > meas.last_lio_update_time + 1.0e-5)
          lio_target_time = std::min(lio_target_time, pending_end);
        if (!lid_raw_data_buffer.empty() && !lid_raw_data_buffer.front()->points.empty()) {
          const auto &next_scan = lid_raw_data_buffer.front()->points;
          const auto last_point = std::max_element(
              next_scan.begin(), next_scan.end(),
              [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
          const double scan_end = lid_header_time_buffer.front() + last_point->curvature / 1000.0;
          if (scan_end > meas.last_lio_update_time + 1.0e-5)
            lio_target_time = std::min(lio_target_time, scan_end);
        }
      }
      double imu_newest_time = runtime_imu_available && !imu_buffer.empty()
                                   ? imu_buffer.back()->header.stamp.toSec()
                                   : std::numeric_limits<double>::infinity();

      if (have_image && img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        const double image_lag = meas.last_lio_update_time - img_capture_time;
        {
          std::lock_guard<std::mutex> lock(mtx_buffer);
          if (!img_buffer.empty()) img_buffer.pop_front();
          if (!img_time_buffer.empty()) img_time_buffer.pop_front();
        }
        ROS_WARN_THROTTLE(1.0,
                          "[VIO] Dropping stale image %.1f ms behind LiDAR state",
                          image_lag * 1000.0);
        return false;
      }

      if (preserve_lidar_cadence && !std::isfinite(lio_target_time)) return false;
      if (preserve_lidar_cadence &&
          (lio_target_time > lid_newest_time + 1.0e-5 ||
           (runtime_imu_available && lio_target_time > imu_newest_time + 1.0e-5)))
        return false;
      if (!preserve_lidar_cadence &&
          (img_capture_time > lid_newest_time ||
           (runtime_imu_available && img_capture_time > imu_newest_time)))
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = lio_target_time;
      livo_image_due_ = !preserve_lidar_cadence ||
                         (have_image && lio_target_time >= img_capture_time - 1.0e-5);
      mtx_buffer.lock();
      while (runtime_imu_available && !imu_buffer.empty())
      {
        if (imu_buffer.front()->header.stamp.toSec() > m.lio_time) break;

        if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // imu_buffer.front()->header.stamp.toSec());
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      if (preserve_lidar_cadence) {
        PointCloudXYZI pending;
        pending.swap(*meas.pcl_proc_next);
        meas.pcl_proc_cur->clear();
        const double cut_ms = (m.lio_time - meas.last_lio_update_time) * 1000.0;
        for (auto point : pending.points) {
          if (point.curvature <= cut_ms) meas.pcl_proc_cur->push_back(point);
          else {
            point.curvature -= cut_ms;
            meas.pcl_proc_next->push_back(point);
          }
        }
      } else {
        *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
        PointCloudXYZI().swap(*meas.pcl_proc_next);
      }

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > m.lio_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms ||
              (preserve_lidar_cadence && pcl[i].curvature <= max_offs_time_ms))
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      if (preserve_lidar_cadence)
        std::sort(meas.pcl_proc_cur->points.begin(), meas.pcl_proc_cur->points.end(), time_list);
      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      if (!livo_image_due_) {
        // The preceding event was a LiDAR scan boundary. Keep the image queued
        // and consume the next chronological event before running its VIO.
        meas.lio_vio_flg = VIO;
        return false;
      }
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = imu_buffer.front()->header.stamp.toSec();
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   imu_buffer.front()->header.stamp.toSec());
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  ROS_ERROR("out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  bool flush_wait_pub = false;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      flush_wait_pub = true;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes.publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  // pcl_wait_pub is only a temporary publication/colorization buffer. The
  // upstream code cleared it only when RGB projection produced at least one
  // point. With some camera/LiDAR poses a frame can legitimately project zero
  // points, which made this buffer grow without bound across scans and consume
  // multiple GiB. Flush it whenever its configured publication cycle is
  // complete, irrespective of the number of RGB points produced.
  if(flush_wait_pub) PointCloudXYZI().swap(*pcl_wait_pub);
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);

#ifdef __GLIBC__
  // PCL/OpenCV repeatedly allocate large temporary buffers. glibc may keep
  // those freed pages attached to the process, so RSS can look like a leak
  // even after the clouds have been released. Trim infrequently to return
  // genuinely free top-of-heap pages to the OS without touching estimator
  // state or affecting every frame.
  static unsigned int heap_trim_counter = 0;
  if (++heap_trim_counter % 50 == 0) malloc_trim(0);
#endif

}

void LIVMapper::publish_visual_sub_map(const ros::Publisher &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap.publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  const double update_time = currentUpdateTime();
  odomAftMapped.header.stamp = ros::Time().fromSec(update_time > 0.0 ? update_time : ros::Time::now().toSec());
  set_posestamp(odomAftMapped.pose.pose);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped") );
  pubOdomAftMapped.publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher.publish(msg_body_pose);
}

void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}
