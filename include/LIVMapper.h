/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <limits>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <vikit/camera_loader.h>

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void handleMapOnlyFallback();
  void resetLocalVoxelMapForRecovery();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback(const ros::TimerEvent &e);
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void mapCorrectionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void relativeMotionHintCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void step3GlobalLockCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void step3TrackingCorrectionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void applyPendingStep3TrackingCorrection();
  bool activateStep3PriorMap();
  void applyPendingMapCorrection();
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  void readParameters(ros::NodeHandle &nh);
  void updateFailsafeHealthAndMode();
  void markSensorSample(SensorHealth &health, int &recovery_count, int required_samples, const char *name);
  void setFailsafeMode(FailsafeMode mode, const std::string &reason);
  void publishFailsafeStatus(bool force = false);
  void trimStaleBuffers(double estimator_time);
  bool syncLidarImuFallback(LidarMeasureGroup &meas);
  bool syncLidarNoImuMapFallback(LidarMeasureGroup &meas);
  bool syncVisionImuFallback(LidarMeasureGroup &meas);
  double currentUpdateTime() const;
  void inflateFallbackCovariance(double scale);
  static const char *sensorHealthName(SensorHealth health);
  static const char *failsafeModeName(FailsafeMode mode);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  // Runtime failsafe state. Static img_en/lidar_en/imu_en remain configuration
  // flags and are never changed to represent a runtime sensor fault.
  bool failsafe_enable_ = true;
  // Offline diagnostic only: preserve camera subscriptions but schedule
  // 10 Hz LiDAR/IMU updates instead of image-timed LIVO event splitting.
  bool diagnostic_force_lidar_sync_en_ = false;
  bool failsafe_initialized_ = false;
  bool visual_constraints_ok_ = true;
  bool lidar_rejoin_guard_ = false;
  FailsafeMode failsafe_mode_ = FailsafeMode::FULL_LIVO;
  SensorHealth camera_health_ = SensorHealth::UNKNOWN;
  SensorHealth lidar_health_ = SensorHealth::UNKNOWN;
  SensorHealth imu_health_ = SensorHealth::UNKNOWN;
  // Continuous [0,1] confidence is separate from timeout health.  Health
  // chooses the active sensor subset; confidence controls how strongly a
  // surviving measurement is allowed to influence the one persistent ESIKF.
  double camera_confidence_ = 1.0;
  double lidar_confidence_ = 1.0;
  double imu_confidence_ = 1.0;
  // Visual-update hysteresis is deliberately separate from packet health.
  // A camera may be streaming while its direct-photometric constraints are
  // weak; in that case keep LiDAR+IMU as the ESIKF authority until several
  // consecutive strong visual frames are observed again.
  bool healthy_visual_gate_enable_ = true;
  bool gated_visual_fastpath_enable_ = false;
  bool preserve_lidar_cadence_ = false;
  bool livo_image_due_ = true;
  // On this recorded cart, direct healthy-mode photometric residuals accumulate
  // a slow pose bias even when every individual correction is small. Keep them
  // available for bootstrap and LiDAR-loss recovery, but let LIO own healthy
  // motion unless explicitly re-enabled.
  bool healthy_visual_pose_fusion_enable_ = false;
  // Guard only implausibly large *single LiDAR ESIKF* corrections, never
  // overwrite a legitimate IMU/ESIKF prediction with a global-map pose.
  bool healthy_lidar_update_guard_enable_ = false;
  double healthy_lidar_max_update_translation_m_ = 0.75;
  double healthy_lidar_max_update_rotation_deg_ = 7.0;
  double healthy_lidar_max_update_velocity_mps_ = 3.0;
  bool visual_fusion_enabled_ = true;
  int healthy_visual_min_points_ = 24;
  int healthy_visual_bootstrap_frames_ = 24;
  int healthy_visual_reenable_samples_ = 8;
  int visual_frames_seen_ = 0;
  int visual_recovery_good_frames_ = 0;
  double healthy_visual_min_confidence_ = 0.55;
  double healthy_visual_reenable_confidence_ = 0.70;
  double healthy_visual_max_translation_update_ = 0.20;
  double healthy_visual_max_rotation_deg_ = 2.0;
  double healthy_visual_max_velocity_update_ = 1.0;
  double last_camera_rx_time_ = -1.0, last_lidar_rx_time_ = -1.0, last_imu_rx_time_ = -1.0;
  double camera_timeout_ = 0.5, lidar_timeout_ = 0.5, imu_timeout_ = 0.2;
  // Maximum interval after a valid RGB-D motion observation during LiDAR loss.
  // Separate from camera image timeout: a loaded replay can still receive
  // camera frames while pose-feature processing lags by >400 ms.
  double lidar_fail_rgbd_fresh_window_sec_ = 0.40;
  int camera_recovery_samples_ = 5, lidar_recovery_samples_ = 3, imu_recovery_samples_ = 20;
  int camera_recovery_count_ = 0, lidar_recovery_count_ = 0, imu_recovery_count_ = 0;
  int min_lidar_points_ = 50, min_visual_constraints_ = 8;
  double no_imu_covariance_scale_ = 5.0;
  double camera_fail_covariance_scale_ = 1.05;
  double lidar_fail_covariance_scale_ = 1.10;
  double imu_bridge_covariance_scale_ = 1.50;
  double lidar_rejoin_max_translation_jump_ = 0.8;
  double lidar_rejoin_max_rotation_deg_ = 20.0;
  double imu_rejoin_max_rotation_deg_ = 4.0;
  double imu_rejoin_max_tilt_correction_deg_ = 8.0;
  bool recovery_from_imu_outage_ = false;
  V3D imu_recovery_acc_sum_ = V3D::Zero();
  V3D imu_recovery_gyr_sum_ = V3D::Zero();
  int imu_recovery_sample_count_ = 0;
  double lidar_fail_visual_max_translation_update_ = 0.35;
  double lidar_fail_visual_max_rotation_deg_ = 5.0;
  double lidar_fail_visual_max_velocity_update_ = 1.5;
  bool map_correction_enable_ = true;
  double map_correction_max_age_ = 0.5;
  double map_correction_max_translation_jump_ = 2.5;
  double map_correction_max_rotation_deg_ = 20.0;
  double map_correction_blend_ = 1.0;
  bool map_depth_primary_on_lidar_failure_ = false;
  bool map_primary_on_imu_failure_ = false;
  bool rebuild_voxel_map_on_recovery_ = false;
  int recovery_lio_warmup_scans_ = 5;
  int recovery_lio_warmup_remaining_ = 0;
  double last_recovery_warmup_lio_time_ = -1.0;
  bool map_correction_prev_valid_ = false;
  ros::Time prev_map_correction_stamp_;
  V3D prev_map_correction_pos_ = V3D::Zero();
  M3D prev_map_correction_rot_ = M3D::Identity();
  std::mutex mtx_map_correction_;
  bool map_correction_pending_ = false;
  ros::Time map_correction_stamp_;
  V3D map_correction_pos_ = V3D::Zero();
  M3D map_correction_rot_ = M3D::Identity();
  std::string map_correction_source_;
  double map_correction_quality_ = std::numeric_limits<double>::infinity();
  ros::Time last_applied_map_correction_stamp_;
  std::string last_applied_map_correction_source_;

  // Step 3: automatic global acquisition + immutable prior-map tracking.
  // The ESIKF remains in its continuous camera_init frame.  A one-time
  // map<-camera_init lock transforms the saved prior PCD into that frame;
  // subsequent LiDAR updates use the normal FAST-LIVO2 point-to-plane ESIKF
  // against the immutable map, without moving the global anchor.
  bool step3_prior_enable_ = false;
  bool step3_prior_active_ = false;
  bool step3_prior_lock_pending_ = false;
  // false (default): keep the Step-2 local voxel map and fuse the immutable
  // prior as a second measurement map into the SAME ESIKF.  true exists only
  // for regression testing of the rejected map-replacement experiment.
  bool step3_replace_local_voxel_map_ = false;
  std::string step3_prior_map_path_;
  double step3_prior_leaf_ = 0.30;
  double step3_prior_point_variance_ = 0.01;
  Eigen::Matrix4d step3_T_map_fast_ = Eigen::Matrix4d::Identity();
  std::mutex mtx_step3_prior_;
  PointCloudXYZI::Ptr step3_prior_map_mapframe_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> step3_prior_voxel_map_;
  VoxelMapManagerPtr step3_prior_voxelmap_manager;

  // Step-3 production global tracking: GICP is a measurement front-end only.
  // It never owns a second odometry state and never changes map<-camera_init.
  bool step3_tracking_correction_enable_ = true;
  double step3_tracking_max_age_ = 1.2;
  double step3_tracking_xy_gain_ = 0.65;
  double step3_tracking_max_xy_step_ = 0.30;
  double step3_tracking_yaw_gain_ = 0.55;
  double step3_tracking_max_yaw_step_deg_ = 1.0;
  double step3_tracking_hard_xy_gate_ = 2.0;
  double step3_tracking_hard_yaw_gate_deg_ = 10.0;
  std::mutex mtx_step3_tracking_;
  bool step3_tracking_pending_ = false;
  ros::Time step3_tracking_stamp_;
  V3D step3_tracking_pos_ = V3D::Zero();
  M3D step3_tracking_rot_ = M3D::Identity();
  double step3_tracking_fitness_ = std::numeric_limits<double>::infinity();
  bool step3_tracking_measurement_is_residual_ = false;
  bool step3_tracking_measurement_is_gyro_bias_ = false;
  bool step3_gyro_bias_feedback_enable_ = false;
  bool step3_tracking_measurement_is_anchored_local_ = false;
  bool step3_anchored_local_feedback_enable_ = false;

  // Accepted map measurements are targets for the SAME ESIKF state.  They are
  // released causally over normal estimator cycles instead of teleporting the
  // state when a low-rate global correction arrives.
  bool smooth_map_target_valid_ = false;
  V3D smooth_map_target_pos_ = V3D::Zero();
  M3D smooth_map_target_rot_ = M3D::Identity();
  bool smooth_map_target_rotation_enabled_ = false;
  ros::Time smooth_map_last_apply_time_;
  // Saved-map residuals are released gradually into the persistent ESIKF.
  // The defaults mirror the stable standalone failure experiments: the map
  // may correct drift quickly, but it must never teleport the golf cart.
  double smooth_map_max_pos_rate_ = 1.3;       // m/s correction component
  double smooth_map_max_rot_rate_deg_ = 16.0;  // deg/s correction component
  double smooth_map_max_pos_rate_lidar_fail_ = 1.3;
  double smooth_map_max_rot_rate_lidar_fail_deg_ = 0.0;
  double smooth_map_max_pos_rate_imu_fail_ = 3.0;
  double smooth_map_max_rot_rate_imu_fail_deg_ = 30.0;

  bool lidar_recovery_map_lock_ = false;
  bool degraded_map_alignment_valid_ = false;
  M3D degraded_R_odom_map_ = M3D::Identity();
  V3D degraded_t_odom_map_ = V3D::Zero();
  ros::Time last_relative_motion_stamp_;
  std::string last_relative_motion_source_;
  V3D relative_linear_body_ = V3D::Zero();
  V3D relative_angular_body_ = V3D::Zero();
  ros::Time last_lidar_relative_stamp_, last_depth_relative_stamp_,
      last_rgbd_relative_stamp_;
  V3D lidar_relative_linear_body_ = V3D::Zero();
  V3D lidar_relative_angular_body_ = V3D::Zero();
  V3D depth_relative_linear_body_ = V3D::Zero();
  V3D depth_relative_angular_body_ = V3D::Zero();
  V3D rgbd_relative_linear_body_ = V3D::Zero();
  V3D rgbd_relative_angular_body_ = V3D::Zero();

  // Proven degraded-mode front-ends produce a filtered SE(3) increment.  Keep
  // one causal anchor inside the SAME FAST-LIVO2 state so the exact standalone
  // propagation can be reproduced without launching/merging another odometry.
  bool fallback_increment_anchor_valid_ = false;
  M3D fallback_increment_rot_ = M3D::Identity();
  V3D fallback_increment_pos_ = V3D::Zero();
  ros::Time fallback_increment_stamp_;
  std::string fallback_increment_source_;

  // During LiDAR loss, RGB-D supplies a short-term *relative displacement*
  // constraint to the same ESIKF while the healthy IMU remains attitude
  // authority.  This is not a second odometry state: it measures how far the
  // existing ESIKF moved between consecutive RGB-D observations and nudges
  // that displacement toward the metric RGB-D observation.
  bool rgbd_displacement_constraint_valid_ = false;
  ros::Time rgbd_displacement_constraint_stamp_;
  V3D rgbd_displacement_constraint_state_pos_ = V3D::Zero();
  double rgbd_displacement_gain_ = 0.75;
  double rgbd_displacement_max_step_ = 0.30;
  double rgbd_velocity_weight_ = 0.70;

  // v3 degraded-motion update: consume the exact filtered SE(3) increments
  // from the already-validated standalone front-ends as residual measurements
  // on the ONE persistent ESIKF state.  The current state is never advanced by
  // simply multiplying another odometry trajectory on top of IMU/no-IMU
  // propagation; only the difference to the measured next pose is applied.
  bool relative_delta_residual_enable_ = true;
  bool rgbd_delta_residual_enable_ = true;
  bool lidar_delta_residual_enable_ = true;
  bool lidar_delta_direct_state_update_ = true;
  bool rgbd_delta_direct_position_update_ = true;
  double rgbd_delta_position_gain_ = 1.0;
  double rgbd_delta_rotation_gain_ = 0.20;
  double rgbd_delta_max_position_step_ = 0.45;
  double rgbd_delta_max_rotation_step_deg_ = 2.0;
  double lidar_delta_position_gain_ = 0.90;
  double lidar_delta_rotation_gain_ = 0.85;
  double lidar_delta_max_position_step_ = 0.80;
  double lidar_delta_max_rotation_step_deg_ = 6.0;

  // During IMU loss, consecutive place+LiDAR verified map poses are the
  // strongest available motion observation.  Keep their metric velocity and
  // angular rate separate from raw scan-to-scan hints.
  ros::Time last_verified_map_motion_stamp_;
  V3D verified_map_linear_world_ = V3D::Zero();
  V3D verified_map_angular_body_ = V3D::Zero();
  bool verified_map_motion_valid_ = false;

  V3D lidar_fail_velocity_reference_ = V3D::Zero();
  bool lidar_fail_velocity_valid_ = false;
  double last_status_publish_time_ = -1.0;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  ros::Publisher pubFailsafeMode;
  ros::Publisher pubSensorHealth;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  bool colorize_only_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double visual_map_radius = 60.0;
  int visual_map_prune_every_n = 20;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  ros::Publisher plane_pub;
  ros::Publisher voxel_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Subscriber sub_map_correction;
  ros::Subscriber sub_relative_motion_hint;
  ros::Subscriber sub_step3_global_lock;
  ros::Subscriber sub_step3_tracking_correction;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubNormal;
  ros::Publisher pubSubVisualMap;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubLaserCloudMap;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudDyn;
  ros::Publisher pubLaserCloudDynRmed;
  ros::Publisher pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  ros::Publisher mavros_pose_publisher;
  ros::Timer imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif
