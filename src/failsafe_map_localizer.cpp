#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <mutex>
#include <deque>
#include <string>
#include <vector>

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

Eigen::Matrix3d matrixFromVector(const std::vector<double> &v)
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  if (v.size() == 9)
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        R(r, c) = v[r * 3 + c];
  return R;
}

Eigen::Vector3d vectorFromVector(const std::vector<double> &v)
{
  if (v.size() == 3) return Eigen::Vector3d(v[0], v[1], v[2]);
  return Eigen::Vector3d::Zero();
}

double rotationAngleDeg(const Eigen::Matrix3d &R)
{
  double c = (R.trace() - 1.0) * 0.5;
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c) * 180.0 / M_PI;
}

double wrapAngleRad(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}

class FailsafeMapLocalizer
{
public:
  FailsafeMapLocalizer()
      : nh_(), pnh_("~"), map_(new CloudT), visual_map_(new CloudT),
        map_kdtree_(new pcl::KdTreeFLANN<PointT>),
        visual_map_kdtree_(new pcl::KdTreeFLANN<PointT>), latest_odom_valid_(false)
  {
    pnh_.param<std::string>("mode", mode_, "localize");
    pnh_.param<std::string>("map_path", map_path_, "/results/failsafe_saved_map.pcd");
    pnh_.param<std::string>("visual_map_path", visual_map_path_, "");
    pnh_.param<bool>("visual_map_enable", visual_map_enable_, false);
    pnh_.param<std::string>("registered_cloud_topic", registered_cloud_topic_, "/cloud_registered");
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/fast_livo2/input/lidar");
    pnh_.param<std::string>("depth_topic", depth_topic_, "/camera/depth/image_rect_raw");
    pnh_.param<std::string>("depth_info_topic", depth_info_topic_, "/camera/depth/camera_info");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/aft_mapped_to_init");
    pnh_.param<std::string>("failsafe_mode_topic", failsafe_mode_topic_, "/fast_livo2/failsafe_mode");
    pnh_.param<std::string>("place_candidates_topic", place_candidates_topic_,
                            "/fast_livo2/place_pose_candidates");
    pnh_.param<std::string>("initial_pose_topic", initial_pose_topic_, "/initialpose");
    pnh_.param<std::string>("step3_global_pose_topic", step3_global_pose_topic_,
                            "/fast_livo2/step3_global_pose");
    pnh_.param<double>("step3_global_pose_max_age", step3_global_pose_max_age_, 0.75);
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("global_pose_topic", global_pose_topic_, "/fast_livo2/map_pose");
    pnh_.param<std::string>("global_odom_topic", global_odom_topic_, "/fast_livo2/map_odom");
    pnh_.param<std::string>("global_path_topic", global_path_topic_, "/fast_livo2/map_path");
    pnh_.param<std::string>("localization_status_topic", localization_status_topic_,
                            "/fast_livo2/localization_status");
    pnh_.param<bool>("publish_legacy_outputs", publish_legacy_outputs_, true);
    pnh_.param<std::string>("legacy_pose_topic", legacy_pose_topic_, "/pose");
    pnh_.param<std::string>("legacy_odom_topic", legacy_odom_topic_, "/odom");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<bool>("publish_map_to_base_tf", publish_map_to_base_tf_, true);
    // v2 keeps the persistent FAST-LIVO2 ESIKF as the only trajectory state.
    // The old v1 fallback independently integrated RGB-D/LiDAR deltas in map
    // frame and then rewrote map<-camera_init.  Saved-map measurements were
    // simultaneously correcting the ESIKF, creating two correction paths and
    // visible mode-switch/recovery jumps.  Keep the old path only as an
    // opt-in diagnostic switch; production/default is one-state continuity.
    pnh_.param<bool>("legacy_fallback_map_pose_enable",
                     legacy_fallback_map_pose_enable_, false);
    pnh_.param<bool>("degraded_global_anchor_only",
                     degraded_global_anchor_only_, true);
    pnh_.param<bool>("imu_failure_esikf_map_correction_enable",
                     imu_failure_esikf_map_correction_enable_, true);
    pnh_.param<bool>("external_lidar_relative_frontend",
                     external_lidar_relative_frontend_, true);
    pnh_.param<double>("anchor_refinement_freeze_sec",
                       anchor_refinement_freeze_sec_, 2.0);
    pnh_.param<double>("place_hint_max_age", place_hint_max_age_, 1.25);
    // Feature-verified RGB-D PnP is more expensive than appearance-only
    // retrieval. Only during LiDAR loss may it use bounded, timestamped
    // ESIKF-odometry transport before *fresh* depth-map GICP confirmation.
    // Never relabel old measurement stamps as 'now'.
    pnh_.param<double>("place_metric_lidar_fail_max_age",
                       place_metric_lidar_fail_max_age_, 5.5);
    pnh_.param<double>("place_bootstrap_max_age", place_bootstrap_max_age_, 7.0);
    pnh_.param<double>("rolling_lidar_window_sec", rolling_lidar_window_sec_, 0.60);
    pnh_.param<int>("rolling_lidar_max_scans", rolling_lidar_max_scans_, 5);
    pnh_.param<double>("rolling_depth_window_sec", rolling_depth_window_sec_, 0.45);
    pnh_.param<int>("rolling_depth_max_scans", rolling_depth_max_scans_, 3);
    pnh_.param<double>("rolling_map_translation_gain", rolling_map_translation_gain_, 0.55);
    pnh_.param<double>("rolling_map_max_translation_step", rolling_map_max_translation_step_, 0.45);
    pnh_.param<double>("rolling_map_max_z_step", rolling_map_max_z_step_, 0.15);
    pnh_.param<double>("rolling_map_yaw_gain", rolling_map_yaw_gain_, 0.45);
    pnh_.param<double>("rolling_map_max_yaw_step_deg", rolling_map_max_yaw_step_deg_, 1.0);
    pnh_.param<double>("rolling_map_max_translation_innovation", rolling_map_max_translation_innovation_, 3.0);
    pnh_.param<double>("rolling_map_max_yaw_innovation_deg", rolling_map_max_yaw_innovation_deg_, 12.0);
    pnh_.param<double>("rolling_heading_baseline_m", rolling_heading_baseline_m_, 4.0);
    pnh_.param<double>("rolling_heading_gain", rolling_heading_gain_, 0.30);
    pnh_.param<double>("rolling_heading_filter_gain", rolling_heading_filter_gain_, 0.35);
    pnh_.param<double>("rolling_heading_max_step_deg", rolling_heading_max_step_deg_, 0.35);
    pnh_.param<double>("rolling_heading_max_innovation_deg", rolling_heading_max_innovation_deg_, 8.0);
    pnh_.param<double>("rolling_heading_min_distance_ratio", rolling_heading_min_distance_ratio_, 0.70);
    pnh_.param<double>("rolling_heading_max_distance_ratio", rolling_heading_max_distance_ratio_, 1.30);

    pnh_.param<double>("map_voxel_leaf", map_voxel_leaf_, 0.25);
    pnh_.param<double>("source_voxel_leaf", source_voxel_leaf_, 0.25);
    pnh_.param<double>("local_map_radius", local_map_radius_, 22.0);
    pnh_.param<double>("local_map_z_radius", local_map_z_radius_, 5.0);
    pnh_.param<double>("icp_max_correspondence", icp_max_correspondence_, 2.0);
    pnh_.param<double>("icp_fitness_threshold", icp_fitness_threshold_, 0.7);
    pnh_.param<int>("icp_max_iterations", icp_max_iterations_, 35);
    pnh_.param<double>("max_correction_translation", max_correction_translation_, 2.2);
    pnh_.param<double>("max_correction_rotation_deg", max_correction_rotation_deg_, 18.0);
    pnh_.param<double>("depth_map_fitness_threshold", depth_map_fitness_threshold_, 0.30);
    pnh_.param<double>("depth_map_max_translation", depth_map_max_translation_, 0.8);
    pnh_.param<double>("depth_map_max_rotation_deg", depth_map_max_rotation_deg_, 3.0);
    pnh_.param<double>("depth_bootstrap_max_translation", depth_bootstrap_max_translation_, 1.2);
    pnh_.param<double>("depth_bootstrap_max_rotation_deg", depth_bootstrap_max_rotation_deg_, 5.0);
    // Appearance retrieval identifies the global route location; depth GICP
    // verifies/refines that seed.  On repeated road geometry GICP can have an
    // excellent fitness while sliding several metres along the road.  Bound
    // only the translation actually applied from the visual seed; the raw
    // GICP solution is still used for geometry/continuity validation.
    pnh_.param<double>("depth_place_max_applied_translation",
                       depth_place_max_applied_translation_, 3.0);
    pnh_.param<bool>("depth_place_position_relock_enable",
                     depth_place_position_relock_enable_, false);
    pnh_.param<double>("depth_place_position_relock_max_fitness",
                       depth_place_position_relock_max_fitness_, 0.11);
    pnh_.param<double>("depth_place_position_relock_min_overlap",
                       depth_place_position_relock_min_overlap_, 0.90);
    pnh_.param<double>("depth_place_position_relock_max_seed_translation",
                       depth_place_position_relock_max_seed_translation_, 1.25);
    pnh_.param<double>("depth_place_position_relock_max_seed_rotation_deg",
                       depth_place_position_relock_max_seed_rotation_deg_, 8.0);
    pnh_.param<double>("depth_place_position_relock_max_continuity_translation",
                       depth_place_position_relock_max_continuity_translation_, 6.0);
    pnh_.param<bool>("depth_place_preserve_visual_orientation",
                     depth_place_preserve_visual_orientation_, false);
    // In LiDAR loss the RGB-D predicted global yaw can already be wrong.
    // Only a sequence of independent, strong depth+visual geometry hypotheses
    // may overrule that prediction; one repeated-road match never can.
    pnh_.param<bool>("depth_place_temporal_relock_enable",
                     depth_place_temporal_relock_enable_, false);
    pnh_.param<int>("depth_place_temporal_relock_min_fixes",
                    depth_place_temporal_relock_min_fixes_, 3);
    pnh_.param<double>("depth_place_temporal_relock_max_fitness",
                       depth_place_temporal_relock_max_fitness_, 0.09);
    pnh_.param<double>("depth_place_temporal_relock_min_overlap",
                       depth_place_temporal_relock_min_overlap_, 0.95);
    pnh_.param<double>("depth_place_temporal_relock_max_speed_mps",
                       depth_place_temporal_relock_max_speed_mps_, 3.5);
    pnh_.param<double>("depth_place_temporal_relock_max_accel_mps2",
                       depth_place_temporal_relock_max_accel_mps2_, 4.0);
    pnh_.param<double>("depth_place_temporal_relock_max_course_change_deg",
                       depth_place_temporal_relock_max_course_change_deg_, 65.0);
    pnh_.param<double>("lidar_localization_hz", lidar_localization_hz_, 7.0);
    pnh_.param<double>("depth_localization_hz", depth_localization_hz_, 5.0);
    pnh_.param<bool>("depth_fallback_map_enable", depth_fallback_map_enable_, false);
    pnh_.param<bool>("depth_failover_only", depth_failover_only_, false);
    pnh_.param<double>("anchor_localization_hz", anchor_localization_hz_, 2.0);
    pnh_.param<double>("anchor_max_correspondence", anchor_max_correspondence_, 5.0);
    pnh_.param<double>("anchor_fitness_threshold", anchor_fitness_threshold_, 1.2);
    pnh_.param<double>("anchor_overlap_threshold", anchor_overlap_threshold_, 0.50);
    pnh_.param<double>("anchor_max_translation", anchor_max_translation_, 8.0);
    pnh_.param<double>("anchor_max_rotation_deg", anchor_max_rotation_deg_, 40.0);
    pnh_.param<double>("anchor_source_voxel_leaf", anchor_source_voxel_leaf_, 0.90);
    pnh_.param<double>("anchor_target_voxel_leaf", anchor_target_voxel_leaf_, 1.00);
    pnh_.param<double>("anchor_local_map_radius", anchor_local_map_radius_, 16.0);
    pnh_.param<double>("anchor_local_map_z_radius", anchor_local_map_z_radius_, 4.0);
    pnh_.param<int>("anchor_gicp_iterations", anchor_gicp_iterations_, 12);
    pnh_.param<int>("anchor_correspondence_randomness",
                    anchor_correspondence_randomness_, 12);
    pnh_.param<double>("anchor_overlap_distance", anchor_overlap_distance_, 1.0);
    pnh_.param<bool>("use_online_anchor", use_online_anchor_, false);
    pnh_.param<int>("anchor_lock_samples", anchor_lock_samples_, 3);
    pnh_.param<double>("anchor_consistency_translation",
                       anchor_consistency_translation_, 1.0);
    pnh_.param<double>("anchor_consistency_rotation_deg",
                       anchor_consistency_rotation_deg_, 3.0);
    pnh_.param<double>("anchor_update_max_translation_step",
                       anchor_update_max_translation_step_, 0.15);
    pnh_.param<double>("anchor_update_max_rotation_step_deg",
                       anchor_update_max_rotation_step_deg_, 1.5);
    pnh_.param<double>("anchor_target_max_translation_rate",
                       anchor_target_max_translation_rate_, 3.0);
    pnh_.param<double>("anchor_target_max_rotation_rate_deg",
                       anchor_target_max_rotation_rate_deg_, 20.0);
    pnh_.param<double>("healthy_anchor_target_max_translation_rate",
                       healthy_anchor_target_max_translation_rate_, 1.5);
    pnh_.param<double>("healthy_anchor_target_max_rotation_rate_deg",
                       healthy_anchor_target_max_rotation_rate_deg_, 12.0);
    pnh_.param<double>("degraded_anchor_target_max_translation_jump",
                       degraded_anchor_target_max_translation_jump_, 4.0);
    pnh_.param<double>("degraded_anchor_target_max_rotation_jump_deg",
                       degraded_anchor_target_max_rotation_jump_deg_, 18.0);
    pnh_.param<bool>("fixed_anchor_enable", fixed_anchor_enable_, false);
    std::vector<double> fixed_anchor_t, fixed_anchor_rpy_deg;
    pnh_.getParam("fixed_anchor_translation", fixed_anchor_t);
    pnh_.getParam("fixed_anchor_rpy_deg", fixed_anchor_rpy_deg);
    if (fixed_anchor_enable_ && fixed_anchor_t.size() == 3 &&
        fixed_anchor_rpy_deg.size() == 3)
    {
      const double rr = fixed_anchor_rpy_deg[0] * M_PI / 180.0;
      const double rp = fixed_anchor_rpy_deg[1] * M_PI / 180.0;
      const double ry = fixed_anchor_rpy_deg[2] * M_PI / 180.0;
      const Eigen::Matrix3f R =
          (Eigen::AngleAxisf(static_cast<float>(ry), Eigen::Vector3f::UnitZ()) *
           Eigen::AngleAxisf(static_cast<float>(rp), Eigen::Vector3f::UnitY()) *
           Eigen::AngleAxisf(static_cast<float>(rr), Eigen::Vector3f::UnitX()))
              .toRotationMatrix();
      anchor_T_map_current_.setIdentity();
      anchor_T_map_current_.block<3, 3>(0, 0) = R;
      anchor_T_map_current_.block<3, 1>(0, 3) =
          Eigen::Vector3f(fixed_anchor_t[0], fixed_anchor_t[1], fixed_anchor_t[2]);
      anchor_valid_ = true;
      ROS_WARN("[FAILSAFE MAP] Fixed startup anchor enabled: %.2fm/%.1fdeg",
               anchor_T_map_current_.block<3, 1>(0, 3).norm(),
               rotationAngleDeg(R.cast<double>()));
    }
    pnh_.param<double>("tracker_max_odom_delta_translation",
                       tracker_max_odom_delta_translation_, 1.5);
    pnh_.param<double>("tracker_max_odom_delta_rotation_deg",
                       tracker_max_odom_delta_rotation_deg_, 12.0);
    // LiDAR-loss depth tracking: once Step-3 has seeded the tracker from a
    // trusted global pose, prefer tight local RGB-D -> saved visual-map GICP
    // for short-horizon absolute corrections. Appearance retrieval is then a
    // reacquisition mechanism, not a mandatory precursor to every depth fix.
    pnh_.param<double>("depth_tracker_max_age_sec",
                       depth_tracker_max_age_sec_, 2.5);
    pnh_.param<double>("depth_local_fitness_threshold",
                       depth_local_fitness_threshold_, 0.10);
    pnh_.param<double>("depth_local_overlap_threshold",
                       depth_local_overlap_threshold_, 0.75);
    pnh_.param<double>("depth_local_max_translation",
                       depth_local_max_translation_, 0.90);
    pnh_.param<double>("depth_local_max_rotation_deg",
                       depth_local_max_rotation_deg_, 8.0);
    pnh_.param<bool>("structural_filter_enable", structural_filter_enable_, true);
    pnh_.param<double>("structural_min_body_z", structural_min_body_z_, -0.9);
    pnh_.param<double>("structural_max_body_z", structural_max_body_z_, 4.0);
    pnh_.param<double>("structural_min_range", structural_min_range_, 1.0);
    pnh_.param<int>("structural_min_points", structural_min_points_, 250);
    pnh_.param<double>("relative_max_correspondence", relative_max_correspondence_, 1.2);
    pnh_.param<double>("relative_fitness_threshold", relative_fitness_threshold_, 0.45);
    pnh_.param<double>("relative_max_translation", relative_max_translation_, 1.5);
    pnh_.param<double>("relative_max_rotation_deg", relative_max_rotation_deg_, 12.0);
    pnh_.param<int>("depth_stride", depth_stride_, 5);
    pnh_.param<double>("depth_scale", depth_scale_, 0.001);
    pnh_.param<double>("depth_min_m", depth_min_m_, 0.5);
    pnh_.param<double>("depth_max_m", depth_max_m_, 12.0);
    pnh_.param<double>("depth_fx", depth_fx_, 0.0);
    pnh_.param<double>("depth_fy", depth_fy_, 0.0);
    pnh_.param<double>("depth_cx", depth_cx_, 0.0);
    pnh_.param<double>("depth_cy", depth_cy_, 0.0);
    pnh_.param<int>("map_accumulate_every_n", map_accumulate_every_n_, 2);
    pnh_.param<int>("map_downsample_every_n", map_downsample_every_n_, 20);

    std::vector<double> extR_vec, extT_vec, Rcl_vec, Pcl_vec;
    if (!nh_.getParam("/extrin_calib/extrinsic_R", extR_vec))
      pnh_.getParam("extrinsic_R", extR_vec);
    if (!nh_.getParam("/extrin_calib/extrinsic_T", extT_vec))
      pnh_.getParam("extrinsic_T", extT_vec);
    if (!nh_.getParam("/extrin_calib/Rcl", Rcl_vec))
      pnh_.getParam("Rcl", Rcl_vec);
    if (!nh_.getParam("/extrin_calib/Pcl", Pcl_vec))
      pnh_.getParam("Pcl", Pcl_vec);
    extR_ = matrixFromVector(extR_vec);
    extT_ = vectorFromVector(extT_vec);
    Rcl_ = matrixFromVector(Rcl_vec);
    Pcl_ = vectorFromVector(Pcl_vec);

    std::vector<double> depth_R, depth_T;
    pnh_.getParam("depth_to_color_R", depth_R);
    pnh_.getParam("depth_to_color_T", depth_T);
    R_depth_color_ = matrixFromVector(depth_R);
    t_depth_color_ = vectorFromVector(depth_T);

    correction_pub_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fast_livo2/map_correction", 3);
    relative_motion_pub_ =
        nh_.advertise<geometry_msgs::TwistStamped>("/fast_livo2/relative_motion_hint", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/fast_livo2/map_localizer_status", 3, true);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fast_livo2/map_aligned_source", 1);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fast_livo2/saved_map", 1, true);
    global_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(global_pose_topic_, 20);
    global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odom_topic_, 20);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>(global_path_topic_, 2);
    depth_failover_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/fast_livo2/step3_depth_map_pose", 3);
    localization_status_pub_ =
        nh_.advertise<std_msgs::String>(localization_status_topic_, 3, true);
    if (publish_legacy_outputs_)
    {
      legacy_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(legacy_pose_topic_, 20);
      legacy_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(legacy_odom_topic_, 20);
    }

    if (mode_ == "build")
    {
      cloud_sub_ = nh_.subscribe(registered_cloud_topic_, 2,
                                 &FailsafeMapLocalizer::registeredCloudCallback, this);
      save_timer_ = nh_.createTimer(ros::Duration(5.0),
                                    &FailsafeMapLocalizer::saveTimerCallback, this);
      ROS_INFO("[FAILSAFE MAP] Build mode: accumulating %s -> %s",
               registered_cloud_topic_.c_str(), map_path_.c_str());
    }
    else
    {
      if (!depth_failover_only_)
      {
        if (pcl::io::loadPCDFile<PointT>(map_path_, *map_) != 0 || map_->empty())
          throw std::runtime_error("Failsafe saved map missing/empty: " + map_path_);
        downsampleInPlace(map_, map_voxel_leaf_);
        map_kdtree_->setInputCloud(map_);
      }
      if (visual_map_enable_ && !visual_map_path_.empty())
      {
        if (pcl::io::loadPCDFile<PointT>(visual_map_path_, *visual_map_) != 0 ||
            visual_map_->empty())
          throw std::runtime_error("Visual/RGB-D saved map missing/empty: " + visual_map_path_);
        downsampleInPlace(visual_map_, map_voxel_leaf_);
        visual_map_kdtree_->setInputCloud(visual_map_);
        ROS_INFO("[FAILSAFE MAP] loaded %zu camera/RGB-D voxel-map points from %s",
                 visual_map_->size(), visual_map_path_.c_str());
      }
      if (!depth_failover_only_) publishMap();

      odom_sub_ = nh_.subscribe(odom_topic_, 20, &FailsafeMapLocalizer::odomCallback, this);
      mode_sub_ = nh_.subscribe(failsafe_mode_topic_, 10, &FailsafeMapLocalizer::modeCallback, this);
      initial_pose_sub_ = nh_.subscribe(initial_pose_topic_, 2,
                                       &FailsafeMapLocalizer::initialPoseCallback, this);
      step3_global_pose_sub_ = nh_.subscribe(
          step3_global_pose_topic_, 20,
          &FailsafeMapLocalizer::step3GlobalPoseCallback, this);
      // RGB-D relative motion history is lightweight and is required by the
      // LiDAR-outage global predictor even in depth-failover-only mode.
      rgbd_relative_delta_sub_ = nh_.subscribe(
          "/fast_livo2/rgbd_relative_motion_hint", 50,
          &FailsafeMapLocalizer::relativeDeltaCallback, this);
      // Visual place candidates are just coarse measurements and are required
      // by the Step-3 depth failover frontend as well. Keep this subscriber
      // alive even when healthy LiDAR/map tracking is disabled. The candidate
      // cannot affect localization until a depth callback geometrically
      // verifies it against the saved visual voxel map.
      place_sub_ = nh_.subscribe(place_candidates_topic_, 2,
                                 &FailsafeMapLocalizer::placeCandidatesCallback, this);
      if (!depth_failover_only_)
      {
        relative_delta_sub_ = nh_.subscribe(
            "/fast_livo2/relative_motion_hint", 50,
            &FailsafeMapLocalizer::relativeDeltaCallback, this);
        lidar_sub_ = nh_.subscribe(lidar_topic_, 3, &FailsafeMapLocalizer::lidarCallback, this);
      }
      depth_info_sub_ = nh_.subscribe(depth_info_topic_, 1,
                                      &FailsafeMapLocalizer::depthInfoCallback, this);
      if (depth_failover_only_)
      {
        publish_legacy_outputs_ = false;
        publish_map_to_base_tf_ = false;
        ROS_INFO("[FAILSAFE MAP] Step3 depth-failover-only mode: healthy LiDAR/map tracking disabled; depth stream dormant");
      }
      else
      {
        depth_sub_ = nh_.subscribe(depth_topic_, 2, &FailsafeMapLocalizer::depthCallback, this);
        ROS_INFO("[FAILSAFE MAP] Localize mode: loaded %zu map points from %s",
                 map_->size(), map_path_.c_str());
      }
      publishLocalizationStatus("startup");
    }
  }

  ~FailsafeMapLocalizer()
  {
    if (mode_ == "build") saveMap();
  }

private:
  void downsampleInPlace(CloudT::Ptr cloud, double leaf)
  {
    if (!cloud || cloud->empty() || leaf <= 0.0) return;
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setInputCloud(cloud);
    CloudT filtered;
    vg.filter(filtered);
    cloud->swap(filtered);
  }

  CloudT::Ptr structuralFilter(const CloudT::Ptr &input)
  {
    if (!structural_filter_enable_ || !input || input->empty()) return input;
    CloudT::Ptr out(new CloudT);
    out->reserve(input->size());
    for (const auto &p : input->points)
    {
      Eigen::Vector3d pl(p.x, p.y, p.z);
      if (!pl.allFinite() || pl.norm() < structural_min_range_) continue;
      const Eigen::Vector3d pi = extR_ * pl + extT_;
      if (pi.z() < structural_min_body_z_ || pi.z() > structural_max_body_z_) continue;
      out->push_back(p);
    }
    out->width = out->size();
    out->height = 1;
    if (out->size() < static_cast<size_t>(std::max(50, structural_min_points_)))
      return input;
    return out;
  }

  void registeredCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    if (++build_frame_counter_ % std::max(1, map_accumulate_every_n_) != 0) return;
    CloudT cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) return;
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_->points.insert(map_->points.end(), cloud.points.begin(), cloud.points.end());
    map_->width = map_->points.size();
    map_->height = 1;
    map_dirty_ = true;
    if (++build_add_counter_ % std::max(1, map_downsample_every_n_) == 0)
      downsampleInPlace(map_, map_voxel_leaf_);
  }

  void saveTimerCallback(const ros::TimerEvent &) { saveMap(); }

  void saveMap()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!map_dirty_ || map_->empty()) return;
    downsampleInPlace(map_, map_voxel_leaf_);
    const int rc = pcl::io::savePCDFileBinaryCompressed(map_path_, *map_);
    if (rc == 0)
    {
      map_dirty_ = false;
      ROS_INFO("[FAILSAFE MAP] Saved %zu points -> %s", map_->size(), map_path_.c_str());
      publishMap();
    }
    else ROS_ERROR("[FAILSAFE MAP] Failed saving %s", map_path_.c_str());
  }

  void publishMap()
  {
    if (!map_ || map_->empty()) return;
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*map_, out);
    out.header.frame_id = map_frame_;
    out.header.stamp = ros::Time::now();
    map_pub_.publish(out);
  }

  void updateDegradedAnchorFromRelativePrediction(
      const Eigen::Matrix3d &Rwi,
      const Eigen::Vector3d &twi,
      const ros::Time &stamp)
  {
    if (!degraded_global_anchor_only_ || stamp.isZero()) return;

    std::string mode;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      mode = failsafe_mode_;
    }
    const bool imu_fail = mode == "LV_IMU_FAILED";
    const bool lidar_fail =
        mode == "VI_LIDAR_FAILED" || mode == "IMU_ONLY_BRIDGE";
    if (!imu_fail && !lidar_fail) return;

    Eigen::Matrix4f predicted_map_lidar;
    if (!predictFromVerifiedPlaceHistory(
            stamp, imu_fail, predicted_map_lidar))
      return;

    // Step-3 LiDAR-loss path: the predictor starts from the last trusted
    // global map pose at the failure boundary and is transported only by the
    // validated RGB-D relative SE(3) history.  Expose that causal map<-body
    // measurement to the single Step-3 map<-FAST anchor.  This is not another
    // odometry state and does not reset the ESIKF; FAST-LIVO2/IMU remains the
    // smooth local-motion authority while this measurement prevents its map
    // frame from drifting away from the lanelet during a LiDAR outage.
    if (lidar_fail && depth_failover_only_)
    {
      const Eigen::Matrix4f predicted_map_body =
          bodyFromMapLidar(predicted_map_lidar);
      Eigen::Quaternionf q(predicted_map_body.block<3, 3>(0, 0));
      if (q.norm() > 1.0e-6f && predicted_map_body.allFinite())
      {
        q.normalize();
        geometry_msgs::PoseStamped out;
        out.header.stamp = stamp;
        // Relative RGB-D propagation only.  Keep this distinguishable from a
        // geometrically verified saved-map observation so Step-3 cannot feed
        // odometry prediction back as absolute map authority.
        out.header.frame_id = "map_rgbd_predict";
        out.pose.position.x = predicted_map_body(0, 3);
        out.pose.position.y = predicted_map_body(1, 3);
        out.pose.position.z = predicted_map_body(2, 3);
        out.pose.orientation.x = q.x();
        out.pose.orientation.y = q.y();
        out.pose.orientation.z = q.z();
        out.pose.orientation.w = q.w();
        depth_failover_pose_pub_.publish(out);
        ROS_INFO_THROTTLE(
            0.5,
            "[STEP3 RGBD MAP] predicted map-body xyz=[%.2f %.2f %.2f]",
            predicted_map_body(0, 3), predicted_map_body(1, 3),
            predicted_map_body(2, 3));
      }
    }

    const Eigen::Matrix4f current_local_lidar = currentWorldLidar(Rwi, twi);
    const Eigen::Matrix4f candidate_anchor =
        predicted_map_lidar * current_local_lidar.inverse();
    queueAnchorTarget(candidate_anchor,
                      imu_fail ? "imu_fail_relative_map_prediction"
                               : "lidar_fail_relative_map_prediction");
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    const auto &p = msg->pose.pose.position;
    const auto &qmsg = msg->pose.pose.orientation;
    Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
    if (q.norm() < 1.0e-6) return;
    q.normalize();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_Rwi_ = q.toRotationMatrix();
      latest_twi_ << p.x, p.y, p.z;
      latest_odom_stamp_ = msg->header.stamp;
      latest_odom_valid_ = true;
    }
    if (!msg->header.stamp.isZero())
    {
      const Eigen::Matrix4f T_current_lidar =
          currentWorldLidar(q.toRotationMatrix(), Eigen::Vector3d(p.x, p.y, p.z));
      std::lock_guard<std::mutex> lock(odom_history_mutex_);
      odom_lidar_history_.emplace_back(msg->header.stamp, T_current_lidar);
      // Keep enough lightweight local-pose history to propagate delayed map
      // observations through long degraded intervals as well as healthy
      // re-acquisition. This is only a deque of 4x4 matrices (not point
      // clouds), so retaining 90 s is cheap and comfortably spans the
      // benchmark LiDAR-outage window.
      while (!odom_lidar_history_.empty() &&
             (msg->header.stamp - odom_lidar_history_.front().first).toSec() > 90.0)
        odom_lidar_history_.pop_front();
      while (odom_lidar_history_.size() > 1800)
        odom_lidar_history_.pop_front();
    }
    updateDegradedAnchorFromRelativePrediction(
        q.toRotationMatrix(), Eigen::Vector3d(p.x, p.y, p.z), msg->header.stamp);
    // Smooth the *vehicle pose* produced by map<-camera_init rather than
    // smoothing the parent transform around the camera_init origin.  Once the
    // cart is tens of metres from that origin, even a small anchor-yaw step can
    // sweep the public /pose through a multi-metre arc.  That was the source of
    // the saw-tooth/loop artefacts in otherwise accurate healthy map locks.
    Eigen::Matrix4f T_current_state = Eigen::Matrix4f::Identity();
    T_current_state.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
    T_current_state.block<3, 1>(0, 3) =
        Eigen::Vector3f(static_cast<float>(p.x), static_cast<float>(p.y),
                        static_cast<float>(p.z));
    applyPendingRecoveryAnchorRebase(T_current_state);
    advanceAnchorTowardTarget(msg->header.stamp, T_current_state);
    if (legacy_fallback_map_pose_enable_)
      syncAnchorToFallback(q.toRotationMatrix(), Eigen::Vector3d(p.x, p.y, p.z));
    publishGlobalPose(*msg);
  }

  void advanceAnchorTowardTarget(const ros::Time &stamp,
                                 const Eigen::Matrix4f &T_current_state)
  {
    std::lock_guard<std::mutex> lock(anchor_mutex_);
    if (!anchor_valid_ || !anchor_target_valid_) return;

    const ros::Time now = stamp.isZero() ? ros::Time::now() : stamp;
    if (anchor_target_last_step_stamp_.isZero())
    {
      anchor_target_last_step_stamp_ = now;
      return;
    }

    const double dt = (now - anchor_target_last_step_stamp_).toSec();
    anchor_target_last_step_stamp_ = now;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5) return;

    if (!T_current_state.allFinite()) return;

    // IMPORTANT: clamp the correction in map-frame VEHICLE-pose space.
    // Interpolating map<-camera_init directly is not equivalent: rotating that
    // parent transform about camera_init moves a vehicle 60-80 m away by metres
    // even for a 1-2 degree yaw step.  Compose both anchors with the current
    // FAST-LIVO2 state, move that public pose toward the target, then recover a
    // new rigid map<-camera_init anchor.  ESIKF local motion remains untouched.
    const Eigen::Matrix4f T_map_state_current =
        anchor_T_map_current_ * T_current_state;
    const Eigen::Matrix4f T_map_state_target =
        anchor_target_T_map_current_ * T_current_state;
    Eigen::Matrix4f T_map_state_next = T_map_state_current;

    Eigen::Vector3f d = T_map_state_target.block<3, 1>(0, 3) -
                        T_map_state_current.block<3, 1>(0, 3);
    const float dn = d.norm();
    const float max_t = static_cast<float>(
        std::max(0.0, anchor_target_translation_rate_active_) * dt);
    if (dn > 1.0e-7f)
    {
      if (max_t > 0.0f && dn > max_t) d *= max_t / dn;
      T_map_state_next.block<3, 1>(0, 3) += d;
    }

    Eigen::Quaternionf qa(T_map_state_current.block<3, 3>(0, 0));
    Eigen::Quaternionf qb(T_map_state_target.block<3, 3>(0, 0));
    qa.normalize();
    qb.normalize();
    const float angle = Eigen::AngleAxisf(qa.toRotationMatrix().transpose() *
                                           qb.toRotationMatrix()).angle();
    const float max_r = static_cast<float>(
        std::max(0.0, anchor_target_rotation_rate_active_deg_) * M_PI / 180.0 * dt);
    float alpha = 1.0f;
    if (angle > 1.0e-7f && max_r > 0.0f && angle > max_r)
      alpha = max_r / angle;
    T_map_state_next.block<3, 3>(0, 0) =
        qa.slerp(alpha, qb).normalized().toRotationMatrix();

    anchor_T_map_current_ = T_map_state_next * T_current_state.inverse();

    const float remaining_t =
        (T_map_state_target.block<3, 1>(0, 3) -
         T_map_state_next.block<3, 1>(0, 3)).norm();
    const float remaining_r = static_cast<float>(rotationAngleDeg(
        (T_map_state_next.block<3, 3>(0, 0).transpose() *
         T_map_state_target.block<3, 3>(0, 0)).cast<double>()));
    if (remaining_t < 0.02f && remaining_r < 0.20f)
      anchor_target_valid_ = false;
  }

  void applyPendingRecoveryAnchorRebase(const Eigen::Matrix4f &T_current_state)
  {
    std::lock_guard<std::mutex> lock(anchor_mutex_);
    if (!recovery_anchor_rebase_pending_) return;
    recovery_anchor_rebase_pending_ = false;
    if (!anchor_valid_ || !recovery_hold_map_state_.allFinite() ||
        !T_current_state.allFinite())
      return;

    // LiDAR/IMU recovery can legitimately change the internal ESIKF local
    // state when the recovered measurement stream is rejoined.  The saved-map
    // frame must not inherit that internal coordinate jump. Preserve the last
    // public map-frame vehicle pose and re-express map<-camera_init around the
    // recovered local state; later verified map tracking may then correct it
    // smoothly. This is a frame rebase only -- no estimator reset/correction.
    anchor_T_map_current_ = recovery_hold_map_state_ * T_current_state.inverse();
    anchor_target_valid_ = false;
    anchor_target_last_step_stamp_ = ros::Time();
    anchor_candidate_valid_ = false;
    anchor_consistency_count_ = 0;
    degraded_anchor_candidate_valid_ = false;
    ROS_WARN("[FAILSAFE MAP] recovery local-state frame changed; rebased map anchor to preserve /pose continuity");
  }

  bool applyIncrementalMapCorrection(const Eigen::Matrix4f &T_map_lidar_match,
                                     const Eigen::Matrix4f &T_current_lidar,
                                     const std::string &source,
                                     bool allow_large_verified = false)
  {
    if (!T_map_lidar_match.allFinite() || !T_current_lidar.allFinite()) return false;
    std::lock_guard<std::mutex> lock(anchor_mutex_);
    if (!anchor_valid_)
    {
      anchor_T_map_current_ = T_map_lidar_match * T_current_lidar.inverse();
      anchor_valid_ = true;
      anchor_target_valid_ = false;
      anchor_target_last_step_stamp_ = ros::Time();
      ROS_WARN("[ROLLING MAP] bootstrap anchor initialized from %s", source.c_str());
      return true;
    }

    const Eigen::Matrix4f T_map_lidar_pred = anchor_T_map_current_ * T_current_lidar;
    Eigen::Vector3f err = T_map_lidar_match.block<3, 1>(0, 3) -
                          T_map_lidar_pred.block<3, 1>(0, 3);
    const double trans_innov = err.norm();
    const Eigen::Matrix3f Rerr = T_map_lidar_match.block<3, 3>(0, 0) *
                                 T_map_lidar_pred.block<3, 3>(0, 0).transpose();
    const double yaw_innov = std::atan2(static_cast<double>(Rerr(1, 0)),
                                        static_cast<double>(Rerr(0, 0)));
    const double yaw_innov_deg = std::abs(yaw_innov) * 180.0 / M_PI;
    const double trans_gate = allow_large_verified
                                  ? std::max(5.0, rolling_map_max_translation_innovation_)
                                  : rolling_map_max_translation_innovation_;
    const double yaw_gate = allow_large_verified
                                ? std::max(20.0, rolling_map_max_yaw_innovation_deg_)
                                : rolling_map_max_yaw_innovation_deg_;
    if (trans_innov > trans_gate || yaw_innov_deg > yaw_gate)
    {
      ROS_WARN_THROTTLE(0.5,
          "[ROLLING MAP] reject %s innovation %.2fm/%.1fdeg",
          source.c_str(), trans_innov, yaw_innov_deg);
      return false;
    }

    Eigen::Vector3f step = static_cast<float>(rolling_map_translation_gain_) * err;
    step.z() = std::max(static_cast<float>(-rolling_map_max_z_step_),
                        std::min(static_cast<float>(rolling_map_max_z_step_), step.z()));
    const float sn = step.norm();
    if (sn > rolling_map_max_translation_step_ && sn > 1.0e-6f)
      step *= static_cast<float>(rolling_map_max_translation_step_ / sn);

    // Local healthy scan-to-map tracking is intentionally translation-only.
    // FAST-LIVO2's healthy ESIKF heading is the short-term orientation
    // authority; repeated LiDAR geometry can produce a plausible map match
    // with an 8-10 degree yaw ambiguity. Applying that yaw to map<-local at a
    // 60-80 m lever arm creates the large arcs/sways we are trying to remove.
    // A place-verified reacquisition is independent enough to contribute a
    // small bounded yaw update; ordinary rolling LiDAR tracking is not.
    const bool translation_only_healthy =
        source == "healthy_rolling_lidar_map";
    double yaw_step = 0.0;
    if (translation_only_healthy)
    {
      // Estimate map<-local heading from several metres of accepted position
      // motion.  This behaves like a low-rate loop-closure heading constraint
      // and does not trust the ambiguous orientation of one GICP solution.
      const Eigen::Vector2f map_xy = T_map_lidar_match.block<2, 1>(0, 3);
      const Eigen::Vector2f local_xy = T_current_lidar.block<2, 1>(0, 3);
      if (!rolling_heading_ref_valid_)
      {
        rolling_heading_ref_map_ = map_xy;
        rolling_heading_ref_local_ = local_xy;
        rolling_heading_ref_valid_ = true;
      }
      else
      {
        const Eigen::Vector2f dm = map_xy - rolling_heading_ref_map_;
        const Eigen::Vector2f dl = local_xy - rolling_heading_ref_local_;
        const double map_dist = dm.norm();
        const double local_dist = dl.norm();
        if (map_dist >= rolling_heading_baseline_m_ &&
            local_dist >= rolling_heading_baseline_m_)
        {
          const double ratio = map_dist / std::max(1.0e-6, local_dist);
          const double observed = wrapAngleRad(
              std::atan2(dm.y(), dm.x()) - std::atan2(dl.y(), dl.x()));
          const double current_anchor_yaw = std::atan2(
              anchor_T_map_current_(1, 0), anchor_T_map_current_(0, 0));
          const double raw_innov = wrapAngleRad(observed - current_anchor_yaw);
          const double max_innov = rolling_heading_max_innovation_deg_ * M_PI / 180.0;
          if (ratio >= rolling_heading_min_distance_ratio_ &&
              ratio <= rolling_heading_max_distance_ratio_ &&
              std::abs(raw_innov) <= max_innov)
          {
            if (!rolling_heading_filtered_valid_)
            {
              rolling_heading_filtered_yaw_ = observed;
              rolling_heading_filtered_valid_ = true;
            }
            else
            {
              rolling_heading_filtered_yaw_ = wrapAngleRad(
                  rolling_heading_filtered_yaw_ + rolling_heading_filter_gain_ *
                  wrapAngleRad(observed - rolling_heading_filtered_yaw_));
            }
            const double filt_innov = wrapAngleRad(
                rolling_heading_filtered_yaw_ - current_anchor_yaw);
            const double max_step = rolling_heading_max_step_deg_ * M_PI / 180.0;
            yaw_step = std::max(-max_step,
                std::min(max_step, rolling_heading_gain_ * filt_innov));
            ROS_INFO("[ROLLING HEADING] map=%.2fm local=%.2fm ratio=%.2f obs=%.2fdeg innov=%.2fdeg step=%.2fdeg",
                     map_dist, local_dist, ratio,
                     observed * 180.0 / M_PI,
                     filt_innov * 180.0 / M_PI,
                     yaw_step * 180.0 / M_PI);
          }
          else
          {
            ROS_WARN_THROTTLE(1.0,
                "[ROLLING HEADING] rejected ratio=%.2f innov=%.1fdeg",
                ratio, raw_innov * 180.0 / M_PI);
          }
          rolling_heading_ref_map_ = map_xy;
          rolling_heading_ref_local_ = local_xy;
        }
      }
    }
    else
    {
      const double yaw_step_lim = rolling_map_max_yaw_step_deg_ * M_PI / 180.0;
      yaw_step = std::max(-yaw_step_lim,
          std::min(yaw_step_lim, rolling_map_yaw_gain_ * yaw_innov));
    }
    Eigen::Matrix4f T_map_lidar_next = T_map_lidar_pred;
    T_map_lidar_next.block<3, 1>(0, 3) += step;
    const Eigen::Matrix3f Rz =
        Eigen::AngleAxisf(static_cast<float>(yaw_step), Eigen::Vector3f::UnitZ()).toRotationMatrix();
    T_map_lidar_next.block<3, 3>(0, 0) = Rz * T_map_lidar_pred.block<3, 3>(0, 0);

    anchor_T_map_current_ = T_map_lidar_next * T_current_lidar.inverse();
    anchor_target_valid_ = false;
    anchor_target_last_step_stamp_ = ros::Time();
    healthy_anchor_stamp_ = ros::Time::now();
    ROS_INFO_THROTTLE(0.25,
        "[ROLLING MAP] %s innov=%.2fm/%.1fdeg applied=%.2fm/%.2fdeg",
        source.c_str(), trans_innov, yaw_innov_deg, step.norm(),
        std::abs(yaw_step) * 180.0 / M_PI);
    return true;
  }

  void queueAnchorTarget(const Eigen::Matrix4f &candidate,
                         const std::string &source)
  {
    if (!candidate.allFinite()) return;
    std::lock_guard<std::mutex> lock(anchor_mutex_);

    // Degraded saved-map fixes arrive from repeated campus geometry. Even when
    // visual retrieval + GICP is locally strong, one false place can produce a
    // large global anchor target and a visible map-pose spike. Compare every
    // new target with the previous accepted degraded target (or current anchor
    // for the first one) and reject discontinuous target jumps. This does not
    // reject normal accumulated local drift because successive verified fixes
    // arrive at roughly 1 Hz and the anchor target evolves continuously.
    const Eigen::Matrix4f &reference =
        degraded_anchor_candidate_valid_ ? degraded_anchor_candidate_
                                         : anchor_T_map_current_;
    const Eigen::Matrix4f dd = reference.inverse() * candidate;
    const double dtrans = dd.block<3, 1>(0, 3).norm();
    const double drot =
        rotationAngleDeg(dd.block<3, 3>(0, 0).cast<double>());
    const bool healthy_verified = source == "healthy_place_verified_map";
    // A returning-LiDAR registration has already passed the saved-map GICP
    // fitness/correction/overlap checks before it reaches this function.  Its
    // anchor transform can legitimately be far from the degraded anchor: that
    // difference is accumulated outage drift in the persistent ESIKF local
    // frame, not a discontinuous global vehicle-pose proposal.  Treat this
    // exactly like a geometrically verified healthy fix in anchor-transform
    // space, while retaining all registration gates and the smooth target
    // application below.  This does not reset or directly correct the ESIKF.
    const bool recovery_verified = source == "recovery_saved_map_verified";
    const bool healthy_tracking = source == "healthy_saved_map_tracking";
    const bool healthy_target = healthy_verified || recovery_verified || healthy_tracking;
    const double target_translation_gate =
        healthy_tracking ? 4.0 : degraded_anchor_target_max_translation_jump_;
    const double target_rotation_gate =
        healthy_tracking ? 12.0 : degraded_anchor_target_max_rotation_jump_deg_;

    // A healthy place-verified target has already passed visual retrieval,
    // coarse+fine LiDAR GICP, seed-correction bounds, timestamp-matched local
    // pose composition, physical map-pose continuity and consecutive temporal
    // confirmation.  Do not re-apply a continuity gate in *anchor-transform*
    // space here: map<-camera_init is deliberately allowed to deform over the
    // route so the local FAST-LIVO2 ESIKF trajectory can remain registered to
    // the loop-closed saved map.  Around the campus bend the correct global
    // vehicle pose stays continuous while this parent transform legitimately
    // changes by much more than the degraded-mode jump gate.  Gating that
    // transform caused verified global fixes to be dropped and the map pose to
    // diverge after ~35 s.  Degraded and blind healthy-tracking targets retain
    // the strict jump gate below.
    const bool enforce_anchor_jump_gate = !(healthy_verified || recovery_verified);
    if (anchor_valid_ && enforce_anchor_jump_gate &&
        (dtrans > target_translation_gate || drot > target_rotation_gate))
    {
      ROS_WARN_THROTTLE(0.5,
          "[FAILSAFE MAP] rejected discontinuous map-anchor target from %s: %.2fm/%.1fdeg",
          source.c_str(), dtrans, drot);
      return;
    }
    if (!anchor_valid_)
    {
      anchor_T_map_current_ = candidate;
      anchor_valid_ = true;
      anchor_target_valid_ = false;
      anchor_target_last_step_stamp_ = ros::Time();
      ROS_WARN("[FAILSAFE MAP] map anchor initialized from %s", source.c_str());
      return;
    }
    anchor_target_T_map_current_ = candidate;
    anchor_target_valid_ = true;
    // Healthy map lock receives frequent small verified corrections, so give
    // it a lower correction bandwidth than sensor-failure recovery. This keeps
    // /pose continuous for planning while preserving the existing degraded
    // recovery rates and gates.
    anchor_target_translation_rate_active_ =
        healthy_target ? healthy_anchor_target_max_translation_rate_
                       : anchor_target_max_translation_rate_;
    anchor_target_rotation_rate_active_deg_ =
        healthy_target ? healthy_anchor_target_max_rotation_rate_deg_
                       : anchor_target_max_rotation_rate_deg_;
    degraded_anchor_candidate_ = candidate;
    degraded_anchor_candidate_valid_ = true;
    if (anchor_target_last_step_stamp_.isZero())
      anchor_target_last_step_stamp_ = ros::Time::now();
    ROS_INFO_THROTTLE(0.75,
        "[FAILSAFE MAP] queued smooth map-anchor target from %s", source.c_str());
  }

  void publishLocalizationStatus(const std::string &detail = std::string())
  {
    if (!localization_status_pub_) return;
    std::string mode;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      mode = failsafe_mode_;
    }
    bool anchored = false;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      anchored = anchor_valid_;
    }

    std_msgs::String out;
    if (!anchored)
      out.data = "WAITING_FOR_MAP_LOCK";
    else if (mode == "FULL_LIVO")
      out.data = "LOCALIZED mode=FULL_LIVO map_frame=" + map_frame_;
    else if (mode == "LIO_CAMERA_FAILED")
      out.data = "LOCALIZED_DEGRADED_CAMERA mode=LIO_CAMERA_FAILED map_frame=" + map_frame_;
    else if (mode == "LV_IMU_FAILED")
      out.data = "LOCALIZED_DEGRADED_IMU mode=LV_IMU_FAILED map_frame=" + map_frame_;
    else if (mode == "VI_LIDAR_FAILED")
      out.data = "LOCALIZED_DEGRADED_LIDAR mode=VI_LIDAR_FAILED map_frame=" + map_frame_;
    else if (mode == "IMU_ONLY_BRIDGE")
      out.data = "LOCALIZED_DEGRADED_LIDAR mode=IMU_ONLY_BRIDGE map_frame=" + map_frame_;
    else
      out.data = "LOCALIZATION_INVALID mode=" + mode;
    if (!detail.empty()) out.data += " detail=" + detail;
    localization_status_pub_.publish(out);
  }

  Eigen::Matrix4f bodyPose(const Eigen::Matrix3d &Rwi,
                           const Eigen::Vector3d &twi) const
  {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = Rwi.cast<float>();
    T.block<3, 1>(0, 3) = twi.cast<float>();
    return T;
  }

  Eigen::Matrix4f bodyFromMapLidar(const Eigen::Matrix4f &T_map_lidar) const
  {
    Eigen::Matrix4f T_body_lidar = Eigen::Matrix4f::Identity();
    T_body_lidar.block<3, 3>(0, 0) = extR_.cast<float>();
    T_body_lidar.block<3, 1>(0, 3) = extT_.cast<float>();
    return T_map_lidar * T_body_lidar.inverse();
  }

  static Eigen::Matrix3f rotationFromVector(const Eigen::Vector3f &rv)
  {
    const float angle = rv.norm();
    if (!std::isfinite(angle) || angle < 1.0e-8f)
      return Eigen::Matrix3f::Identity();
    return Eigen::AngleAxisf(angle, rv / angle).toRotationMatrix();
  }

  void syncAnchorToFallback(const Eigen::Matrix3d &Rwi,
                            const Eigen::Vector3d &twi)
  {
    Eigen::Matrix4f fallback;
    {
      std::lock_guard<std::mutex> lock(fallback_mutex_);
      if (!fallback_pose_valid_) return;
      fallback = fallback_T_map_body_;
    }

    const Eigen::Matrix4f anchor = fallback * bodyPose(Rwi, twi).inverse();
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      anchor_T_map_current_ = anchor;
      anchor_valid_ = true;
    }
  }

  void seedFallbackFromCurrent()
  {
    Eigen::Matrix3d Rwi;
    Eigen::Vector3d twi;
    std::string mode;
    ros::Time stamp;
    if (!snapshotState(Rwi, twi, mode, stamp)) return;

    Eigen::Matrix4f anchor = Eigen::Matrix4f::Identity();
    bool have_anchor = false;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      have_anchor = anchor_valid_;
      if (have_anchor) anchor = anchor_T_map_current_;
    }
    if (!have_anchor) return;

    {
      std::lock_guard<std::mutex> lock(fallback_mutex_);
      fallback_T_map_body_ = anchor * bodyPose(Rwi, twi);
      fallback_pose_valid_ = true;
      fallback_delta_source_.clear();
      fallback_last_delta_stamp_ = ros::Time();
      fallback_pending_translation_.setZero();
      fallback_pending_rotvec_.setZero();
      fallback_pending_valid_ = false;
      fallback_map_locked_ = false;
    }
    ROS_WARN("[FAILSAFE MAP] degraded map-frame fallback seeded from current FAST-LIVO2 pose");
  }

  void relativeDeltaCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
  {
    if (!msg) return;
    const bool rgbd = msg->header.frame_id == "rgbd_relative_delta";
    const bool lidar = msg->header.frame_id == "lidar_relative_delta";
    if (!rgbd && !lidar) return;

    // Keep short timestamped histories of the validated relative increments.
    // These are NOT second navigation trajectories. They are only used to
    // transport delayed global/map measurements to the timestamp of the live
    // degraded estimator using the same sensor that owns short-term motion:
    //   IMU fail   -> LiDAR relative SE(3)
    //   LiDAR fail -> RGB-D relative SE(3)
    if ((lidar || rgbd) && !msg->header.stamp.isZero())
    {
      const Eigen::Vector3f ht(static_cast<float>(msg->twist.linear.x),
                               static_cast<float>(msg->twist.linear.y),
                               static_cast<float>(msg->twist.linear.z));
      const Eigen::Vector3f hr(static_cast<float>(msg->twist.angular.x),
                               static_cast<float>(msg->twist.angular.y),
                               static_cast<float>(msg->twist.angular.z));
      if (ht.allFinite() && hr.allFinite() && ht.norm() < 2.5f && hr.norm() < 0.6f)
      {
        Eigen::Matrix4f D = Eigen::Matrix4f::Identity();
        D.block<3, 3>(0, 0) = rotationFromVector(hr);
        D.block<3, 1>(0, 3) = ht;
        std::lock_guard<std::mutex> hlock(relative_history_mutex_);
        auto &history = lidar ? lidar_delta_history_ : rgbd_delta_history_;
        // The RGB-D delta can arrive on both the shared ESIKF topic and the
        // dedicated map-history mirror. De-duplicate by sensor timestamp so a
        // single physical increment is never composed twice.
        if (!history.empty() &&
            std::abs((msg->header.stamp - history.back().first).toSec()) < 1.0e-6)
          return;
        history.emplace_back(msg->header.stamp, D);
        if (rgbd)
          ROS_INFO_THROTTLE(0.75,
              "[FAILSAFE MAP] RGB-D history ingest size=%zu stamp=%.3f",
              history.size(), msg->header.stamp.toSec());
        // Keep enough relative SE(3) history to transport the last trusted
        // global pose across long LiDAR outages, plus margin for delayed
        // visual/GICP corrections. A 30 s window was sufficient for the short
        // validation case but silently made global prediction impossible late
        // in a 40 s outage when the seed epoch fell out of the buffer.
        while (!history.empty() &&
               (msg->header.stamp - history.front().first).toSec() > 90.0)
          history.pop_front();
        while (history.size() > 500)
          history.pop_front();
      }
    }

    // v2: relative SE(3) belongs to the persistent FAST-LIVO2 ESIKF update
    // path. Do not integrate a second map-frame trajectory here.
    if (!legacy_fallback_map_pose_enable_) return;

    std::string mode;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      mode = failsafe_mode_;
    }
    const bool valid_for_mode =
        (rgbd && (mode == "VI_LIDAR_FAILED" || mode == "IMU_ONLY_BRIDGE")) ||
        (lidar && mode == "LV_IMU_FAILED");
    if (!valid_for_mode) return;

    const Eigen::Vector3f delta_t(static_cast<float>(msg->twist.linear.x),
                                  static_cast<float>(msg->twist.linear.y),
                                  static_cast<float>(msg->twist.linear.z));
    const Eigen::Vector3f delta_rv(static_cast<float>(msg->twist.angular.x),
                                   static_cast<float>(msg->twist.angular.y),
                                   static_cast<float>(msg->twist.angular.z));
    if (!delta_t.allFinite() || !delta_rv.allFinite() ||
        delta_t.norm() > 2.5f || delta_rv.norm() > 0.6f)
      return;

    Eigen::Matrix4f updated = Eigen::Matrix4f::Identity();
    {
      std::lock_guard<std::mutex> lock(fallback_mutex_);
      if (!fallback_pose_valid_) return;

      Eigen::Matrix4f D = Eigen::Matrix4f::Identity();
      D.block<3, 3>(0, 0) = rotationFromVector(delta_rv);
      D.block<3, 1>(0, 3) = delta_t;
      fallback_T_map_body_ = fallback_T_map_body_ * D;

      // Exact correction release used by the validated standalone fallback.
      if (fallback_pending_valid_)
      {
        const float n = fallback_pending_translation_.norm();
        if (n > 1.0e-6f)
        {
          const Eigen::Vector3f step =
              fallback_pending_translation_ * std::min(1.0f, 0.12f / n);
          fallback_T_map_body_.block<3, 1>(0, 3) += step;
          fallback_pending_translation_ -= step;
        }

        const float rn = fallback_pending_rotvec_.norm();
        if (rn > 1.0e-8f)
        {
          const float max_step = static_cast<float>(2.0 * M_PI / 180.0);
          const Eigen::Vector3f rstep =
              fallback_pending_rotvec_ * std::min(1.0f, max_step / rn);
          fallback_T_map_body_.block<3, 3>(0, 0) =
              fallback_T_map_body_.block<3, 3>(0, 0) * rotationFromVector(rstep);
          fallback_pending_rotvec_ -= rstep;
        }

        if (fallback_pending_translation_.norm() < 0.025f &&
            fallback_pending_rotvec_.norm() < static_cast<float>(0.20 * M_PI / 180.0))
          fallback_pending_valid_ = false;
      }

      fallback_last_delta_stamp_ = msg->header.stamp;
      fallback_delta_source_ = msg->header.frame_id;
      updated = fallback_T_map_body_;
    }

    Eigen::Matrix3d Rwi;
    Eigen::Vector3d twi;
    std::string current_mode;
    ros::Time stamp;
    if (snapshotState(Rwi, twi, current_mode, stamp))
      syncAnchorToFallback(Rwi, twi);

    ROS_INFO_THROTTLE(0.5,
        "[FAILSAFE MAP TRACK] %s pose=(%.2f %.2f %.2f)",
        msg->header.frame_id.c_str(), updated(0, 3), updated(1, 3), updated(2, 3));
  }

  void queueFallbackAbsolute(const Eigen::Matrix4f &T_map_body,
                             const std::string &source,
                             bool place_verified)
  {
    if (!legacy_fallback_map_pose_enable_) return;
    std::lock_guard<std::mutex> lock(fallback_mutex_);
    if (!fallback_pose_valid_) return;

    // Do not let repeated local geometry establish the global section of the
    // two-loop map.  Match the validated standalone order: visual place
    // retrieval first, geometric refinement second, local tracking afterwards.
    if (!fallback_map_locked_ && !place_verified)
      return;

    const Eigen::Vector3f delta_t =
        T_map_body.block<3, 1>(0, 3) - fallback_T_map_body_.block<3, 1>(0, 3);
    const Eigen::Matrix3f dR =
        fallback_T_map_body_.block<3, 3>(0, 0).transpose() *
        T_map_body.block<3, 3>(0, 0);
    Eigen::AngleAxisf aa(dR);
    Eigen::Vector3f rv = Eigen::Vector3f::Zero();
    if (std::isfinite(aa.angle()) && aa.axis().allFinite())
      rv = aa.axis() * aa.angle();
    const float rdeg = rv.norm() * 180.0f / static_cast<float>(M_PI);

    const float translation_gate =
        (!fallback_map_locked_ && place_verified) ? 8.0f : 5.0f;
    if (delta_t.norm() < translation_gate && rdeg < 25.0f)
    {
      fallback_pending_translation_ = delta_t;
      fallback_pending_rotvec_ = rv;
      fallback_pending_valid_ = true;
      if (place_verified)
        fallback_map_locked_ = true;
      ROS_INFO_THROTTLE(0.5,
          "[FAILSAFE MAP TRACK] queued %s residual %.2fm/%.1fdeg lock=%d",
          source.c_str(), delta_t.norm(), rdeg,
          static_cast<int>(fallback_map_locked_));
    }
  }

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    if (!msg) return;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_)
    {
      ROS_ERROR("[FAILSAFE MAP] Rejected initial pose in frame %s (expected %s)",
                msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    Eigen::Matrix3d R_current;
    Eigen::Vector3d t_current;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!latest_odom_valid_)
      {
        ROS_WARN("[FAILSAFE MAP] Initial pose ignored until FAST-LIVO2 has published odometry");
        return;
      }
      R_current = latest_Rwi_;
      t_current = latest_twi_;
    }

    const auto &p = msg->pose.pose.position;
    const auto &qm = msg->pose.pose.orientation;
    Eigen::Quaternionf q_map(static_cast<float>(qm.w), static_cast<float>(qm.x),
                             static_cast<float>(qm.y), static_cast<float>(qm.z));
    if (q_map.norm() < 1.0e-5f) return;
    q_map.normalize();

    Eigen::Matrix4f T_map_state = Eigen::Matrix4f::Identity();
    T_map_state.block<3, 3>(0, 0) = q_map.toRotationMatrix();
    T_map_state.block<3, 1>(0, 3) =
        Eigen::Vector3f(static_cast<float>(p.x), static_cast<float>(p.y),
                        static_cast<float>(p.z));
    Eigen::Matrix4f T_current_state = Eigen::Matrix4f::Identity();
    T_current_state.block<3, 3>(0, 0) = R_current.cast<float>();
    T_current_state.block<3, 1>(0, 3) = t_current.cast<float>();

    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      anchor_T_map_current_ = T_map_state * T_current_state.inverse();
      anchor_valid_ = true;
      anchor_target_valid_ = false;
      anchor_target_last_step_stamp_ = ros::Time();
      anchor_candidate_valid_ = false;
      anchor_consistency_count_ = 0;
      healthy_anchor_stamp_ = ros::Time::now();
    }
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      tracker_valid_ = false;
      tracker_last_source_.reset();
      tracker_source_name_.clear();
    }
    ROS_WARN("[FAILSAFE MAP] New map<-FAST-LIVO2 startup anchor accepted from %s; LiDAR geometry will refine it",
             initial_pose_topic_.c_str());
    publishLocalizationStatus("initial_pose");
  }

  void step3GlobalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (!msg) return;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_)
      return;
    const auto &p = msg->pose.position;
    const auto &qm = msg->pose.orientation;
    Eigen::Quaternionf q(static_cast<float>(qm.w), static_cast<float>(qm.x),
                         static_cast<float>(qm.y), static_cast<float>(qm.z));
    if (q.norm() < 1.0e-5f) return;
    q.normalize();
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T.block<3, 1>(0, 3) =
        Eigen::Vector3f(static_cast<float>(p.x), static_cast<float>(p.y),
                        static_cast<float>(p.z));
    if (!T.allFinite()) return;
    std::lock_guard<std::mutex> lock(step3_global_pose_mutex_);
    latest_step3_T_map_body_ = T;
    latest_step3_global_pose_stamp_ = msg->header.stamp.isZero()
                                          ? ros::Time::now()
                                          : msg->header.stamp;
    latest_step3_global_pose_valid_ = true;
  }

  void publishGlobalPose(const nav_msgs::Odometry &local_odom)
  {
    Eigen::Matrix4f anchor;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      if (!anchor_valid_) return;
      anchor = anchor_T_map_current_;
    }

    const auto &p = local_odom.pose.pose.position;
    const auto &qm = local_odom.pose.pose.orientation;
    Eigen::Quaternionf q_current(static_cast<float>(qm.w), static_cast<float>(qm.x),
                                 static_cast<float>(qm.y), static_cast<float>(qm.z));
    if (q_current.norm() < 1.0e-5f) return;
    q_current.normalize();
    Eigen::Matrix4f T_current_state = Eigen::Matrix4f::Identity();
    T_current_state.block<3, 3>(0, 0) = q_current.toRotationMatrix();
    T_current_state.block<3, 1>(0, 3) =
        Eigen::Vector3f(static_cast<float>(p.x), static_cast<float>(p.y),
                        static_cast<float>(p.z));
    const Eigen::Matrix4f T_map_state = anchor * T_current_state;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      last_global_T_map_state_ = T_map_state;
      last_global_pose_valid_ = true;
    }
    Eigen::Quaternionf q_map(T_map_state.block<3, 3>(0, 0));
    q_map.normalize();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = local_odom.header.stamp.isZero() ? ros::Time::now()
                                                         : local_odom.header.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = T_map_state(0, 3);
    pose.pose.position.y = T_map_state(1, 3);
    pose.pose.position.z = T_map_state(2, 3);
    pose.pose.orientation.x = q_map.x();
    pose.pose.orientation.y = q_map.y();
    pose.pose.orientation.z = q_map.z();
    pose.pose.orientation.w = q_map.w();
    global_pose_pub_.publish(pose);

    if (publish_legacy_outputs_)
      legacy_pose_pub_.publish(pose);

    nav_msgs::Odometry odom = local_odom;
    odom.header = pose.header;
    odom.pose.pose = pose.pose;
    global_odom_pub_.publish(odom);

    if (publish_legacy_outputs_)
    {
      nav_msgs::Odometry legacy_odom = odom;
      legacy_odom.child_frame_id = base_frame_;
      legacy_odom_pub_.publish(legacy_odom);
    }

    // Publish map -> FAST-LIVO2 local frame.  FAST-LIVO2 itself already owns
    // camera_init -> aft_mapped, so this composes cleanly with the native TF
    // instead of creating a second competing map -> vehicle transform.
    const std::string local_frame = local_odom.header.frame_id.empty()
                                      ? "camera_init" : local_odom.header.frame_id;
    Eigen::Quaternionf q_anchor(anchor.block<3, 3>(0, 0));
    q_anchor.normalize();
    tf::Transform map_to_local;
    map_to_local.setOrigin(tf::Vector3(anchor(0, 3), anchor(1, 3), anchor(2, 3)));
    map_to_local.setRotation(tf::Quaternion(q_anchor.x(), q_anchor.y(),
                                            q_anchor.z(), q_anchor.w()));
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(map_to_local, pose.header.stamp, map_frame_, local_frame));

    // ALiVE's planning/perception stack is base_link-centric. Publish the same
    // authoritative map-frame vehicle pose directly as map->base_link. This is
    // a separate branch from FAST-LIVO2's native camera_init->aft_mapped TF, so
    // it introduces no TF cycle and lets Lanelet/top-level visualize the car in
    // the exact saved-LiDAR-map frame.
    if (publish_map_to_base_tf_)
    {
      tf::Transform map_to_base;
      map_to_base.setOrigin(tf::Vector3(T_map_state(0, 3), T_map_state(1, 3),
                                       T_map_state(2, 3)));
      map_to_base.setRotation(tf::Quaternion(q_map.x(), q_map.y(), q_map.z(), q_map.w()));
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(map_to_base, pose.header.stamp, map_frame_, base_frame_));
    }

    std::lock_guard<std::mutex> lock(global_path_mutex_);
    if (last_global_path_stamp_.isZero() ||
        (pose.header.stamp - last_global_path_stamp_).toSec() >= 0.10)
    {
      global_path_.header = pose.header;
      global_path_.poses.push_back(pose);
      if (global_path_.poses.size() > 10000)
        global_path_.poses.erase(global_path_.poses.begin(),
                                 global_path_.poses.begin() + 1000);
      global_path_pub_.publish(global_path_);
      last_global_path_stamp_ = pose.header.stamp;
    }
  }

  static bool sameFailsafeFamily(const std::string &a, const std::string &b)
  {
    if (a == b) return true;
    // Camera loss leaves the LiDAR+IMU motion source and saved-map tracker
    // unchanged. Treat FULL_LIVO <-> LIO_CAMERA_FAILED as one continuity
    // family so a camera outage cannot throw away a valid global LiDAR seed.
    const bool a_lidar_imu_family =
        a == "FULL_LIVO" || a == "LIO_CAMERA_FAILED";
    const bool b_lidar_imu_family =
        b == "FULL_LIVO" || b == "LIO_CAMERA_FAILED";
    if (a_lidar_imu_family && b_lidar_imu_family) return true;
    const bool a_lidar_fail =
        a == "VI_LIDAR_FAILED" || a == "IMU_ONLY_BRIDGE";
    const bool b_lidar_fail =
        b == "VI_LIDAR_FAILED" || b == "IMU_ONLY_BRIDGE";
    return a_lidar_fail && b_lidar_fail;
  }

  void modeCallback(const std_msgs::String::ConstPtr &msg)
  {
    std::string previous;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      previous = failsafe_mode_;
      const ros::Time transition_now = ros::Time::now();
      if (previous != msg->data || current_mode_enter_stamp_.isZero())
        current_mode_enter_stamp_ = transition_now;
      // Keep a separate causal epoch for the tracking family.  VI_LIDAR_FAILED
      // <-> IMU_ONLY_BRIDGE is only an internal health/source oscillation during
      // the same LiDAR outage; advancing the predictor epoch there discards
      // valid RGB-D motion accumulated since the actual failure boundary.
      if (tracking_family_enter_stamp_.isZero() ||
          !sameFailsafeFamily(previous, msg->data))
        tracking_family_enter_stamp_ = transition_now;
      failsafe_mode_ = msg->data;
    }
    if (depth_failover_only_)
    {
      const bool need_depth =
          msg->data == "VI_LIDAR_FAILED" || msg->data == "IMU_ONLY_BRIDGE";
      if (need_depth && !depth_failover_stream_active_)
      {
        depth_sub_ = nh_.subscribe(depth_topic_, 1,
                                   &FailsafeMapLocalizer::depthCallback, this);
        depth_failover_stream_active_ = true;
        ROS_WARN("[STEP3 DEPTH MAP] subscribed to depth stream for LiDAR outage");
      }
      else if (!need_depth && depth_failover_stream_active_)
      {
        depth_sub_.shutdown();
        depth_failover_stream_active_ = false;
        {
          std::lock_guard<std::mutex> lock(depth_history_mutex_);
          depth_cloud_history_.clear();
        }
        ROS_INFO("[STEP3 DEPTH MAP] depth stream dormant");
      }
    }
    const bool same_tracking_family = sameFailsafeFamily(previous, msg->data);
    if (previous != msg->data && !same_tracking_family &&
        (previous == "FULL_LIVO" || msg->data == "FULL_LIVO"))
    {
      {
        std::lock_guard<std::mutex> lock(tracker_mutex_);
        tracker_valid_ = false;
        tracker_last_source_.reset();
        tracker_source_name_.clear();
        last_tracker_accept_stamp_ = ros::Time();
      }
      {
        std::lock_guard<std::mutex> lock(place_confirm_mutex_);
        place_confirm_valid_ = false;
        place_confirm_streak_ = 0;
        place_confirm_source_.clear();
        place_confirm_stamp_ = ros::Time();
      }
      {
        std::lock_guard<std::mutex> lock(depth_relock_mutex_);
        depth_relock_valid_ = false;
        depth_relock_streak_ = 0;
        depth_relock_stamp_ = ros::Time();
        depth_relock_velocity_valid_ = false;
      }
    }
    const bool same_failure_family = same_tracking_family;
    if (previous != msg->data && !same_failure_family)
    {
      {
        std::lock_guard<std::mutex> lock(relative_mutex_);
        lidar_rel_filter_valid_ = false;
        lidar_filt_trans_vel_.setZero();
        lidar_filt_ang_vel_.setZero();
      }
      resetImuPlaceHistory();
      {
        std::lock_guard<std::mutex> hlock(relative_history_mutex_);
        lidar_delta_history_.clear();
        rgbd_delta_history_.clear();
      }
      {
        std::lock_guard<std::mutex> vlock(verified_place_history_mutex_);
        verified_place_history_valid_ = false;
        verified_place_history_pose_ = Eigen::Matrix4f::Identity();
        verified_place_history_stamp_ = ros::Time();
        verified_place_history_mode_.clear();
      }
      {
        std::lock_guard<std::mutex> alock(anchor_mutex_);
        degraded_anchor_candidate_valid_ = false;
        degraded_anchor_candidate_ = Eigen::Matrix4f::Identity();
      }
    }

    const bool entering_degraded =
        previous == "FULL_LIVO" &&
        (msg->data == "VI_LIDAR_FAILED" ||
         msg->data == "IMU_ONLY_BRIDGE" ||
         msg->data == "LV_IMU_FAILED");
    if (entering_degraded && legacy_fallback_map_pose_enable_)
      seedFallbackFromCurrent();

    if (entering_degraded)
    {
      {
        std::lock_guard<std::mutex> lock(anchor_mutex_);
        anchor_target_valid_ = false;
        anchor_target_last_step_stamp_ = ros::Time();
        anchor_candidate_valid_ = false;
        anchor_consistency_count_ = 0;
        degraded_anchor_candidate_valid_ = false;
      }

      // Seed the degraded global predictor from the last trustworthy healthy
      // map pose. Relative RGB-D/LiDAR measurements then transport this pose
      // causally between absolute saved-map corrections; no second estimator
      // or new origin is created.
      Eigen::Matrix3d seed_Rwi;
      Eigen::Vector3d seed_twi;
      std::string seed_mode;
      ros::Time seed_stamp;
      if (snapshotState(seed_Rwi, seed_twi, seed_mode, seed_stamp))
      {
        Eigen::Matrix4f anchor = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f last_global = Eigen::Matrix4f::Identity();
        bool have_anchor = false;
        bool have_last_global = false;
        {
          std::lock_guard<std::mutex> lock(anchor_mutex_);
          have_anchor = anchor_valid_;
          if (have_anchor) anchor = anchor_T_map_current_;
          have_last_global = last_global_pose_valid_;
          if (have_last_global) last_global = last_global_T_map_state_;
        }
        Eigen::Matrix4f step3_map_body = Eigen::Matrix4f::Identity();
        ros::Time step3_pose_stamp;
        bool have_fresh_step3_pose = false;
        {
          std::lock_guard<std::mutex> slock(step3_global_pose_mutex_);
          if (latest_step3_global_pose_valid_ &&
              !latest_step3_global_pose_stamp_.isZero())
          {
            const double age = std::abs(
                (tracking_family_enter_stamp_ - latest_step3_global_pose_stamp_).toSec());
            if (std::isfinite(age) && age <= std::max(0.1, step3_global_pose_max_age_))
            {
              step3_map_body = latest_step3_T_map_body_;
              step3_pose_stamp = latest_step3_global_pose_stamp_;
              have_fresh_step3_pose = true;
            }
          }
        }

        if (have_anchor || have_fresh_step3_pose)
        {
          // snapshotState() can lag the mode transition by roughly one second
          // under benchmark load.  Seeding relative-history integration at
          // that old stamp makes the first fresh RGB-D delta look like a
          // >max_gap discontinuity, after which every prediction is rejected.
          // The already-published global pose is the freshest authoritative
          // saved-map state at the transition, so use it (and transition time)
          // when available.  This changes only the degraded predictor seed;
          // it does not reset or correct the persistent ESIKF state.
          // last_global_T_map_state_ is map<-ESIKF-state (IMU/body), while
          // verified_place_history_pose_ and the relative-history propagator
          // are explicitly map<-LiDAR.  Feeding the state pose directly into
          // that history mixed frames; on this cart the LiDAR extrinsic tilt
          // made the very first degraded target differ by ~22 deg and it was
          // correctly rejected by the discontinuity gate.  Convert the fresh
          // published state pose to the same LiDAR frame before seeding.
          Eigen::Matrix4f T_state_lidar = Eigen::Matrix4f::Identity();
          T_state_lidar.block<3, 3>(0, 0) = extR_.cast<float>();
          T_state_lidar.block<3, 1>(0, 3) = extT_.cast<float>();
          // Step-3 is the global authority while healthy.  The failover-only
          // localizer is deliberately dormant then, so its own last_global can
          // be many metres stale even though Step-3 is perfectly localized.
          // Seed LiDAR-loss RGB-D propagation from the actual latest Step-3
          // map<-body pose, not from that dormant anchor.
          const Eigen::Matrix4f seed_map_lidar = have_fresh_step3_pose
              ? step3_map_body * T_state_lidar
              : (have_last_global
                     ? last_global * T_state_lidar
                     : anchor * currentWorldLidar(seed_Rwi, seed_twi));
          const ros::Time predictor_seed_stamp = have_fresh_step3_pose
              ? step3_pose_stamp
              : (have_last_global ? tracking_family_enter_stamp_ : seed_stamp);

          if (have_fresh_step3_pose)
          {
            // Keep the dormant failover anchor coherent too.  This does not
            // touch FAST-LIVO2; it only ensures any failure-only helper uses
            // the same map frame as Step-3 from this boundary onward.
            const Eigen::Matrix4f T_current_body = bodyPose(seed_Rwi, seed_twi);
            std::lock_guard<std::mutex> alock(anchor_mutex_);
            anchor_T_map_current_ = step3_map_body * T_current_body.inverse();
            anchor_valid_ = true;
            last_global_T_map_state_ = step3_map_body;
            last_global_pose_valid_ = true;
          }
          // This is the mode-transition barrier itself, not an asynchronous
          // place-verification result.  Install the new causal epoch directly
          // while holding the history mutex so an in-flight healthy callback
          // cannot leave the degraded predictor on its pre-failure epoch.
          {
            std::lock_guard<std::mutex> vlock(verified_place_history_mutex_);
            verified_place_history_pose_ = seed_map_lidar;
            verified_place_history_stamp_ = predictor_seed_stamp;
            verified_place_history_mode_ = msg->data;
            verified_place_history_valid_ = !predictor_seed_stamp.isZero();
          }
          // LiDAR-loss absolute tracking does not need a fresh visual-place
          // retrieval when we just left a trusted Step-3 map lock. Seed the
          // depth->visual-voxel tracker from that exact global LiDAR pose.
          // Subsequent RGB-D motion predicts locally and depth GICP only
          // refines within the nearby saved visual map. A global place matcher
          // remains a reacquisition option, not a prerequisite for each fix.
          if (msg->data == "VI_LIDAR_FAILED" || msg->data == "IMU_ONLY_BRIDGE")
          {
            const Eigen::Matrix4f T_current_lidar_seed =
                currentWorldLidar(seed_Rwi, seed_twi);
            std::lock_guard<std::mutex> tlock(tracker_mutex_);
            tracker_T_map_lidar_ = seed_map_lidar;
            tracker_last_current_lidar_ = T_current_lidar_seed;
            tracker_last_source_.reset();
            tracker_source_name_ = "depth_saved_map_icp";
            tracker_valid_ = true;
            last_tracker_accept_stamp_ = predictor_seed_stamp;
            ROS_WARN("[STEP3 DEPTH MAP] seeded local visual-voxel tracker from trusted Step-3 pose");
          }
          if (msg->data == "LV_IMU_FAILED")
            acceptImuPlaceHistory(seed_map_lidar, predictor_seed_stamp);
          ROS_WARN("[FAILSAFE MAP] seeded degraded global predictor atomically at %.3f source=%s",
                   predictor_seed_stamp.toSec(),
                   have_fresh_step3_pose ? "step3_global_pose" : "internal_anchor");
        }
      }
    }

    if (msg->data == "FULL_LIVO" && previous != "FULL_LIVO")
    {
      // Hold the current global alignment while recovered sensors settle.  A
      // first noisy scan must not move map<-camera_init immediately after the
      // ESIKF has already been corrected/rejoined.
      anchor_refinement_resume_stamp_ =
          ros::Time::now() + ros::Duration(std::max(0.0, anchor_refinement_freeze_sec_));
      {
        std::lock_guard<std::mutex> lock(anchor_mutex_);
        const bool local_state_may_rebase =
            previous == "VI_LIDAR_FAILED" ||
            previous == "IMU_ONLY_BRIDGE" ||
            previous == "LV_IMU_FAILED";
        if (local_state_may_rebase && last_global_pose_valid_)
        {
          recovery_hold_map_state_ = last_global_T_map_state_;
          recovery_anchor_rebase_pending_ = true;
        }
        anchor_target_valid_ = false;
        anchor_target_last_step_stamp_ = ros::Time();
        anchor_candidate_valid_ = false;
        anchor_consistency_count_ = 0;
      }
      if (legacy_fallback_map_pose_enable_)
      {
        std::lock_guard<std::mutex> lock(fallback_mutex_);
        fallback_pose_valid_ = false;
        fallback_pending_valid_ = false;
        fallback_map_locked_ = false;
        fallback_delta_source_.clear();
      }
    }
    publishLocalizationStatus(previous + "->" + msg->data);
  }

  void placeCandidatesCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
  {
    if (!msg || msg->poses.empty()) return;
    std::vector<Eigen::Matrix4f> candidates;
    candidates.reserve(std::min<size_t>(4, msg->poses.size()));
    for (size_t i = 0; i < msg->poses.size() && i < 4; ++i)
    {
      const auto &p = msg->poses[i];
      Eigen::Quaternionf q(
          static_cast<float>(p.orientation.w),
          static_cast<float>(p.orientation.x),
          static_cast<float>(p.orientation.y),
          static_cast<float>(p.orientation.z));
      if (q.norm() < 1.0e-5f) continue;
      q.normalize();
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block<3, 3>(0, 0) = q.toRotationMatrix();
      T.block<3, 1>(0, 3) =
          Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
      candidates.push_back(T);
    }
    if (candidates.empty()) return;
    {
      std::lock_guard<std::mutex> lock(place_mutex_);
      place_candidates_ = candidates;
      place_candidates_stamp_ =
          msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
      place_candidates_metric_rgbd_ =
          msg->header.frame_id == "saved_map_lidar_rgbd_metric_candidates";
    }
    ROS_INFO_THROTTLE(0.5,
        "[FAILSAFE MAP] received %zu visual place candidates",
        candidates.size());
  }

  bool snapshotPlaceCandidates(std::vector<Eigen::Matrix4f> &out,
                               ros::Time *stamp_out = nullptr,
                               bool *metric_rgbd_out = nullptr,
                               double max_age_override = -1.0)
  {
    std::lock_guard<std::mutex> lock(place_mutex_);
    if (place_candidates_.empty() || place_candidates_stamp_.isZero())
      return false;
    const double age = std::abs((ros::Time::now() - place_candidates_stamp_).toSec());
    const bool verified_metric_lidar_fail =
        place_candidates_metric_rgbd_ &&
        (failsafe_mode_ == "VI_LIDAR_FAILED" ||
         failsafe_mode_ == "IMU_ONLY_BRIDGE");
    const double max_age = max_age_override > 0.0 ? max_age_override :
        (verified_metric_lidar_fail ?
         std::max(place_hint_max_age_, place_metric_lidar_fail_max_age_) :
         place_hint_max_age_);
    if (age > max_age)
    {
      ROS_INFO_THROTTLE(0.5,
          "[FAILSAFE MAP BOOT] place snapshot stale age=%.3fs max=%.3fs",
          age, max_age);
      return false;
    }
    out = place_candidates_;
    if (stamp_out) *stamp_out = place_candidates_stamp_;
    if (metric_rgbd_out) *metric_rgbd_out = place_candidates_metric_rgbd_;
    return !out.empty();
  }

  bool propagateMapLidarWithHistory(
      const Eigen::Matrix4f &T_map_lidar_query,
      const ros::Time &query_stamp,
      const ros::Time &target_stamp,
      bool use_lidar_history,
      Eigen::Matrix4f &T_map_lidar_target)
  {
    if (query_stamp.isZero() || target_stamp.isZero() || target_stamp < query_stamp)
      return false;
    if ((target_stamp - query_stamp).toSec() < 0.03)
    {
      T_map_lidar_target = T_map_lidar_query;
      return true;
    }

    std::vector<std::pair<ros::Time, Eigen::Matrix4f>> deltas;
    {
      std::lock_guard<std::mutex> lock(relative_history_mutex_);
      const auto &history =
          use_lidar_history ? lidar_delta_history_ : rgbd_delta_history_;
      for (const auto &entry : history)
        if (entry.first > query_stamp && entry.first <= target_stamp)
          deltas.push_back(entry);
    }
    const size_t min_deltas = use_lidar_history ? 3u : 2u;
    if (deltas.size() < min_deltas)
    {
      if (!use_lidar_history &&
          (failsafe_mode_ == "VI_LIDAR_FAILED" || failsafe_mode_ == "IMU_ONLY_BRIDGE"))
        ROS_WARN_THROTTLE(0.75,
            "[FAILSAFE MAP] RGB-D global predictor waiting: deltas=%zu need=%zu query=%.3f target=%.3f",
            deltas.size(), min_deltas, query_stamp.toSec(), target_stamp.toSec());
      return false;
    }
    const double head_gap = (deltas.front().first - query_stamp).toSec();
    const double tail_gap = (target_stamp - deltas.back().first).toSec();
    const double max_gap = use_lidar_history ? 0.35 : 1.20;
    if (head_gap > max_gap || tail_gap > max_gap)
    {
      if (!use_lidar_history &&
          (failsafe_mode_ == "VI_LIDAR_FAILED" || failsafe_mode_ == "IMU_ONLY_BRIDGE"))
        ROS_WARN_THROTTLE(0.75,
            "[FAILSAFE MAP] RGB-D global predictor gap reject: deltas=%zu head=%.3f tail=%.3f max=%.3f query=%.3f target=%.3f first=%.3f last=%.3f",
            deltas.size(), head_gap, tail_gap, max_gap, query_stamp.toSec(),
            target_stamp.toSec(), deltas.front().first.toSec(),
            deltas.back().first.toSec());
      return false;
    }

    Eigen::Matrix4f T_body_lidar = Eigen::Matrix4f::Identity();
    T_body_lidar.block<3, 3>(0, 0) = extR_.cast<float>();
    T_body_lidar.block<3, 1>(0, 3) = extT_.cast<float>();
    Eigen::Matrix4f T_map_body = T_map_lidar_query * T_body_lidar.inverse();
    for (const auto &entry : deltas)
      T_map_body = T_map_body * entry.second;
    T_map_lidar_target = T_map_body * T_body_lidar;

    ROS_INFO_THROTTLE(0.75,
        "[FAILSAFE MAP] propagated delayed map pose %.2fs with %zu %s deltas",
        (target_stamp - query_stamp).toSec(), deltas.size(),
        use_lidar_history ? "LiDAR" : "RGB-D");
    return true;
  }

  bool propagatePlaceCandidateWithLidarHistory(
      const Eigen::Matrix4f &T_map_lidar_query,
      const ros::Time &query_stamp,
      const ros::Time &target_stamp,
      Eigen::Matrix4f &T_map_lidar_target)
  {
    return propagateMapLidarWithHistory(
        T_map_lidar_query, query_stamp, target_stamp, true,
        T_map_lidar_target);
  }

  bool propagatePlaceCandidateWithRgbdHistory(
      const Eigen::Matrix4f &T_map_lidar_query,
      const ros::Time &query_stamp,
      const ros::Time &target_stamp,
      Eigen::Matrix4f &T_map_lidar_target)
  {
    return propagateMapLidarWithHistory(
        T_map_lidar_query, query_stamp, target_stamp, false,
        T_map_lidar_target);
  }

  bool propagatePlaceCandidateWithOdomHistory(
      const Eigen::Matrix4f &T_map_lidar_query,
      const ros::Time &query_stamp,
      const ros::Time &target_stamp,
      Eigen::Matrix4f &T_map_lidar_target)
  {
    if (query_stamp.isZero() || target_stamp.isZero() || target_stamp < query_stamp)
      return false;
    if ((target_stamp - query_stamp).toSec() < 0.03)
    {
      T_map_lidar_target = T_map_lidar_query;
      return true;
    }

    Eigen::Matrix4f T_query = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_target = Eigen::Matrix4f::Identity();
    double best_q = 1e9, best_t = 1e9;
    {
      std::lock_guard<std::mutex> lock(odom_history_mutex_);
      for (const auto &entry : odom_lidar_history_)
      {
        const double dq = std::abs((entry.first - query_stamp).toSec());
        if (dq < best_q) { best_q = dq; T_query = entry.second; }
        const double dt = std::abs((entry.first - target_stamp).toSec());
        if (dt < best_t) { best_t = dt; T_target = entry.second; }
      }
    }
    if (best_q > 0.25 || best_t > 0.25) return false;
    const Eigen::Matrix4f D = T_query.inverse() * T_target;
    const double dtrans = D.block<3, 1>(0, 3).norm();
    const double drot = rotationAngleDeg(D.block<3, 3>(0, 0).cast<double>());
    if (!std::isfinite(dtrans) || !std::isfinite(drot) ||
        dtrans > 16.0 || drot > 75.0)
      return false;
    T_map_lidar_target = T_map_lidar_query * D;
    ROS_INFO_THROTTLE(0.75,
        "[FAILSAFE MAP] propagated healthy visual place %.2fs with odometry %.2fm/%.1fdeg",
        (target_stamp - query_stamp).toSec(), dtrans, drot);
    return true;
  }

  bool predictFromVerifiedPlaceHistory(const ros::Time &target_stamp,
                                       bool use_lidar_history,
                                       Eigen::Matrix4f &predicted)
  {
    Eigen::Matrix4f last = Eigen::Matrix4f::Identity();
    ros::Time last_stamp;
    std::string last_mode;
    {
      std::lock_guard<std::mutex> lock(verified_place_history_mutex_);
      if (!verified_place_history_valid_ || verified_place_history_stamp_.isZero())
        return false;
      last = verified_place_history_pose_;
      last_stamp = verified_place_history_stamp_;
      last_mode = verified_place_history_mode_;
    }
    // A mode transition is an integration barrier.  An odometry callback can
    // race the failure-mode callback: it may snapshot the old healthy epoch,
    // then call this function after the degraded epoch has been installed.
    // Conversely, an in-flight request must never splice relative motion from
    // before the current degraded epoch onto its authoritative transition
    // pose.  Clamp only the request time to that epoch; the pose and history
    // remain the same persistent saved-map authority and the normal max-gap
    // checks below are intentionally unchanged.
    ros::Time mode_enter;
    std::string current_mode;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      mode_enter = tracking_family_enter_stamp_;
      current_mode = failsafe_mode_;
    }
    const bool degraded =
        current_mode == "VI_LIDAR_FAILED" ||
        current_mode == "IMU_ONLY_BRIDGE" ||
        current_mode == "LV_IMU_FAILED";
    if (degraded && !mode_enter.isZero() && last_stamp < mode_enter)
    {
      ROS_WARN_THROTTLE(0.75,
          "[FAILSAFE MAP] clamping stale verified predictor epoch %.3f -> %.3f at degraded transition",
          last_stamp.toSec(), mode_enter.toSec());
      last_stamp = mode_enter;
    }
    if (!sameFailsafeFamily(last_mode, failsafe_mode_))
    {
      ROS_WARN_THROTTLE(0.75,
          "[FAILSAFE MAP] verified predictor mode-family mismatch: last=%s current=%s",
          last_mode.c_str(), failsafe_mode_.c_str());
      return false;
    }
    // During LiDAR loss, RGB-D remains the actual relative measurement source
    // feeding the persistent ESIKF, but its image-processing timestamps can
    // trail wall/ROS processing time by ~1-2 s.  For *timestamp transport* of
    // an already verified map pose, prefer the resulting current ESIKF local
    // odometry history.  This avoids extrapolating stale RGB-D deltas while
    // keeping a single estimator/state.  Fall back to raw RGB-D history if the
    // local odometry history is unavailable at either endpoint.
    if (!use_lidar_history &&
        (current_mode == "VI_LIDAR_FAILED" ||
         current_mode == "IMU_ONLY_BRIDGE"))
    {
      if (propagatePlaceCandidateWithOdomHistory(
              last, last_stamp, target_stamp, predicted))
        return true;
    }
    return propagateMapLidarWithHistory(
        last, last_stamp, target_stamp, use_lidar_history, predicted);
  }

  bool predictHealthyFromVerifiedPlaceHistory(
      const ros::Time &target_stamp, Eigen::Matrix4f &predicted)
  {
    Eigen::Matrix4f last = Eigen::Matrix4f::Identity();
    ros::Time last_stamp;
    std::string last_mode;
    {
      std::lock_guard<std::mutex> lock(verified_place_history_mutex_);
      if (!verified_place_history_valid_ || verified_place_history_stamp_.isZero())
        return false;
      last = verified_place_history_pose_;
      last_stamp = verified_place_history_stamp_;
      last_mode = verified_place_history_mode_;
    }
    if (last_mode != "FULL_LIVO" || failsafe_mode_ != "FULL_LIVO")
      return false;

    // Healthy continuity must be judged in VEHICLE/map-pose space, not against
    // the live map<-camera_init anchor.  The anchor is the quantity we are
    // trying to repair and can be badly rotated after local FAST-LIVO2 drift.
    // Propagate the last independently verified map<-LiDAR pose with the
    // corresponding local FAST-LIVO2 odometry increment instead.
    return propagatePlaceCandidateWithOdomHistory(
        last, last_stamp, target_stamp, predicted);
  }

  void acceptVerifiedPlaceHistory(const Eigen::Matrix4f &pose,
                                  const ros::Time &stamp)
  {
    if (stamp.isZero() || !pose.allFinite()) return;
    // A healthy place-verification job can finish after the failure-mode
    // callback because saved-map ICP runs asynchronously.  Never let such a
    // pre-transition result overwrite the freshly seeded degraded predictor:
    // doing so moves its integration epoch backwards and makes every fresh
    // RGB-D/LiDAR delta fail the causal max-gap gate.
    ros::Time mode_enter;
    std::string mode;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      mode_enter = tracking_family_enter_stamp_;
      mode = failsafe_mode_;
    }
    if (mode != "FULL_LIVO" && !mode_enter.isZero() && stamp < mode_enter)
    {
      ROS_WARN_THROTTLE(1.0,
          "[FAILSAFE MAP] ignoring stale pre-failure verified-place history (age %.3fs)",
          (mode_enter - stamp).toSec());
      return;
    }
    std::lock_guard<std::mutex> lock(verified_place_history_mutex_);
    // Verified history is a causal integration epoch.  Async saved-map jobs
    // may complete out of order, especially exactly across a sensor-failure
    // transition.  Never move that epoch backwards: doing so would splice
    // fresh relative motion onto an older absolute pose and either trip the
    // max-gap gate or create a discontinuous global prediction.
    if (verified_place_history_valid_ && !verified_place_history_stamp_.isZero() &&
        stamp < verified_place_history_stamp_)
    {
      ROS_WARN_THROTTLE(1.0,
          "[FAILSAFE MAP] ignoring out-of-order verified-place history (backstep %.3fs)",
          (verified_place_history_stamp_ - stamp).toSec());
      return;
    }
    verified_place_history_pose_ = pose;
    verified_place_history_stamp_ = stamp;
    verified_place_history_mode_ = mode;
    verified_place_history_valid_ = true;
  }

  static double yawRad(const Eigen::Matrix4f &T)
  {
    return std::atan2(static_cast<double>(T(1, 0)),
                      static_cast<double>(T(0, 0)));
  }

  static double wrapRad(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  bool imuPlaceMotionConsistent(const Eigen::Matrix4f &candidate,
                                const ros::Time &stamp,
                                double &distance_from_last,
                                double &yaw_delta_deg)
  {
    distance_from_last = 0.0;
    yaw_delta_deg = 0.0;
    std::lock_guard<std::mutex> lock(imu_place_history_mutex_);
    if (!imu_place_history_valid_ || imu_place_history_stamp_.isZero() || stamp.isZero())
      return true;

    const double dt = (stamp - imu_place_history_stamp_).toSec();
    if (dt <= 0.15) return false;
    // A long gap means global vision was temporarily unavailable. Strict
    // visual+LiDAR geometry may reacquire; after acceptance history is reset.
    if (dt > 5.0) return true;

    const Eigen::Vector3d p = candidate.block<3, 1>(0, 3).cast<double>();
    const Eigen::Vector3d p0 = imu_place_history_pose_.block<3, 1>(0, 3).cast<double>();
    const Eigen::Vector3d vel = (p - p0) / dt;
    const double dyaw = wrapRad(yawRad(candidate) - yawRad(imu_place_history_pose_));
    const double yaw_rate = dyaw / dt;
    distance_from_last = (p - p0).norm();
    yaw_delta_deg = std::abs(dyaw) * 180.0 / M_PI;

    if (!vel.allFinite() || vel.norm() >= 6.5 ||
        std::abs(yaw_rate) >= 35.0 * M_PI / 180.0)
      return false;

    if (imu_place_motion_valid_)
    {
      const double acc = (vel - imu_place_velocity_).norm() / dt;
      const double yaw_acc = std::abs(yaw_rate - imu_place_yaw_rate_) / dt;
      if (acc >= 6.5 || yaw_acc >= 90.0 * M_PI / 180.0)
        return false;
    }
    return true;
  }

  void acceptImuPlaceHistory(const Eigen::Matrix4f &pose, const ros::Time &stamp)
  {
    std::lock_guard<std::mutex> lock(imu_place_history_mutex_);
    if (imu_place_history_valid_ && !imu_place_history_stamp_.isZero() && !stamp.isZero())
    {
      const double dt = (stamp - imu_place_history_stamp_).toSec();
      if (dt > 0.15 && dt < 5.0)
      {
        const Eigen::Vector3d p = pose.block<3, 1>(0, 3).cast<double>();
        const Eigen::Vector3d p0 = imu_place_history_pose_.block<3, 1>(0, 3).cast<double>();
        const Eigen::Vector3d vel = (p - p0) / dt;
        const double yaw_rate = wrapRad(yawRad(pose) - yawRad(imu_place_history_pose_)) / dt;
        if (vel.allFinite() && vel.norm() < 6.5 && std::isfinite(yaw_rate))
        {
          imu_place_velocity_ = vel;
          imu_place_yaw_rate_ = yaw_rate;
          imu_place_motion_valid_ = true;
        }
      }
      else
      {
        imu_place_motion_valid_ = false;
        imu_place_velocity_.setZero();
        imu_place_yaw_rate_ = 0.0;
      }
    }
    else
    {
      imu_place_motion_valid_ = false;
      imu_place_velocity_.setZero();
      imu_place_yaw_rate_ = 0.0;
    }
    imu_place_history_pose_ = pose;
    imu_place_history_stamp_ = stamp;
    imu_place_history_valid_ = true;
  }

  void resetImuPlaceHistory()
  {
    std::lock_guard<std::mutex> lock(imu_place_history_mutex_);
    imu_place_history_valid_ = false;
    imu_place_motion_valid_ = false;
    imu_place_history_pose_ = Eigen::Matrix4f::Identity();
    imu_place_history_stamp_ = ros::Time();
    imu_place_velocity_.setZero();
    imu_place_yaw_rate_ = 0.0;
  }

  double overlapRatio(const CloudT::Ptr &aligned,
                      const CloudT::Ptr &target,
                      double max_dist)
  {
    if (!aligned || !target || aligned->empty() || target->empty()) return 0.0;
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(target);
    std::vector<int> idx(1);
    std::vector<float> d2(1);
    const float gate = static_cast<float>(max_dist * max_dist);
    size_t good = 0;
    for (const auto &p : aligned->points)
      if (tree.nearestKSearch(p, 1, idx, d2) > 0 && d2[0] <= gate)
        ++good;
    return static_cast<double>(good) /
           static_cast<double>(std::max<size_t>(1, aligned->size()));
  }

  void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
  {
    if (!msg || msg->K[0] <= 0.0 || msg->K[4] <= 0.0) return;
    depth_fx_ = msg->K[0];
    depth_fy_ = msg->K[4];
    depth_cx_ = msg->K[2];
    depth_cy_ = msg->K[5];
  }

  bool snapshotState(Eigen::Matrix3d &Rwi, Eigen::Vector3d &twi,
                     std::string &mode, ros::Time &stamp)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_odom_valid_) return false;
    Rwi = latest_Rwi_;
    twi = latest_twi_;
    mode = failsafe_mode_;
    stamp = latest_odom_stamp_;
    return true;
  }

  Eigen::Matrix4f currentWorldLidar(const Eigen::Matrix3d &Rwi,
                                    const Eigen::Vector3d &twi) const
  {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    const Eigen::Matrix3d Rwl = Rwi * extR_;
    const Eigen::Vector3d twl = Rwi * extT_ + twi;
    T.block<3, 3>(0, 0) = Rwl.cast<float>();
    T.block<3, 1>(0, 3) = twl.cast<float>();
    return T;
  }

  Eigen::Matrix4f mapFrameGuess(const Eigen::Matrix4f &T_current_lidar)
  {
    // The startup/global anchor remains valid even when continuous healthy
    // anchor refinement is disabled. `use_online_anchor_` controls only the
    // expensive healthy-mode GICP refresh; degraded saved-map localization
    // must still start from map<-camera_init * current_lidar.
    std::lock_guard<std::mutex> lock(anchor_mutex_);
    return anchor_valid_ ? anchor_T_map_current_ * T_current_lidar : T_current_lidar;
  }

  bool publishRelativeMotionHint(const CloudT::Ptr &raw_source,
                                 const std::string &source_name,
                                 const ros::Time &stamp)
  {
    if (!raw_source || raw_source->size() < 80) return false;
    const bool is_depth = source_name.find("depth") != std::string::npos;

    CloudT::Ptr source(new CloudT(*raw_source));
    const double relative_leaf = is_depth ? 0.40 : 0.45;
    downsampleInPlace(source, relative_leaf);
    if (is_depth)
    {
      source = structuralFilter(source);
    }
    else
    {
      // Match continuous_imu_visual_lidar_stable_v2 exactly: finite VLP-16
      // points between 1 m and 60 m, without the additional structural crop.
      CloudT::Ptr clean(new CloudT);
      clean->reserve(source->size());
      for (const auto &p : source->points)
      {
        const double r2 = p.x * p.x + p.y * p.y + p.z * p.z;
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
            r2 > 1.0 && r2 < 3600.0)
          clean->push_back(p);
      }
      source = clean;
    }
    if (source->size() < 60) return false;

    // Always advance the reference slot, even if another ICP is still running.
    // Therefore the next successful registration uses the newest available
    // scan instead of a scan from ~1 second ago.
    CloudT::Ptr previous;
    ros::Time previous_stamp;
    {
      std::lock_guard<std::mutex> lock(relative_mutex_);
      CloudT::Ptr &slot =
          is_depth ? relative_last_depth_source_ : relative_last_lidar_source_;
      ros::Time &time_slot =
          is_depth ? relative_last_depth_stamp_ : relative_last_lidar_stamp_;
      if (slot) previous.reset(new CloudT(*slot));
      previous_stamp = time_slot;
      slot.reset(new CloudT(*source));
      time_slot = stamp;
    }
    if (!previous || previous_stamp.isZero() || stamp.isZero()) return false;

    const double dt = (stamp - previous_stamp).toSec();
    if (dt < 0.03 || dt > (is_depth ? 0.8 : 0.20)) return false;

    std::unique_lock<std::mutex> relative_registration_lock(
        relative_registration_mutex_, std::try_to_lock);
    if (!relative_registration_lock.owns_lock()) return false;

    pcl::IterativeClosestPoint<PointT, PointT> rel;
    rel.setInputSource(source);
    rel.setInputTarget(previous);
    rel.setMaximumIterations(is_depth ? 8 : 18);
    rel.setMaxCorrespondenceDistance(is_depth ? 1.2 : 2.2);
    rel.setTransformationEpsilon(is_depth ? 2.0e-4 : 1.0e-5);
    rel.setEuclideanFitnessEpsilon(is_depth ? 2.0e-4 : 1.0e-4);
    CloudT aligned;
    rel.align(aligned, Eigen::Matrix4f::Identity());
    if (!rel.hasConverged()) return false;

    const Eigen::Matrix4f T_lprev_lcur = rel.getFinalTransformation();
    const double fit = rel.getFitnessScore(is_depth ? 1.2 : 1.5);
    const double trans = T_lprev_lcur.block<3, 1>(0, 3).norm();
    const double rot =
        rotationAngleDeg(T_lprev_lcur.block<3, 3>(0, 0).cast<double>());
    const double fit_gate = is_depth ? 0.18 : std::numeric_limits<double>::infinity();
    const double trans_gate = is_depth ? 1.8 : 2.0;
    const double rot_gate = is_depth ? 12.0 : 25.0;
    if (!std::isfinite(fit) || fit > fit_gate ||
        trans > trans_gate || rot > rot_gate)
      return false;

    Eigen::Matrix4f T_bl = Eigen::Matrix4f::Identity();
    T_bl.block<3, 3>(0, 0) = extR_.cast<float>();
    T_bl.block<3, 1>(0, 3) = extT_.cast<float>();
    Eigen::Matrix4f T_increment_lidar = T_lprev_lcur;
    if (!is_depth)
    {
      // Exact causal relative-motion filter from the successful standalone
      // IMU-failure v2 implementation.  Filter in LiDAR coordinates first,
      // then convert the filtered SE(3) increment to body coordinates.
      const Eigen::Vector3d v_lidar =
          T_lprev_lcur.block<3, 1>(0, 3).cast<double>() / dt;
      Eigen::AngleAxisd aa_lidar(T_lprev_lcur.block<3, 3>(0, 0).cast<double>());
      Eigen::Vector3d w_lidar = Eigen::Vector3d::Zero();
      if (std::isfinite(aa_lidar.angle()) && aa_lidar.axis().allFinite())
        w_lidar = aa_lidar.axis() * (aa_lidar.angle() / dt);
      if (!v_lidar.allFinite() || !w_lidar.allFinite() ||
          v_lidar.norm() >= 8.0 || w_lidar.norm() >= 2.2)
        return false;

      if (!lidar_rel_filter_valid_)
      {
        lidar_filt_trans_vel_ = v_lidar;
        lidar_filt_ang_vel_ = w_lidar;
        lidar_rel_filter_valid_ = true;
      }
      else
      {
        Eigen::Vector3d dv = v_lidar - lidar_filt_trans_vel_;
        const double max_dv = 4.0 * dt;
        if (dv.norm() > max_dv && dv.norm() > 1.0e-9)
          dv *= max_dv / dv.norm();
        Eigen::Vector3d dw = w_lidar - lidar_filt_ang_vel_;
        const double max_dw = 1.2 * dt;
        if (dw.norm() > max_dw && dw.norm() > 1.0e-9)
          dw *= max_dw / dw.norm();
        const Eigen::Vector3d vc = lidar_filt_trans_vel_ + dv;
        const Eigen::Vector3d wc = lidar_filt_ang_vel_ + dw;
        lidar_filt_trans_vel_ = 0.55 * lidar_filt_trans_vel_ + 0.45 * vc;
        lidar_filt_ang_vel_ = 0.55 * lidar_filt_ang_vel_ + 0.45 * wc;
      }

      T_increment_lidar.setIdentity();
      T_increment_lidar.block<3, 1>(0, 3) =
          (lidar_filt_trans_vel_ * dt).cast<float>();
      const Eigen::Vector3d rv = lidar_filt_ang_vel_ * dt;
      const double ang = rv.norm();
      Eigen::Matrix3d Rf = Eigen::Matrix3d::Identity();
      if (ang > 1.0e-12)
        Rf = Eigen::AngleAxisd(ang, rv / ang).toRotationMatrix();
      T_increment_lidar.block<3, 3>(0, 0) = Rf.cast<float>();
    }

    const Eigen::Matrix4f T_bprev_bcur =
        T_bl * T_increment_lidar * T_bl.inverse();

    const Eigen::Vector3d v_body =
        T_bprev_bcur.block<3, 1>(0, 3).cast<double>() / dt;
    Eigen::AngleAxisd aa(T_bprev_bcur.block<3, 3>(0, 0).cast<double>());
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    if (std::isfinite(aa.angle()) && aa.axis().allFinite())
      omega = aa.axis() * (aa.angle() / dt);

    if (!v_body.allFinite() || !omega.allFinite() ||
        v_body.norm() > 15.0 || omega.norm() > 3.0)
      return false;

    geometry_msgs::TwistStamped hint;
    hint.header.stamp = stamp;
    hint.header.frame_id =
        is_depth ? "depth_relative_motion" : "lidar_relative_motion";
    hint.twist.linear.x = v_body.x();
    hint.twist.linear.y = v_body.y();
    hint.twist.linear.z = v_body.z();
    hint.twist.angular.x = omega.x();
    hint.twist.angular.y = omega.y();
    hint.twist.angular.z = omega.z();
    relative_motion_pub_.publish(hint);

    if (!is_depth)
    {
      geometry_msgs::TwistStamped delta;
      delta.header.stamp = stamp;
      delta.header.frame_id = "lidar_relative_delta";
      const Eigen::Vector3d dt_body =
          T_bprev_bcur.block<3, 1>(0, 3).cast<double>();
      Eigen::AngleAxisd daa(T_bprev_bcur.block<3, 3>(0, 0).cast<double>());
      Eigen::Vector3d drv = Eigen::Vector3d::Zero();
      if (std::isfinite(daa.angle()) && daa.axis().allFinite())
        drv = daa.axis() * daa.angle();
      delta.twist.linear.x = dt_body.x();
      delta.twist.linear.y = dt_body.y();
      delta.twist.linear.z = dt_body.z();
      delta.twist.angular.x = drv.x();
      delta.twist.angular.y = drv.y();
      delta.twist.angular.z = drv.z();
      relative_motion_pub_.publish(delta);
    }

    ROS_INFO_THROTTLE(0.5,
        "[FAILSAFE REL] %s fit=%.3f dt=%.3f v=%.2fm/s w=%.1fdeg/s pts=%zu",
        hint.header.frame_id.c_str(), fit, dt, v_body.norm(),
        omega.norm() * 180.0 / M_PI, source->size());
    return true;
  }

  Eigen::Matrix4f trackingGuess(const Eigen::Matrix4f &T_current_lidar,
                                 const CloudT::Ptr &source,
                                 const std::string &source_name)
  {
    // During LiDAR failure keep the saved-map tracker globally anchored.
    // Propagate the last accepted map pose only by the *incremental* ESIKF
    // motion since that accepted correction.  Never reseed from the absolute
    // odometry pose: once LiDAR is absent that global odometry frame can drift
    // quickly even though short-term IMU motion remains useful.
    if (source_name == "depth_saved_map_icp")
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      if (!tracker_valid_ || tracker_source_name_ != source_name)
        return mapFrameGuess(T_current_lidar);

      const Eigen::Matrix4f T_last_current_delta =
          tracker_last_current_lidar_.inverse() * T_current_lidar;
      const double trans = T_last_current_delta.block<3, 1>(0, 3).norm();
      const double rot =
          rotationAngleDeg(T_last_current_delta.block<3, 3>(0, 0).cast<double>());
      if (trans <= tracker_max_odom_delta_translation_ &&
          rot <= tracker_max_odom_delta_rotation_deg_)
        return tracker_T_map_lidar_ * T_last_current_delta;

      // If several depth updates were missed, hold the last trusted global pose
      // and let map registration reacquire locally rather than following a
      // potentially large inertial drift.
      return tracker_T_map_lidar_;
    }

    std::lock_guard<std::mutex> lock(tracker_mutex_);
    if (!tracker_valid_ || !tracker_last_source_ ||
        tracker_source_name_ != source_name)
      return mapFrameGuess(T_current_lidar);

    // Healthy FAST-LIVO2 has trustworthy short-horizon local odometry. Once a
    // visual+LiDAR match has acquired the saved map, propagate that last global
    // LiDAR pose with the local odometry increment and let scan-to-map GICP
    // refine it. This is cheaper and more stable than a second scan-to-scan ICP.
    if ((failsafe_mode_ == "FULL_LIVO" ||
         failsafe_mode_ == "LIO_CAMERA_FAILED") &&
        source_name == "lidar_saved_map_icp")
    {
      // Healthy rolling localization must never become its own drifting
      // odometry source. Seed every saved-map refinement from the CURRENT
      // globally-predicted FAST-LIVO2 pose (map<-local * local<-lidar), not
      // from the previous ICP result propagated forward. The immutable saved
      // map may refine this prediction locally, but cannot recursively drag
      // the next seed into a nearby repeated-geometry basin.
      return mapFrameGuess(T_current_lidar);
    }

    // Sensor-local odometry for the degraded mode. Align the current available
    // geometry to the last accepted source cloud, then compose that relative
    // motion with the last trusted global/map pose.
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> rel;
    rel.setInputSource(source);
    rel.setInputTarget(tracker_last_source_);
    rel.setMaximumIterations(25);
    rel.setMaxCorrespondenceDistance(relative_max_correspondence_);
    rel.setTransformationEpsilon(1.0e-5);
    rel.setEuclideanFitnessEpsilon(1.0e-4);
    rel.setCorrespondenceRandomness(15);
    CloudT aligned;
    rel.align(aligned, Eigen::Matrix4f::Identity());
    if (rel.hasConverged())
    {
      const Eigen::Matrix4f T_last_current = rel.getFinalTransformation();
      const double trans = T_last_current.block<3, 1>(0, 3).norm();
      const double rot =
          rotationAngleDeg(T_last_current.block<3, 3>(0, 0).cast<double>());
      const double fit = rel.getFitnessScore(relative_max_correspondence_);
      if (std::isfinite(fit) && fit <= relative_fitness_threshold_ &&
          trans <= relative_max_translation_ &&
          rot <= relative_max_rotation_deg_)
      {
        ROS_INFO_THROTTLE(1.0,
            "[FAILSAFE MAP] relative %s motion %.2fm/%.1fdeg fit=%.3f",
            source_name.c_str(), trans, rot, fit);
        return tracker_T_map_lidar_ * T_last_current;
      }
    }

    // If relative registration is uncertain, hold the last trusted pose rather
    // than following a degraded estimator jump.
    return tracker_T_map_lidar_;
  }

  void acceptTrackedPose(const Eigen::Matrix4f &T_map_lidar,
                         const Eigen::Matrix4f &T_current_lidar,
                         const CloudT::Ptr &source,
                         const std::string &source_name,
                         const ros::Time &stamp)
  {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    tracker_T_map_lidar_ = T_map_lidar;
    tracker_last_current_lidar_ = T_current_lidar;
    tracker_last_source_.reset(new CloudT(*source));
    tracker_source_name_ = source_name;
    tracker_valid_ = true;
    last_tracker_accept_stamp_ = stamp;
  }

  bool trackerHasPoseFor(const std::string &source_name)
  {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    return tracker_valid_ && tracker_source_name_ == source_name;
  }

  bool estimateAnchor(const CloudT::Ptr &source_lidar,
                      const Eigen::Matrix3d &Rwi,
                      const Eigen::Vector3d &twi)
  {
    if (!source_lidar || source_lidar->size() < 80) return false;

    CloudT::Ptr source(new CloudT(*source_lidar));
    downsampleInPlace(source, 0.65);
    source = structuralFilter(source);
    if (source->size() < 80) return false;

    const Eigen::Matrix4f T_current_lidar = currentWorldLidar(Rwi, twi);

    Eigen::Matrix4f prior_anchor = Eigen::Matrix4f::Identity();
    bool have_prior = false;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      have_prior = anchor_valid_;
      if (have_prior) prior_anchor = anchor_T_map_current_;
    }

    // The healthy FAST-LIVO2 odometry pose must first be mapped into the saved
    // map frame.  The previous/fixed anchor is only an initializer; current
    // LiDAR geometry refines it locally so loop-closure deformation/drift does
    // not force one global rigid transform over the whole route.
    const Eigen::Matrix4f map_guess =
        have_prior ? prior_anchor * T_current_lidar : T_current_lidar;
    CloudT::Ptr target = localTarget(map_guess.block<3, 1>(0, 3));
    CloudT::Ptr target_ds(new CloudT(*target));
    downsampleInPlace(target_ds, 0.70);

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> reg;
    reg.setInputSource(source);
    reg.setInputTarget(target_ds);
    reg.setMaximumIterations(22);
    reg.setMaxCorrespondenceDistance(1.8);
    reg.setTransformationEpsilon(1.0e-5);
    reg.setEuclideanFitnessEpsilon(1.0e-4);
    reg.setCorrespondenceRandomness(18);
    CloudT aligned;
    reg.align(aligned, map_guess);
    if (!reg.hasConverged()) return false;

    const double fitness = reg.getFitnessScore(1.8);
    const Eigen::Matrix4f T_map_lidar = reg.getFinalTransformation();
    const double ov =
        overlapRatio(CloudT::Ptr(new CloudT(aligned)), target_ds, 0.85);
    const Eigen::Matrix4f candidate_anchor =
        T_map_lidar * T_current_lidar.inverse();

    Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
    double delta_t = 0.0, delta_r = 0.0;
    if (have_prior)
    {
      delta = prior_anchor.inverse() * candidate_anchor;
      delta_t = delta.block<3, 1>(0, 3).norm();
      delta_r =
          rotationAngleDeg(delta.block<3, 3>(0, 0).cast<double>());
    }

    const bool good =
        std::isfinite(fitness) && fitness < 0.18 &&
        ov > 0.55 &&
        (!have_prior || (delta_t < 2.0 && delta_r < 8.0));
    if (!good)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[FAILSAFE MAP] Healthy anchor refinement rejected fit=%.3f ov=%.2f delta=%.2fm/%.1fdeg",
          fitness, ov, delta_t, delta_r);
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      if (!anchor_valid_)
      {
        anchor_T_map_current_ = candidate_anchor;
        anchor_valid_ = true;
      }
      else
      {
        // Follow slow healthy odometry-vs-map deformation without reacting to
        // one noisy registration.
        const float alpha_t = 0.35f;
        const float alpha_r = 0.30f;
        anchor_T_map_current_.block<3, 1>(0, 3) =
            (1.0f - alpha_t) * anchor_T_map_current_.block<3, 1>(0, 3) +
            alpha_t * candidate_anchor.block<3, 1>(0, 3);
        Eigen::Quaternionf qa(anchor_T_map_current_.block<3, 3>(0, 0));
        Eigen::Quaternionf qb(candidate_anchor.block<3, 3>(0, 0));
        anchor_T_map_current_.block<3, 3>(0, 0) =
            qa.slerp(alpha_r, qb).normalized().toRotationMatrix();
      }
      healthy_anchor_stamp_ = ros::Time::now();
    }

    ROS_INFO_THROTTLE(
        0.75,
        "[FAILSAFE MAP] Healthy map<-ESIKF anchor updated fit=%.3f ov=%.2f delta=%.2fm/%.1fdeg",
        fitness, ov, delta_t, delta_r);
    return true;
  }

  CloudT::Ptr localTarget(const Eigen::Vector3f &center,
                          double xy_radius = -1.0,
                          double z_radius = -1.0,
                          bool use_visual_map = false)
  {
    // Localize mode uses an immutable saved map.  A CropBox over the complete
    // 4M+ point map for every GICP query is O(N) and was starving the primary
    // FAST-LIVO2 estimator at rosbag 1x.  Build a KD-tree once at startup and
    // query only the spatial neighborhood needed by the registration.
    CloudT::Ptr local(new CloudT);
    CloudT::Ptr active_map =
        (use_visual_map && visual_map_enable_ && visual_map_ && !visual_map_->empty())
            ? visual_map_ : map_;
    pcl::KdTreeFLANN<PointT>::Ptr active_tree =
        (use_visual_map && visual_map_enable_ && visual_map_kdtree_ &&
         visual_map_ && !visual_map_->empty())
            ? visual_map_kdtree_ : map_kdtree_;
    if (!active_tree || !active_map || active_map->empty()) return local;

    const double rxy = xy_radius > 0.0 ? xy_radius : local_map_radius_;
    const double rz = z_radius > 0.0 ? z_radius : local_map_z_radius_;

    PointT query;
    query.x = center.x();
    query.y = center.y();
    query.z = center.z();
    const double search_radius = std::sqrt(
        2.0 * rxy * rxy + rz * rz);
    std::vector<int> indices;
    std::vector<float> sqdist;
    if (active_tree->radiusSearch(query, search_radius, indices, sqdist) <= 0)
      return local;

    local->reserve(indices.size());
    for (const int idx : indices)
    {
      if (idx < 0 || static_cast<size_t>(idx) >= active_map->size()) continue;
      const PointT &p = active_map->points[static_cast<size_t>(idx)];
      if (std::abs(p.x - center.x()) > rxy ||
          std::abs(p.y - center.y()) > rxy ||
          std::abs(p.z - center.z()) > rz)
        continue;
      local->push_back(p);
    }
    local->width = local->size();
    local->height = 1;
    local->is_dense = false;
    if (local->size() < 500)
      ROS_WARN_THROTTLE(1.0,
          "[FAILSAFE MAP] Local KD target sparse (%zu pts) near %.1f %.1f %.1f",
          local->size(), center.x(), center.y(), center.z());
    return local;
  }

  bool registerToMap(const CloudT::Ptr &source_lidar,
                     const Eigen::Matrix3d &Rwi,
                     const Eigen::Vector3d &twi,
                     const std::string &source_name,
                     const ros::Time &stamp,
                     bool allow_place = true)
  {
    // Registration is CPU-heavy. With an AsyncSpinner, keep only one GICP
    // active at a time while allowing odometry/health callbacks to continue
    // updating concurrently for latency compensation.
    std::unique_lock<std::mutex> registration_lock(registration_mutex_, std::try_to_lock);
    if (!registration_lock.owns_lock())
    {
      if (failsafe_mode_ == "FULL_LIVO" && !allow_place)
        ROS_WARN_THROTTLE(1.0, "[FAILSAFE MAP] healthy tracker skipped: registration busy");
      return false;
    }
    if (!source_lidar || source_lidar->size() < 80) return false;
    const bool is_depth = source_name.find("depth") != std::string::npos;
    const bool use_visual_target = is_depth && visual_map_enable_ && visual_map_ && !visual_map_->empty();
    CloudT::Ptr source(new CloudT(*source_lidar));
    // Match the source resolution used by the validated standalone tests:
    // RGB-D 18 cm, LiDAR 45 cm before fine registration.
    downsampleInPlace(source, is_depth ? 0.18 : 0.45);
    source = structuralFilter(source);
    if (source->size() < 60) return false;

    const Eigen::Matrix4f T_current_lidar = currentWorldLidar(Rwi, twi);
    // Current global prediction from the healthy pre-failure anchor. The
    // successful standalone place matcher rejected visually plausible but
    // geometrically repeated locations that implied an implausible jump from
    // the continuously propagated pose. Preserve that guard here.
    const Eigen::Matrix4f T_map_lidar_prior = mapFrameGuess(T_current_lidar);
    const bool lidar_depth_fail =
        is_depth &&
        (failsafe_mode_ == "VI_LIDAR_FAILED" ||
         failsafe_mode_ == "IMU_ONLY_BRIDGE");

    // The tracker is seeded atomically from the trusted Step-3 map pose at the
    // LiDAR-failure boundary. If it has been refreshed recently by accepted
    // depth-map geometry, stay inside that local metric basin. This prevents
    // a visually similar road keyframe from dragging a perfectly good local
    // depth alignment several metres along repeated campus geometry.
    bool depth_tracker_recent = false;
    double depth_tracker_age_sec = std::numeric_limits<double>::infinity();
    if (lidar_depth_fail)
    {
      std::lock_guard<std::mutex> tlock(tracker_mutex_);
      if (tracker_valid_ && tracker_source_name_ == source_name &&
          !last_tracker_accept_stamp_.isZero() && !stamp.isZero())
      {
        depth_tracker_age_sec = (stamp - last_tracker_accept_stamp_).toSec();
        depth_tracker_recent = depth_tracker_max_age_sec_ > 0.0 &&
            depth_tracker_age_sec >= -0.05 &&
            depth_tracker_age_sec <= depth_tracker_max_age_sec_;
      }
    }

    const bool can_use_place = allow_place &&
        (source_name == "depth_saved_map_icp" ||
         source_name == "lidar_saved_map_icp") &&
        !(lidar_depth_fail && depth_tracker_recent);

    Eigen::Matrix4f guess = trackingGuess(T_current_lidar, source, source_name);
    Eigen::Matrix4f T_wl = Eigen::Matrix4f::Identity();
    CloudT::Ptr target;
    CloudT aligned;
    double fitness = std::numeric_limits<double>::infinity();
    double correction_translation = std::numeric_limits<double>::infinity();
    double correction_rotation = std::numeric_limits<double>::infinity();
    double selected_overlap = 0.0;
    bool place_guided = false;
    bool registration_ok = false;

    std::vector<Eigen::Matrix4f> place_candidates;
    ros::Time place_candidate_stamp;
    bool place_metric_rgbd = false;
    bool have_anchor_for_place = false;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      have_anchor_for_place = anchor_valid_;
    }
    const double registration_place_max_age =
        (!have_anchor_for_place && failsafe_mode_ == "FULL_LIVO")
            ? place_bootstrap_max_age_ : -1.0;
    if (can_use_place && snapshotPlaceCandidates(
            place_candidates, &place_candidate_stamp, &place_metric_rgbd,
            registration_place_max_age))
    {
      double best_score = std::numeric_limits<double>::infinity();
      for (size_t ci = 0; ci < place_candidates.size() && ci < 4; ++ci)
      {
        Eigen::Matrix4f candidate_guess = place_candidates[ci];
        if (failsafe_mode_ == "FULL_LIVO" && !is_depth)
        {
          Eigen::Matrix4f propagated;
          if (!propagatePlaceCandidateWithOdomHistory(
                  candidate_guess, place_candidate_stamp, stamp, propagated))
            continue;
          candidate_guess = propagated;
        }
        else if (failsafe_mode_ == "LV_IMU_FAILED" && !is_depth)
        {
          Eigen::Matrix4f propagated;
          if (!propagatePlaceCandidateWithLidarHistory(
                  candidate_guess, place_candidate_stamp, stamp, propagated))
            continue;
          candidate_guess = propagated;
        }
        else if ((failsafe_mode_ == "VI_LIDAR_FAILED" ||
                  failsafe_mode_ == "IMU_ONLY_BRIDGE") && is_depth)
        {
          Eigen::Matrix4f propagated;
          // The RGB-D frontend is intentionally processed at a reduced frame
          // rate and can be ~1-2 s behind under load.  The persistent ESIKF
          // odometry already contains those fallback measurements and is
          // timestamp-current, so use its local motion only to transport the
          // delayed visual-place hypothesis to the depth-source timestamp.
          // Raw RGB-D delta history remains a safe fallback if local odometry
          // does not bracket both timestamps.
          const bool propagated_with_esikf =
              propagatePlaceCandidateWithOdomHistory(
                  candidate_guess, place_candidate_stamp, stamp, propagated);
          if (!propagated_with_esikf &&
              !propagatePlaceCandidateWithRgbdHistory(
                  candidate_guess, place_candidate_stamp, stamp, propagated))
            continue;
          candidate_guess = propagated;
        }
        CloudT::Ptr candidate_target =
            localTarget(candidate_guess.block<3, 1>(0, 3), -1.0, -1.0, use_visual_target);

        // Exact geometry from the standalone-confirmed matcher:
        //   coarse GICP in a wider basin
        //   -> fine GICP in a tight basin
        // This is deliberately stricter than the previous fast ICP shortcut.
        CloudT::Ptr coarse_src(new CloudT(*source));
        CloudT::Ptr coarse_tgt(new CloudT(*candidate_target));
        CloudT::Ptr fine_src(new CloudT(*source));
        CloudT::Ptr fine_tgt(new CloudT(*candidate_target));

        if (is_depth)
        {
          downsampleInPlace(coarse_src, 0.35);
          downsampleInPlace(coarse_tgt, 0.60);
          downsampleInPlace(fine_src, 0.18);
          downsampleInPlace(fine_tgt, 0.35);
        }
        else
        {
          downsampleInPlace(coarse_src, 0.75);
          downsampleInPlace(coarse_tgt, 0.80);
          downsampleInPlace(fine_src, 0.45);
          downsampleInPlace(fine_tgt, 0.50);
        }

        if (coarse_src->size() < 60 || coarse_tgt->size() < 300 ||
            fine_src->size() < 80 || fine_tgt->size() < 400)
          continue;

        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> coarse;
        coarse.setInputSource(coarse_src);
        coarse.setInputTarget(coarse_tgt);
        coarse.setMaximumIterations(is_depth ? 30 : 25);
        coarse.setMaxCorrespondenceDistance(2.0);
        coarse.setTransformationEpsilon(1.0e-5);
        coarse.setEuclideanFitnessEpsilon(1.0e-4);
        coarse.setCorrespondenceRandomness(15);
        CloudT coarse_aligned;
        coarse.align(coarse_aligned, candidate_guess);
        if (!coarse.hasConverged()) continue;

        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> fine;
        fine.setInputSource(fine_src);
        fine.setInputTarget(fine_tgt);
        fine.setMaximumIterations(is_depth ? 35 : 30);
        fine.setMaxCorrespondenceDistance(is_depth ? 0.90 : 1.25);
        fine.setTransformationEpsilon(1.0e-6);
        fine.setEuclideanFitnessEpsilon(1.0e-5);
        fine.setCorrespondenceRandomness(20);
        CloudT::Ptr candidate_aligned(new CloudT);
        fine.align(*candidate_aligned, coarse.getFinalTransformation());
        if (!fine.hasConverged()) continue;

        const Eigen::Matrix4f T_candidate = fine.getFinalTransformation();
        const Eigen::Matrix4f d = candidate_guess.inverse() * T_candidate;
        const double ctrans = d.block<3, 1>(0, 3).norm();
        const double crot =
            rotationAngleDeg(d.block<3, 3>(0, 0).cast<double>());
        const double cfit =
            fine.getFitnessScore(is_depth ? 0.90 : 1.25);
        const double coverlap =
            overlapRatio(candidate_aligned, fine_tgt,
                         is_depth ? 0.55 : 0.75);

        const Eigen::Matrix4f prior_delta =
            T_map_lidar_prior.inverse() * T_candidate;
        const double prior_trans = prior_delta.block<3, 1>(0, 3).norm();
        const double prior_rot =
            rotationAngleDeg(prior_delta.block<3, 3>(0, 0).cast<double>());

        // In IMU failure the live map pose is not a safe continuity prior: the
        // global anchor can itself be wrong, while the saved visual database is
        // in the optimized-map frame. Use the previous independently verified
        // place+LiDAR pose instead, reproducing standalone v3c's speed/yaw/
        // acceleration consistency gates. Other modes keep the local prior.
        double continuity_trans = prior_trans;
        double continuity_rot = prior_rot;
        bool continuity_ok = true;
        if (failsafe_mode_ == "FULL_LIVO" && !is_depth)
        {
          // Do NOT use the current map anchor as healthy continuity truth:
          // globallock13 showed excellent visual+LiDAR GICP candidates being
          // rejected only because that live anchor had already drifted by
          // 90-160 deg.  Predict from the last independently verified global
          // LiDAR pose using FAST-LIVO2's local odometry increment instead.
          Eigen::Matrix4f predicted;
          if (predictHealthyFromVerifiedPlaceHistory(stamp, predicted))
          {
            const Eigen::Matrix4f pd = predicted.inverse() * T_candidate;
            continuity_trans = pd.block<3, 1>(0, 3).norm();
            continuity_rot =
                rotationAngleDeg(pd.block<3, 3>(0, 0).cast<double>());
            // This is a short-horizon physical vehicle-pose gate. It is
            // deliberately tighter than the old anchor-space 12 m/30 deg gate
            // while still allowing small loop-closure/map deformation.
            // Normal healthy continuity remains strict.  If local odometry has
            // drifted far enough that this predictor is no longer trustworthy,
            // permit only an independently *very strong* saved-map hypothesis
            // into the existing two-frame temporal confirmation stage below.
            // This is not a wider jump gate: it requires tight visual seed->GICP
            // correction plus high geometric overlap/fitness, and still needs a
            // second physically-consistent global observation before acceptance.
            const bool strong_global_reacquisition =
                cfit < 0.10 && coverlap > 0.85 &&
                ctrans < 1.00 && crot < 6.0;
            // Preserve the validated healthy verified-motion/reacquisition
            // behavior here.  Metric RGB-D has its own independent seed->GICP
            // agreement gate below; tightening both gates at once starved
            // otherwise valid global corrections in rgbd_consistency.
            continuity_ok =
                (continuity_trans < 4.0 && continuity_rot < 20.0) ||
                strong_global_reacquisition;
          }
          else
          {
            // Before the first verified global fix, retain a bounded bootstrap
            // basin around the initialized map pose.
            continuity_trans = prior_trans;
            continuity_rot = prior_rot;
            continuity_ok = prior_trans < 12.0 && prior_rot < 30.0;
          }
        }
        else if (failsafe_mode_ == "LV_IMU_FAILED" && !is_depth)
        {
          // Strict visual retrieval + current-LiDAR coarse/fine GICP has
          // already established the global hypothesis. The live map prior can
          // drift during IMU loss, so do not reject a good global fix against
          // that failed quantity. Preserve the standalone-style physical
          // continuity check between independently verified global fixes.
          double hist_trans = 0.0, hist_rot = 0.0;
          const bool history_ok = imuPlaceMotionConsistent(
              T_candidate, stamp, hist_trans, hist_rot);
          Eigen::Matrix4f predicted;
          if (predictFromVerifiedPlaceHistory(stamp, true, predicted))
          {
            const Eigen::Matrix4f pd = predicted.inverse() * T_candidate;
            continuity_trans = pd.block<3, 1>(0, 3).norm();
            continuity_rot =
                rotationAngleDeg(pd.block<3, 3>(0, 0).cast<double>());
            continuity_ok = history_ok &&
                            continuity_trans < 3.0 &&
                            continuity_rot < 15.0;
          }
          else
          {
            // First verified fix must still be local to the pre-failure map
            // lock; subsequent fixes use the sensor-propagated verified pose.
            continuity_trans = prior_trans;
            continuity_rot = prior_rot;
            continuity_ok = history_ok &&
                            prior_trans < 4.0 && prior_rot < 18.0;
          }
        }
        else if ((failsafe_mode_ == "VI_LIDAR_FAILED" ||
                  failsafe_mode_ == "IMU_ONLY_BRIDGE") && is_depth)
        {
          // A saved visual/depth map is the *absolute* authority, while the
          // propagated RGB-D/ESIKF pose is only a short-term prediction.
          // When prediction has drifted by tens of metres or rotated by 90+
          // degrees, the old continuity gate rejects even excellent map
          // matches.  Require several independent geometric observations to
          // form a physically continuous *map-frame trajectory* before a
          // strong candidate can bypass that drifting-prediction veto.
          bool temporal_relock = false;
          const bool strong_independent_depth =
              depth_place_temporal_relock_enable_ && ci == 0 &&
              std::isfinite(cfit) &&
              cfit <= depth_place_temporal_relock_max_fitness_ &&
              coverlap >= depth_place_temporal_relock_min_overlap_ &&
              ctrans <= 2.25 && crot <= 10.0;
          if (strong_independent_depth && !stamp.isZero())
          {
            std::lock_guard<std::mutex> lock(depth_relock_mutex_);
            bool consistent = false;
            Eigen::Vector2f candidate_velocity = Eigen::Vector2f::Zero();
            bool candidate_velocity_valid = false;
            if (depth_relock_valid_ && stamp > depth_relock_stamp_)
            {
              const double dt = (stamp - depth_relock_stamp_).toSec();
              const Eigen::Matrix4f motion =
                  depth_relock_pose_.inverse() * T_candidate;
              candidate_velocity = (T_candidate.block<2,1>(0,3) -
                                    depth_relock_pose_.block<2,1>(0,3)) / dt;
              candidate_velocity_valid = candidate_velocity.allFinite() && dt <= 3.5;
              const double speed = motion.block<3,1>(0,3).norm() / dt;
              const double yaw_rate =
                  rotationAngleDeg(motion.block<3,3>(0,0).cast<double>()) / dt;
              consistent = candidate_velocity_valid &&
                  speed <= std::max(1.0,depth_place_temporal_relock_max_speed_mps_) &&
                  yaw_rate <= 35.0;
              if (consistent && depth_relock_velocity_valid_)
              {
                const double accel =
                    (candidate_velocity - depth_relock_velocity_).norm()/dt;
                consistent = accel <= depth_place_temporal_relock_max_accel_mps2_;
                if (consistent && candidate_velocity.norm() > 0.8f &&
                    depth_relock_velocity_.norm() > 0.8f)
                {
                  const double cosine = std::max(-1.0,std::min(1.0,
                      static_cast<double>(candidate_velocity.normalized().dot(
                          depth_relock_velocity_.normalized()))));
                  const double turn_deg = std::acos(cosine)*180.0/M_PI;
                  consistent = turn_deg <=
                      depth_place_temporal_relock_max_course_change_deg_;
                }
              }
            }
            depth_relock_streak_ = consistent ? depth_relock_streak_ + 1 : 1;
            // Do not carry a rejected road-alias velocity into the new
            // sequence; two subsequent independent motions must agree first.
            depth_relock_velocity_valid_ = consistent && candidate_velocity_valid;
            if (candidate_velocity_valid) depth_relock_velocity_ = candidate_velocity;
            depth_relock_pose_ = T_candidate;
            depth_relock_stamp_ = stamp;
            depth_relock_valid_ = true;
            temporal_relock = depth_relock_streak_ >=
                std::max(3, depth_place_temporal_relock_min_fixes_);
            ROS_INFO_THROTTLE(0.35,
                "[STEP3 DEPTH RELOCK] independent geometry fit=%.3f overlap=%.2f streak=%d/%d confirmed=%d",
                cfit, coverlap, depth_relock_streak_,
                std::max(3, depth_place_temporal_relock_min_fixes_),
                temporal_relock ? 1 : 0);
          }
          Eigen::Matrix4f predicted;
          if (predictFromVerifiedPlaceHistory(stamp, false, predicted))
          {
            const Eigen::Matrix4f pd = predicted.inverse() * T_candidate;
            continuity_trans = pd.block<3, 1>(0, 3).norm();
            continuity_rot =
                rotationAngleDeg(pd.block<3, 3>(0, 0).cast<double>());
            // RGB-D is the relative-motion source during LiDAR loss. Over a
            // long outage its heading can drift even while the independently
            // saved visual/depth map gives an excellent absolute hypothesis.
            // Permit only a top-ranked, exceptionally strong map hypothesis
            // past that drifting-yaw veto. It is still NOT accepted here:
            // LiDAR-loss keeps the existing two-consecutive-observation
            // confirmation gate below before any global correction is used.
            const bool strong_depth_relock =
                ci == 0 &&
                cfit < 0.09 && coverlap > 0.90 &&
                ctrans < 1.8 && crot < 10.0;
            const bool strong_depth_relock_near_route =
                strong_depth_relock &&
                prior_trans < 6.0 && prior_rot < 35.0;
            // During LiDAR loss the short-horizon RGB-D/IMU yaw predictor can
            // drift before position does.  Do not let that failed yaw veto an
            // independently strong *position* observation.  This bypass is
            // intentionally narrow: top appearance candidate only, excellent
            // depth-map geometry, small seed->GICP displacement, bounded
            // translational continuity, and it still goes through the existing
            // two-observation temporal confirmation below.  The corresponding
            // GICP orientation is not applied when the position-only mode is
            // enabled, so repeated-road yaw minima cannot rotate the map pose.
            const bool strong_depth_position_relock =
                depth_place_position_relock_enable_ && ci == 0 &&
                cfit <= depth_place_position_relock_max_fitness_ &&
                coverlap >= depth_place_position_relock_min_overlap_ &&
                ctrans <= depth_place_position_relock_max_seed_translation_ &&
                crot <= depth_place_position_relock_max_seed_rotation_deg_ &&
                continuity_trans <=
                    depth_place_position_relock_max_continuity_translation_;
            // Metric RGB-D candidates have already passed saved-image ORB,
            // depth-backed PnP, location uniqueness, and cross-frame checks.
            // Require independent tight live-depth GICP too before allowing
            // that measurement to overrule an already-drifting ESIKF prior.
            const bool metric_feature_relock =
                place_metric_rgbd && ci == 0 &&
                cfit <= 0.11 && coverlap >= 0.78 &&
                ctrans <= 2.25 && crot <= 10.0;
            continuity_ok =
                (continuity_trans < 3.0 && continuity_rot < 15.0) ||
                (strong_depth_relock &&
                 continuity_trans < 3.5 && continuity_rot < 18.0) ||
                strong_depth_relock_near_route ||
                strong_depth_position_relock || temporal_relock ||
                metric_feature_relock;
          }
          else
          {
            continuity_trans = prior_trans;
            continuity_rot = prior_rot;
            const bool strong_depth_relock =
                ci == 0 &&
                cfit < 0.09 && coverlap > 0.90 &&
                ctrans < 1.8 && crot < 10.0;
            const bool strong_depth_position_relock =
                depth_place_position_relock_enable_ && ci == 0 &&
                cfit <= depth_place_position_relock_max_fitness_ &&
                coverlap >= depth_place_position_relock_min_overlap_ &&
                ctrans <= depth_place_position_relock_max_seed_translation_ &&
                crot <= depth_place_position_relock_max_seed_rotation_deg_ &&
                prior_trans <=
                    depth_place_position_relock_max_continuity_translation_;
            const bool metric_feature_relock =
                place_metric_rgbd && ci == 0 &&
                cfit <= 0.11 && coverlap >= 0.78 &&
                ctrans <= 2.25 && crot <= 10.0;
            continuity_ok =
                (prior_trans < 5.0 && prior_rot < 25.0) ||
                (strong_depth_relock &&
                 prior_trans < 6.0 && prior_rot < 35.0) ||
                strong_depth_position_relock || temporal_relock ||
                metric_feature_relock;
          }
        }
        else
          continuity_ok =
              prior_trans < (is_depth ? 5.0 : 4.0) &&
              prior_rot < (is_depth ? 25.0 : 18.0);

        const bool valid =
            std::isfinite(cfit) &&
            cfit < (is_depth ? 0.16 : 0.18) &&
            coverlap > (is_depth ? 0.32 : 0.55) &&
            ctrans < 3.0 && crot < 12.0 && continuity_ok;
        if (!valid)
        {
          if (failsafe_mode_ == "FULL_LIVO" && !is_depth)
            ROS_WARN_THROTTLE(0.35,
                "[FAILSAFE MAP] healthy candidate reject idx=%zu fit=%.3f ov=%.2f seedcorr=%.2fm/%.1fdeg prior=%.2fm/%.1fdeg verified_cont=%.2fm/%.1fdeg cont=%d",
                ci, cfit, coverlap, ctrans, crot, prior_trans, prior_rot,
                continuity_trans, continuity_rot, continuity_ok ? 1 : 0);
          else if (is_depth && (failsafe_mode_ == "VI_LIDAR_FAILED" || failsafe_mode_ == "IMU_ONLY_BRIDGE"))
            ROS_WARN_THROTTLE(0.35,
                "[FAILSAFE MAP] lidar-fail visual candidate reject idx=%zu fit=%.3f ov=%.2f seedcorr=%.2fm/%.1fdeg prior=%.2fm/%.1fdeg verified_cont=%.2fm/%.1fdeg cont=%d",
                ci, cfit, coverlap, ctrans, crot, prior_trans, prior_rot,
                continuity_trans, continuity_rot, continuity_ok ? 1 : 0);
          continue;
        }

        const double score =
            cfit +
            (is_depth ? 0.12 : 0.15) * (1.0 - coverlap) +
            (is_depth ? 0.015 : 0.012) * ctrans +
            0.035 * std::min(continuity_trans, 3.0) +
            0.0025 * std::min(continuity_rot, 12.0) +
            0.002 * static_cast<double>(ci);

        if (score < best_score)
        {
          Eigen::Matrix4f selected_pose = T_candidate;
          double selected_translation_correction = ctrans;
          if (lidar_depth_fail &&
              depth_place_max_applied_translation_ > 0.0 &&
              ctrans > depth_place_max_applied_translation_)
          {
            Eigen::Matrix4f bounded_delta = d;
            bounded_delta.block<3, 1>(0, 3) *=
                static_cast<float>(depth_place_max_applied_translation_ / ctrans);
            selected_pose = candidate_guess * bounded_delta;
            selected_translation_correction =
                depth_place_max_applied_translation_;
            ROS_INFO_THROTTLE(
                0.5,
                "[STEP3 DEPTH PLACE] bounded visual-seed GICP translation %.2fm -> %.2fm",
                ctrans, depth_place_max_applied_translation_);
          }
          if (lidar_depth_fail && depth_place_preserve_visual_orientation_)
          {
            selected_pose.block<3, 3>(0, 0) =
                candidate_guess.block<3, 3>(0, 0);
            ROS_INFO_THROTTLE(
                0.5,
                "[STEP3 DEPTH PLACE] preserving visual/predicted orientation; raw GICP dRot=%.1fdeg",
                crot);
          }

          best_score = score;
          guess = candidate_guess;
          T_wl = selected_pose;
          target = fine_tgt;
          aligned = *candidate_aligned;
          fitness = cfit;
          selected_overlap = coverlap;
          correction_translation = selected_translation_correction;
          correction_rotation =
              (lidar_depth_fail && depth_place_preserve_visual_orientation_)
                  ? 0.0 : crot;
          place_guided = true;
          registration_ok = true;

          // Candidates arrive ranked by the visual front-end.  Once the
          // highest-ranked candidate passes the SAME strict coarse+fine GICP
          // gates as the standalone matcher, stop.  Rank-2/3 are fallbacks
          // only when earlier hypotheses fail geometry.
          break;
        }
      }

      // RGB-D/LiDAR-loss keeps the conservative consecutive-confirmation gate.
      // During IMU loss, however, the validated standalone visual+LiDAR
      // pipeline accepted every visual hypothesis that independently passed
      // strict coarse+fine LiDAR GICP. Requiring a second expensive visual
      // query here stretches absolute corrections to ~5-10 s on this CPU and
      // lets no-IMU drift run away. Match the standalone behavior for
      // LV_IMU_FAILED: one visual candidate + strict LiDAR geometry is enough.
      // LiDAR-loss keeps conservative consecutive confirmation. During IMU
      // loss the visual front-end is already constrained by LiDAR-relative
      // propagation, strict coarse/fine GICP, verified-history consistency and
      // the global anchor target gate, so accept each strict verified fix.
      // Healthy and LiDAR-loss keep conservative temporal confirmation to
      // reject repeated-campus-geometry aliases.  IMU loss is the one exception:
      // its validated standalone path needs frequent LiDAR-backed global fixes,
      // and already has LiDAR-relative/history consistency gates.
      // Bootstrap needs to acquire the map frame promptly. A bootstrap
      // hypothesis has already passed visual place retrieval plus coarse/fine
      // saved-map GICP, overlap and correction gates, and there is no existing
      // global anchor to protect yet. Requiring a second expensive visual
      // query can delay initialization by many seconds on the cart CPU.
      // After bootstrap, healthy reacquisition still requires temporal
      // confirmation; IMU-fail retains its validated single-fix behavior.
      const bool healthy_bootstrap =
          failsafe_mode_ == "FULL_LIVO" && !is_depth && !have_anchor_for_place;
      const bool require_consecutive_confirmation =
          !healthy_bootstrap && !(failsafe_mode_ == "LV_IMU_FAILED" && !is_depth);
      if (registration_ok && place_guided && require_consecutive_confirmation)
      {
        bool confirmed = false;
        {
          std::lock_guard<std::mutex> lock(place_confirm_mutex_);
          bool consistent = false;
          if (place_confirm_valid_ &&
              place_confirm_source_ == source_name &&
              !place_confirm_stamp_.isZero() && !stamp.isZero())
          {
            const double dt = (stamp - place_confirm_stamp_).toSec();
            const double confirm_max_dt =
                (failsafe_mode_ == "FULL_LIVO" && !have_anchor_for_place)
                    ? place_bootstrap_max_age_ : 2.5;
            if (dt > 0.03 && dt < confirm_max_dt)
            {
              const Eigen::Matrix4f delta =
                  place_confirm_pose_.inverse() * T_wl;
              const double trans =
                  delta.block<3, 1>(0, 3).norm();
              const double rot_deg =
                  rotationAngleDeg(delta.block<3, 3>(0, 0).cast<double>());
              const double speed = trans / dt;
              const double rot_rate = rot_deg / dt;
              consistent =
                  speed < 6.5 && rot_rate < 35.0;
            }
          }

          if (!place_confirm_valid_ ||
              place_confirm_source_ != source_name || !consistent)
            place_confirm_streak_ = 1;
          else
            ++place_confirm_streak_;

          place_confirm_pose_ = T_wl;
          place_confirm_stamp_ = stamp;
          place_confirm_source_ = source_name;
          place_confirm_valid_ = true;
          confirmed = place_confirm_streak_ >= 2;
        }

        if (!confirmed)
        {
          ROS_INFO_THROTTLE(
              0.5,
              "[FAILSAFE MAP] %s candidate geometrically valid; waiting for consecutive confirmation",
              source_name.c_str());
          return false;
        }
      }

      if (registration_ok)
      {
        if (failsafe_mode_ == "LV_IMU_FAILED" && !is_depth && place_guided)
          acceptImuPlaceHistory(T_wl, stamp);
        if (place_guided)
          acceptVerifiedPlaceHistory(T_wl, stamp);
        ROS_INFO_THROTTLE(0.5,
            "[FAILSAFE MAP] PLACE-VERIFIED %s fit=%.3f overlap=%.2f corr=%.2fm/%.1fdeg",
            source_name.c_str(), fitness, selected_overlap,
            correction_translation, correction_rotation);
      }
      else
        ROS_WARN_THROTTLE(0.75,
            "[FAILSAFE MAP] %zu visual place candidates but none passed geometric verification",
            place_candidates.size());
    }

    // During degraded operation the short-term frontend already provides
    // relative motion. Blind local map ICP is NOT a safe absolute authority on
    // repeated campus geometry. Require visual place + current geometry for
    // both failure cases:
    //   IMU fail   -> visual place + LiDAR GICP
    //   LiDAR fail -> visual place + depth GICP
    const bool lidar_imu_tracker_seeded =
        (failsafe_mode_ == "FULL_LIVO" ||
         failsafe_mode_ == "LIO_CAMERA_FAILED") && !is_depth &&
        trackerHasPoseFor(source_name);
    const bool require_verified_global_seed =
        ((failsafe_mode_ == "FULL_LIVO" ||
          failsafe_mode_ == "LIO_CAMERA_FAILED") &&
         !is_depth && !lidar_imu_tracker_seeded) ||
        (failsafe_mode_ == "LV_IMU_FAILED" && !is_depth) ||
        // LiDAR failure: NEVER let blind depth-to-map ICP become an absolute
        // authority on repeated campus geometry. RGB-D carries relative motion;
        // absolute depth corrections require a visual-place seed plus strict
        // coarse/fine GICP verification.
        ((failsafe_mode_ == "VI_LIDAR_FAILED" ||
          failsafe_mode_ == "IMU_ONLY_BRIDGE") && is_depth &&
         !depth_tracker_recent);
    if (!registration_ok && require_verified_global_seed)
    {
      ROS_WARN_THROTTLE(0.75,
          "[FAILSAFE MAP] %s fail: no verified visual place; skipping blind absolute map correction",
          failsafe_mode_ == "LV_IMU_FAILED" ? "IMU" : "LiDAR");
      return false;
    }

    // Other modes may retain the strict local tracker as a fallback.
    if (!registration_ok)
    {
      guess = trackingGuess(T_current_lidar, source, source_name);
      target = localTarget(guess.block<3, 1>(0, 3), -1.0, -1.0, use_visual_target);
      if (failsafe_mode_ == "FULL_LIVO" && !allow_place)
        ROS_INFO_THROTTLE(1.0,
            "[FAILSAFE MAP] healthy tracker attempt target=%zu guess=(%.1f,%.1f,%.1f)",
            target ? target->size() : 0u, guess(0,3), guess(1,3), guess(2,3));
      if (!target || target->size() < 80)
      {
        if (failsafe_mode_ == "FULL_LIVO" && !allow_place)
          ROS_WARN_THROTTLE(1.0, "[FAILSAFE MAP] healthy tracker fail: sparse target %zu",
                            target ? target->size() : 0u);
        return false;
      }

      CloudT::Ptr tracking_target = target;
      const bool healthy_local_tracking =
          (failsafe_mode_ == "FULL_LIVO" ||
           failsafe_mode_ == "LIO_CAMERA_FAILED") &&
          !is_depth && lidar_imu_tracker_seeded && !allow_place;
      const bool depth_local_tracking =
          lidar_depth_fail && depth_tracker_recent && !place_guided;
      if (healthy_local_tracking)
      {
        tracking_target.reset(new CloudT(*target));
        downsampleInPlace(tracking_target, 0.45);
      }
      else if (depth_local_tracking)
      {
        tracking_target.reset(new CloudT(*target));
        downsampleInPlace(tracking_target, 0.35);
      }

      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
      icp.setInputSource(source);
      icp.setInputTarget(tracking_target);
      icp.setMaximumIterations(healthy_local_tracking ? 18 :
                               (depth_local_tracking ? 25 : icp_max_iterations_));
      icp.setMaxCorrespondenceDistance(healthy_local_tracking ? 1.2 :
                                       (depth_local_tracking ? 0.90 : icp_max_correspondence_));
      icp.setTransformationEpsilon(1.0e-5);
      icp.setEuclideanFitnessEpsilon(1.0e-4);
      icp.setCorrespondenceRandomness((healthy_local_tracking || depth_local_tracking) ? 15 : 20);
      icp.align(aligned, guess);
      if (!icp.hasConverged())
      {
        if (failsafe_mode_ == "FULL_LIVO" && !allow_place)
          ROS_WARN_THROTTLE(1.0, "[FAILSAFE MAP] healthy tracker fail: GICP did not converge");
        return false;
      }

      fitness = icp.getFitnessScore(depth_local_tracking ? 0.90 : icp_max_correspondence_);
      T_wl = icp.getFinalTransformation();
      const Eigen::Matrix4f delta = guess.inverse() * T_wl;
      correction_translation = delta.block<3, 1>(0, 3).norm();
      correction_rotation =
          rotationAngleDeg(delta.block<3, 3>(0, 0).cast<double>());

      const bool bootstrap = !trackerHasPoseFor(source_name);
      const double fitness_gate =
          is_depth ? depth_map_fitness_threshold_
                   : (bootstrap ? std::min(0.35, icp_fitness_threshold_)
                                : icp_fitness_threshold_);
      const double translation_gate =
          is_depth ? (bootstrap ? depth_bootstrap_max_translation_
                                : depth_map_max_translation_)
                   : (bootstrap ? std::max(3.0, max_correction_translation_)
                                : max_correction_translation_);
      const double rotation_gate =
          is_depth ? (bootstrap ? depth_bootstrap_max_rotation_deg_
                                : depth_map_max_rotation_deg_)
                   : (bootstrap ? std::max(15.0, max_correction_rotation_deg_)
                                : max_correction_rotation_deg_);
      double local_overlap = 1.0;
      bool healthy_tracking_ok = true;
      bool depth_local_tracking_ok = true;
      if ((failsafe_mode_ == "FULL_LIVO" ||
           failsafe_mode_ == "LIO_CAMERA_FAILED") &&
          !is_depth && lidar_imu_tracker_seeded)
      {
        CloudT::Ptr aligned_ptr(new CloudT(aligned));
        local_overlap = overlapRatio(aligned_ptr, tracking_target, 0.80);
        // Healthy FAST-LIVO2 already provides a smooth, observable heading.
        // The saved-map GICP is used here as a *position drift* constraint,
        // not as a second orientation estimator. Repetitive campus geometry
        // often gives an excellent positional alignment while the local GICP
        // optimum differs by ~8-10 deg in yaw. Rejecting the whole match on
        // that yaw alone starves global position updates for 10+ seconds.
        // Keep the geometry gates strong (fitness/overlap/translation), allow
        // only a moderate orientation disagreement for acceptance, and later
        // apply only a tiny bounded yaw innovation to the map anchor.
        healthy_tracking_ok =
            fitness < 0.20 && local_overlap > 0.65 &&
            correction_translation < 3.0 && correction_rotation < 12.0;
      }
      if (depth_local_tracking)
      {
        CloudT::Ptr aligned_ptr(new CloudT(aligned));
        local_overlap = overlapRatio(aligned_ptr, tracking_target, 0.55);
        depth_local_tracking_ok =
            std::isfinite(fitness) &&
            fitness <= depth_local_fitness_threshold_ &&
            local_overlap >= depth_local_overlap_threshold_;
      }
      const double effective_translation_gate =
          healthy_local_tracking ? 3.0 :
          (depth_local_tracking ? std::min(depth_local_max_translation_, depth_map_max_translation_)
                                : translation_gate);
      const double effective_rotation_gate =
          healthy_local_tracking ? 12.0 :
          (depth_local_tracking ? std::min(depth_local_max_rotation_deg_, depth_map_max_rotation_deg_)
                                : rotation_gate);
      const double effective_fitness_gate =
          depth_local_tracking ? std::min(depth_local_fitness_threshold_, fitness_gate)
                               : fitness_gate;
      if (!std::isfinite(fitness) || fitness > effective_fitness_gate ||
          correction_translation > effective_translation_gate ||
          correction_rotation > effective_rotation_gate || !healthy_tracking_ok ||
          !depth_local_tracking_ok)
      {
        ROS_WARN_THROTTLE(1.0,
            "[FAILSAFE MAP] %s ICP rejected%s fitness=%.3f ov=%.2f corr=%.2fm/%.1fdeg",
            source_name.c_str(), bootstrap ? " (bootstrap)" : "",
            fitness, local_overlap, correction_translation, correction_rotation);
        return false;
      }

      if (depth_local_tracking)
        ROS_INFO_THROTTLE(0.5,
            "[STEP3 DEPTH LOCAL] geometrically verified local RGB-D voxel fix fit=%.3f ov=%.2f corr=%.2fm/%.1fdeg age=%.2fs",
            fitness, local_overlap, correction_translation, correction_rotation,
            depth_tracker_age_sec);

      // During Step-3 LiDAR failover, depth-to-visual-map GICP is an absolute
      // POSITION measurement, not a second heading estimator. Repeated visual
      // geometry can give an excellent fitness while rotating the local optimum
      // by several degrees. At a long map lever arm that destroys the next
      // prediction. Keep the translation refined by GICP, but preserve the
      // causal RGB-D/FAST predicted orientation. Step-3 estimates slow global
      // yaw from the multi-frame trajectory itself.
      if (depth_failover_only_ && is_depth &&
          (failsafe_mode_ == "VI_LIDAR_FAILED" ||
           failsafe_mode_ == "IMU_ONLY_BRIDGE"))
      {
        const double raw_rot = correction_rotation;
        T_wl.block<3, 3>(0, 0) = guess.block<3, 3>(0, 0);
        correction_rotation = 0.0;
        ROS_INFO_THROTTLE(0.5,
            "[STEP3 DEPTH LOCAL] accepted translation-only fit=%.3f dxyz=%.2fm raw_drot=%.1fdeg",
            fitness, correction_translation, raw_rot);
      }
      registration_ok = true;
    }

    // GICP on the full saved map can take several hundred milliseconds.
    // The estimator keeps moving while registration is running, so T_wl is a
    // correction for the odometry snapshot taken at callback entry. Propagate
    // that corrected pose to the latest estimator time using only the relative
    // odometry motion that occurred during GICP. This prevents a geometrically
    // valid registration from being discarded as stale or applied to an old
    // vehicle pose.
    Eigen::Matrix4f T_map_lidar_publish = T_wl;
    ros::Time correction_stamp = stamp;
    Eigen::Matrix3d latest_Rwi;
    Eigen::Vector3d latest_twi;
    std::string latest_mode;
    ros::Time latest_stamp;
    // Compensate expensive registration latency with the SAME short-term
    // sensor that owns degraded motion. Using no-IMU odometry here was the
    // source of 12-25 m / 40-100 deg "corrections" from otherwise-good GICP.
    // Likewise, RGB-D history is safer than a stale depth tracker during
    // LiDAR loss. Only fall back to estimator odometry if sensor history is
    // unavailable.
    bool latency_compensated = false;
    if (snapshotState(latest_Rwi, latest_twi, latest_mode, latest_stamp))
    {
      if (failsafe_mode_ == "LV_IMU_FAILED" && !is_depth)
      {
        Eigen::Matrix4f propagated;
        if (propagatePlaceCandidateWithLidarHistory(
                T_wl, stamp, latest_stamp, propagated))
        {
          T_map_lidar_publish = propagated;
          correction_stamp = latest_stamp;
          latency_compensated = true;
        }
      }
      else if ((failsafe_mode_ == "VI_LIDAR_FAILED" ||
                failsafe_mode_ == "IMU_ONLY_BRIDGE") && is_depth)
      {
        Eigen::Matrix4f propagated;
        if (propagatePlaceCandidateWithRgbdHistory(
                T_wl, stamp, latest_stamp, propagated))
        {
          T_map_lidar_publish = propagated;
          correction_stamp = latest_stamp;
          latency_compensated = true;
        }
      }

      if (!latency_compensated)
      {
        const Eigen::Matrix4f T_current_lidar_latest =
            currentWorldLidar(latest_Rwi, latest_twi);
        const Eigen::Matrix4f T_source_to_latest =
            T_current_lidar.inverse() * T_current_lidar_latest;
        const double latency_motion_translation =
            T_source_to_latest.block<3, 1>(0, 3).norm();
        const double latency_motion_rotation =
            rotationAngleDeg(T_source_to_latest.block<3, 3>(0, 0).cast<double>());
        if (latency_motion_translation < 3.0 &&
            latency_motion_rotation < 15.0)
        {
          T_map_lidar_publish = T_wl * T_source_to_latest;
          correction_stamp = latest_stamp.isZero() ? ros::Time::now() : latest_stamp;
          latency_compensated = true;
          ROS_INFO_THROTTLE(1.0,
              "[FAILSAFE MAP] estimator latency fallback %.2fm/%.1fdeg",
              latency_motion_translation, latency_motion_rotation);
        }
      }
    }
    // If no newer odometry snapshot is available, keep the source timestamp;
    // LIVMapper's age gate will reject it rather than applying an unpropagated
    // stale global pose.

    // In v3 the persistent FAST-LIVO2 ESIKF owns LOCAL motion while the saved
    // PCD owns GLOBAL alignment. During a sensor outage, do not force an
    // absolute map pose into local ESIKF coordinates through an anchor that is
    // itself being corrected. Instead update map<-camera_init smoothly. This
    // avoids the 12-25 m / 40-100 deg local-pose pulls observed during IMU
    // failure and directly keeps /pose + map->base_link usable by Lanelet.
    const bool imu_esikf_map_correction =
        failsafe_mode_ == "LV_IMU_FAILED" &&
        imu_failure_esikf_map_correction_enable_;
    const bool healthy_anchor_update =
        (failsafe_mode_ == "FULL_LIVO" ||
         failsafe_mode_ == "LIO_CAMERA_FAILED") &&
        !is_depth && registration_ok;
    const bool depth_measurement_frontend_only =
        depth_failover_only_ && is_depth &&
        (failsafe_mode_ == "VI_LIDAR_FAILED" ||
         failsafe_mode_ == "IMU_ONLY_BRIDGE");
    const bool degraded_anchor_update =
        degraded_global_anchor_only_ && !imu_esikf_map_correction &&
        !depth_measurement_frontend_only &&
        (failsafe_mode_ == "LV_IMU_FAILED" ||
         failsafe_mode_ == "VI_LIDAR_FAILED" ||
         failsafe_mode_ == "IMU_ONLY_BRIDGE");
    if (healthy_anchor_update || degraded_anchor_update)
    {
      // Keep both sides of map<-current = (map<-lidar) * (current<-lidar)^-1
      // at the SAME timestamp. If expensive GICP could not propagate the
      // registered map pose to latest estimator time, pairing an old map pose
      // with the latest local pose creates a fictitious multi-metre anchor
      // jump. Use latest local pose only when latency compensation succeeded.
      Eigen::Matrix4f T_current_lidar_for_anchor = T_current_lidar;
      if (latency_compensated && !latest_stamp.isZero())
        T_current_lidar_for_anchor = currentWorldLidar(latest_Rwi, latest_twi);
      const Eigen::Matrix4f candidate_anchor =
          T_map_lidar_publish * T_current_lidar_for_anchor.inverse();

      // Diagnostic-only trace for healthy saved-map fixes.  This intentionally
      // does not change estimator or anchor behaviour; it records the exact
      // transform chain used to turn an accepted map<-LiDAR measurement into
      // the queued map<-local anchor target so offline analysis can distinguish
      // bad measurements from bad anchor application/smoothing.
      if (healthy_anchor_update && place_guided)
      {
        Eigen::Matrix4f T_body_lidar = Eigen::Matrix4f::Identity();
        T_body_lidar.block<3, 3>(0, 0) = extR_.cast<float>();
        T_body_lidar.block<3, 1>(0, 3) = extT_.cast<float>();
        const Eigen::Matrix4f T_current_body_for_anchor =
            T_current_lidar_for_anchor * T_body_lidar.inverse();

        Eigen::Matrix4f anchor_before = Eigen::Matrix4f::Identity();
        bool anchor_before_valid = false;
        {
          std::lock_guard<std::mutex> lock(anchor_mutex_);
          anchor_before_valid = anchor_valid_;
          if (anchor_valid_) anchor_before = anchor_T_map_current_;
        }
        const Eigen::Matrix4f T_map_body_before =
            anchor_before * T_current_body_for_anchor;

        const Eigen::Quaternionf q_red(T_map_lidar_publish.block<3, 3>(0, 0));
        const Eigen::Quaternionf q_local(T_current_body_for_anchor.block<3, 3>(0, 0));
        const Eigen::Quaternionf q_blue(T_map_body_before.block<3, 3>(0, 0));
        const Eigen::Quaternionf q_ab(anchor_before.block<3, 3>(0, 0));
        const Eigen::Quaternionf q_target(candidate_anchor.block<3, 3>(0, 0));
        const ros::Time anchor_stamp =
            (latency_compensated && !latest_stamp.isZero()) ? latest_stamp : stamp;

        ROS_INFO(
            "[ANCHOR DIAG PRE] src=%.9f app=%.9f latency_comp=%d anchor_valid=%d "
            "red_lidar_p=%.6f,%.6f,%.6f red_lidar_q=%.7f,%.7f,%.7f,%.7f "
            "local_body_p=%.6f,%.6f,%.6f local_body_q=%.7f,%.7f,%.7f,%.7f "
            "blue_before_p=%.6f,%.6f,%.6f blue_before_q=%.7f,%.7f,%.7f,%.7f "
            "anchor_before_p=%.6f,%.6f,%.6f anchor_before_q=%.7f,%.7f,%.7f,%.7f "
            "anchor_target_p=%.6f,%.6f,%.6f anchor_target_q=%.7f,%.7f,%.7f,%.7f",
            stamp.toSec(), anchor_stamp.toSec(), static_cast<int>(latency_compensated),
            static_cast<int>(anchor_before_valid),
            T_map_lidar_publish(0, 3), T_map_lidar_publish(1, 3), T_map_lidar_publish(2, 3),
            q_red.x(), q_red.y(), q_red.z(), q_red.w(),
            T_current_body_for_anchor(0, 3), T_current_body_for_anchor(1, 3),
            T_current_body_for_anchor(2, 3), q_local.x(), q_local.y(), q_local.z(), q_local.w(),
            T_map_body_before(0, 3), T_map_body_before(1, 3), T_map_body_before(2, 3),
            q_blue.x(), q_blue.y(), q_blue.z(), q_blue.w(),
            anchor_before(0, 3), anchor_before(1, 3), anchor_before(2, 3),
            q_ab.x(), q_ab.y(), q_ab.z(), q_ab.w(),
            candidate_anchor(0, 3), candidate_anchor(1, 3), candidate_anchor(2, 3),
            q_target.x(), q_target.y(), q_target.z(), q_target.w());
      }
      bool rolling_global_accept = true;
      bool used_incremental_global_correction = false;
      if (healthy_anchor_update)
      {
        const std::string rolling_source =
            place_guided ? "healthy_place_reacquisition" : "healthy_rolling_lidar_map";
        used_incremental_global_correction = true;
        rolling_global_accept =
            applyIncrementalMapCorrection(T_map_lidar_publish,
                                          T_current_lidar_for_anchor,
                                          rolling_source, place_guided);
      }
      else if (degraded_anchor_update && is_depth && use_visual_target)
      {
        used_incremental_global_correction = true;
        rolling_global_accept =
            applyIncrementalMapCorrection(T_map_lidar_publish,
                                          T_current_lidar_for_anchor,
                                          "lidar_fail_rolling_rgbd_voxel",
                                          place_guided);
      }
      else
      {
        queueAnchorTarget(candidate_anchor,
                          place_guided ? "degraded_place_verified_map"
                                       : "degraded_saved_map");
      }

      if (healthy_anchor_update && place_guided)
      {
        Eigen::Matrix4f anchor_after = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f target_after = Eigen::Matrix4f::Identity();
        bool anchor_after_valid = false;
        bool target_after_valid = false;
        {
          std::lock_guard<std::mutex> lock(anchor_mutex_);
          anchor_after_valid = anchor_valid_;
          target_after_valid = anchor_target_valid_;
          if (anchor_valid_) anchor_after = anchor_T_map_current_;
          if (anchor_target_valid_) target_after = anchor_target_T_map_current_;
        }
        const Eigen::Quaternionf q_after(anchor_after.block<3, 3>(0, 0));
        const Eigen::Quaternionf q_tafter(target_after.block<3, 3>(0, 0));
        ROS_INFO(
            "[ANCHOR DIAG POST] anchor_valid=%d target_valid=%d "
            "anchor_current_p=%.6f,%.6f,%.6f anchor_current_q=%.7f,%.7f,%.7f,%.7f "
            "anchor_target_p=%.6f,%.6f,%.6f anchor_target_q=%.7f,%.7f,%.7f,%.7f",
            static_cast<int>(anchor_after_valid), static_cast<int>(target_after_valid),
            anchor_after(0, 3), anchor_after(1, 3), anchor_after(2, 3),
            q_after.x(), q_after.y(), q_after.z(), q_after.w(),
            target_after(0, 3), target_after(1, 3), target_after(2, 3),
            q_tafter.x(), q_tafter.y(), q_tafter.z(), q_tafter.w());
      }

      // A locally plausible GICP solution can still be a repeated-geometry
      // alias globally. If the rolling global-consistency gate rejects it,
      // do NOT let that rejected pose become the tracker's next seed. Keeping
      // the last trusted tracker pose lets the next rolling query recover or
      // the caller fall back to visual-place reacquisition.
      if (used_incremental_global_correction && !rolling_global_accept)
      {
        ROS_WARN_THROTTLE(
            0.5,
            "[ROLLING MAP] globally rejected registration; tracker state preserved");
        return false;
      }
    }

    // Convert every accepted saved-map pose into the existing FAST-LIVO2
    // frame only when the legacy absolute-ESIKF correction path is requested.
    Eigen::Matrix4f T_corrected_current_lidar = T_map_lidar_publish;
    bool healthy_anchor_applied = false;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      if (anchor_valid_)
      {
        T_corrected_current_lidar =
            anchor_T_map_current_.inverse() * T_map_lidar_publish;
        healthy_anchor_applied = true;
      }
    }

    const Eigen::Matrix3d Rwl =
        T_corrected_current_lidar.block<3, 3>(0, 0).cast<double>();
    const Eigen::Vector3d twl =
        T_corrected_current_lidar.block<3, 1>(0, 3).cast<double>();
    const Eigen::Matrix3d corrected_Rwi = Rwl * extR_.transpose();
    const Eigen::Vector3d corrected_twi = twl - corrected_Rwi * extT_;
    Eigen::Quaterniond q(corrected_Rwi);
    q.normalize();

    // Keep the geometric tracker tied to the source scan itself. Only the
    // correction sent to the live ESIKF is latency-compensated.
    acceptTrackedPose(T_wl, T_current_lidar, source, source_name, stamp);

    // In Step-3 depth-failover-only mode this node is a measurement frontend,
    // not another estimator. Publish the geometrically verified map<-body pose
    // and let the single Step-3 map<-FAST anchor decide how much correction to
    // apply. Never inject the same depth registration directly into the ESIKF.
    if (depth_failover_only_ && is_depth)
    {
      const Eigen::Matrix4f T_map_body = bodyFromMapLidar(T_map_lidar_publish);
      Eigen::Quaternionf q_map_body(T_map_body.block<3, 3>(0, 0));
      q_map_body.normalize();
      geometry_msgs::PoseStamped out;
      out.header.stamp = correction_stamp.isZero() ? stamp : correction_stamp;
      // Independent absolute observation: visual-place seeded and geometrically
      // verified against the saved visual/depth map.
      out.header.frame_id = "map_depth_gicp";
      out.pose.position.x = T_map_body(0, 3);
      out.pose.position.y = T_map_body(1, 3);
      out.pose.position.z = T_map_body(2, 3);
      out.pose.orientation.x = q_map_body.x();
      out.pose.orientation.y = q_map_body.y();
      out.pose.orientation.z = q_map_body.z();
      out.pose.orientation.w = q_map_body.w();
      depth_failover_pose_pub_.publish(out);
      ROS_INFO_THROTTLE(0.5,
          "[STEP3 DEPTH MAP] verified map-body pose fitness=%.3f xyz=[%.2f %.2f %.2f]",
          fitness, T_map_body(0, 3), T_map_body(1, 3), T_map_body(2, 3));
    }

    // IMPORTANT v2: this saved-map measurement corrects the persistent ESIKF.
    // Do NOT also retarget map<-camera_init here. Doing both applies the same
    // absolute correction twice (once to the ESIKF and once to its parent
    // frame), which is exactly the source of outage/recovery map-pose jumps.
    // The global anchor is established/refined only in healthy localization;
    // degraded localization changes the vehicle state, not the map frame.

    geometry_msgs::PoseWithCovarianceStamped correction;
    correction.header.stamp =
        correction_stamp.isZero() ? ros::Time::now() : correction_stamp;
    if (place_guided)
      correction.header.frame_id =
          healthy_anchor_applied
              ? (is_depth ? "depth_place_verified_esikf"
                          : "lidar_place_verified_esikf")
              : (is_depth ? "depth_place_verified_map"
                          : "lidar_place_verified_map");
    else if (healthy_anchor_applied && source_name == "depth_saved_map_icp")
      correction.header.frame_id = "depth_saved_map_esikf";
    else if (healthy_anchor_applied && source_name == "lidar_saved_map_icp")
      correction.header.frame_id = "lidar_saved_map_esikf";
    else if (healthy_anchor_applied && source_name == "lidar_recovery_saved_map_icp")
      correction.header.frame_id = "lidar_recovery_saved_map_esikf";
    else
      correction.header.frame_id = source_name;
    correction.pose.pose.position.x = corrected_twi.x();
    correction.pose.pose.position.y = corrected_twi.y();
    correction.pose.pose.position.z = corrected_twi.z();
    correction.pose.pose.orientation.x = q.x();
    correction.pose.pose.orientation.y = q.y();
    correction.pose.pose.orientation.z = q.z();
    correction.pose.pose.orientation.w = q.w();
    correction.pose.covariance[0] = fitness;
    correction.pose.covariance[7] = fitness;
    correction.pose.covariance[14] = fitness;

    // Recovery handshake: in global-anchor-only mode the returning LiDAR can
    // be geometrically re-locked to the saved PCD without applying that same
    // absolute pose into the ESIKF.  LIVMapper still needs to know that this
    // geometric lock succeeded before it is allowed to rejoin FULL_LIVO. Send
    // a lock-only marker through the existing correction queue; LIVMapper
    // consumes the marker without changing pose/state.
    const bool lidar_recovery_lock_only =
        degraded_global_anchor_only_ &&
        source_name == "lidar_recovery_saved_map_icp";
    if (lidar_recovery_lock_only)
    {
      correction.header.frame_id = "lidar_recovery_lock_only";
      correction_pub_.publish(correction);
      ROS_WARN_THROTTLE(0.5,
          "[FAILSAFE MAP] Returning LiDAR saved-map geometry verified; sent lock-only handshake");
    }
    else if (!degraded_global_anchor_only_ || imu_esikf_map_correction)
      correction_pub_.publish(correction);

    if (legacy_fallback_map_pose_enable_ &&
        (failsafe_mode_ == "VI_LIDAR_FAILED" ||
        failsafe_mode_ == "IMU_ONLY_BRIDGE" ||
        failsafe_mode_ == "LV_IMU_FAILED"))
      queueFallbackAbsolute(bodyFromMapLidar(T_map_lidar_publish), source_name,
                            place_guided);

    sensor_msgs::PointCloud2 dbg;
    pcl::toROSMsg(aligned, dbg);
    dbg.header.stamp = ros::Time::now();
    dbg.header.frame_id = map_frame_;
    aligned_pub_.publish(dbg);
    std_msgs::String status;
    status.data = (place_guided ? "PLACE_VERIFIED " : "") + source_name +
                  " accepted fitness=" + std::to_string(fitness) +
                  " correction_m=" + std::to_string(correction_translation);
    status_pub_.publish(status);
    publishLocalizationStatus(place_guided ? "place_verified_map_fix" : "saved_map_fix");
    ROS_INFO_THROTTLE(0.5,
        "[FAILSAFE MAP] %s%s ICP accepted fitness=%.3f corr=%.2fm/%.1fdeg points=%zu",
        place_guided ? "PLACE-VERIFIED " : "", source_name.c_str(),
        fitness, correction_translation, correction_rotation, source->size());
    if (place_guided && failsafe_mode_ == "FULL_LIVO")
    {
      const Eigen::Vector3f mp = T_map_lidar_publish.block<3, 1>(0, 3);
      ROS_INFO("[FAILSAFE MAP MEAS] stamp=%.9f x=%.6f y=%.6f z=%.6f fit=%.6f overlap=%.4f",
               stamp.toSec(), mp.x(), mp.y(), mp.z(), fitness, selected_overlap);
    }
    return true;
  }

  bool bufferedLidarNear(const ros::Time &stamp,
                         sensor_msgs::PointCloud2::ConstPtr &msg_out,
                         ros::Time &actual_stamp)
  {
    if (stamp.isZero()) return false;
    std::lock_guard<std::mutex> lock(lidar_history_mutex_);
    if (lidar_msg_history_.empty()) return false;
    double best = 1e9;
    sensor_msgs::PointCloud2::ConstPtr best_msg;
    ros::Time best_stamp;
    for (const auto &entry : lidar_msg_history_)
    {
      const double d = std::abs((entry.first - stamp).toSec());
      if (d < best)
      {
        best = d;
        best_msg = entry.second;
        best_stamp = entry.first;
      }
    }
    if (!best_msg || best > 0.30) return false;
    msg_out = best_msg;
    actual_stamp = best_stamp;
    return true;
  }

  bool localBodyPoseNear(const ros::Time &stamp,
                         Eigen::Matrix3d &Rwi,
                         Eigen::Vector3d &twi)
  {
    if (stamp.isZero()) return false;
    Eigen::Matrix4f T_local_lidar = Eigen::Matrix4f::Identity();
    double best = 1e9;
    {
      std::lock_guard<std::mutex> lock(odom_history_mutex_);
      if (odom_lidar_history_.empty()) return false;
      for (const auto &entry : odom_lidar_history_)
      {
        const double d = std::abs((entry.first - stamp).toSec());
        if (d < best)
        {
          best = d;
          T_local_lidar = entry.second;
        }
      }
    }
    if (best > 0.30) return false;
    Eigen::Matrix4f T_body_lidar = Eigen::Matrix4f::Identity();
    T_body_lidar.block<3, 3>(0, 0) = extR_.cast<float>();
    T_body_lidar.block<3, 1>(0, 3) = extT_.cast<float>();
    const Eigen::Matrix4f T_local_body = T_local_lidar * T_body_lidar.inverse();
    Rwi = T_local_body.block<3, 3>(0, 0).cast<double>();
    twi = T_local_body.block<3, 1>(0, 3).cast<double>();
    return true;
  }

  CloudT::Ptr buildRollingLidarCloud(const ros::Time &reference_stamp,
                                    const Eigen::Matrix3d &Rwi_ref,
                                    const Eigen::Vector3d &twi_ref)
  {
    CloudT::Ptr merged(new CloudT);
    std::vector<std::pair<ros::Time, sensor_msgs::PointCloud2::ConstPtr>> selected;
    {
      std::lock_guard<std::mutex> lock(lidar_history_mutex_);
      for (auto it = lidar_msg_history_.rbegin(); it != lidar_msg_history_.rend(); ++it)
      {
        if (it->first > reference_stamp + ros::Duration(0.03)) continue;
        if ((reference_stamp - it->first).toSec() > rolling_lidar_window_sec_) break;
        selected.push_back(*it);
        if (static_cast<int>(selected.size()) >= std::max(1, rolling_lidar_max_scans_)) break;
      }
    }
    const Eigen::Matrix4f T_ref = currentWorldLidar(Rwi_ref, twi_ref);
    for (auto it = selected.rbegin(); it != selected.rend(); ++it)
    {
      Eigen::Matrix3d Ri; Eigen::Vector3d ti;
      if (!localBodyPoseNear(it->first, Ri, ti)) continue;
      const Eigen::Matrix4f Ti = currentWorldLidar(Ri, ti);
      CloudT c, ct;
      pcl::fromROSMsg(*it->second, c);
      if (c.empty()) continue;
      pcl::transformPointCloud(c, ct, T_ref.inverse() * Ti);
      merged->points.insert(merged->points.end(), ct.points.begin(), ct.points.end());
    }
    merged->width = merged->size(); merged->height = 1;
    downsampleInPlace(merged, 0.25);
    return merged;
  }

  CloudT::Ptr buildRollingDepthCloud(const ros::Time &reference_stamp,
                                    const Eigen::Matrix3d &Rwi_ref,
                                    const Eigen::Vector3d &twi_ref)
  {
    CloudT::Ptr merged(new CloudT);
    std::vector<std::pair<ros::Time, CloudT::Ptr>> selected;
    {
      std::lock_guard<std::mutex> lock(depth_history_mutex_);
      for (auto it = depth_cloud_history_.rbegin(); it != depth_cloud_history_.rend(); ++it)
      {
        if (it->first > reference_stamp + ros::Duration(0.03)) continue;
        if ((reference_stamp - it->first).toSec() > rolling_depth_window_sec_) break;
        selected.push_back(*it);
        if (static_cast<int>(selected.size()) >= std::max(1, rolling_depth_max_scans_)) break;
      }
    }
    const Eigen::Matrix4f T_ref = currentWorldLidar(Rwi_ref, twi_ref);
    for (auto it = selected.rbegin(); it != selected.rend(); ++it)
    {
      Eigen::Matrix3d Ri; Eigen::Vector3d ti;
      if (!localBodyPoseNear(it->first, Ri, ti)) continue;
      const Eigen::Matrix4f Ti = currentWorldLidar(Ri, ti);
      CloudT ct;
      pcl::transformPointCloud(*it->second, ct, T_ref.inverse() * Ti);
      merged->points.insert(merged->points.end(), ct.points.begin(), ct.points.end());
    }
    merged->width = merged->size(); merged->height = 1;
    downsampleInPlace(merged, 0.18);
    return merged;
  }

  void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    if (!msg) return;
    const ros::Time lidar_stamp =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    {
      std::lock_guard<std::mutex> lock(lidar_history_mutex_);
      lidar_msg_history_.emplace_back(lidar_stamp, msg);
      while (!lidar_msg_history_.empty() &&
             (lidar_stamp - lidar_msg_history_.front().first).toSec() > 8.0)
        lidar_msg_history_.pop_front();
      while (lidar_msg_history_.size() > 160)
        lidar_msg_history_.pop_front();
    }

    Eigen::Matrix3d Rwi;
    Eigen::Vector3d twi;
    std::string mode;
    ros::Time odom_stamp;
    if (!snapshotState(Rwi, twi, mode, odom_stamp)) return;
    const ros::Time now = ros::Time::now();
    ros::Time mode_enter_stamp;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      mode_enter_stamp = current_mode_enter_stamp_;
    }

    // A heavy GICP callback can leave a few pre-failure LiDAR messages queued.
    // Once health has declared LiDAR failed, those old scans must never be
    // interpreted as live recovery geometry. Their sensor stamp predates the
    // mode transition even if the callback executes seconds later.
    if ((mode == "VI_LIDAR_FAILED" || mode == "IMU_ONLY_BRIDGE") &&
        !mode_enter_stamp.isZero() &&
        lidar_stamp + ros::Duration(0.02) < mode_enter_stamp)
    {
      ROS_WARN_THROTTLE(1.0,
          "[FAILSAFE MAP] dropping queued pre-failure LiDAR scan (age across mode transition %.3fs)",
          (mode_enter_stamp - lidar_stamp).toSec());
      return;
    }

    if (mode == "FULL_LIVO")
    {
      if (!use_online_anchor_) return;
      if (!anchor_refinement_resume_stamp_.isZero() &&
          now < anchor_refinement_resume_stamp_)
      {
        ROS_INFO_THROTTLE(0.5,
            "[FAILSAFE MAP] Holding map anchor during recovery settle window");
        return;
      }
      if (!last_anchor_localize_.isZero() &&
          (now - last_anchor_localize_).toSec() < 1.0 / std::max(0.05, anchor_localization_hz_))
        return;
      // Bootstrap globally with visual place + LiDAR geometry once. After that,
      // FAST-LIVO2 remains the motion authority and a short rolling LiDAR window
      // is registered near the predicted saved-map pose at ~1 Hz. Each accepted
      // match applies only a bounded current-pose correction; healthy no longer
      // chases a full SE(3) parent-anchor target from isolated place fixes.
      std::vector<Eigen::Matrix4f> healthy_place_candidates;
      ros::Time healthy_place_stamp;
      bool have_anchor = false;
      {
        std::lock_guard<std::mutex> lock(anchor_mutex_);
        have_anchor = anchor_valid_;
      }
      if (have_anchor && trackerHasPoseFor("lidar_saved_map_icp"))
      {
        Eigen::Matrix3d source_Rwi; Eigen::Vector3d source_twi;
        if (localBodyPoseNear(lidar_stamp, source_Rwi, source_twi))
        {
          CloudT::Ptr rolling = buildRollingLidarCloud(lidar_stamp, source_Rwi, source_twi);
          if (rolling && rolling->size() >= 100)
          {
            last_anchor_localize_ = now;
            if (registerToMap(rolling, source_Rwi, source_twi,
                              "lidar_saved_map_icp", lidar_stamp, false))
              return;
          }
        }
        // A failed local rolling match falls through to visual-place
        // reacquisition rather than recursively trusting a drifting tracker.
      }
      const double healthy_place_max_age =
          have_anchor ? -1.0 : place_bootstrap_max_age_;
      if (!snapshotPlaceCandidates(healthy_place_candidates, &healthy_place_stamp,
                                   nullptr, healthy_place_max_age))
      {
        ROS_INFO_THROTTLE(0.5,
            "[FAILSAFE MAP BOOT] healthy waiting for usable place snapshot anchor=%d",
            have_anchor ? 1 : 0);
        return;
      }
      ROS_INFO_THROTTLE(0.5,
          "[FAILSAFE MAP BOOT] healthy place snapshot accepted anchor=%d stamp=%.3f age=%.3f",
          have_anchor ? 1 : 0, healthy_place_stamp.toSec(),
          std::abs((now - healthy_place_stamp).toSec()));
      if (!last_healthy_place_attempt_stamp_.isZero() &&
          healthy_place_stamp <= last_healthy_place_attempt_stamp_)
        return;

      sensor_msgs::PointCloud2::ConstPtr buffered_msg;
      ros::Time buffered_stamp;
      Eigen::Matrix3d source_Rwi;
      Eigen::Vector3d source_twi;
      if (!bufferedLidarNear(healthy_place_stamp, buffered_msg, buffered_stamp) ||
          !localBodyPoseNear(buffered_stamp, source_Rwi, source_twi))
      {
        ROS_WARN_THROTTLE(1.0,
            "[FAILSAFE MAP] healthy exact-time verification waiting for buffered LiDAR/odom");
        return;
      }

      last_healthy_place_attempt_stamp_ = healthy_place_stamp;
      last_anchor_localize_ = now;
      CloudT::Ptr cloud = buildRollingLidarCloud(buffered_stamp, source_Rwi, source_twi);
      if (!cloud || cloud->size() < 100)
      {
        cloud.reset(new CloudT);
        pcl::fromROSMsg(*buffered_msg, *cloud);
      }
      registerToMap(cloud, source_Rwi, source_twi,
                    "lidar_saved_map_icp", buffered_stamp, true);
      return;
    }

    if (mode == "LIO_CAMERA_FAILED")
    {
      // Camera loss does NOT remove either of the sensors needed for reliable
      // short-horizon LIO.  Keep the already-acquired saved-map track alive by
      // propagating it with LiDAR+IMU odometry and refining the current LiDAR
      // scan locally against the canonical PCD.  This is the same global map
      // authority used in healthy mode; it is not a second localizer and it
      // never touches/reset the persistent ESIKF state.
      //
      // Do not attempt a blind global reacquisition here: without camera place
      // retrieval, repeated campus geometry is ambiguous.  We require the
      // tracker acquired before the outage and only perform bounded local GICP.
      if (!use_online_anchor_ || !trackerHasPoseFor("lidar_saved_map_icp"))
      {
        ROS_WARN_THROTTLE(1.0,
            "[FAILSAFE MAP] camera-fail map tracking waiting for pre-failure LiDAR map seed");
        return;
      }
      if (!last_anchor_localize_.isZero() &&
          (now - last_anchor_localize_).toSec() <
              1.0 / std::max(0.05, anchor_localization_hz_))
        return;
      last_anchor_localize_ = now;

      Eigen::Matrix3d source_Rwi;
      Eigen::Vector3d source_twi;
      if (!localBodyPoseNear(lidar_stamp, source_Rwi, source_twi))
      {
        ROS_WARN_THROTTLE(1.0,
            "[FAILSAFE MAP] camera-fail map tracking waiting for timestamp-matched odometry");
        return;
      }
      CloudT::Ptr cloud = buildRollingLidarCloud(lidar_stamp, source_Rwi, source_twi);
      if (!cloud || cloud->size() < 100)
      {
        cloud.reset(new CloudT);
        pcl::fromROSMsg(*msg, *cloud);
      }
      registerToMap(cloud, source_Rwi, source_twi,
                    "lidar_saved_map_icp", lidar_stamp, false);
      return;
    }

    if (mode != "LV_IMU_FAILED" &&
        mode != "VI_LIDAR_FAILED" &&
        mode != "IMU_ONLY_BRIDGE")
      return;

    // Degraded modes intentionally consume every surviving LiDAR scan for the
    // short-term relative-motion front-end, so deserialize only here.
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);

    // A dedicated lightweight front-end can own scan-to-scan LiDAR motion so
    // expensive saved-map GICP cannot starve the relative update cadence.
    if (!external_lidar_relative_frontend_)
      publishRelativeMotionHint(
          cloud,
          mode == "LV_IMU_FAILED" ? "lidar_saved_map_icp"
                                  : "lidar_recovery_saved_map_icp",
          now);

    // Absolute saved-map GICP can run slower.
    if (!last_lidar_localize_.isZero() &&
        (now - last_lidar_localize_).toSec() <
            1.0 / std::max(0.5, lidar_localization_hz_))
      return;
    last_lidar_localize_ = now;

    if (mode == "LV_IMU_FAILED")
      registerToMap(cloud, Rwi, twi, "lidar_saved_map_icp", now);
    else
      registerToMap(cloud, Rwi, twi, "lidar_recovery_saved_map_icp", now);
  }

  CloudT::Ptr depthToLidarCloud(const sensor_msgs::Image::ConstPtr &msg)
  {
    if (depth_fx_ <= 0.0 || depth_fy_ <= 0.0) return CloudT::Ptr();
    cv_bridge::CvImageConstPtr cv_ptr;
    const bool float_depth = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    try
    {
      if (float_depth)
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      else
      {
        ROS_WARN_THROTTLE(1.0, "[FAILSAFE MAP] Unsupported depth encoding: %s",
                          msg->encoding.c_str());
        return CloudT::Ptr();
      }
    }
    catch (const cv_bridge::Exception &e)
    {
      ROS_WARN_THROTTLE(1.0, "[FAILSAFE MAP] Depth conversion failed: %s", e.what());
      return CloudT::Ptr();
    }
    CloudT::Ptr cloud(new CloudT);
    cloud->reserve((cv_ptr->image.rows / std::max(1, depth_stride_)) *
                   (cv_ptr->image.cols / std::max(1, depth_stride_)));
    for (int v = 0; v < cv_ptr->image.rows; v += std::max(1, depth_stride_))
    {
      for (int u = 0; u < cv_ptr->image.cols; u += std::max(1, depth_stride_))
      {
        double raw = 0.0;
        if (float_depth)
        {
          raw = static_cast<double>(cv_ptr->image.ptr<float>(v)[u]);
          if (!std::isfinite(raw) || raw <= 0.0) continue;
        }
        else
        {
          raw = static_cast<double>(cv_ptr->image.ptr<uint16_t>(v)[u]);
          if (raw == 0.0) continue;
        }
        const double z = raw * depth_scale_;
        if (z < depth_min_m_ || z > depth_max_m_) continue;
        const Eigen::Vector3d p_depth((u - depth_cx_) * z / depth_fx_,
                                     (v - depth_cy_) * z / depth_fy_, z);
        const Eigen::Vector3d p_color = R_depth_color_ * p_depth + t_depth_color_;
        const Eigen::Vector3d p_lidar = Rcl_.transpose() * (p_color - Pcl_);
        if (!p_lidar.allFinite()) continue;
        PointT p;
        p.x = p_lidar.x();
        p.y = p_lidar.y();
        p.z = p_lidar.z();
        p.intensity = static_cast<float>(z);
        cloud->push_back(p);
      }
    }
    cloud->width = cloud->size();
    cloud->height = 1;
    return cloud;
  }

  void depthCallback(const sensor_msgs::Image::ConstPtr &msg)
  {
    Eigen::Matrix3d Rwi;
    Eigen::Vector3d twi;
    std::string mode;
    ros::Time odom_stamp;
    if (!snapshotState(Rwi, twi, mode, odom_stamp)) return;
    if (!(mode == "VI_LIDAR_FAILED" || mode == "IMU_ONLY_BRIDGE"))
      return;

    // Once a genuinely post-failure LiDAR scan has returned, give returning-
    // LiDAR saved-map verification priority over optional depth GICP.  FULL_LIVO
    // is intentionally held until that geometric lock succeeds, so otherwise
    // depth callbacks can keep winning registration_mutex_ while the mode still
    // says VI_LIDAR_FAILED and delay recovery by several seconds.  RGB-D VO
    // continues unaffected; only the optional absolute depth registration yields.
    // Pre-failure queued scans do not trigger this because their sensor stamp is
    // older than the tracking-family causal boundary.
    ros::Time family_enter_stamp;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      family_enter_stamp = tracking_family_enter_stamp_;
    }
    bool returning_lidar_seen = false;
    {
      std::lock_guard<std::mutex> lock(lidar_history_mutex_);
      returning_lidar_seen = !lidar_msg_history_.empty() &&
          !family_enter_stamp.isZero() &&
          lidar_msg_history_.back().first + ros::Duration(0.02) >= family_enter_stamp;
    }
    if (returning_lidar_seen)
    {
      ROS_INFO_THROTTLE(0.5,
          "[FAILSAFE MAP] yielding depth GICP to returning-LiDAR map lock");
      return;
    }

    // RGB-D VO is the primary LiDAR-loss measurement. Building a dense depth
    // cloud + GICP here can starve that high-rate frontend on the cart CPU.
    // Keep this optional for experiments; default production fallback relies
    // on the pre-failure global map anchor + RGB-D/IMU motion.
    if (!depth_fallback_map_enable_) return;
    const ros::Time now = ros::Time::now();
    ros::Time depth_stamp = msg->header.stamp.isZero() ? now : msg->header.stamp;
    // Archived RealSense depth headers are in the camera hardware clock while
    // FAST-LIVO2/Step-3 odometry uses rebased rosbag /clock.  Normalize depth
    // to the same epoch using the sensor adapter's single shared offset.  Do
    // not double-apply it when a future driver already publishes ROS time.
    double camera_clock_offset_sec = 0.0;
    if (!depth_stamp.isZero() && !now.isZero() &&
        std::abs((depth_stamp - now).toSec()) > 5.0 &&
        nh_.getParam("/fast_livo2/camera_clock_offset_sec", camera_clock_offset_sec) &&
        std::isfinite(camera_clock_offset_sec))
    {
      depth_stamp += ros::Duration(camera_clock_offset_sec);
    }
    CloudT::Ptr cloud = depthToLidarCloud(msg);
    if (!cloud || cloud->size() < 80) return;
    downsampleInPlace(cloud, 0.18);
    {
      std::lock_guard<std::mutex> lock(depth_history_mutex_);
      depth_cloud_history_.emplace_back(depth_stamp, CloudT::Ptr(new CloudT(*cloud)));
      while (!depth_cloud_history_.empty() &&
             (depth_stamp - depth_cloud_history_.front().first).toSec() > 1.5)
        depth_cloud_history_.pop_front();
      while (depth_cloud_history_.size() > 12) depth_cloud_history_.pop_front();
    }

    // Do NOT publish a second depth scan-to-scan motion stream here. The
    // dedicated RGB-D VO already supplies the validated high-rate LiDAR-loss
    // relative motion. Publishing depth ICP as another velocity cue made the
    // same persistent ESIKF chase two translation sources and regressed the
    // previously-good ~2 m LiDAR-fail trajectory. Depth is used here only for
    // low-rate visual-place-verified absolute saved-map registration.

    if (!last_depth_localize_.isZero() &&
        (now - last_depth_localize_).toSec() <
            1.0 / std::max(0.5, depth_localization_hz_))
      return;
    last_depth_localize_ = now;
    Eigen::Matrix3d source_Rwi = Rwi; Eigen::Vector3d source_twi = twi;
    localBodyPoseNear(depth_stamp, source_Rwi, source_twi);
    CloudT::Ptr rolling = buildRollingDepthCloud(depth_stamp, source_Rwi, source_twi);
    if (rolling && rolling->size() >= 100) cloud = rolling;
    registerToMap(cloud, source_Rwi, source_twi, "depth_saved_map_icp", depth_stamp);
  }

  ros::NodeHandle nh_, pnh_;
  std::string mode_, map_path_, visual_map_path_, registered_cloud_topic_, lidar_topic_, depth_topic_,
      depth_info_topic_, odom_topic_, failsafe_mode_topic_, place_candidates_topic_,
      initial_pose_topic_, step3_global_pose_topic_, map_frame_, global_pose_topic_, global_odom_topic_, global_path_topic_,
      localization_status_topic_, legacy_pose_topic_, legacy_odom_topic_, base_frame_,
      failsafe_mode_ = "FULL_LIVO";
  ros::Subscriber cloud_sub_, odom_sub_, mode_sub_, place_sub_, initial_pose_sub_, lidar_sub_, depth_sub_,
      depth_info_sub_, relative_delta_sub_, rgbd_relative_delta_sub_, step3_global_pose_sub_;
  ros::Publisher correction_pub_, relative_motion_pub_, status_pub_, aligned_pub_, map_pub_,
      depth_failover_pose_pub_, global_pose_pub_, global_odom_pub_, global_path_pub_, localization_status_pub_,
      legacy_pose_pub_, legacy_odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex global_path_mutex_;
  nav_msgs::Path global_path_;
  ros::Time last_global_path_stamp_;
  ros::Timer save_timer_;
  CloudT::Ptr map_, visual_map_;
  pcl::KdTreeFLANN<PointT>::Ptr map_kdtree_, visual_map_kdtree_;
  std::mutex map_mutex_, state_mutex_, registration_mutex_, relative_mutex_,
      relative_registration_mutex_;
  std::mutex relative_history_mutex_;
  std::mutex odom_history_mutex_;
  std::deque<std::pair<ros::Time, Eigen::Matrix4f>> odom_lidar_history_;
  std::mutex lidar_history_mutex_;
  std::deque<std::pair<ros::Time, sensor_msgs::PointCloud2::ConstPtr>> lidar_msg_history_;
  std::mutex depth_history_mutex_;
  std::deque<std::pair<ros::Time, CloudT::Ptr>> depth_cloud_history_;
  std::deque<std::pair<ros::Time, Eigen::Matrix4f>> lidar_delta_history_;
  std::deque<std::pair<ros::Time, Eigen::Matrix4f>> rgbd_delta_history_;
  std::mutex verified_place_history_mutex_;
  bool verified_place_history_valid_ = false;
  Eigen::Matrix4f verified_place_history_pose_ = Eigen::Matrix4f::Identity();
  ros::Time verified_place_history_stamp_;
  std::string verified_place_history_mode_;
  std::mutex step3_global_pose_mutex_;
  bool latest_step3_global_pose_valid_ = false;
  Eigen::Matrix4f latest_step3_T_map_body_ = Eigen::Matrix4f::Identity();
  ros::Time latest_step3_global_pose_stamp_;
  double step3_global_pose_max_age_ = 0.75;
  std::mutex fallback_mutex_;
  bool fallback_pose_valid_ = false;
  Eigen::Matrix4f fallback_T_map_body_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f fallback_pending_translation_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f fallback_pending_rotvec_ = Eigen::Vector3f::Zero();
  bool fallback_pending_valid_ = false;
  bool fallback_map_locked_ = false;
  bool legacy_fallback_map_pose_enable_ = false;
  bool degraded_global_anchor_only_ = true;
  bool imu_failure_esikf_map_correction_enable_ = true;
  bool external_lidar_relative_frontend_ = true;
  bool depth_fallback_map_enable_ = false;
  bool depth_failover_only_ = false;
  bool depth_failover_stream_active_ = false;
  bool visual_map_enable_ = false;
  bool publish_legacy_outputs_ = true;
  bool publish_map_to_base_tf_ = true;
  double anchor_refinement_freeze_sec_ = 2.0;
  ros::Time anchor_refinement_resume_stamp_;
  ros::Time fallback_last_delta_stamp_;
  std::string fallback_delta_source_;
  CloudT::Ptr relative_last_lidar_source_, relative_last_depth_source_;
  ros::Time relative_last_lidar_stamp_, relative_last_depth_stamp_;
  bool lidar_rel_filter_valid_ = false;
  Eigen::Vector3d lidar_filt_trans_vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d lidar_filt_ang_vel_ = Eigen::Vector3d::Zero();
  bool map_dirty_ = false;
  int build_frame_counter_ = 0, build_add_counter_ = 0;
  int map_accumulate_every_n_ = 2, map_downsample_every_n_ = 20;
  double map_voxel_leaf_ = 0.25, source_voxel_leaf_ = 0.25;
  double local_map_radius_ = 22.0, local_map_z_radius_ = 5.0;
  double icp_max_correspondence_ = 2.0, icp_fitness_threshold_ = 0.7;
  int icp_max_iterations_ = 35;
  double max_correction_translation_ = 2.2, max_correction_rotation_deg_ = 18.0;
  double depth_map_fitness_threshold_ = 0.30;
  double depth_map_max_translation_ = 0.8;
  double depth_map_max_rotation_deg_ = 3.0;
  double depth_bootstrap_max_translation_ = 1.2;
  double depth_bootstrap_max_rotation_deg_ = 5.0;
  double depth_place_max_applied_translation_ = 3.0;
  bool depth_place_position_relock_enable_ = false;
  double depth_place_position_relock_max_fitness_ = 0.11;
  double depth_place_position_relock_min_overlap_ = 0.90;
  double depth_place_position_relock_max_seed_translation_ = 1.25;
  double depth_place_position_relock_max_seed_rotation_deg_ = 8.0;
  double depth_place_position_relock_max_continuity_translation_ = 6.0;
  bool depth_place_preserve_visual_orientation_ = false;
  bool depth_place_temporal_relock_enable_ = false;
  int depth_place_temporal_relock_min_fixes_ = 3;
  double depth_place_temporal_relock_max_fitness_ = 0.09;
  double depth_place_temporal_relock_min_overlap_ = 0.95;
  double depth_place_temporal_relock_max_speed_mps_ = 3.5;
  double depth_place_temporal_relock_max_accel_mps2_ = 4.0;
  double depth_place_temporal_relock_max_course_change_deg_ = 65.0;
  std::mutex depth_relock_mutex_;
  bool depth_relock_valid_ = false;
  int depth_relock_streak_ = 0;
  ros::Time depth_relock_stamp_;
  Eigen::Matrix4f depth_relock_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector2f depth_relock_velocity_ = Eigen::Vector2f::Zero();
  bool depth_relock_velocity_valid_ = false;
  double lidar_localization_hz_ = 7.0, depth_localization_hz_ = 5.0, anchor_localization_hz_ = 2.0;
  double rolling_lidar_window_sec_ = 0.60, rolling_depth_window_sec_ = 0.45;
  int rolling_lidar_max_scans_ = 5, rolling_depth_max_scans_ = 3;
  double rolling_map_translation_gain_ = 0.55;
  double rolling_map_max_translation_step_ = 0.45, rolling_map_max_z_step_ = 0.15;
  double rolling_map_yaw_gain_ = 0.45, rolling_map_max_yaw_step_deg_ = 1.0;
  double rolling_map_max_translation_innovation_ = 3.0;
  double rolling_map_max_yaw_innovation_deg_ = 12.0;
  double rolling_heading_baseline_m_ = 4.0;
  double rolling_heading_gain_ = 0.30, rolling_heading_filter_gain_ = 0.35;
  double rolling_heading_max_step_deg_ = 0.35, rolling_heading_max_innovation_deg_ = 8.0;
  double rolling_heading_min_distance_ratio_ = 0.70, rolling_heading_max_distance_ratio_ = 1.30;
  bool rolling_heading_ref_valid_ = false, rolling_heading_filtered_valid_ = false;
  Eigen::Vector2f rolling_heading_ref_map_ = Eigen::Vector2f::Zero();
  Eigen::Vector2f rolling_heading_ref_local_ = Eigen::Vector2f::Zero();
  double rolling_heading_filtered_yaw_ = 0.0;
  double anchor_max_correspondence_ = 5.0, anchor_fitness_threshold_ = 1.2;
  double anchor_overlap_threshold_ = 0.50;
  double anchor_max_translation_ = 8.0, anchor_max_rotation_deg_ = 40.0;
  double anchor_source_voxel_leaf_ = 0.90;
  double anchor_target_voxel_leaf_ = 1.00;
  double anchor_local_map_radius_ = 16.0;
  double anchor_local_map_z_radius_ = 4.0;
  int anchor_gicp_iterations_ = 12;
  int anchor_correspondence_randomness_ = 12;
  double anchor_overlap_distance_ = 1.0;
  double anchor_update_max_translation_step_ = 0.15;
  double anchor_update_max_rotation_step_deg_ = 1.5;
  double anchor_target_max_translation_rate_ = 3.0;
  double anchor_target_max_rotation_rate_deg_ = 20.0;
  double healthy_anchor_target_max_translation_rate_ = 1.5;
  double healthy_anchor_target_max_rotation_rate_deg_ = 12.0;
  double anchor_target_translation_rate_active_ = 3.0;
  double anchor_target_rotation_rate_active_deg_ = 20.0;
  double degraded_anchor_target_max_translation_jump_ = 4.0;
  double degraded_anchor_target_max_rotation_jump_deg_ = 18.0;
  bool use_online_anchor_ = false;
  bool fixed_anchor_enable_ = false;
  int anchor_lock_samples_ = 3;
  double anchor_consistency_translation_ = 1.0;
  double anchor_consistency_rotation_deg_ = 3.0;
  bool anchor_candidate_valid_ = false;
  int anchor_consistency_count_ = 0;
  Eigen::Matrix4f anchor_candidate_ = Eigen::Matrix4f::Identity();
  double tracker_max_odom_delta_translation_ = 1.5;
  double tracker_max_odom_delta_rotation_deg_ = 12.0;
  double depth_tracker_max_age_sec_ = 2.5;
  double depth_local_fitness_threshold_ = 0.10;
  double depth_local_overlap_threshold_ = 0.75;
  double depth_local_max_translation_ = 0.90;
  double depth_local_max_rotation_deg_ = 8.0;
  bool structural_filter_enable_ = true;
  double structural_min_body_z_ = -0.9;
  double structural_max_body_z_ = 4.0;
  double structural_min_range_ = 1.0;
  int structural_min_points_ = 250;
  double relative_max_correspondence_ = 1.2;
  double relative_fitness_threshold_ = 0.45;
  double relative_max_translation_ = 1.5;
  double relative_max_rotation_deg_ = 12.0;
  int depth_stride_ = 5;
  double depth_scale_ = 0.001, depth_min_m_ = 0.5, depth_max_m_ = 12.0;
  double depth_fx_ = 0.0, depth_fy_ = 0.0, depth_cx_ = 0.0, depth_cy_ = 0.0;
  Eigen::Matrix3d extR_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d extT_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d Rcl_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d Pcl_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_depth_color_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_depth_color_ = Eigen::Vector3d::Zero();
  std::mutex anchor_mutex_;
  bool anchor_valid_ = false;
  Eigen::Matrix4f anchor_T_map_current_ = Eigen::Matrix4f::Identity();
  bool last_global_pose_valid_ = false;
  Eigen::Matrix4f last_global_T_map_state_ = Eigen::Matrix4f::Identity();
  bool recovery_anchor_rebase_pending_ = false;
  Eigen::Matrix4f recovery_hold_map_state_ = Eigen::Matrix4f::Identity();
  bool anchor_target_valid_ = false;
  Eigen::Matrix4f anchor_target_T_map_current_ = Eigen::Matrix4f::Identity();
  ros::Time anchor_target_last_step_stamp_;
  bool degraded_anchor_candidate_valid_ = false;
  Eigen::Matrix4f degraded_anchor_candidate_ = Eigen::Matrix4f::Identity();
  ros::Time healthy_anchor_stamp_;
  std::mutex tracker_mutex_;
  bool tracker_valid_ = false;
  Eigen::Matrix4f tracker_T_map_lidar_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tracker_last_current_lidar_ = Eigen::Matrix4f::Identity();
  CloudT::Ptr tracker_last_source_;
  std::string tracker_source_name_;
  ros::Time last_tracker_accept_stamp_;
  bool latest_odom_valid_;
  Eigen::Matrix3d latest_Rwi_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d latest_twi_ = Eigen::Vector3d::Zero();
  ros::Time current_mode_enter_stamp_;
  // Causal boundary of the current estimator/tracking family. Unlike the exact
  // mode timestamp, this does not advance on VI_LIDAR_FAILED <-> IMU_ONLY_BRIDGE.
  ros::Time tracking_family_enter_stamp_;
  ros::Time latest_odom_stamp_, last_lidar_localize_, last_depth_localize_, last_anchor_localize_;
  ros::Time last_healthy_place_attempt_stamp_;
  std::mutex place_mutex_;
  std::vector<Eigen::Matrix4f> place_candidates_;
  ros::Time place_candidates_stamp_;
  bool place_candidates_metric_rgbd_ = false;
  double place_hint_max_age_ = 1.25;
  double place_metric_lidar_fail_max_age_ = 5.5;
  double place_bootstrap_max_age_ = 7.0;

  std::mutex place_confirm_mutex_;
  bool place_confirm_valid_ = false;
  int place_confirm_streak_ = 0;
  Eigen::Matrix4f place_confirm_pose_ = Eigen::Matrix4f::Identity();
  ros::Time place_confirm_stamp_;
  std::string place_confirm_source_;

  // Motion history of independently verified global visual+LiDAR matches.
  // Used only during IMU failure; this is a measurement-consistency guard, not
  // a second navigation state.
  std::mutex imu_place_history_mutex_;
  bool imu_place_history_valid_ = false;
  bool imu_place_motion_valid_ = false;
  Eigen::Matrix4f imu_place_history_pose_ = Eigen::Matrix4f::Identity();
  ros::Time imu_place_history_stamp_;
  Eigen::Vector3d imu_place_velocity_ = Eigen::Vector3d::Zero();
  double imu_place_yaw_rate_ = 0.0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_livo2_failsafe_map_localizer");
  try
  {
    FailsafeMapLocalizer node;
    // GICP must not block odometry updates. registerToMap() is explicitly
    // serialized, while this spinner lets state callbacks run in parallel.
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[FAILSAFE MAP] %s", e.what());
    return 1;
  }
  return 0;
}
