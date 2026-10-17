#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fast_livo2/StampedFloat64.h>

namespace {

using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;

double wrapAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double yawOf(const Eigen::Isometry3d& pose)
{
  return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
}

bool msgPose(const geometry_msgs::Pose& msg, Eigen::Isometry3d& pose)
{
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                       msg.orientation.y, msg.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1.0e-9 ||
      !std::isfinite(msg.position.x) || !std::isfinite(msg.position.y) ||
      !std::isfinite(msg.position.z))
    return false;
  q.normalize();
  pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() << msg.position.x, msg.position.y, msg.position.z;
  return pose.matrix().allFinite();
}

geometry_msgs::Pose toMsg(const Eigen::Isometry3d& pose)
{
  geometry_msgs::Pose msg;
  const Eigen::Quaterniond q(pose.linear());
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

Eigen::Isometry3d blend(const Eigen::Isometry3d& current,
                        const Eigen::Isometry3d& measured,
                        double alpha)
{
  alpha = std::max(0.0, std::min(1.0, alpha));
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
      (1.0 - alpha) * current.translation() + alpha * measured.translation();
  Eigen::Quaterniond a(current.linear());
  Eigen::Quaterniond b(measured.linear());
  result.linear() = a.slerp(alpha, b).normalized().toRotationMatrix();
  return result;
}

double percentileZ(const Cloud::Ptr& cloud, double fraction)
{
  std::vector<float> values;
  values.reserve(cloud->size());
  for (const auto& p : cloud->points)
    if (std::isfinite(p.z)) values.push_back(p.z);
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const size_t index = std::min(values.size() - 1,
      static_cast<size_t>(std::max(0.0, std::min(1.0, fraction)) *
                          static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

}  // namespace

class PcdFastLocalizer
{
 public:
  PcdFastLocalizer() : nh_(), pnh_("~")
  {
    loadParameters();
    if (!loadCalibration() || !loadMap())
      throw std::runtime_error("FAST-LIVO2 PCD localizer initialization failed");

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 20);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 5);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic_, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 1, true);
    quality_pub_ = nh_.advertise<fast_livo2::StampedFloat64>(quality_topic_, 5);

    initial_pose_sub_ = nh_.subscribe("/initialpose", 2,
        &PcdFastLocalizer::initialPoseCallback, this);
    fast_odom_sub_ = nh_.subscribe(fast_odom_topic_, 200,
        &PcdFastLocalizer::fastOdomCallback, this);
    lidar_sub_ = nh_.subscribe(lidar_topic_, 4,
        &PcdFastLocalizer::lidarCallback, this);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_, map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = ros::Time(0);
    map_pub_.publish(map_msg);

    global_path_.header.frame_id = map_frame_;
    publishStatus("WAITING_FOR_INITIALPOSE");
    worker_ = std::thread(&PcdFastLocalizer::registrationLoop, this);
  }

  ~PcdFastLocalizer()
  {
    stopping_ = true;
    scan_cv_.notify_all();
    fast_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  struct TimedPose
  {
    ros::Time stamp;
    Eigen::Isometry3d fast_from_base = Eigen::Isometry3d::Identity();
  };

  struct Candidate
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double overlap = 0.0;
    double fitness = std::numeric_limits<double>::infinity();
  };

  struct RegistrationResult
  {
    bool valid = false;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double fitness = std::numeric_limits<double>::infinity();
    double inlier_fraction = 0.0;
  };

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber initial_pose_sub_, fast_odom_sub_, lidar_sub_;
  ros::Publisher odom_pub_, pose_pub_, path_pub_, status_pub_, map_pub_, quality_pub_;

  std::string map_path_;
  std::string lidar_topic_ = "/lidar103/velodyne_points";
  std::string fast_odom_topic_ = "/aft_mapped_to_init";
  std::string odom_topic_ = "/odom";
  std::string pose_topic_ = "/pose";
  std::string trajectory_topic_ = "/fast_livo2/trajectory";
  std::string status_topic_ = "/fast_livo2/localization_status";
  std::string map_topic_ = "/apriori_map";
  std::string quality_topic_ = "/fast_livo2/registration_quality";
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  std::string fast_parent_frame_ = "camera_init";
  std::string fast_child_frame_ = "aft_mapped";

  double map_leaf_ = 0.40;
  double source_leaf_ = 0.35;
  double coarse_leaf_ = 1.0;
  double local_map_radius_ = 35.0;
  double initial_search_xy_ = 5.0;
  double initial_search_yaw_deg_ = 25.0;
  double initial_grid_xy_step_ = 2.0;
  double initial_grid_yaw_step_deg_ = 10.0;
  int coarse_candidates_ = 8;
  int refine_candidates_ = 3;
  double ndt_resolution_ = 1.5;
  int ndt_iterations_ = 30;
  double gicp_max_correspondence_ = 1.5;
  int gicp_iterations_ = 40;
  double initial_max_fitness_ = 0.65;
  double check_max_fitness_ = 0.45;
  double min_inlier_fraction_ = 0.25;
  double inlier_distance_ = 1.0;
  double periodic_check_sec_ = 3.0;
  int failures_before_reacquire_ = 3;
  double reacquire_search_xy_ = 5.0;
  double reacquire_search_yaw_deg_ = 20.0;
  double max_periodic_translation_correction_ = 2.0;
  double max_periodic_rotation_deg_ = 12.0;
  double anchor_correction_alpha_ = 0.25;
  double fast_wait_sec_ = 0.35;
  double fast_max_time_error_ = 0.15;
  double min_range_ = 0.5;
  double max_range_ = 35.0;
  bool remove_vehicle_ = true;
  double vehicle_length_ = 3.590;
  double vehicle_width_ = 2.2;
  double vehicle_crop_min_z_ = -1.5;
  double vehicle_crop_max_z_ = 10.0;
  double vehicle_crop_pitch_rad_ = -0.35;

  Eigen::Isometry3d base_from_lidar_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d imu_from_base_ = Eigen::Isometry3d::Identity();

  Cloud::Ptr map_{new Cloud};
  Cloud::Ptr map_xy_{new Cloud};
  pcl::KdTreeFLANN<PointT> map_tree_;
  pcl::KdTreeFLANN<PointT> map_xy_tree_;

  std::mutex fast_mutex_;
  std::condition_variable fast_cv_;
  std::deque<TimedPose> fast_poses_;

  std::mutex seed_mutex_;
  bool seed_pending_ = false;
  Eigen::Isometry3d pending_seed_ = Eigen::Isometry3d::Identity();

  std::mutex anchor_mutex_;
  bool localized_ = false;
  Eigen::Isometry3d map_from_fast_ = Eigen::Isometry3d::Identity();
  int check_failures_ = 0;
  ros::Time last_check_stamp_;
  ros::Time last_path_stamp_;
  nav_msgs::Path global_path_;

  std::mutex scan_mutex_;
  std::condition_variable scan_cv_;
  std::deque<sensor_msgs::PointCloud2ConstPtr> scan_queue_;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  tf::TransformBroadcaster tf_broadcaster_;

  void loadParameters()
  {
    pnh_.param<std::string>("map_path", map_path_, std::string());
    pnh_.param<std::string>("lidar_topic", lidar_topic_, lidar_topic_);
    pnh_.param<std::string>("fast_odom_topic", fast_odom_topic_, fast_odom_topic_);
    pnh_.param<std::string>("odom_topic", odom_topic_, odom_topic_);
    pnh_.param<std::string>("pose_topic", pose_topic_, pose_topic_);
    pnh_.param<std::string>("trajectory_topic", trajectory_topic_, trajectory_topic_);
    pnh_.param<std::string>("map_frame", map_frame_, map_frame_);
    pnh_.param<std::string>("base_frame", base_frame_, base_frame_);
    pnh_.param<std::string>("fast_parent_frame", fast_parent_frame_, fast_parent_frame_);
    pnh_.param<std::string>("fast_child_frame", fast_child_frame_, fast_child_frame_);
    pnh_.param("map_leaf", map_leaf_, map_leaf_);
    pnh_.param("source_leaf", source_leaf_, source_leaf_);
    pnh_.param("coarse_leaf", coarse_leaf_, coarse_leaf_);
    pnh_.param("local_map_radius", local_map_radius_, local_map_radius_);
    pnh_.param("initial_search_xy", initial_search_xy_, initial_search_xy_);
    pnh_.param("initial_search_yaw_deg", initial_search_yaw_deg_, initial_search_yaw_deg_);
    pnh_.param("initial_grid_xy_step", initial_grid_xy_step_, initial_grid_xy_step_);
    pnh_.param("initial_grid_yaw_step_deg", initial_grid_yaw_step_deg_, initial_grid_yaw_step_deg_);
    pnh_.param("coarse_candidates", coarse_candidates_, coarse_candidates_);
    pnh_.param("refine_candidates", refine_candidates_, refine_candidates_);
    pnh_.param("ndt_resolution", ndt_resolution_, ndt_resolution_);
    pnh_.param("ndt_iterations", ndt_iterations_, ndt_iterations_);
    pnh_.param("gicp_max_correspondence", gicp_max_correspondence_, gicp_max_correspondence_);
    pnh_.param("gicp_iterations", gicp_iterations_, gicp_iterations_);
    pnh_.param("initial_max_fitness", initial_max_fitness_, initial_max_fitness_);
    pnh_.param("check_max_fitness", check_max_fitness_, check_max_fitness_);
    pnh_.param("min_inlier_fraction", min_inlier_fraction_, min_inlier_fraction_);
    pnh_.param("inlier_distance", inlier_distance_, inlier_distance_);
    pnh_.param("periodic_check_sec", periodic_check_sec_, periodic_check_sec_);
    pnh_.param("failures_before_reacquire", failures_before_reacquire_, failures_before_reacquire_);
    pnh_.param("reacquire_search_xy", reacquire_search_xy_, reacquire_search_xy_);
    pnh_.param("reacquire_search_yaw_deg", reacquire_search_yaw_deg_, reacquire_search_yaw_deg_);
    pnh_.param("max_periodic_translation_correction", max_periodic_translation_correction_, max_periodic_translation_correction_);
    pnh_.param("max_periodic_rotation_deg", max_periodic_rotation_deg_, max_periodic_rotation_deg_);
    pnh_.param("anchor_correction_alpha", anchor_correction_alpha_, anchor_correction_alpha_);
    pnh_.param("fast_wait_sec", fast_wait_sec_, fast_wait_sec_);
    pnh_.param("fast_max_time_error", fast_max_time_error_, fast_max_time_error_);
    pnh_.param("min_range", min_range_, min_range_);
    pnh_.param("max_range", max_range_, max_range_);
    pnh_.param("remove_vehicle_footprint", remove_vehicle_, remove_vehicle_);
    pnh_.param("vehicle_length", vehicle_length_, vehicle_length_);
    pnh_.param("vehicle_width", vehicle_width_, vehicle_width_);
    pnh_.param("vehicle_crop_min_z", vehicle_crop_min_z_, vehicle_crop_min_z_);
    pnh_.param("vehicle_crop_max_z", vehicle_crop_max_z_, vehicle_crop_max_z_);
    pnh_.param("vehicle_crop_pitch_rad", vehicle_crop_pitch_rad_, vehicle_crop_pitch_rad_);
  }

  bool loadTransform(const std::string& prefix, Eigen::Isometry3d& result)
  {
    std::vector<double> t, r;
    if (!nh_.getParam(prefix + "_translation", t) ||
        !nh_.getParam(prefix + "_rotation", r) || t.size() != 3 || r.size() != 9)
    {
      ROS_FATAL("Missing transform parameters %s_translation/rotation", prefix.c_str());
      return false;
    }
    Eigen::Matrix3d R;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col) R(row, col) = r[3 * row + col];
    if (!R.allFinite() || std::abs(R.determinant() - 1.0) > 1.0e-3)
    {
      ROS_FATAL("Invalid rotation for %s", prefix.c_str());
      return false;
    }
    result = Eigen::Isometry3d::Identity();
    result.linear() = R;
    result.translation() << t[0], t[1], t[2];
    return result.matrix().allFinite();
  }

  bool loadCalibration()
  {
    bool ready = false;
    nh_.param("calibration/ready", ready, false);
    if (!ready)
    {
      ROS_FATAL("calibration/ready is false; refusing to localize with unknown extrinsics");
      return false;
    }
    Eigen::Isometry3d imu_from_lidar;
    if (!loadTransform("calibration/lidar_to_base", base_from_lidar_) ||
        !loadTransform("calibration/lidar_to_imu", imu_from_lidar))
      return false;
    imu_from_base_ = imu_from_lidar * base_from_lidar_.inverse();
    return true;
  }

  bool loadMap()
  {
    if (map_path_.empty() || pcl::io::loadPCDFile(map_path_, *map_) != 0)
    {
      ROS_FATAL("Cannot read PCD map: %s", map_path_.c_str());
      return false;
    }
    map_->is_dense = false;
    std::vector<int> valid;
    pcl::removeNaNFromPointCloud(*map_, *map_, valid);
    Cloud::Ptr filtered(new Cloud);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(map_leaf_, map_leaf_, map_leaf_);
    voxel.setInputCloud(map_);
    voxel.filter(*filtered);
    map_.swap(filtered);
    if (map_->size() < 1000)
    {
      ROS_FATAL("PCD map has too little geometry after filtering (%zu points)", map_->size());
      return false;
    }
    map_xy_->resize(map_->size());
    for (size_t i = 0; i < map_->size(); ++i)
    {
      (*map_xy_)[i] = (*map_)[i];
      (*map_xy_)[i].z = 0.0f;
    }
    map_tree_.setInputCloud(map_);
    map_xy_tree_.setInputCloud(map_xy_);
    ROS_INFO("PCD localizer loaded %zu map points at %.2f m voxel resolution",
             map_->size(), map_leaf_);
    return true;
  }

  void publishStatus(const std::string& text)
  {
    std_msgs::String msg;
    msg.data = text;
    status_pub_.publish(msg);
    ROS_INFO_STREAM_THROTTLE(2.0, "PCD localization: " << text);
  }

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_)
    {
      ROS_ERROR("Rejected /initialpose in frame %s (expected %s)",
                msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    Eigen::Isometry3d pose;
    if (!msgPose(msg->pose.pose, pose))
    {
      ROS_ERROR("Rejected invalid /initialpose");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(seed_mutex_);
      pending_seed_ = pose;
      seed_pending_ = true;
    }
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      localized_ = false;
      check_failures_ = 0;
      global_path_.poses.clear();
      last_check_stamp_ = ros::Time(0);
    }
    publishStatus("ALIGNING: rough pose received");
    scan_cv_.notify_all();
  }

  void fastOdomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    if ((!fast_parent_frame_.empty() && msg->header.frame_id != fast_parent_frame_) ||
        (!fast_child_frame_.empty() && msg->child_frame_id != fast_child_frame_))
    {
      ROS_WARN_THROTTLE(5.0, "Ignoring FAST odom frame %s -> %s",
                        msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
      return;
    }
    Eigen::Isometry3d fast_from_imu;
    if (!msgPose(msg->pose.pose, fast_from_imu)) return;
    TimedPose sample;
    sample.stamp = msg->header.stamp;
    sample.fast_from_base = fast_from_imu * imu_from_base_;
    {
      std::lock_guard<std::mutex> lock(fast_mutex_);
      if (!fast_poses_.empty() && sample.stamp < fast_poses_.back().stamp - ros::Duration(1.0))
        fast_poses_.clear();
      auto pos = std::lower_bound(fast_poses_.begin(), fast_poses_.end(), sample.stamp,
          [](const TimedPose& a, const ros::Time& stamp) { return a.stamp < stamp; });
      if (pos != fast_poses_.end() && pos->stamp == sample.stamp) *pos = sample;
      else fast_poses_.insert(pos, sample);
      while (fast_poses_.size() > 3000) fast_poses_.pop_front();
    }
    fast_cv_.notify_all();

    Eigen::Isometry3d anchor;
    bool localized = false;
    {
      std::lock_guard<std::mutex> lock(anchor_mutex_);
      localized = localized_;
      anchor = map_from_fast_;
    }
    if (localized) publishGlobalPose(msg->header.stamp, anchor * sample.fast_from_base);
  }

  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_queue_.push_back(msg);
    while (scan_queue_.size() > 5) scan_queue_.pop_front();
    scan_cv_.notify_one();
  }

  bool fastPoseAt(const ros::Time& stamp, Eigen::Isometry3d& pose)
  {
    std::unique_lock<std::mutex> lock(fast_mutex_);
    fast_cv_.wait_for(lock, std::chrono::duration<double>(fast_wait_sec_), [&]() {
      return stopping_ || (!fast_poses_.empty() && fast_poses_.back().stamp >= stamp);
    });
    if (stopping_ || fast_poses_.empty()) return false;
    auto upper = std::lower_bound(fast_poses_.begin(), fast_poses_.end(), stamp,
        [](const TimedPose& a, const ros::Time& s) { return a.stamp < s; });
    if (upper == fast_poses_.end()) return false;
    if (upper->stamp == stamp)
    {
      pose = upper->fast_from_base;
      return true;
    }
    if (upper == fast_poses_.begin()) return false;
    auto lower = std::prev(upper);
    const double dt0 = (stamp - lower->stamp).toSec();
    const double dt1 = (upper->stamp - stamp).toSec();
    const double span = (upper->stamp - lower->stamp).toSec();
    if (span <= 1.0e-9 || dt0 < 0.0 || dt1 < 0.0 ||
        std::max(dt0, dt1) > fast_max_time_error_)
      return false;
    const double alpha = dt0 / span;
    pose = Eigen::Isometry3d::Identity();
    pose.translation() = (1.0 - alpha) * lower->fast_from_base.translation() +
                         alpha * upper->fast_from_base.translation();
    Eigen::Quaterniond a(lower->fast_from_base.linear());
    Eigen::Quaterniond b(upper->fast_from_base.linear());
    pose.linear() = a.slerp(alpha, b).normalized().toRotationMatrix();
    return true;
  }

  Cloud::Ptr preprocess(const sensor_msgs::PointCloud2ConstPtr& msg) const
  {
    Cloud::Ptr raw(new Cloud);
    pcl::fromROSMsg(*msg, *raw);
    raw->is_dense = false;
    std::vector<int> valid;
    pcl::removeNaNFromPointCloud(*raw, *raw, valid);

    Cloud::Ptr range(new Cloud);
    range->reserve(raw->size());
    const double min2 = min_range_ * min_range_;
    const double max2 = max_range_ * max_range_;
    for (const auto& p : raw->points)
    {
      const double d2 = p.x * p.x + p.y * p.y + p.z * p.z;
      if (d2 >= min2 && d2 <= max2) range->push_back(p);
    }
    if (remove_vehicle_)
    {
      pcl::CropBox<PointT> box;
      box.setInputCloud(range);
      box.setMin(Eigen::Vector4f(-vehicle_length_ / 3.0, -vehicle_width_ / 2.0,
                                 vehicle_crop_min_z_, 1.0));
      box.setMax(Eigen::Vector4f(2.0 * vehicle_length_ / 3.0, vehicle_width_ / 2.0,
                                 vehicle_crop_max_z_, 1.0));
      box.setRotation(Eigen::Vector3f(0.0f, static_cast<float>(vehicle_crop_pitch_rad_), 0.0f));
      box.setNegative(true);
      Cloud::Ptr cropped(new Cloud);
      box.filter(*cropped);
      range.swap(cropped);
    }

    Cloud::Ptr base(new Cloud);
    pcl::transformPointCloud(*range, *base, base_from_lidar_.matrix().cast<float>());
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(source_leaf_, source_leaf_, source_leaf_);
    voxel.setInputCloud(base);
    Cloud::Ptr filtered(new Cloud);
    voxel.filter(*filtered);
    return filtered;
  }

  Cloud::Ptr localMap(double x, double y, double radius) const
  {
    PointT query;
    query.x = x;
    query.y = y;
    query.z = 0.0f;
    std::vector<int> indices;
    std::vector<float> distances;
    map_xy_tree_.radiusSearch(query, radius, indices, distances);
    Cloud::Ptr local(new Cloud);
    local->reserve(indices.size());
    for (const int index : indices) local->push_back((*map_)[index]);
    return local;
  }

  double overlapScore(const Cloud::Ptr& source, const Eigen::Isometry3d& pose) const
  {
    if (source->empty()) return 0.0;
    const size_t stride = std::max<size_t>(1, source->size() / 300);
    size_t tested = 0, matched = 0;
    std::vector<int> index(1);
    std::vector<float> distance(1);
    for (size_t i = 0; i < source->size(); i += stride)
    {
      const Eigen::Vector3d world = pose * source->points[i].getVector3fMap().cast<double>();
      PointT query;
      query.x = world.x(); query.y = world.y(); query.z = world.z();
      if (map_tree_.nearestKSearch(query, 1, index, distance) == 1 && distance[0] <= 4.0f)
        ++matched;
      ++tested;
    }
    return tested ? static_cast<double>(matched) / tested : 0.0;
  }

  double inlierFraction(const Cloud::Ptr& source, const Cloud::Ptr& target,
                        const Eigen::Isometry3d& pose) const
  {
    if (source->empty() || target->empty()) return 0.0;
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(target);
    const float max_sq = static_cast<float>(inlier_distance_ * inlier_distance_);
    std::vector<int> index(1);
    std::vector<float> distance(1);
    size_t matched = 0;
    for (const auto& p : source->points)
    {
      const Eigen::Vector3d world = pose * p.getVector3fMap().cast<double>();
      PointT query;
      query.x = world.x(); query.y = world.y(); query.z = world.z();
      if (tree.nearestKSearch(query, 1, index, distance) == 1 && distance[0] <= max_sq)
        ++matched;
    }
    return static_cast<double>(matched) / source->size();
  }

  RegistrationResult fineAlign(const Cloud::Ptr& source, const Cloud::Ptr& target,
                               const Eigen::Isometry3d& guess,
                               double max_fitness) const
  {
    RegistrationResult result;
    if (source->size() < 80 || target->size() < 300) return result;
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    gicp.setMaximumIterations(gicp_iterations_);
    gicp.setMaxCorrespondenceDistance(gicp_max_correspondence_);
    gicp.setTransformationEpsilon(1.0e-4);
    gicp.setEuclideanFitnessEpsilon(1.0e-4);
    Cloud aligned;
    gicp.align(aligned, guess.matrix().cast<float>());
    if (!gicp.hasConverged()) return result;
    result.pose.matrix() = gicp.getFinalTransformation().cast<double>();
    result.fitness = gicp.getFitnessScore(gicp_max_correspondence_);
    result.inlier_fraction = inlierFraction(source, target, result.pose);
    result.valid = result.pose.matrix().allFinite() &&
                   std::isfinite(result.fitness) && result.fitness <= max_fitness &&
                   result.inlier_fraction >= min_inlier_fraction_;
    return result;
  }

  bool withinSeedBounds(const Eigen::Isometry3d& seed,
                        const Eigen::Isometry3d& pose,
                        double search_xy,
                        double search_yaw_deg) const
  {
    const double xy_error = std::hypot(pose.translation().x() - seed.translation().x(),
                                       pose.translation().y() - seed.translation().y());
    const double yaw_error = std::abs(wrapAngle(yawOf(pose) - yawOf(seed)));
    const Eigen::Matrix3d relative = seed.linear().transpose() * pose.linear();
    const double tilt_error = std::acos(std::max(-1.0, std::min(1.0, relative(2, 2))));
    // GICP is a 6-DoF optimizer.  Without this gate it can leave the
    // operator's local basin, flip onto a repeated road segment, and still
    // report a numerically excellent fitness.  The rough RViz pose is a
    // bounded acquisition prior, so reject those physically different
    // solutions instead of falsely declaring LOCALIZED.
    return std::isfinite(xy_error) && std::isfinite(yaw_error) &&
           std::isfinite(tilt_error) &&
           xy_error <= search_xy + 1.0 &&
           yaw_error <= (search_yaw_deg + 10.0) * M_PI / 180.0 &&
           tilt_error <= 30.0 * M_PI / 180.0;
  }

  RegistrationResult acquire(const Cloud::Ptr& source, Eigen::Isometry3d seed,
                             double search_xy, double search_yaw_deg,
                             double max_fitness)
  {
    RegistrationResult failure;
    Cloud::Ptr target = localMap(seed.translation().x(), seed.translation().y(),
                                 local_map_radius_ + search_xy);
    if (target->size() < 500)
    {
      ROS_WARN("No useful PCD geometry near rough seed (target=%zu)", target->size());
      return failure;
    }

    // RViz's 2D Pose Estimate supplies z=0.  Estimate base height from the low
    // envelope of both clouds so NDT is not asked to bridge a 30-40 m campus
    // elevation offset before it can even establish correspondences.
    const double source_ground = percentileZ(source, 0.10);
    const double map_ground = percentileZ(target, 0.10);
    if (std::isfinite(source_ground) && std::isfinite(map_ground))
      seed.translation().z() = map_ground - source_ground;
    const Eigen::Isometry3d bounded_seed = seed;

    Cloud::Ptr coarse_source(new Cloud), coarse_target(new Cloud);
    pcl::VoxelGrid<PointT> coarse_filter;
    coarse_filter.setLeafSize(coarse_leaf_, coarse_leaf_, coarse_leaf_);
    coarse_filter.setInputCloud(source);
    coarse_filter.filter(*coarse_source);
    coarse_filter.setInputCloud(target);
    coarse_filter.filter(*coarse_target);
    if (coarse_source->size() < 50 || coarse_target->size() < 200) return failure;

    std::vector<Candidate, Eigen::aligned_allocator<Candidate>> candidates;
    const int xy_steps = search_xy > 0.0
        ? static_cast<int>(std::ceil(search_xy / std::max(0.5, initial_grid_xy_step_))) : 0;
    const int yaw_steps = search_yaw_deg > 0.0
        ? static_cast<int>(std::ceil(search_yaw_deg / std::max(1.0, initial_grid_yaw_step_deg_))) : 0;
    for (int xi = -xy_steps; xi <= xy_steps; ++xi)
      for (int yi = -xy_steps; yi <= xy_steps; ++yi)
        for (int yiw = -yaw_steps; yiw <= yaw_steps; ++yiw)
        {
          Candidate c;
          c.pose = seed;
          c.pose.translation().x() += xi * (xy_steps ? search_xy / xy_steps : 0.0);
          c.pose.translation().y() += yi * (xy_steps ? search_xy / xy_steps : 0.0);
          const double dyaw = yiw * (yaw_steps ? search_yaw_deg / yaw_steps : 0.0) * M_PI / 180.0;
          c.pose.linear() = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * seed.linear();
          c.overlap = overlapScore(coarse_source, c.pose);
          candidates.push_back(c);
        }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.overlap > b.overlap; });
    if (candidates.empty()) return failure;

    std::vector<Candidate, Eigen::aligned_allocator<Candidate>> ndt_results;
    const int coarse_count = std::min<int>(coarse_candidates_, candidates.size());
    for (int i = 0; i < coarse_count; ++i)
    {
      pcl::NormalDistributionsTransform<PointT, PointT> ndt;
      ndt.setInputSource(coarse_source);
      ndt.setInputTarget(coarse_target);
      ndt.setResolution(ndt_resolution_);
      ndt.setStepSize(0.3);
      ndt.setTransformationEpsilon(0.02);
      ndt.setMaximumIterations(ndt_iterations_);
      Cloud aligned;
      ndt.align(aligned, candidates[i].pose.matrix().cast<float>());
      if (!ndt.hasConverged()) continue;
      Candidate c = candidates[i];
      c.pose.matrix() = ndt.getFinalTransformation().cast<double>();
      c.fitness = ndt.getFitnessScore(3.0);
      if (c.pose.matrix().allFinite() && std::isfinite(c.fitness)) ndt_results.push_back(c);
    }
    if (ndt_results.empty())
    {
      ROS_WARN("NDT did not converge for any coarse seed");
      return failure;
    }
    std::sort(ndt_results.begin(), ndt_results.end(),
              [](const Candidate& a, const Candidate& b) { return a.fitness < b.fitness; });

    RegistrationResult best;
    const int refine_count = std::min<int>(refine_candidates_, ndt_results.size());
    for (int i = 0; i < refine_count; ++i)
    {
      RegistrationResult result = fineAlign(source, target, ndt_results[i].pose, max_fitness);
      if (result.valid && !withinSeedBounds(bounded_seed, result.pose,
                                            search_xy, search_yaw_deg))
      {
        ROS_WARN("Rejecting PCD solution outside rough-pose basin: seed=[%.2f %.2f yaw %.1f] result=[%.2f %.2f yaw %.1f]",
                 bounded_seed.translation().x(), bounded_seed.translation().y(),
                 yawOf(bounded_seed) * 180.0 / M_PI,
                 result.pose.translation().x(), result.pose.translation().y(),
                 yawOf(result.pose) * 180.0 / M_PI);
        result.valid = false;
      }
      if (result.valid && (!best.valid || result.fitness < best.fitness)) best = result;
    }
    if (best.valid)
      ROS_INFO("PCD acquisition valid: fitness=%.4f inliers=%.1f%% xyz=[%.2f %.2f %.2f] yaw=%.1fdeg",
               best.fitness, 100.0 * best.inlier_fraction,
               best.pose.translation().x(), best.pose.translation().y(), best.pose.translation().z(),
               yawOf(best.pose) * 180.0 / M_PI);
    return best;
  }

  void publishQuality(const ros::Time& stamp, const RegistrationResult& result)
  {
    fast_livo2::StampedFloat64 msg;
    msg.header.stamp = stamp;
    msg.data = result.fitness;
    msg.data2 = result.inlier_fraction;
    quality_pub_.publish(msg);
  }

  void setAnchor(const Eigen::Isometry3d& map_from_base,
                 const Eigen::Isometry3d& fast_from_base,
                 bool smooth)
  {
    const Eigen::Isometry3d measured = map_from_base * fast_from_base.inverse();
    std::lock_guard<std::mutex> lock(anchor_mutex_);
    map_from_fast_ = smooth && localized_
        ? blend(map_from_fast_, measured, anchor_correction_alpha_) : measured;
    localized_ = true;
    check_failures_ = 0;
  }

  void registrationLoop()
  {
    while (ros::ok() && !stopping_)
    {
      sensor_msgs::PointCloud2ConstPtr msg;
      {
        std::unique_lock<std::mutex> lock(scan_mutex_);
        scan_cv_.wait_for(lock, std::chrono::milliseconds(100),
                          [&]() { return stopping_ || !scan_queue_.empty(); });
        if (stopping_) break;
        if (scan_queue_.empty()) continue;
        msg = scan_queue_.back();
        scan_queue_.clear();
      }

      Cloud::Ptr source = preprocess(msg);
      if (source->size() < 100)
      {
        ROS_WARN_THROTTLE(3.0, "LiDAR scan too sparse for PCD localization (%zu points)", source->size());
        continue;
      }

      bool localized = false;
      Eigen::Isometry3d anchor;
      {
        std::lock_guard<std::mutex> lock(anchor_mutex_);
        localized = localized_;
        anchor = map_from_fast_;
      }

      if (!localized)
      {
        Eigen::Isometry3d seed;
        bool have_seed = false;
        {
          std::lock_guard<std::mutex> lock(seed_mutex_);
          if (seed_pending_)
          {
            seed = pending_seed_;
            have_seed = true;
          }
        }
        if (!have_seed)
        {
          publishStatus("WAITING_FOR_INITIALPOSE");
          continue;
        }
        publishStatus("ALIGNING: NDT coarse search + GICP refinement");
        RegistrationResult result = acquire(source, seed, initial_search_xy_,
                                            initial_search_yaw_deg_, initial_max_fitness_);
        publishQuality(msg->header.stamp, result);
        if (!result.valid)
        {
          publishStatus("ALIGNING: no validated PCD match yet");
          continue;
        }
        // Acquisition is intentionally performed while the bag is paused.
        // FAST has no odometry sample yet in that state, so the frozen
        // LiDAR-to-PCD result is the first global pose.  Once playback is
        // resumed FAST supplies the local motion used by the anchor.
        Eigen::Isometry3d fast_from_base = Eigen::Isometry3d::Identity();
        if (!fastPoseAt(msg->header.stamp, fast_from_base))
          ROS_INFO("No FAST pose while paused; publishing the frozen LiDAR/PCD pose");
        setAnchor(result.pose, fast_from_base, false);
        {
          std::lock_guard<std::mutex> lock(seed_mutex_);
          seed_pending_ = false;
        }
        last_check_stamp_ = msg->header.stamp;
        publishGlobalPose(msg->header.stamp, result.pose);
        publishStatus("LOCALIZED");
        continue;
      }

      Eigen::Isometry3d fast_from_base;
      if (!fastPoseAt(msg->header.stamp, fast_from_base))
      {
        ROS_WARN_THROTTLE(3.0,
          "PCD pose is locked; waiting for FAST-LIVO2 odometry after playback resumes");
        continue;
      }

      if (!last_check_stamp_.isZero() &&
          (msg->header.stamp - last_check_stamp_).toSec() < periodic_check_sec_)
        continue;
      last_check_stamp_ = msg->header.stamp;
      const Eigen::Isometry3d predicted = anchor * fast_from_base;
      Cloud::Ptr target = localMap(predicted.translation().x(), predicted.translation().y(),
                                   local_map_radius_);
      RegistrationResult check = fineAlign(source, target, predicted, check_max_fitness_);
      publishQuality(msg->header.stamp, check);

      if (check.valid)
      {
        const Eigen::Isometry3d delta = predicted.inverse() * check.pose;
        const double translation = delta.translation().norm();
        const double rotation_deg = Eigen::AngleAxisd(delta.linear()).angle() * 180.0 / M_PI;
        if (translation <= max_periodic_translation_correction_ &&
            rotation_deg <= max_periodic_rotation_deg_)
        {
          setAnchor(check.pose, fast_from_base, true);
          publishStatus("LOCALIZED: periodic PCD check OK");
          continue;
        }
        ROS_WARN("Rejected large periodic correction %.2fm / %.1fdeg", translation, rotation_deg);
        check.valid = false;
      }

      int failures = 0;
      {
        std::lock_guard<std::mutex> lock(anchor_mutex_);
        failures = ++check_failures_;
      }
      if (failures < failures_before_reacquire_)
      {
        publishStatus("DEGRADED: FAST-LIVO2 continuing; PCD check failed");
        continue;
      }

      publishStatus("RELOCALIZING: coarse PCD search around FAST prediction");
      RegistrationResult reacquired = acquire(source, predicted,
          reacquire_search_xy_, reacquire_search_yaw_deg_, initial_max_fitness_);
      publishQuality(msg->header.stamp, reacquired);
      if (reacquired.valid)
      {
        setAnchor(reacquired.pose, fast_from_base, false);
        publishStatus("LOCALIZED: PCD re-acquired");
      }
      else
      {
        publishStatus("DEGRADED: re-acquisition failed; FAST-LIVO2 still propagating");
      }
    }
  }

  void publishGlobalPose(const ros::Time& stamp, const Eigen::Isometry3d& pose)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = toMsg(pose);
    pose_pub_.publish(pose_msg);

    nav_msgs::Odometry odom;
    odom.header = pose_msg.header;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = pose_msg.pose;
    odom_pub_.publish(odom);

    tf::Transform tf_pose;
    tf_pose.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(),
                                 pose.translation().z()));
    const Eigen::Quaterniond q(pose.linear());
    tf_pose.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_pose, stamp, map_frame_, base_frame_));

    std::lock_guard<std::mutex> lock(anchor_mutex_);
    if (last_path_stamp_.isZero() || (stamp - last_path_stamp_).toSec() >= 0.10)
    {
      global_path_.header.stamp = stamp;
      global_path_.poses.push_back(pose_msg);
      if (global_path_.poses.size() > 10000)
        global_path_.poses.erase(global_path_.poses.begin(), global_path_.poses.begin() + 1000);
      path_pub_.publish(global_path_);
      last_path_stamp_ = stamp;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fast_livo2_pcd_localizer");
  try
  {
    PcdFastLocalizer localizer;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("PCD localizer failed: %s", e.what());
    return 1;
  }
  return 0;
}
