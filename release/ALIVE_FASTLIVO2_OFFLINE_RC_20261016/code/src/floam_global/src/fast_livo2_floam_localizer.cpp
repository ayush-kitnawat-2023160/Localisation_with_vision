// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// c++ lib
#include <cmath>
#include <vector>
#include <deque>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <math.h>
#include <string.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <pcl/io/pcd_io.h>
#include <fast_livo2/StampedFloat64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// local lib
#include "lidar.h"
#include "odomEstimationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;
std::deque<nav_msgs::OdometryConstPtr> fastLocalOdomBuf;
std::mutex fast_local_odom_mutex;
std::condition_variable fast_local_odom_cv;
uint64_t fast_local_generation = 0;

nav_msgs::Odometry laserOdometry;
nav_msgs::Path globalTrajectory;

ros::Publisher pubLaserOdometry;
ros::Publisher pubLaserPose;
ros::Publisher pubInitEdge;
ros::Publisher pubInitSurface;
ros::Publisher pubCost;
ros::Publisher pubTrajectory;
ros::Publisher pubStatus, pubMap;
ros::Publisher pubFastPredictionCount;
uint64_t fast_prediction_count = 0;
pcl::PointCloud<pcl::PointXYZI>::Ptr full_map(new pcl::PointCloud<pcl::PointXYZI>);
std::mutex initial_pose_mutex;
geometry_msgs::PoseWithCovarianceStamped pending_initial_pose;
bool initial_pose_pending = false;
bool global_lock = false;
int consecutive_failures = 0;
Eigen::Isometry3d floam_motion = Eigen::Isometry3d::Identity();
bool search_pending = false;
double search_xy = 5.0, search_yaw = 20.0, search_z = 2.0;

void publishStatus(const std::string& state)
{
  static std::string previous;
  if (state == previous) return;
  previous = state;
  std_msgs::String msg;
  msg.data = state;
  pubStatus.publish(msg);
}

std::atomic<bool> is_odom_inited(false);

bool relocalize = false;



double sensor_height_m = 1.5;

bool fast_local_anchor_set = false;
Eigen::Isometry3d fast_local_anchor = Eigen::Isometry3d::Identity();
ros::Time fast_local_anchor_stamp;
uint64_t fast_local_anchor_generation = 0;
double fast_local_max_time_error = 0.15;
double fast_local_wait_timeout = 0.3;
double fast_local_time_offset = 0.0;
std::string fast_local_odom_topic = "/aft_mapped_to_init";
bool use_fast_local_prediction = true;
int fast_prediction_warmup_scans = 5;
int floam_correction_scans = 0;
double floam_fallback_max_translation_per_scan = 0.8;
double floam_fallback_max_rotation_deg_per_scan = 8.0;
double floam_fallback_max_tilt_deg = 20.0;
std::string map_frame = "map";
std::string base_frame = "base_link";
std::string fast_local_parent_frame = "camera_init";
std::string fast_local_child_frame = "aft_mapped";
Eigen::Isometry3d imu_from_base = Eigen::Isometry3d::Identity();
Eigen::Vector3d base_from_lidar_translation = Eigen::Vector3d::Zero();

bool loadRigidTransform(ros::NodeHandle& nh, const std::string& prefix,
                        Eigen::Isometry3d& transform)
{
  std::vector<double> translation;
  std::vector<double> rotation;
  if (!nh.getParam(prefix + "_translation", translation) ||
      !nh.getParam(prefix + "_rotation", rotation) ||
      translation.size() != 3 || rotation.size() != 9)
  {
    ROS_FATAL("Missing or invalid rigid transform parameters: %s_translation/rotation",
              prefix.c_str());
    return false;
  }

  Eigen::Matrix3d matrix;
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      matrix(row, column) = rotation[3 * row + column];
  const Eigen::Vector3d vector(translation[0], translation[1], translation[2]);
  const double orthogonality_error =
      (matrix.transpose() * matrix - Eigen::Matrix3d::Identity()).norm();
  if (!matrix.allFinite() || !vector.allFinite() ||
      std::abs(matrix.determinant() - 1.0) > 1.0e-3 ||
      orthogonality_error > 1.0e-3)
  {
    ROS_FATAL("Invalid rigid transform %s (det %.9f, orthogonality error %.3e)",
              prefix.c_str(), matrix.determinant(), orthogonality_error);
    return false;
  }
  transform = Eigen::Isometry3d::Identity();
  transform.linear() = matrix;
  transform.translation() = vector;
  return true;
}

bool odometryPose(const nav_msgs::Odometry& odometry,
                  Eigen::Isometry3d& pose)
{
  const auto& orientation = odometry.pose.pose.orientation;
  Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y,
                       orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1.0e-9)
    return false;
  const auto& position = odometry.pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z))
    return false;
  q.normalize();
  pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() << position.x, position.y, position.z;
  return true;
}

bool fastLocalPoseAt(const ros::Time& floam_stamp,
                     Eigen::Isometry3d& pose,
                     ros::Time& interpolated_stamp,
                     double& bracket_error,
                     uint64_t& generation)
{
  const ros::Time target_stamp =
      floam_stamp + ros::Duration(fast_local_time_offset);
  std::unique_lock<std::mutex> lock(fast_local_odom_mutex);
  // Do not stall FLOAM for an absent/stale predictor. When FAST is active it
  // needs future IMU/camera samples to bracket the raw scan timestamp, so
  // allow a bounded wait instead of always outrunning its publication.
  if (fastLocalOdomBuf.empty() ||
      (target_stamp - fastLocalOdomBuf.back()->header.stamp).toSec() > 0.5)
    return false;
  fast_local_odom_cv.wait_for(
      lock, std::chrono::duration<double>(fast_local_wait_timeout),
      [&target_stamp]() {
        return !ros::ok() ||
               (!fastLocalOdomBuf.empty() &&
                fastLocalOdomBuf.back()->header.stamp >= target_stamp);
      });
  if (!ros::ok() || fastLocalOdomBuf.empty())
    return false;

  const auto upper = std::lower_bound(
      fastLocalOdomBuf.begin(), fastLocalOdomBuf.end(), target_stamp,
      [](const nav_msgs::OdometryConstPtr& sample, const ros::Time& stamp) {
        return sample->header.stamp < stamp;
      });
  if (upper == fastLocalOdomBuf.end())
    return false;

  Eigen::Isometry3d upper_pose;
  if (!odometryPose(**upper, upper_pose))
    return false;

  const double upper_error = ((*upper)->header.stamp - target_stamp).toSec();
  if (std::abs(upper_error) < 1.0e-9)
  {
    pose = upper_pose;
    interpolated_stamp = target_stamp;
    bracket_error = 0.0;
    generation = fast_local_generation;
    return true;
  }
  if (upper == fastLocalOdomBuf.begin())
    return false;

  const auto lower = std::prev(upper);
  const double lower_error = (target_stamp - (*lower)->header.stamp).toSec();
  const double span = ((*upper)->header.stamp - (*lower)->header.stamp).toSec();
  bracket_error = std::max(lower_error, upper_error);
  if (span <= 1.0e-9 || lower_error < 0.0 || upper_error < 0.0 ||
      bracket_error > fast_local_max_time_error)
    return false;

  Eigen::Isometry3d lower_pose;
  if (!odometryPose(**lower, lower_pose))
    return false;
  const double alpha = std::max(0.0, std::min(1.0, lower_error / span));
  Eigen::Quaterniond q_lower(lower_pose.linear());
  Eigen::Quaterniond q_upper(upper_pose.linear());
  Eigen::Quaterniond q = q_lower.slerp(alpha, q_upper).normalized();
  pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() =
      (1.0 - alpha) * lower_pose.translation() + alpha * upper_pose.translation();
  interpolated_stamp = target_stamp;
  generation = fast_local_generation;
  return pose.matrix().allFinite();
}

void fastLocalOdomHandler(const nav_msgs::OdometryConstPtr& msg)
{
  if (msg->header.stamp.isZero() ||
      (!fast_local_parent_frame.empty() &&
       msg->header.frame_id != fast_local_parent_frame) ||
      (!fast_local_child_frame.empty() &&
       msg->child_frame_id != fast_local_child_frame))
  {
    ROS_ERROR_THROTTLE(
        5.0,
        "Rejected FAST odometry with stamp/frame [%s -> %s]; expected [%s -> %s]",
        msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
        fast_local_parent_frame.c_str(), fast_local_child_frame.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(fast_local_odom_mutex);
    if (!fastLocalOdomBuf.empty() &&
        msg->header.stamp < fastLocalOdomBuf.back()->header.stamp - ros::Duration(1.0))
    {
      ROS_WARN("FAST odometry clock moved backwards; resetting interpolation buffer");
      fastLocalOdomBuf.clear();
      ++fast_local_generation;
    }
    const auto position = std::lower_bound(
        fastLocalOdomBuf.begin(), fastLocalOdomBuf.end(), msg->header.stamp,
        [](const nav_msgs::OdometryConstPtr& sample, const ros::Time& stamp) {
          return sample->header.stamp < stamp;
        });
    if (position != fastLocalOdomBuf.end() &&
        (*position)->header.stamp == msg->header.stamp)
      *position = msg;
    else
      fastLocalOdomBuf.insert(position, msg);
    while (fastLocalOdomBuf.size() > 2000)
      fastLocalOdomBuf.pop_front();
  }
  fast_local_odom_cv.notify_all();
}

bool fastPredictionAt(const ros::Time& stamp, Eigen::Isometry3d& prediction)
{
  Eigen::Isometry3d fast_imu_pose;
  ros::Time interpolated_stamp;
  double bracket_error = 0.0;
  uint64_t generation = 0;
  if (!fastLocalPoseAt(stamp, fast_imu_pose, interpolated_stamp,
                       bracket_error, generation))
    return false;

  // FAST publishes world<-IMU. FLOAM optimizes map<-base_link. Compose the
  // calibrated IMU<-base_link rigid transform before taking the local delta.
  const Eigen::Isometry3d fast_base_pose = fast_imu_pose * imu_from_base;
  if (!fast_local_anchor_set || generation != fast_local_anchor_generation)
  {
    fast_local_anchor = fast_base_pose;
    fast_local_anchor_stamp = interpolated_stamp;
    fast_local_anchor_generation = generation;
    fast_local_anchor_set = true;
    prediction = odomEstimation.odom * (odomEstimation.last_odom.inverse() * odomEstimation.odom);
    ROS_INFO("Anchored interpolated FAST base pose at t=%.6f (max bracket %.4f s)",
             interpolated_stamp.toSec(), bracket_error);
    return true;
  }

  if (interpolated_stamp <= fast_local_anchor_stamp)
  {
    ROS_WARN_THROTTLE(5.0, "Ignoring non-monotonic interpolated FAST pose");
    return false;
  }

  const Eigen::Isometry3d fast_delta =
      fast_local_anchor.inverse() * fast_base_pose;
  prediction = odomEstimation.odom * fast_delta;
  fast_local_anchor = fast_base_pose;
  fast_local_anchor_stamp = interpolated_stamp;
  if (!prediction.matrix().allFinite()) return false;
  std_msgs::UInt64 count;
  count.data = ++fast_prediction_count;
  pubFastPredictionCount.publish(count);
  return true;
}

/* Subscribe initialPose published from rviz */
void initialPoseHandler(geometry_msgs::PoseWithCovarianceStamped msg)
{
  if (!relocalize) return;
  if (!msg.header.frame_id.empty() && msg.header.frame_id != map_frame)
  {
    ROS_ERROR("Rejected /initialpose in frame %s; expected %s",
              msg.header.frame_id.c_str(), map_frame.c_str());
    return;
  }
  nav_msgs::Odometry candidate;
  candidate.pose = msg.pose;
  Eigen::Isometry3d pose;
  if (!odometryPose(candidate, pose))
  {
    ROS_ERROR("Rejected non-finite /initialpose or invalid quaternion");
    return;
  }
  std::lock_guard<std::mutex> lock(initial_pose_mutex);
  pending_initial_pose = msg;
  initial_pose_pending = true;
}

/** Subscribe to plane features topic published by feature extractor **/
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  std::lock_guard<std::mutex> lock(mutex_lock);
  pointCloudSurfBuf.push(laserCloudMsg);
  while (pointCloudSurfBuf.size() > 50)
    pointCloudSurfBuf.pop();
}

/* Subscribe to edge features topic published by feature extractor */
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  std::lock_guard<std::mutex> lock(mutex_lock);
  pointCloudEdgeBuf.push(laserCloudMsg);
  while (pointCloudEdgeBuf.size() > 50)
    pointCloudEdgeBuf.pop();
}

void odom_estimation()
{
  while (ros::ok())
  {
    std::unique_lock<std::mutex> cloud_lock(mutex_lock);
    if (pointCloudEdgeBuf.empty() || pointCloudSurfBuf.empty())
    {
      cloud_lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    if (pointCloudSurfBuf.front()->header.stamp.toSec() <
        pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period)
    {
      pointCloudSurfBuf.pop();
      ROS_WARN("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
      continue;
    }

    if (pointCloudEdgeBuf.front()->header.stamp.toSec() <
        pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period)
    {
      pointCloudEdgeBuf.pop();
      ROS_WARN("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
      continue;
    }

    const sensor_msgs::PointCloud2ConstPtr edge_msg = pointCloudEdgeBuf.front();
    const sensor_msgs::PointCloud2ConstPtr surf_msg = pointCloudSurfBuf.front();
    pointCloudEdgeBuf.pop();
    pointCloudSurfBuf.pop();
    cloud_lock.unlock();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*edge_msg, *pointcloud_edge_in);
    pcl::fromROSMsg(*surf_msg, *pointcloud_surf_in);
    const ros::Time pointcloud_time = surf_msg->header.stamp;

    if (edge_msg->header.frame_id != base_frame ||
        surf_msg->header.frame_id != base_frame)
    {
      ROS_ERROR_THROTTLE(5.0, "FLOAM rejected features outside configured base frame");
      continue;
    }

    // Keep the first paired feature scan in this worker while the bag is
    // paused and the operator chooses the approximate RViz pose.  The old
    // loop discarded feature pairs whenever /initialpose had not arrived;
    // after pausing there was then no scan left to run the FLOAM seed search.
    // Holding this already-received pair makes the paused workflow
    // deterministic and costs no extra registration work.
    if (relocalize && !is_odom_inited)
    {
      while (ros::ok())
      {
        bool pending = false;
        {
          std::lock_guard<std::mutex> lock(initial_pose_mutex);
          pending = initial_pose_pending;
        }
        if (pending) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    {
      std::lock_guard<std::mutex> lock(initial_pose_mutex);
      if (initial_pose_pending)
      {
        initial_pose_pending = false;
        const auto& requested = pending_initial_pose.pose.pose;
        std::vector<double> heights;
        for (const auto& point : full_map->points)
        {
          const double dx = point.x - requested.position.x;
          const double dy = point.y - requested.position.y;
          const double d2 = dx * dx + dy * dy;
          if (d2 > 9.0 && d2 < 25.0 && std::isfinite(point.z))
            heights.push_back(point.z);
        }
        global_lock = false;
        is_odom_inited = false;
        floam_correction_scans = 0;
        consecutive_failures = 0;
        floam_motion = Eigen::Isometry3d::Identity();
        fast_local_anchor_set = false;
        globalTrajectory.poses.clear();
        globalTrajectory.header.stamp = pointcloud_time;
        pubTrajectory.publish(globalTrajectory);
        if (heights.size() < 10)
        {
          publishStatus("WAITING_FOR_INITIALPOSE: no map ground near seed");
          ROS_ERROR("No map support near /initialpose; choose a position on the map and retry");
        }
        else
        {
          // Ground may be only a few returns below a dense tree canopy.
          // Search around the low envelope to refine the initial height.
          const size_t index = 0;
          std::nth_element(heights.begin(), heights.begin() + index, heights.end());
          nav_msgs::Odometry seed;
          seed.pose = pending_initial_pose.pose;
          odometryPose(seed, odomEstimation.odom);
          // sensor_height is measured at the LiDAR origin, not base_link.
          const double base_to_lidar_z =
              (odomEstimation.odom.linear() * base_from_lidar_translation).z();
          odomEstimation.odom.translation().z() = heights[index] + sensor_height_m - base_to_lidar_z;
          odomEstimation.last_odom = odomEstimation.odom;
          is_odom_inited = true;
          search_pending = true;
          publishStatus("ALIGNING");
          ROS_INFO("FLOAM seed accepted at %.3f %.3f %.3f; waiting for validated scan-to-map matches",
                   odomEstimation.odom.translation().x(), odomEstimation.odom.translation().y(),
                   odomEstimation.odom.translation().z());
        }
      }
    }
    if (!is_odom_inited)
    {
      if (!relocalize)
      {
        odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
        is_odom_inited = true;
      }
      else ROS_WARN_THROTTLE(5.0, "FLOAM waiting for /initialpose in map frame");
      continue;
    }

    if (search_pending)
    {
      search_pending = false;
      if (!odomEstimation.initializeFromSeed(pointcloud_edge_in, pointcloud_surf_in,
                                            search_xy, search_yaw, search_z))
        ROS_WARN("FLOAM seed search found no validated solution; refine /initialpose and retry");
    }
    const Eigen::Isometry3d previous_pose = odomEstimation.odom;
    Eigen::Isometry3d motion_prediction = odomEstimation.odom * floam_motion;
    bool used_fast_prediction = false;
    odomEstimation.setExternalPrediction(motion_prediction);
    if (!global_lock)
    {
      // Initial numerical corrections are not vehicle velocity. Extrapolating
      // them caused the seed to accelerate out of the map correspondence basin.
      odomEstimation.setExternalPrediction(odomEstimation.odom);
      fast_local_anchor_set = false;
    }
    else if (use_fast_local_prediction)
    {
      Eigen::Isometry3d prediction;
      if (fastPredictionAt(pointcloud_time, prediction))
      {
        motion_prediction = prediction;
        odomEstimation.setExternalPrediction(prediction);
        used_fast_prediction = true;
      }
      else
      {
        // Otherwise the next delta spans scans already advanced by FLOAM.
        fast_local_anchor_set = false;
        ROS_WARN_THROTTLE(5.0, "No bracketed FAST pose; using FLOAM and resetting FAST anchor");
      }
    }
    odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in, relocalize);

    // FLOAM-only fallback has no independent motion source. A scan-to-map
    // solve can therefore look numerically good while snapping to a nearby
    // repeated surface; feeding that correction into the next extrapolation
    // creates the large jumps/tilt observed on the faculty loop.  Apply a
    // second, temporal continuity gate only when FAST did not provide the
    // prediction.  The normal FAST+FLOAM path remains governed by FAST motion
    // plus FLOAM's existing correction-to-prediction validation.
    if (odomEstimation.registration_valid && global_lock && !used_fast_prediction)
    {
      const Eigen::Isometry3d accepted_delta =
          previous_pose.inverse() * odomEstimation.odom;
      const double translation_step = accepted_delta.translation().norm();
      const double rotation_step = Eigen::AngleAxisd(accepted_delta.linear()).angle();

      const Eigen::Vector3d body_z = odomEstimation.odom.linear().col(2);
      const double tilt = std::acos(std::max(-1.0, std::min(1.0, body_z.z())));
      const double max_rotation = floam_fallback_max_rotation_deg_per_scan * M_PI / 180.0;
      const double max_tilt = floam_fallback_max_tilt_deg * M_PI / 180.0;

      if (!std::isfinite(translation_step) || !std::isfinite(rotation_step) ||
          !std::isfinite(tilt) ||
          translation_step > floam_fallback_max_translation_per_scan ||
          rotation_step > max_rotation || tilt > max_tilt)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "FLOAM continuity rejected pose: step=%.3f m rot=%.2f deg tilt=%.2f deg",
            translation_step, rotation_step * 180.0 / M_PI,
            tilt * 180.0 / M_PI);
        odomEstimation.registration_valid = false;
        odomEstimation.odom = previous_pose;
        odomEstimation.last_odom = previous_pose;
      }
    }
    fast_livo2::StampedFloat64 cost_msg;
    cost_msg.header.stamp = pointcloud_time;
    cost_msg.data = odomEstimation.final_cost;
    cost_msg.data2 = odomEstimation.average_cost;
    pubCost.publish(cost_msg);
    if (!odomEstimation.registration_valid)
    {
      ++consecutive_failures;
      if (global_lock && consecutive_failures <= 5)
      {
        // Bridge a short occlusion internally using motion alone. Publish no
        // unvalidated global measurement, and never apply the rejected solve.
        odomEstimation.odom = motion_prediction;
        odomEstimation.last_odom = previous_pose;
        publishStatus("DEGRADED: awaiting a valid map correction");
      }
      else
      {
        floam_correction_scans = 0;
        global_lock = false;
        fast_local_anchor_set = false;
        floam_motion = Eigen::Isometry3d::Identity();
        publishStatus("ALIGNING: scan-to-map validation failed; retry /initialpose if persistent");
      }
      ROS_WARN_THROTTLE(2.0, "FLOAM match rejected: residuals=%zu fraction=%.3f cost=%.5f geometry=%.6f",
                       odomEstimation.matched_points, odomEstimation.matched_fraction,
                       odomEstimation.average_cost, odomEstimation.geometry_ratio);
      continue;
    }
    consecutive_failures = 0;
    if (global_lock)
    {
      // Map correction noise is not instantaneous vehicle velocity. Smooth
      // the fallback delta to prevent alternating corrections from feeding
      // back as accelerating translation/rotation on subsequent scans.
      const Eigen::Isometry3d measured_motion = previous_pose.inverse() * odomEstimation.odom;
      floam_motion.translation() = .8 * floam_motion.translation() + .2 * measured_motion.translation();
      floam_motion.linear() = Eigen::Quaterniond(floam_motion.linear()).slerp(
          .2, Eigen::Quaterniond(measured_motion.linear())).normalized().toRotationMatrix();
    }
    ++floam_correction_scans;
    if (!global_lock && floam_correction_scans >= std::max(1, fast_prediction_warmup_scans))
    {
      global_lock = true;
      publishStatus("LOCALIZED");
      ROS_INFO("FLOAM global map lock validated on %d consecutive scans; FAST prediction %s",
               floam_correction_scans, use_fast_local_prediction ? "enabled" : "disabled");
    }
    if (!global_lock) continue;
    publishStatus("LOCALIZED");

      Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
      Eigen::Vector3d t_current = odomEstimation.odom.translation();

      geometry_msgs::PoseStamped carCoord;
      carCoord.header.frame_id = map_frame;
      carCoord.header.stamp = pointcloud_time;
      carCoord.pose.position.x = t_current.x();
      carCoord.pose.position.y = t_current.y();
      carCoord.pose.position.z = t_current.z();
      carCoord.pose.orientation.x = q_current.x();
      carCoord.pose.orientation.y = q_current.y();
      carCoord.pose.orientation.z = q_current.z();
      carCoord.pose.orientation.w = q_current.w();
      pubLaserPose.publish(carCoord);

      globalTrajectory.header.frame_id = map_frame;
      globalTrajectory.header.stamp = pointcloud_time;
      globalTrajectory.poses.push_back(carCoord);
      if (globalTrajectory.poses.size() > 10000)
        globalTrajectory.poses.erase(globalTrajectory.poses.begin(), globalTrajectory.poses.begin() + 1000);
      pubTrajectory.publish(globalTrajectory);

      // This standalone stack has exactly one owner for the global dynamic
      // transform: the FLOAM localizer.  This is the optimized map<-base
      // pose itself, not a relabelled FAST local pose. FAST-LIVO2 owns only
      // its independent camera_init -> aft_mapped local transform.
      static tf::TransformBroadcaster global_tf_broadcaster;
      tf::Transform global_tf;
      global_tf.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
      global_tf.setRotation(tf::Quaternion(q_current.x(), q_current.y(),
                                           q_current.z(), q_current.w()));
      global_tf_broadcaster.sendTransform(
          tf::StampedTransform(global_tf, pointcloud_time, map_frame, base_frame));

      // publish odometry
      laserOdometry.header.frame_id = map_frame;
      laserOdometry.child_frame_id = base_frame;
      laserOdometry.header.stamp = pointcloud_time;
      laserOdometry.pose.pose.orientation = carCoord.pose.orientation;
      laserOdometry.pose.pose.position = carCoord.pose.position;

      pubLaserOdometry.publish(laserOdometry);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  int scan_line = 64;
  double vertical_angle = 2.0;
  double scan_period = 0.1;
  double max_dis = 60.0;
  double min_dis = 2.0;
  double map_resolution = 0.4;
  int ceres_num_threads = 2;
  int ceres_max_iterations = 4;
  std::string lidarPoseTopic = "/pose";
  std::string odomTopic = "/odom";
  std::string trajectoryTopic = "/fast_livo2/trajectory";
  std::string edgeTopic = "/laser_cloud_edge";
  std::string surfTopic = "/laser_cloud_surf";

  nh.getParam("/relocalization", relocalize);

  nh.getParam("/scan_period", scan_period);
  nh.getParam("/vertical_angle", vertical_angle);
  nh.getParam("/max_dis", max_dis);
  nh.getParam("/min_dis", min_dis);
  nh.getParam("/scan_line", scan_line);
  nh.getParam("/map_resolution", map_resolution);
  nh.param("floam_initial_search_xy_m", search_xy, 5.0);
  nh.param("floam_initial_search_yaw_deg", search_yaw, 20.0);
  nh.param("floam_initial_search_z_m", search_z, 2.0);
  if (!std::isfinite(search_xy) || search_xy < 0.0 || search_xy > 15.0 ||
      !std::isfinite(search_yaw) || search_yaw < 0.0 || search_yaw > 90.0 ||
      !std::isfinite(search_z) || search_z < 0.0 || search_z > 5.0)
  {
    ROS_FATAL("Invalid bounded FLOAM seed search radii (XY 0..15 m, yaw 0..90 deg, Z 0..5 m)");
    return 1;
  }
  nh.param("floam_ceres_num_threads", ceres_num_threads, 2);
  nh.param("floam_ceres_max_iterations", ceres_max_iterations, 4);
  nh.getParam("/lidarPoseTopic", lidarPoseTopic);
  nh.getParam("/floam_odom_topic", odomTopic);
  nh.getParam("/floam_trajectory_topic", trajectoryTopic);
  nh.getParam("/floam_edge_topic", edgeTopic);
  nh.getParam("/floam_surf_topic", surfTopic);
  nh.getParam("/fast_local_odom_topic", fast_local_odom_topic);
  nh.param<std::string>("map_frame", map_frame, "map");
  nh.param<std::string>("base_frame", base_frame, "base_link");
  nh.param<std::string>("fast_local_parent_frame", fast_local_parent_frame,
                        "camera_init");
  nh.param<std::string>("fast_local_child_frame", fast_local_child_frame,
                        "aft_mapped");
  nh.param("fast_local_max_time_error", fast_local_max_time_error, 0.15);
  nh.param("fast_local_wait_timeout", fast_local_wait_timeout, 0.3);
  nh.param("use_fast_local_prediction", use_fast_local_prediction, true);
  nh.param("fast_prediction_warmup_scans", fast_prediction_warmup_scans, 5);
  nh.param("floam_fallback_max_translation_per_scan",
           floam_fallback_max_translation_per_scan, 0.8);
  nh.param("floam_fallback_max_rotation_deg_per_scan",
           floam_fallback_max_rotation_deg_per_scan, 8.0);
  nh.param("floam_fallback_max_tilt_deg", floam_fallback_max_tilt_deg, 20.0);
  if (!std::isfinite(floam_fallback_max_translation_per_scan) ||
      floam_fallback_max_translation_per_scan <= 0.0 ||
      !std::isfinite(floam_fallback_max_rotation_deg_per_scan) ||
      floam_fallback_max_rotation_deg_per_scan <= 0.0 ||
      !std::isfinite(floam_fallback_max_tilt_deg) ||
      floam_fallback_max_tilt_deg <= 0.0 || floam_fallback_max_tilt_deg >= 90.0)
  {
    ROS_FATAL("Invalid FLOAM fallback continuity limits");
    return 1;
  }
  nh.param("calibration/fast_to_floam_time_offset_sec",
           fast_local_time_offset, 0.0);

  bool calibration_ready = false;
  nh.param("calibration/ready", calibration_ready, false);
  nh.param("calibration/sensor_height_m", sensor_height_m, 0.0);
  if (!calibration_ready || !std::isfinite(sensor_height_m) ||
      sensor_height_m <= 0.0)
  {
    ROS_FATAL("Calibration profile is not ready or has invalid sensor_height_m");
    return 1;
  }
  Eigen::Isometry3d imu_from_lidar;
  Eigen::Isometry3d base_from_lidar;
  if (!loadRigidTransform(nh, "calibration/lidar_to_imu", imu_from_lidar) ||
      !loadRigidTransform(nh, "calibration/lidar_to_base", base_from_lidar))
    return 1;
  imu_from_base = imu_from_lidar * base_from_lidar.inverse();
  base_from_lidar_translation = base_from_lidar.translation();
  ROS_INFO("FAST/FLOAM rigid-body composition loaded; |IMU<-base translation| = %.6f m",
           imu_from_base.translation().norm());

  // initialise odometry covariances
  // TODO use covariances based on ICP quality
  std::vector<double> pose_covar;
  if (!nh_private.getParam("pose_covariance", pose_covar) ||
      pose_covar.size() != laserOdometry.pose.covariance.size())
  {
    ROS_FATAL("pose_covariance must contain exactly 36 values");
    return 1;
  }
  auto it_pose = laserOdometry.pose.covariance.begin();
  std::cout << "pose:"
            << "\n";
  for (auto var : pose_covar)
  {
    std::cout << var << " ";
    *it_pose = var;
    ++it_pose;
  }
  std::cout << "\n";
  std::vector<double> twist_covar;
  if (!nh_private.getParam("twist_covariance", twist_covar) ||
      twist_covar.size() != laserOdometry.twist.covariance.size())
  {
    ROS_FATAL("twist_covariance must contain exactly 36 values");
    return 1;
  }
  auto it_twist = laserOdometry.twist.covariance.begin();
  std::cout << "twist:"
            << "\n";
  for (auto var : twist_covar)
  {
    std::cout << var << " ";
    *it_twist = var;
    ++it_twist;
  }
  std::cout << "\n";
  lidar_param.setScanPeriod(scan_period);
  lidar_param.setVerticalAngle(vertical_angle);
  lidar_param.setLines(scan_line);
  lidar_param.setMaxDistance(max_dis);
  lidar_param.setMinDistance(min_dis);

  odomEstimation.init(lidar_param, map_resolution);
  odomEstimation.setSolverOptions(ceres_num_threads, ceres_max_iterations);

  ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(edgeTopic, 1, velodyneEdgeHandler);
  ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(surfTopic, 1, velodyneSurfHandler);
  ros::Subscriber initialPosition = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseHandler);
  ros::Subscriber fastLocalOdom = nh.subscribe<nav_msgs::Odometry>(fast_local_odom_topic, 100, fastLocalOdomHandler);
  pubLaserOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 100);
  pubLaserPose = nh.advertise<geometry_msgs::PoseStamped>(lidarPoseTopic, 1);
  pubTrajectory = nh.advertise<nav_msgs::Path>(trajectoryTopic, 1, true);

  pubInitEdge = nh.advertise<sensor_msgs::PointCloud2>("/edge_init", 1, true);
  pubInitSurface = nh.advertise<sensor_msgs::PointCloud2>("/surf_init", 1, true);
  pubCost = nh.advertise<fast_livo2::StampedFloat64>("/ceres_cost", 1);
  globalTrajectory.header.frame_id = map_frame;

  pubStatus = nh.advertise<std_msgs::String>("/fast_livo2/localization_status", 1, true);
  pubFastPredictionCount = nh.advertise<std_msgs::UInt64>("/fast_livo2/fast_prediction_count", 1, true);
  pubFastPredictionCount.publish(std_msgs::UInt64());
  pubMap = nh.advertise<sensor_msgs::PointCloud2>("/apriori_map", 1, true);
  if (relocalize)
  {
    std::string map_path;
    nh.getParam("mapFileName", map_path);
    if (map_path.empty() || pcl::io::loadPCDFile(map_path, *full_map) != 0)
    {
      ROS_FATAL("Cannot read FLOAM map: %s", map_path.c_str());
      return 1;
    }
    full_map->is_dense = false;
    std::vector<int> valid;
    pcl::removeNaNFromPointCloud(*full_map, *full_map, valid);
    if (full_map->size() < 50)
    {
      ROS_FATAL("FLOAM map is empty or has insufficient finite geometry");
      return 1;
    }
    // Saved maps are unordered global geometry, never VLP-16 sweeps.  Keep
    // the complete PCD as the surface association map.  The optional dense
    // structural-edge PCA pass scans every reduced map point with a KD-tree
    // and makes startup take minutes on this 2M-point campus map; it is not
    // needed for the point-to-plane FLOAM seed solve and is disabled here.
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_edges(new pcl::PointCloud<pcl::PointXYZI>);
    odomEstimation.initMapWithPoints(no_edges, full_map, false);
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*full_map, map_msg);
    map_msg.header.frame_id = map_frame;
    map_msg.header.stamp = ros::Time(0);  // static geometry, also available with a paused bag
    pubMap.publish(map_msg);
    pcl::toROSMsg(*odomEstimation.laserCloudCornerMap, map_msg);
    map_msg.header.frame_id = map_frame;
    pubInitEdge.publish(map_msg);
    pcl::toROSMsg(*odomEstimation.laserCloudSurfMap, map_msg);
    map_msg.header.frame_id = map_frame;
    pubInitSurface.publish(map_msg);
    ROS_INFO("Loaded complete FLOAM map: %zu points (no scan-ring filtering)", full_map->size());
  }
  publishStatus("WAITING_FOR_INITIALPOSE");
  std::thread odom_estimation_process{ odom_estimation };

  ros::spin();

  fast_local_odom_cv.notify_all();
  if (odom_estimation_process.joinable())
    odom_estimation_process.join();

  return 0;
}
