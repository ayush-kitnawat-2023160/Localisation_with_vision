// FLOAM feature extraction must run in the sensor frame. A pitched mount
// changes elevation angles in base_link and therefore destroys inferred rings.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include "laserProcessingClass.h"

class FloamFrontend
{
  ros::NodeHandle nh_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_{buffer_};
  LaserProcessingClass processing_;
  ros::Subscriber sub_;
  ros::Publisher edge_, surf_, filtered_, transformed_;
  std::string base_frame_;
  bool crop_, allow_identity_;
  double length_, width_, min_z_, max_z_, pitch_;

  void scan(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = buffer_.lookupTransform(base_frame_, msg->header.frame_id,
                                           msg->header.stamp, ros::Duration(0.05));
    }
    catch (const tf2::TransformException& error)
    {
      if (!allow_identity_)
      {
        ROS_WARN_THROTTLE(5.0, "FLOAM missing LiDAR mount TF: %s", error.what());
        return;
      }
      ROS_WARN_THROTTLE(5.0, "FLOAM explicitly configured to use identity LiDAR mount");
      transform.transform.rotation.w = 1.0;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *raw);
    raw->is_dense = false;
    std::vector<int> valid;
    pcl::removeNaNFromPointCloud(*raw, *raw, valid);
    if (crop_)
    {
      pcl::CropBox<pcl::PointXYZI> box;
      box.setInputCloud(raw);
      box.setMin(Eigen::Vector4f(-length_ / 3., -width_ / 2., min_z_, 1.));
      box.setMax(Eigen::Vector4f(2. * length_ / 3., width_ / 2., max_z_, 1.));
      box.setRotation(Eigen::Vector3f(0., pitch_, 0.));
      box.setNegative(true);
      box.filter(*raw);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr edges(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfaces(new pcl::PointCloud<pcl::PointXYZI>);
    processing_.featureExtraction(raw, edges, surfaces);
    auto publish = [&](const pcl::PointCloud<pcl::PointXYZI>& cloud,
                       ros::Publisher& publisher) {
      sensor_msgs::PointCloud2 sensor, base;
      pcl::toROSMsg(cloud, sensor);
      sensor.header = msg->header;
      pcl_ros::transformPointCloud(base_frame_, transform.transform, sensor, base);
      base.header.stamp = msg->header.stamp;
      publisher.publish(base);
    };
    publish(*raw, transformed_);
    publish(*edges, edge_);
    publish(*surfaces, surf_);
    ROS_INFO_THROTTLE(5.0, "FLOAM sensor-frame features: edge=%zu surface=%zu",
                      edges->size(), surfaces->size());
    *edges += *surfaces;
    publish(*edges, filtered_);
  }

public:
  FloamFrontend()
  {
    lidar::Lidar lidar;
    int lines;
    double period, angle, min_distance, max_distance;
    nh_.param("scan_line", lines, 16);
    nh_.param("scan_period", period, 0.1);
    nh_.param("vertical_angle", angle, 1.0);
    nh_.param("min_dis", min_distance, 0.5);
    nh_.param("floam_max_scan_distance", max_distance, 30.0);
    lidar.setLines(lines);
    lidar.setScanPeriod(period);
    lidar.setVerticalAngle(angle);
    lidar.setMinDistance(min_distance);
    lidar.setMaxDistance(max_distance);
    processing_.init(lidar);
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_.param("allow_identity_lidar_tf", allow_identity_, false);
    nh_.param("floam_remove_vehicle_footprint", crop_, true);
    nh_.param("vehicle_length", length_, 3.590);
    nh_.param("vehicle_width", width_, 2.2);
    nh_.param("floam_vehicle_crop_min_z", min_z_, -1.5);
    nh_.param("floam_vehicle_crop_max_z", max_z_, 10.0);
    nh_.param("floam_vehicle_crop_pitch_rad", pitch_, -0.35);
    std::string topic, edge_topic, surf_topic;
    nh_.param<std::string>("lidarTopic", topic, "/lidar103/velodyne_points");
    nh_.param<std::string>("floam_edge_topic", edge_topic, "/laser_cloud_edge");
    nh_.param<std::string>("floam_surf_topic", surf_topic, "/laser_cloud_surf");
    edge_ = nh_.advertise<sensor_msgs::PointCloud2>(edge_topic, 2);
    surf_ = nh_.advertise<sensor_msgs::PointCloud2>(surf_topic, 2);
    filtered_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1);
    transformed_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_transformed", 1);
    sub_ = nh_.subscribe(topic, 2, &FloamFrontend::scan, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fast_livo2_floam_frontend");
  FloamFrontend frontend;
  ros::spin();
  return 0;
}
