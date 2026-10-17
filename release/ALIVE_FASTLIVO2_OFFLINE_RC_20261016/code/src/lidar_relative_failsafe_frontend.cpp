#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>

class LidarRelativeFailsafeFrontend {
 public:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;

  LidarRelativeFailsafeFrontend() : pnh_("~") {
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/fast_livo2/input/lidar");
    pnh_.param<std::string>("mode_topic", mode_topic_, "/fast_livo2/failsafe_mode");
    pnh_.param<std::string>("output_topic", output_topic_, "/fast_livo2/relative_motion_hint");
    pnh_.param<double>("voxel_leaf", voxel_leaf_, 0.45);
    pnh_.param<double>("min_range", min_range_, 1.0);
    pnh_.param<double>("max_range", max_range_, 60.0);
    pnh_.param<int>("icp_iterations", icp_iterations_, 18);
    pnh_.param<double>("max_correspondence", max_correspondence_, 2.2);
    pnh_.param<double>("min_dt", min_dt_, 0.03);
    pnh_.param<double>("max_dt", max_dt_, 0.20);
    pnh_.param<double>("max_speed", max_speed_, 8.0);
    pnh_.param<double>("max_omega", max_omega_, 2.2);
    pnh_.param<double>("max_linear_accel", max_linear_accel_, 4.0);
    pnh_.param<double>("max_angular_accel", max_angular_accel_, 1.2);
    pnh_.param<double>("filter_alpha", filter_alpha_, 0.45);

    std::vector<double> r, t;
    ros::param::get("/extrin_calib/extrinsic_R", r);
    ros::param::get("/extrin_calib/extrinsic_T", t);
    Rbl_.setIdentity(); tbl_.setZero();
    if (r.size() == 9) {
      for (int i=0;i<3;++i) for (int j=0;j<3;++j) Rbl_(i,j)=r[3*i+j];
    }
    if (t.size() == 3) tbl_ << t[0],t[1],t[2];

    pub_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_, 20);
    mode_sub_ = nh_.subscribe(mode_topic_, 10, &LidarRelativeFailsafeFrontend::modeCb, this);
    // Stay completely unsubscribed from the high-bandwidth LiDAR stream while
    // healthy. modeCb attaches it only for LV_IMU_FAILED.
    ROS_INFO("[FAILSAFE LIDAR REL] ready dormant: %s -> %s", lidar_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void resetLocked() {
    prev_.reset(); prev_stamp_ = ros::Time(); filter_valid_ = false;
    filt_v_.setZero(); filt_w_.setZero();
  }

  void modeCb(const std_msgs::String::ConstPtr& msg) {
    if (!msg) return;
    bool should_activate = false;
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      changed = mode_ != msg->data;
      if (changed) {
        mode_ = msg->data;
        resetLocked();
      }
      should_activate = mode_ == "LV_IMU_FAILED";
    }
    if (should_activate && !lidar_active_) {
      lidar_sub_ = nh_.subscribe(lidar_topic_, 1,
          &LidarRelativeFailsafeFrontend::lidarCb, this,
          ros::TransportHints().tcpNoDelay());
      lidar_active_ = true;
      ROS_INFO("[FAILSAFE LIDAR REL] subscribed to LiDAR stream");
    } else if (!should_activate && lidar_active_) {
      lidar_sub_.shutdown();
      lidar_active_ = false;
      ROS_INFO("[FAILSAFE LIDAR REL] unsubscribed from LiDAR stream");
    }
    if (changed) ROS_INFO("[FAILSAFE LIDAR REL] mode -> %s", msg->data.c_str());
  }

  CloudT::Ptr prepare(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    CloudT::Ptr raw(new CloudT), ds(new CloudT), clean(new CloudT);
    pcl::fromROSMsg(*msg, *raw);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    vg.setInputCloud(raw); vg.filter(*ds);
    clean->reserve(ds->size());
    const double rmin2=min_range_*min_range_, rmax2=max_range_*max_range_;
    for (const auto &p:*ds) {
      const double r2=p.x*p.x+p.y*p.y+p.z*p.z;
      if (std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&r2>rmin2&&r2<rmax2)
        clean->push_back(p);
    }
    clean->width=clean->size(); clean->height=1; clean->is_dense=false;
    return clean;
  }

  void lidarCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!msg) return;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (mode_ != "LV_IMU_FAILED") return;
    }
    CloudT::Ptr scan=prepare(msg);
    if (!scan || scan->size()<150) return;

    CloudT::Ptr prev;
    ros::Time prev_stamp;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      prev=prev_; prev_stamp=prev_stamp_;
      prev_.reset(new CloudT(*scan)); prev_stamp_=msg->header.stamp;
    }
    if (!prev || prev_stamp.isZero() || msg->header.stamp.isZero()) return;
    const double dt=(msg->header.stamp-prev_stamp).toSec();
    if (dt<=min_dt_ || dt>=max_dt_) return;

    pcl::IterativeClosestPoint<PointT,PointT> icp;
    icp.setInputSource(scan); icp.setInputTarget(prev);
    icp.setMaximumIterations(icp_iterations_);
    icp.setMaxCorrespondenceDistance(max_correspondence_);
    icp.setTransformationEpsilon(1e-5);
    icp.setEuclideanFitnessEpsilon(1e-4);
    CloudT aligned; icp.align(aligned);
    if (!icp.hasConverged()) return;
    Eigen::Matrix4d D=icp.getFinalTransformation().cast<double>(); // prev lidar <- current lidar
    Eigen::Vector3d v_l=D.block<3,1>(0,3)/dt;
    Eigen::AngleAxisd aa(D.block<3,3>(0,0));
    Eigen::Vector3d w_l=Eigen::Vector3d::Zero();
    if (std::isfinite(aa.angle()) && aa.axis().allFinite()) w_l=aa.axis()*(aa.angle()/dt);
    if (!v_l.allFinite() || !w_l.allFinite() || v_l.norm()>=max_speed_ || w_l.norm()>=max_omega_) return;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!filter_valid_) { filt_v_=v_l; filt_w_=w_l; filter_valid_=true; }
      else {
        Eigen::Vector3d dv=v_l-filt_v_; double lim=max_linear_accel_*dt;
        if (dv.norm()>lim && dv.norm()>1e-9) dv*=lim/dv.norm();
        Eigen::Vector3d dw=w_l-filt_w_; double limw=max_angular_accel_*dt;
        if (dw.norm()>limw && dw.norm()>1e-9) dw*=limw/dw.norm();
        Eigen::Vector3d vc=filt_v_+dv, wc=filt_w_+dw;
        double a=std::max(0.0,std::min(1.0,filter_alpha_));
        filt_v_=(1.0-a)*filt_v_+a*vc; filt_w_=(1.0-a)*filt_w_+a*wc;
      }
      v_l=filt_v_; w_l=filt_w_;
    }

    Eigen::Matrix4d Df=Eigen::Matrix4d::Identity();
    Df.block<3,1>(0,3)=v_l*dt;
    const double ang=(w_l*dt).norm();
    if (ang>1e-12) Df.block<3,3>(0,0)=Eigen::AngleAxisd(ang,(w_l*dt)/ang).toRotationMatrix();

    Eigen::Matrix4d T_bl=Eigen::Matrix4d::Identity();
    T_bl.block<3,3>(0,0)=Rbl_; T_bl.block<3,1>(0,3)=tbl_;
    Eigen::Matrix4d Db=T_bl*Df*T_bl.inverse(); // prev body <- current body
    Eigen::Vector3d t_b=Db.block<3,1>(0,3);
    Eigen::AngleAxisd aab(Db.block<3,3>(0,0));
    Eigen::Vector3d rv_b=Eigen::Vector3d::Zero();
    if (std::isfinite(aab.angle()) && aab.axis().allFinite()) rv_b=aab.axis()*aab.angle();

    geometry_msgs::TwistStamped motion;
    motion.header.stamp=msg->header.stamp; motion.header.frame_id="lidar_relative_motion";
    Eigen::Vector3d vb=t_b/dt, wb=rv_b/dt;
    motion.twist.linear.x=vb.x(); motion.twist.linear.y=vb.y(); motion.twist.linear.z=vb.z();
    motion.twist.angular.x=wb.x(); motion.twist.angular.y=wb.y(); motion.twist.angular.z=wb.z();
    pub_.publish(motion);

    geometry_msgs::TwistStamped delta;
    delta.header.stamp=msg->header.stamp; delta.header.frame_id="lidar_relative_delta";
    delta.twist.linear.x=t_b.x(); delta.twist.linear.y=t_b.y(); delta.twist.linear.z=t_b.z();
    delta.twist.angular.x=rv_b.x(); delta.twist.angular.y=rv_b.y(); delta.twist.angular.z=rv_b.z();
    pub_.publish(delta);

    ++accepted_;
    ROS_INFO_THROTTLE(0.5,"[FAILSAFE LIDAR REL] accepted=%zu fit=%.3f dt=%.3f motion=%.2fm/%.1fdeg",
                      accepted_, icp.getFitnessScore(1.5), dt, t_b.norm(), rv_b.norm()*180.0/M_PI);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber lidar_sub_, mode_sub_; ros::Publisher pub_;
  std::string lidar_topic_, mode_topic_, output_topic_, mode_="FULL_LIVO";
  std::mutex mtx_; CloudT::Ptr prev_; ros::Time prev_stamp_;
  bool lidar_active_=false;
  bool filter_valid_=false; Eigen::Vector3d filt_v_=Eigen::Vector3d::Zero(), filt_w_=Eigen::Vector3d::Zero();
  Eigen::Matrix3d Rbl_=Eigen::Matrix3d::Identity(); Eigen::Vector3d tbl_=Eigen::Vector3d::Zero();
  double voxel_leaf_=0.45,min_range_=1.0,max_range_=60.0,max_correspondence_=2.2,min_dt_=0.03,max_dt_=0.20,max_speed_=8.0,max_omega_=2.2,max_linear_accel_=4.0,max_angular_accel_=1.2,filter_alpha_=0.45;
  int icp_iterations_=18; size_t accepted_=0;
};

int main(int argc,char**argv){
  ros::init(argc,argv,"fast_livo2_lidar_relative_failsafe");
  LidarRelativeFailsafeFrontend node;
  ros::AsyncSpinner spinner(2); spinner.start(); ros::waitForShutdown();
  return 0;
}
