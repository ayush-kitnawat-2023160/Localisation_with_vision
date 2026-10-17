// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

//std lib
#include <string>
#include <math.h>
#include <limits>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>

class OdomEstimationClass 
{

    public:
    	OdomEstimationClass();
    	
		void init(lidar::Lidar lidar_param, double map_resolution);	
		void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
		                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in,
		                       bool use_dense_edge_candidates = false);
		void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in, bool relocalize);
		// FAST-LIVO2 supplies the local-motion prediction.  The FLOAM map
		// association, KD-trees, and Ceres residuals remain unchanged.
		void setExternalPrediction(const Eigen::Isometry3d& prediction);
		void setSolverOptions(int num_threads, int max_iterations);
        bool initializeFromSeed(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edges,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr& surfaces,
                                double xy_radius, double yaw_radius_deg, double z_radius);
		void getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap);

		Eigen::Isometry3d odom;
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;
		Eigen::Isometry3d last_odom;
		bool registration_valid = false;
        size_t matched_points = 0;
        double matched_fraction = 0.0;
        double geometry_ratio = 0.0;
        double final_cost = std::numeric_limits<double>::infinity();
		double average_cost = std::numeric_limits<double>::infinity();
		std::vector<ceres::ResidualBlockId> edge_blocks, surface_blocks;

	private:
		// optimization variable in eigen style
		double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
		Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

		//kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfMap;

		//points downsampling before add to map
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;

		//local map
		pcl::CropBox<pcl::PointXYZI> cropBoxFilter;

		//optimization count 
		int optimization_count;
		int solver_num_threads = 2;
		int solver_max_iterations = 4;
		double map_resolution_ = 0.4;
		bool external_prediction_valid = false;
		Eigen::Isometry3d external_prediction = Eigen::Isometry3d::Identity();

		Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
        void addInformation(const Eigen::Vector3d& point, const Eigen::Vector3d& normal);
        //function
		void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud);
		void pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);
		void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out);

		double max_yawchange = INFINITY;
		double max_xchange = INFINITY;
		double max_ychange = INFINITY;
};

#endif // _ODOM_ESTIMATION_CLASS_H_
