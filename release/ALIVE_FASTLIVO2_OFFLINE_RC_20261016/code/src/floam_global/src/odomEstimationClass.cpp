// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/octree/octree_pointcloud.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>

void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution){
    //init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    odom = Eigen::Isometry3d::Identity();   
    last_odom = Eigen::Isometry3d::Identity();
    optimization_count=4;
    map_resolution_ = map_resolution;
    
}

void OdomEstimationClass::initMapWithPoints(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in,
    bool use_dense_edge_candidates){
    if (use_dense_edge_candidates)
    {
        // A global PCD is not an ordered LiDAR sweep, so the legacy FLOAM
        // scan-line feature selector produces only ~1.9k edge candidates for
        // the whole campus.  However, treating every dense map point as an
        // edge candidate is also wrong: planar neighbourhoods can then create
        // spurious point-to-line residuals and pull the pose catastrophically.
        //
        // Build a structural line map instead.  First voxelize the dense PCD,
        // then retain only points whose local PCA neighbourhood is strongly
        // one-dimensional (largest eigenvalue dominates the second largest).
        // These are suitable candidates for FLOAM's point-to-line residuals.
        pcl::PointCloud<pcl::PointXYZI>::Ptr dense(
            new pcl::PointCloud<pcl::PointXYZI>());
        *dense += *surf_in;
        *dense += *edge_in;

        pcl::PointCloud<pcl::PointXYZI>::Ptr reduced(
            new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> coarse_filter;
        const float coarse_leaf = static_cast<float>(
            std::max(0.6, 2.0 * map_resolution_));
        coarse_filter.setLeafSize(coarse_leaf, coarse_leaf, coarse_leaf);
        coarse_filter.setInputCloud(dense);
        coarse_filter.filter(*reduced);

        pcl::KdTreeFLANN<pcl::PointXYZI> structural_tree;
        structural_tree.setInputCloud(reduced);
        pcl::PointCloud<pcl::PointXYZI>::Ptr structural_lines(
            new pcl::PointCloud<pcl::PointXYZI>());
        structural_lines->reserve(reduced->size() / 8 + edge_in->size());

        constexpr int kNeighbours = 8;
        constexpr double kLinearityRatio = 6.0;
        constexpr double kMaxNeighbourSqDistance = 4.0;  // 2 m
        std::vector<int> indices(kNeighbours);
        std::vector<float> sq_distances(kNeighbours);

        for (const auto& point : reduced->points)
        {
            const int found = structural_tree.nearestKSearch(
                point, kNeighbours, indices, sq_distances);
            if (found < kNeighbours ||
                sq_distances[found - 1] > kMaxNeighbourSqDistance)
                continue;

            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (int j = 0; j < found; ++j)
            {
                const auto& neighbour = reduced->points[indices[j]];
                center += Eigen::Vector3d(neighbour.x, neighbour.y, neighbour.z);
            }
            center /= static_cast<double>(found);

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (int j = 0; j < found; ++j)
            {
                const auto& neighbour = reduced->points[indices[j]];
                const Eigen::Vector3d delta(
                    neighbour.x - center.x(),
                    neighbour.y - center.y(),
                    neighbour.z - center.z());
                covariance.noalias() += delta * delta.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
            if (solver.info() != Eigen::Success ||
                !solver.eigenvalues().allFinite())
                continue;

            const Eigen::Vector3d eigenvalues = solver.eigenvalues();
            const double largest = eigenvalues[2];
            const double second = eigenvalues[1];
            if (largest > 1.0e-6 &&
                largest > kLinearityRatio * std::max(second, 1.0e-9))
                structural_lines->push_back(point);
        }

        // Preserve the few legacy edge points too; they are cheap and may
        // contain useful structure where PCA becomes sparse.
        *structural_lines += *edge_in;

        pcl::VoxelGrid<pcl::PointXYZI> final_edge_filter;
        const float final_leaf = static_cast<float>(
            std::max(0.2, map_resolution_));
        final_edge_filter.setLeafSize(final_leaf, final_leaf, final_leaf);
        final_edge_filter.setInputCloud(structural_lines);
        final_edge_filter.filter(*laserCloudCornerMap);

        ROS_INFO("FLOAM structural edge extraction: dense=%zu, reduced=%zu, line candidates=%zu",
                 dense->size(), reduced->size(), laserCloudCornerMap->size());
    }
    else
    {
        *laserCloudCornerMap += *edge_in;
    }
    pcl::VoxelGrid<pcl::PointXYZI> surface_filter;
    surface_filter.setLeafSize(map_resolution_, map_resolution_, map_resolution_);
    surface_filter.setInputCloud(surf_in);
    surface_filter.filter(*laserCloudSurfMap);
    if (!laserCloudCornerMap->empty()) kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
    kdtreeSurfMap->setInputCloud(laserCloudSurfMap);
    ROS_INFO("FLOAM association maps: edge candidates=%zu, surfaces=%zu, dense edge mode=%s",
             laserCloudCornerMap->size(), laserCloudSurfMap->size(),
             use_dense_edge_candidates ? "true" : "false");
}

void OdomEstimationClass::setExternalPrediction(const Eigen::Isometry3d& prediction){
    external_prediction = prediction;
    external_prediction_valid = true;
}

void OdomEstimationClass::setSolverOptions(int num_threads, int max_iterations){
    solver_num_threads = std::max(1, num_threads);
    solver_max_iterations = std::max(1, max_iterations);
}


void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in, bool relocalize){
    const Eigen::Isometry3d previous_pose = odom;
    registration_valid = false;
    matched_points = 0;
    matched_fraction = 0.0;
    geometry_ratio = 0.0;
    auto start = std::chrono::high_resolution_clock::now();
    final_cost = std::numeric_limits<double>::infinity();
    average_cost = std::numeric_limits<double>::infinity();

    // Use synchronized FAST relative motion when available; otherwise use
    // FLOAM's last accepted motion. Acquisition explicitly supplies a fixed
    // prediction so registration corrections are never treated as velocity.
    Eigen::Isometry3d odom_prediction = external_prediction_valid
        ? external_prediction
        : odom * (last_odom.inverse() * odom);
    external_prediction_valid = false;

    last_odom = odom;
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();
    // std::cout << "q:\n" << q_w_curr.matrix() << "\nt:\n" << t_w_curr.matrix() << "\n";

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_s =  std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    ROS_DEBUG_STREAM("down sampling: " << elapsed_s.count());
    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    
    if(laserCloudSurfMap->points.size()>50){
        // start = std::chrono::high_resolution_clock::now();
        // since the entire map is initially set as KdInputCloud during initialisation
        // we don't need to recreate
        // kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        // kdtreeSurfMap->setInputCloud(laserCloudSurfMap);
        // end = std::chrono::high_resolution_clock::now();
        // elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // ROS_DEBUG_STREAM("kdTree set: " << elapsed_s.count());
        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            start = std::chrono::high_resolution_clock::now();
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            information.setZero();
            if (laserCloudCornerMap->size() >= 5)
                addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
            addSurfCostFactor(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function);
            const size_t edge_blocks_num = edge_blocks.size();
            const size_t surface_blocks_num = surface_blocks.size();
            const size_t residual_blocks_num = edge_blocks_num + surface_blocks_num;
            end = std::chrono::high_resolution_clock::now();
            elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);;
            ROS_DEBUG_STREAM("edge and surface cost: " << elapsed_s.count());

            matched_points = residual_blocks_num;
            // A saved map may contain very few structural lines. Evaluate
            // coverage per feature family so unmatched scan edges cannot
            // veto well-supported, fully observable surface registration.
            matched_fraction = std::max(
                static_cast<double>(surface_blocks_num) / std::max<size_t>(1, downsampledSurfCloud->size()),
                static_cast<double>(edge_blocks_num) / std::max<size_t>(1, downsampledEdgeCloud->size()));
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> geometry(information);
            geometry_ratio = geometry.info() == Eigen::Success && geometry.eigenvalues()[5] > 0.0
                ? std::max(0.0, geometry.eigenvalues()[0]) / geometry.eigenvalues()[5] : 0.0;
            if (residual_blocks_num < 50 || geometry_ratio < 1.0e-5)
            {
                ROS_DEBUG("Insufficient scan-to-map support or unobservable geometry");
                registration_valid = false;
                if (residual_blocks_num == 0) delete loss_function;
                edge_blocks.clear();
                surface_blocks.clear();
                break;
            }

            const std::array<double, 7> parameters_before_solve = {
                parameters[0], parameters[1], parameters[2], parameters[3],
                parameters[4], parameters[5], parameters[6]};
            start = std::chrono::high_resolution_clock::now();
            ceres::Solver::Options options;
            options.num_threads = solver_num_threads;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = solver_max_iterations;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);
            end = std::chrono::high_resolution_clock::now();
            elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);;
            ROS_DEBUG_STREAM("ceres solver: " << elapsed_s.count());

            if (!summary.IsSolutionUsable() || !std::isfinite(summary.final_cost))
            {
                registration_valid = false;
                ROS_ERROR("Ceres returned an unusable/non-finite solution; retaining the motion prediction");
                std::copy(parameters_before_solve.begin(),
                          parameters_before_solve.end(), parameters);
            }
            else
            {
                final_cost = summary.final_cost;
                average_cost = final_cost / static_cast<double>(residual_blocks_num);
                registration_valid = average_cost <= 0.02;
            }
            edge_blocks.clear();
            surface_blocks.clear();
            // edge_costs.clear();
            // surface_costs.clear();
        }
    }
    else
    {
        ROS_WARN("not enough points in map to associate, map error");
    }
    // std::cout << "q:\n" << q_w_curr.matrix() << "\nt:\n" << t_w_curr.matrix() << "\n";
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;

    const double correction_angle = Eigen::AngleAxisd(
        odom_prediction.linear().transpose() * odom.linear()).angle();
    registration_valid = registration_valid && matched_fraction >= 0.15 && odom.matrix().allFinite() &&
        (odom.translation() - odom_prediction.translation()).norm() <= 2.0 &&
        correction_angle <= 15.0 * M_PI / 180.0;
    if (!registration_valid)
    {
        odom = previous_pose;
        last_odom = previous_pose;
    }
    // add points to map only when the translation difference between the previous and current lidar scan is sufficient
    //if((last_odom.translation() - odom.translation()).norm() > 0.30)
    //addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);
    if(!relocalize && registration_valid)
    {
        addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);
    }

}

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);    
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        const int K = 5;
        const int found = kdtreeEdgeMap->nearestKSearch(
            point_temp, K, pointSearchInd, pointSearchSqDis);
        // for points in range 1.0 sq m, find covariance matrix
        int min_avg = 3;
        int in_range = 0;
        for (int neighbor = 0; neighbor < found; ++neighbor)
        {
            if (pointSearchSqDis[neighbor] < 1.0)
            {
                in_range++;
            }
        }
        if (in_range >= min_avg)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < in_range; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);

                center = center + tmp;
                nearCorners.push_back(tmp);
            }

            center = center / static_cast<double>(in_range);

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < in_range; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
            if (saes.info() != Eigen::Success ||
                !saes.eigenvalues().allFinite() ||
                !saes.eigenvectors().allFinite())
                continue;

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            // eigen values are sorted in increasing order
            // https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html#aaf4ed4172a517a4b9f0ab222f629e261
            // for an edge we are requiring largest eigenvalue or variance magnitude must be atleast 3*x larger than other eigenvalues
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            { 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  
                ceres::ResidualBlockId id = problem.AddResidualBlock(cost_function, loss_function, parameters);
                edge_blocks.push_back(id);
                const Eigen::Vector3d normal = unit_direction.unitOrthogonal();
                addInformation(curr_point, normal);
                addInformation(curr_point, unit_direction.cross(normal));
                corner_num++;   
            }
            else
            {
                // ROS_WARN("Not really like an edge");
                ;
            }
        }
        else
        {
            // ROS_WARN("Not enough points in 1m radius for each point!");
        }
    }
    
    if(corner_num<20)
    {
        ROS_WARN_THROTTLE(5.0, "Not enough correct points, corner %d", corner_num);
    }
    else
        ROS_DEBUG("Corners available in scan %d", corner_num);

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        const int K = 5;
        const int found = kdtreeSurfMap->nearestKSearch(
            point_temp, K, pointSearchInd, pointSearchSqDis);

        // Ax + By + Cz + D = 0 or ax + by + cz + 1 = 0
        // ax + by + cz = -1
        int min_avg = 5;
        int in_range = 0;
        for (int neighbor = 0; neighbor < found; ++neighbor)
        {
            if (pointSearchSqDis[neighbor] < 1.0)
            {
                in_range++;
            }
        }
        if (in_range >= min_avg)
        {
            // Fit a centered plane. Ax=-1 cannot represent planes through
            // the map origin and becomes ill-conditioned at campus coordinates.
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (int j = 0; j < in_range; ++j)
                center += map_in->points[pointSearchInd[j]].getVector3fMap().cast<double>();
            center /= in_range;
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (int j = 0; j < in_range; ++j)
            {
                Eigen::Vector3d delta = map_in->points[pointSearchInd[j]].getVector3fMap().cast<double>() - center;
                covariance.noalias() += delta * delta.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> plane(covariance);
            if (plane.info() != Eigen::Success || !plane.eigenvalues().allFinite() ||
                plane.eigenvalues()[1] < 1.e-5 ||
                plane.eigenvalues()[0] > 0.1 * plane.eigenvalues()[1]) continue;
            const Eigen::Vector3d norm = plane.eigenvectors().col(0);
            const double negative_OA_dot_norm = -norm.dot(center);

            bool planeValid = true;
            for (int j = 0; j < in_range; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            // A nearby map neighbourhood can still be the wrong surface,
            // particularly on vegetation or an occlusion boundary.
            const Eigen::Vector3d predicted = q_w_curr * curr_point + t_w_curr;
            if (planeValid && std::abs(norm.dot(predicted) + negative_OA_dot_norm) <= 0.5)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                ceres::ResidualBlockId id = problem.AddResidualBlock(cost_function, loss_function, parameters);
                surface_blocks.push_back(id);
                addInformation(curr_point, norm);
                surf_num++;
            }
        }

    }
    if(surf_num<20){
        ROS_WARN_THROTTLE(5.0, "not enough correct points, surface %d", surf_num);
    }
    else
        ROS_DEBUG("Surfaces available in scan %d", surf_num);

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud){


    // keep the feature maps size as 10k
    // if the size crosses 10k -> remove the excess points from the starting

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp); 
    }
    
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
        laserCloudSurfMap->push_back(point_temp);
    }
/*
    int th = 20000;
    
    if(laserCloudCornerMap->size() > th)
    {
        std::cout<<"Edge feature vector size is more than 10000 "<<laserCloudCornerMap->size()-th<<std::endl;
        laserCloudCornerMap->erase(laserCloudCornerMap->begin(), laserCloudCornerMap->begin() + (laserCloudCornerMap->size() - th));
        std::cout<<"Edge feature vector size is (after erasing)"<<laserCloudCornerMap->size()<<std::endl;
    }

    if(laserCloudSurfMap->size() > th)
    {
        laserCloudSurfMap->erase(laserCloudSurfMap->begin() , laserCloudSurfMap->begin() +(laserCloudSurfMap->size() - th));
    }
*/
    std::cout<<"Edge map size is: "<<laserCloudCornerMap->size()<<std::endl;
    
    double x_min = +odom.translation().x()-100;
    double y_min = +odom.translation().y()-100;
    double z_min = +odom.translation().z()-100;
    double x_max = +odom.translation().x()+100;
    double y_max = +odom.translation().y()+100;
    double z_max = +odom.translation().z()+100;
    
    //ROS_DEBUG("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false);    

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    downSizeFilterSurf.setInputCloud(tmpSurf);
    downSizeFilterSurf.filter(*laserCloudSurfMap);
    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);

}

void OdomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap){
    
    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

OdomEstimationClass::OdomEstimationClass(){

}

void OdomEstimationClass::addInformation(const Eigen::Vector3d& point,
                                         const Eigen::Vector3d& normal)
{
    // Center at the sensor and scale the rotation lever arm to 10 m so map
    // translation (hundreds of metres) does not dominate the rank check.
    Eigen::Matrix<double, 6, 1> row;
    row.head<3>() = (q_w_curr * point).cross(normal) / 10.0;
    row.tail<3>() = normal;
    information.noalias() += row * row.transpose();
}

namespace {
// Ground is a proposal for roll/pitch/height, never proof of global position.
// Score only a bounded, deterministic sample and refine the best plane by PCA.
bool groundPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                 const Eigen::Vector3d& origin, Eigen::Vector3d& normal, double& offset)
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> nearby;
    for (const auto& point : cloud->points)
    {
        const Eigen::Vector3d p = point.getVector3fMap().cast<double>();
        const double radius = (p - origin).head<2>().squaredNorm();
        if (p.allFinite() && radius > 9.0 && radius < 625.0) nearby.push_back(p);
    }
    if (nearby.size() < 100) return false;
    std::vector<double> heights;
    for (const auto& p : nearby) heights.push_back(p.z());
    const size_t middle = heights.size() / 2;
    std::nth_element(heights.begin(), heights.begin() + middle, heights.end());
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
    const size_t stride = std::max<size_t>(1, nearby.size() / 3000);
    for (size_t i = 0; i < nearby.size(); i += stride)
        if (nearby[i].z() <= heights[middle]) points.push_back(nearby[i]);
    if (points.size() < 50) return false;
    std::mt19937 rng(7);
    std::uniform_int_distribution<size_t> pick(0, points.size() - 1);
    size_t best_count = 0;
    for (int iteration = 0; iteration < 400; ++iteration)
    {
        const Eigen::Vector3d a = points[pick(rng)];
        const Eigen::Vector3d b = points[pick(rng)];
        const Eigen::Vector3d c = points[pick(rng)];
        Eigen::Vector3d n = (b - a).cross(c - a);
        if (n.norm() < 1.e-6) continue;
        n.normalize();
        if (std::abs(n.z()) < 0.85) continue;
        if (n.z() < 0.0) n = -n;
        const double d = n.dot(a);
        size_t count = 0;
        for (const auto& p : points) if (std::abs(n.dot(p) - d) < 0.2) ++count;
        if (count > best_count) { best_count = count; normal = n; offset = d; }
    }
    if (best_count < 50 || best_count < points.size() / 5) return false;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> inliers;
    for (const auto& p : points)
        if (std::abs(normal.dot(p) - offset) < 0.2) { center += p; inliers.push_back(p); }
    center /= inliers.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& p : inliers)
    {
        const Eigen::Vector3d delta = p - center;
        covariance.noalias() += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 1.e-3) return false;
    normal = solver.eigenvectors().col(0);
    if (normal.z() < 0.) normal = -normal;
    offset = normal.dot(center);
    return normal.allFinite() && normal.z() >= .85;
}
}

bool OdomEstimationClass::initializeFromSeed(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edges,
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& surfaces,
                                            double xy_radius, double yaw_radius_deg, double z_radius)
{
    Eigen::Isometry3d seed = odom;
    Eigen::Vector3d map_normal, scan_normal;
    double map_offset, scan_offset;
    if (groundPlane(laserCloudSurfMap, seed.translation(), map_normal, map_offset) &&
        groundPlane(surfaces, Eigen::Vector3d::Zero(), scan_normal, scan_offset))
    {
        seed.linear() = Eigen::Quaterniond::FromTwoVectors(seed.linear() * scan_normal,
                                                          map_normal).toRotationMatrix() * seed.linear();
        seed.translation().z() = (map_offset - scan_offset -
            map_normal.head<2>().dot(seed.translation().head<2>())) / map_normal.z();
        ROS_INFO("FLOAM ground proposal: z=%.3f map normal [%.3f %.3f %.3f]",
                 seed.translation().z(), map_normal.x(), map_normal.y(), map_normal.z());
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr reduced(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setLeafSize(1.0f, 1.0f, 1.0f);
    filter.setInputCloud(surfaces);
    filter.filter(*reduced);
    if (reduced->size() < 50) return false;
    struct Candidate { double score; Eigen::Isometry3d pose; };
    std::vector<Candidate, Eigen::aligned_allocator<Candidate>> candidates;
    const int xy_steps = static_cast<int>(std::ceil(xy_radius / 2.0));
    const int yaw_steps = static_cast<int>(std::ceil(yaw_radius_deg / 10.0));
    const int z_steps = static_cast<int>(std::ceil(z_radius));
    const size_t stride = std::max<size_t>(1, reduced->size() / 200);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (int yaw = -yaw_steps; yaw <= yaw_steps; ++yaw)
    {
        const double angle = yaw_steps ? yaw * yaw_radius_deg / yaw_steps * M_PI / 180.0 : 0.0;
        const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() * seed.linear();
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rotated;
        for (size_t i = 0; i < reduced->size(); i += stride)
            rotated.push_back(rotation * reduced->points[i].getVector3fMap().cast<double>());
        for (int x = -xy_steps; x <= xy_steps; ++x)
            for (int y = -xy_steps; y <= xy_steps; ++y)
                for (int z = -z_steps; z <= z_steps; ++z)
                {
                    Candidate candidate;
                    candidate.pose = seed;
                    candidate.pose.linear() = rotation;
                    candidate.pose.translation() += Eigen::Vector3d(
                        xy_steps ? x * xy_radius / xy_steps : 0.0,
                        xy_steps ? y * xy_radius / xy_steps : 0.0,
                        z_steps ? z * z_radius / z_steps : 0.0);
                    candidate.score = 0.;
                    for (const auto& point : rotated)
                    {
                        const Eigen::Vector3d world = point + candidate.pose.translation();
                        pcl::PointXYZI query;
                        query.x = world.x(); query.y = world.y(); query.z = world.z();
                        if (kdtreeSurfMap->nearestKSearch(query, 1, indices, distances) == 1)
                            candidate.score += std::max(0.0, 1.0 - distances[0]);
                    }
                    candidate.score /= rotated.size();
                    candidates.push_back(candidate);
                }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.score > b.score;
    });
    const auto coarse = candidates;
    for (size_t i = 0; i < std::min<size_t>(4, coarse.size()); ++i)
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = -1; z <= 1; ++z)
                    for (int yaw = -1; yaw <= 1; ++yaw)
                    {
                        Candidate candidate = coarse[i];
                        candidate.pose.translation() += Eigen::Vector3d(x, y, .5 * z);
                        candidate.pose.linear() = Eigen::AngleAxisd(yaw * 5.0 * M_PI / 180.0,
                            Eigen::Vector3d::UnitZ()).toRotationMatrix() * candidate.pose.linear();
                        const Eigen::Vector3d delta = candidate.pose.translation() - seed.translation();
                        const double yaw_delta = std::atan2(
                            (candidate.pose.linear() * seed.linear().transpose())(1, 0),
                            (candidate.pose.linear() * seed.linear().transpose())(0, 0));
                        if (std::abs(delta.x()) > xy_radius || std::abs(delta.y()) > xy_radius ||
                            std::abs(delta.z()) > z_radius ||
                            std::abs(yaw_delta) > yaw_radius_deg * M_PI / 180.0 + 1.e-6) continue;
                        candidate.score = 0.;
                        size_t count = 0;
                        for (size_t k = 0; k < reduced->size(); k += stride)
                        {
                            const Eigen::Vector3d world = candidate.pose * reduced->points[k].getVector3fMap().cast<double>();
                            pcl::PointXYZI query;
                            query.x = world.x(); query.y = world.y(); query.z = world.z();
                            if (kdtreeSurfMap->nearestKSearch(query, 1, indices, distances) == 1)
                                candidate.score += std::max(0.0, 1.0 - distances[0]);
                            ++count;
                        }
                        candidate.score /= count;
                        candidates.push_back(candidate);
                    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.score > b.score;
    });
    ROS_INFO("FLOAM seed search: %zu candidates, best overlap score %.3f at %.2f %.2f %.2f",
             candidates.size(), candidates.front().score, candidates.front().pose.translation().x(),
             candidates.front().pose.translation().y(), candidates.front().pose.translation().z());
    double best_score = -1.0;
    Eigen::Isometry3d best_pose = seed;
    // Refine candidates with FLOAM's line/plane Ceres optimizer. The geometric
    // search is only a seed proposal and cannot itself declare localization.
    for (size_t i = 0; i < std::min<size_t>(8, candidates.size()); ++i)
    {
        odom = candidates[i].pose;
        last_odom = odom;
        setExternalPrediction(odom);
        updatePointsToMap(edges, surfaces, true);
        ROS_INFO("FLOAM seed refinement %zu: valid=%d fraction=%.3f cost=%.5f geometry=%.5f",
                 i, registration_valid, matched_fraction, average_cost, geometry_ratio);
        const double score = matched_fraction / (1.0 + 50.0 * average_cost);
        if (registration_valid && score > best_score)
        {
            best_score = score;
            best_pose = odom;
        }
    }
    ROS_INFO("FLOAM refined seed: score=%.3f xyz [%.3f %.3f %.3f] yaw=%.3f",
             best_score, best_pose.translation().x(), best_pose.translation().y(), best_pose.translation().z(),
             std::atan2(best_pose.linear()(1, 0), best_pose.linear()(0, 0)));
    odom = best_pose;
    last_odom = best_pose;
    registration_valid = best_score >= 0.0;
    return registration_valid;
}
