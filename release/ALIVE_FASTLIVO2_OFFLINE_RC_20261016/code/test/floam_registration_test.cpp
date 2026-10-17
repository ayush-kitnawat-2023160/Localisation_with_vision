#include <gtest/gtest.h>
#include "odomEstimationClass.h"

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "floam_registration_test", ros::init_options::AnonymousName);
    ros::Time::init();
    return RUN_ALL_TESTS();
}

Cloud::Ptr scene(const Eigen::Vector3d& offset, bool ground_only = false)
{
    Cloud::Ptr cloud(new Cloud);
    for (int i = -25; i <= 25; ++i)
        for (int j = -25; j <= 25; ++j)
        {
            auto add = [&](double x, double y, double z) {
                pcl::PointXYZI p;
                p.x = x + offset.x(); p.y = y + offset.y(); p.z = z + offset.z();
                p.intensity = 1.; cloud->push_back(p);
            };
            add(i * .2, j * .2, 0.);
            if (!ground_only)
            {
                add(5., i * .2, (j + 25) * .1);
                add(i * .2, 5., (j + 25) * .1);
            }
        }
    return cloud;
}

void verifyRecovery(const Eigen::Vector3d& offset)
{
    OdomEstimationClass estimator;
    estimator.init(lidar::Lidar(), .2);
    estimator.setSolverOptions(1, 12);
    Cloud::Ptr empty(new Cloud);
    estimator.initMapWithPoints(empty, scene(offset));
    estimator.odom.translation() = offset + Eigen::Vector3d(.22, -.18, .16);
    estimator.odom.linear() = Eigen::AngleAxisd(.025, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    estimator.last_odom = estimator.odom;
    estimator.setExternalPrediction(estimator.odom);
    estimator.updatePointsToMap(empty, scene(Eigen::Vector3d::Zero()), true);
    ASSERT_TRUE(estimator.registration_valid);
    EXPECT_LT((estimator.odom.translation() - offset).norm(), .06);
    EXPECT_LT(Eigen::AngleAxisd(estimator.odom.linear()).angle(), .01);
}

TEST(FloamRegistration, RecoversPlanesThroughOrigin) { verifyRecovery(Eigen::Vector3d::Zero()); }
TEST(FloamRegistration, RecoversAtCampusCoordinates) { verifyRecovery(Eigen::Vector3d(-467., 334., 38.)); }

TEST(FloamRegistration, SearchesAndRefinesARoughSeed)
{
    OdomEstimationClass estimator;
    estimator.init(lidar::Lidar(), .2);
    estimator.setSolverOptions(1, 12);
    Cloud::Ptr empty(new Cloud);
    const Eigen::Vector3d offset(-467., 334., 38.);
    estimator.initMapWithPoints(empty, scene(offset));
    estimator.odom.translation() = offset + Eigen::Vector3d(.8, -.9, 1.2);
    estimator.odom.linear() = Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    ASSERT_TRUE(estimator.initializeFromSeed(empty, scene(Eigen::Vector3d::Zero()), 2., 10., 2.));
    EXPECT_LT((estimator.odom.translation() - offset).norm(), .1);
    EXPECT_LT(Eigen::AngleAxisd(estimator.odom.linear()).angle(), .02);
}

TEST(FloamRegistration, RejectsUnobservableGroundAndRetainsPose)
{
    OdomEstimationClass estimator;
    estimator.init(lidar::Lidar(), .2);
    Cloud::Ptr empty(new Cloud);
    estimator.initMapWithPoints(empty, scene(Eigen::Vector3d::Zero(), true));
    estimator.odom.translation().z() = .15;
    const Eigen::Matrix4d before = estimator.odom.matrix();
    estimator.setExternalPrediction(estimator.odom);
    estimator.updatePointsToMap(empty, scene(Eigen::Vector3d::Zero(), true), true);
    EXPECT_FALSE(estimator.registration_valid);
    EXPECT_LT((estimator.odom.matrix() - before).norm(), 1.e-12);
}

TEST(FloamRegistration, SparseMapEdgesDoNotVetoObservableSurfaces)
{
    OdomEstimationClass estimator;
    estimator.init(lidar::Lidar(), .2);
    Cloud::Ptr empty(new Cloud), unmatched_edges(new Cloud);
    auto surfaces = scene(Eigen::Vector3d::Zero());
    estimator.initMapWithPoints(empty, surfaces);
    for (int i = 0; i < 50000; ++i)
    {
        pcl::PointXYZI point;
        point.x = 1000.0 + i * .3; point.y = 0.; point.z = 1.;
        unmatched_edges->push_back(point);
    }
    estimator.setExternalPrediction(estimator.odom);
    estimator.updatePointsToMap(unmatched_edges, surfaces, true);
    EXPECT_TRUE(estimator.registration_valid);
    EXPECT_GT(estimator.matched_fraction, .8);
}

TEST(FloamRegistration, RejectsNoOverlapAndCanRetry)
{
    OdomEstimationClass estimator;
    estimator.init(lidar::Lidar(), .2);
    Cloud::Ptr empty(new Cloud);
    auto scan = scene(Eigen::Vector3d::Zero());
    estimator.initMapWithPoints(empty, scan);
    estimator.odom.translation().x() = 100.;
    const Eigen::Matrix4d before = estimator.odom.matrix();
    estimator.setExternalPrediction(estimator.odom);
    estimator.updatePointsToMap(empty, scan, true);
    EXPECT_FALSE(estimator.registration_valid);
    EXPECT_LT((estimator.odom.matrix() - before).norm(), 1.e-12);
    estimator.odom = Eigen::Isometry3d::Identity();
    estimator.setExternalPrediction(estimator.odom);
    estimator.updatePointsToMap(empty, scan, true);
    EXPECT_TRUE(estimator.registration_valid);
}
