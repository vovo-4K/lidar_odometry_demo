#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <numbers>

#include "../src/cloud_matcher.h"
#include "../src/utils/cloud_transform.h"


bool exactPointCmp(const pcl::PointXYZ& point_a, const pcl::PointXYZ& point_b)
{
    return point_a.x == point_b.x &&
           point_a.y == point_b.y &&
           point_a.z == point_b.z;
}

TEST(VoxelGrid, UniquePoints)
{
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    input_cloud.points.emplace_back(0,0,0);
    input_cloud.points.emplace_back(1,0,0);
    input_cloud.points.emplace_back(0,1,0);
    input_cloud.points.emplace_back(0,0,1);
    input_cloud.points.emplace_back(-1,0,0);
    input_cloud.points.emplace_back(0,-1,0);
    input_cloud.points.emplace_back(0,0,-1);

    VoxelGrid<VoxelWithPoints<1>> voxel_grid(0.5, 1);
    voxel_grid.addCloud(input_cloud);

    ASSERT_EQ(voxel_grid.size(), input_cloud.size());

    auto output_cloud = voxel_grid.getCloud();
    ASSERT_EQ(input_cloud.size(), output_cloud->size());

    for (const pcl::PointXYZ &output_point : output_cloud->points) {
        auto search_point_it = std::find_if(input_cloud.points.begin(), input_cloud.points.end(),
                                            [&output_point](const pcl::PointXYZ& input_point){
                                                return exactPointCmp(output_point, input_point);
                                            });

        ASSERT_NE(search_point_it, input_cloud.end());

        input_cloud.points.erase(search_point_it);
    }
}

TEST(VoxelGrid, DuplicatePoints)
{
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    input_cloud.points.emplace_back(0,0,0);
    input_cloud.points.emplace_back(1,0,0);
    input_cloud.points.emplace_back(0,0,0);
    input_cloud.points.emplace_back(1,0,0);

    VoxelGrid<VoxelWithPoints<1>> voxel_grid(0.5, 1);
    voxel_grid.addCloud(input_cloud);

    ASSERT_EQ(voxel_grid.size(), size_t(2));

    auto output_cloud = voxel_grid.getCloud();

    ASSERT_EQ(output_cloud->size(), size_t(2));

    ASSERT_FALSE(exactPointCmp(output_cloud->points.at(0), output_cloud->points.at(1)));
}

TEST(Pose3D, ComposeRelativeInverse)
{
    std::vector<std::tuple<Pose3D, Pose3D>> test_data{
            {
                    Pose3D({0,0,0}, {1,0,0,0}),
                    Pose3D({0,0,0}, {1,0,0,0})
            },
            {
                    Pose3D({0,0,0}, Eigen::Quaternionf{Eigen::AngleAxisf(0.2, Eigen::Vector3f(0, 0, 1))}),
                    Pose3D({0,0,0}, Eigen::Quaternionf{Eigen::AngleAxisf(0.2, Eigen::Vector3f(0, 0, 1))})
            },
            {
                    Pose3D({0,0,0}, Eigen::Quaternionf{Eigen::AngleAxisf(0, Eigen::Vector3f(0, 0, 1))}),
                    Pose3D({1,0,0}, Eigen::Quaternionf{Eigen::AngleAxisf(std::numbers::pi * 0.5, Eigen::Vector3f(0, 0, 1))})
            },
            {
                    Pose3D({1,0,0}, Eigen::Quaternionf{Eigen::AngleAxisf(0, Eigen::Vector3f(0, 0, 1))}),
                    Pose3D({1,1,1}, Eigen::Quaternionf{Eigen::AngleAxisf(-std::numbers::pi, Eigen::Vector3f(0, 0, 1))})
            },
            {
                    Pose3D({100,100,100}, Eigen::Quaternionf{Eigen::AngleAxisf(0, Eigen::Vector3f(0, 0, 1).normalized())}),
                    Pose3D({150,150,150}, Eigen::Quaternionf{Eigen::AngleAxisf(0, Eigen::Vector3f(0, 0, 1).normalized())})
            },
            {
                    Pose3D({100,100,100}, Eigen::Quaternionf{Eigen::AngleAxisf(0.1, Eigen::Vector3f(0, 0, 1).normalized())}),
                    Pose3D({150,150,150}, Eigen::Quaternionf{Eigen::AngleAxisf(-0.2, Eigen::Vector3f(0, 0, 1).normalized())})
            },
            {
                    Pose3D({1,0.5,-0.5}, Eigen::Quaternionf{Eigen::AngleAxisf(0.456, Eigen::Vector3f(0.1, 0.2, 1).normalized())}),
                    Pose3D({-1,-0.6,0}, Eigen::Quaternionf{Eigen::AngleAxisf(-0.245, Eigen::Vector3f(-0.2, 0, 0).normalized())})
            },
    };

    for (const auto& test_case : test_data) {
        const auto& pose1 = std::get<0>(test_case);
        const auto& pose2 = std::get<1>(test_case);

        auto result_compose = pose1.compose(pose2);
        auto result_relative = pose1.relativeTo(pose2);

        auto result_inverse1 = pose1.inverse();
        auto result_inverse2 = pose2.inverse();

        Eigen::Isometry3f eigen_transform1 = Eigen::Isometry3f::Identity();
        eigen_transform1.translate(pose1.translation);
        eigen_transform1.rotate(pose1.rotation);

        Eigen::Isometry3f eigen_transform2 =  Eigen::Isometry3f::Identity();
        eigen_transform2.translate(pose2.translation);
        eigen_transform2.rotate(pose2.rotation);

        auto eigen_compose = eigen_transform1*eigen_transform2;
        auto eigen_relative = eigen_transform1.inverse() * eigen_transform2;

        auto eigen_inverse1 = eigen_transform1.inverse();
        auto eigen_inverse2 = eigen_transform2.inverse();

        //std::cout<<pose1.translation.transpose()<<"|"<<pose2.translation.transpose()
        //         <<"|"<<result_relative.translation.transpose()<<"|"<<eigen_relative.translation().transpose()<<std::endl;

        ASSERT_LT((eigen_compose.translation() - result_compose.translation).norm(), 1e-6);
        ASSERT_FLOAT_EQ(fabs(result_compose.rotation.dot(Eigen::Quaternionf(eigen_compose.rotation()))), 1.0);

        ASSERT_LT((eigen_relative.translation() - result_relative.translation).norm(), 1e-4);
        ASSERT_FLOAT_EQ(fabs(result_relative.rotation.dot(Eigen::Quaternionf(eigen_relative.rotation()))), 1.0);

        ASSERT_LT((eigen_inverse1.translation() - result_inverse1.translation).norm(), 1e-4);
        ASSERT_FLOAT_EQ(fabs(result_inverse1.rotation.dot(Eigen::Quaternionf(eigen_inverse1.rotation()))), 1.0);

        ASSERT_LT((eigen_inverse2.translation() - result_inverse2.translation).norm(), 1e-4);
        ASSERT_FLOAT_EQ(fabs(result_inverse2.rotation.dot(Eigen::Quaternionf(eigen_inverse2.rotation()))), 1.0);
    }
}

TEST(VoxelWithPlanes, PlaneParameters)
{
    using PlaneParams = std::pair<Eigen::Vector3f, Eigen::Vector3f>; // origin, normal
    using PlanePoints = std::vector<Eigen::Vector3f>;

    std::vector<std::pair<PlanePoints, PlaneParams>> test_cases =
            {
                    // z perpendicular
                   {{{0,0,0}, {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}},
                     {{0,0,0}, {0,0,1}}},
                    // x perpendicular
                    {{{0,20,0}, {0,30,0}, {0,10,0}, {0,20,-50}, {0,20,50}},
                     {{0,20,0}, {1,0,0}}},
                    // y perpendicular
                    {{{5,0,0}, {4,0,0}, {6,0,0}, {5,0,-1}, {5,0,1}},
                     {{5,0,0}, {0,-1,0}}},

            };

    for (const auto &test_case : test_cases) {
        VoxelWithPlanes<5> voxel;
        for (const auto &point : test_case.first) {
            voxel.addPoint(point(0), point(1), point(2));
        }

        auto origin_error = (voxel.plane_origin - test_case.second.first).norm();
        auto normal_error = std::abs(voxel.plane_normal.dot(test_case.second.second));

        ASSERT_LT(origin_error, 0.01);
        ASSERT_GE(normal_error, 0.99);
    }

}

TEST(CloudMatcher, MatchingTest)
{
    auto full_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>("lidar_odometry_test_data/intersection00056.pcd", *full_cloud);

    Keyframe keyframe(0.25, 0.1);
    keyframe.addClouds(*full_cloud, pcl::PointCloud<pcl::PointXYZ>());

    CloudMatcher matcher;

    std::vector<Pose3D> guess_poses{
            Pose3D({0.0, 0.0, 0.1}, Eigen::Quaternionf::Identity()),
            Pose3D({0.1, 0.1, 0.1}, Eigen::Quaternionf::Identity()),
            Pose3D({-0.1, -0.1, -0.1}, Eigen::Quaternionf::Identity()),
            Pose3D({0.1, -0.1, 0}, Eigen::Quaternionf::Identity()),
            Pose3D({0.0, 0.0, 0.0}, Eigen::Quaternionf(Eigen::AngleAxisf(-10.0*std::numbers::pi/180.0, Eigen::Vector3f(0,0,1)))),
            Pose3D({-0.2, 0.0, 0.0}, Eigen::Quaternionf(Eigen::AngleAxisf(12.0*std::numbers::pi/180.0, Eigen::Vector3f(0,0,1)))),
    };

    for (const auto& guess_pose : guess_poses) {
        std::cerr<<"guess transform: t: "<<guess_pose.translation.transpose() <<" q: "<<guess_pose.rotation<<std::endl;

        auto guess_cloud = CloudTransformer::transform(*keyframe.getCloud(), guess_pose.inverse());

        VoxelGrid<VoxelWithPoints<1>> voxel_filter(0.5, 1);
        voxel_filter.addCloud(*guess_cloud);

        auto subsampled_cloud = voxel_filter.getCloud();

        auto final_transform = matcher.align(keyframe, *subsampled_cloud, pcl::PointCloud<pcl::PointXYZ>(), Pose3D());

        //std::cout<<final_transform.translation.transpose()<<std::endl;

        auto error = final_transform.relativeTo(guess_pose);

        //std::cout<<final_transform.rotation<<std::endl;
        //std::cout<<guess_pose.rotation<<std::endl;

        auto rotation_error = 1.0 - fabs(final_transform.rotation.dot(guess_pose.rotation));

        ASSERT_LT(error.translation.norm(), 0.01);
        ASSERT_LT(rotation_error, 0.01);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}