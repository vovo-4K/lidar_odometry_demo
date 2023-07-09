#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <numbers>

#include "../src/cloud_matcher.h"
#include "../src/utils/cloud_transform.h"


bool exactPointCmp(const pcl::PointNormal& point_a, const pcl::PointNormal& point_b)
{
    return point_a.x == point_b.x &&
           point_a.y == point_b.y &&
           point_a.z == point_b.z;
}

TEST(VoxelGrid, UniquePoints)
{
    pcl::PointCloud<pcl::PointNormal> input_cloud;
    input_cloud.points.emplace_back(0,0,0);
    input_cloud.points.emplace_back(1,0,0);
    input_cloud.points.emplace_back(0,1,0);
    input_cloud.points.emplace_back(0,0,1);
    input_cloud.points.emplace_back(-1,0,0);
    input_cloud.points.emplace_back(0,-1,0);
    input_cloud.points.emplace_back(0,0,-1);

    VoxelGrid voxel_grid(0.5, 1);
    voxel_grid.addCloud(input_cloud);

    ASSERT_EQ(voxel_grid.size(), input_cloud.size());

    auto output_cloud = voxel_grid.getCloud();
    ASSERT_EQ(input_cloud.size(), output_cloud->size());

    for (const auto &output_point : output_cloud->points) {
        auto search_point_it = std::find_if(input_cloud.points.begin(), input_cloud.points.end(),
                                            [&output_point](const auto& input_point){
                                                return exactPointCmp(output_point, input_point);
                                            });

        ASSERT_NE(search_point_it, input_cloud.end());

        input_cloud.points.erase(search_point_it);
    }
}

TEST(VoxelGrid, DuplicatePoints)
{
    pcl::PointCloud<pcl::PointNormal> input_cloud;
    input_cloud.points.emplace_back(0,0,0);
    input_cloud.points.emplace_back(1,0,0);
    input_cloud.points.emplace_back(0,0,0);
    input_cloud.points.emplace_back(1,0,0);

    VoxelGrid voxel_grid(0.5, 1);
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
/*
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
        VoxelWithPlanes voxel;
        for (const auto &point : test_case.first) {
            voxel.addPoint(point(0), point(1), point(2));
        }

        auto origin_error = (voxel.plane_origin - test_case.second.first).norm();
        auto normal_error = std::abs(voxel.plane_normal.dot(test_case.second.second));

        ASSERT_LT(origin_error, 0.01);
        ASSERT_GE(normal_error, 0.99);
    }

}
 */

TEST(CloudMatcher, MatchingTest)
{
    auto full_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>("lidar_odometry_test_data/scan_005_subsampled_crop.pcd", *full_cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(full_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.25);
    //ne.setKSearch(20);
    ne.compute(*normals_cloud);

    auto full_cloud_with_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    full_cloud_with_normals->points.reserve(full_cloud->size());
    for (size_t i=0; i < full_cloud->points.size(); i++) {
        pcl::PointNormal output_point;
        output_point.x = full_cloud->points.at(i).x;
        output_point.y = full_cloud->points.at(i).y;
        output_point.z = full_cloud->points.at(i).z;

        output_point.normal_x = normals_cloud->points.at(i).normal_x;
        output_point.normal_y = normals_cloud->points.at(i).normal_y;
        output_point.normal_z = normals_cloud->points.at(i).normal_z;

        if (std::isnan(output_point.normal_x) || std::isnan(output_point.normal_y) || std::isnan(output_point.normal_z)) {
            continue;
        }

        full_cloud_with_normals->points.push_back(output_point);
    }

    VoxelGrid keyframe(0.25, 20);
    keyframe.addCloud(*full_cloud_with_normals);

    VoxelGrid voxel_filter(0.5, 1);
    voxel_filter.addCloudWithoutNormals(*full_cloud);
    auto subsampled_cloud = voxel_filter.getCloudWithoutNormals();

    CloudMatcher matcher;

    std::vector<Pose3D> guess_poses{
            Pose3D({0.0, 0.0, 0.0}, Eigen::Quaternionf::Identity()),
            Pose3D({0.0, 0.0, 0.1}, Eigen::Quaternionf::Identity()),
            Pose3D({0.1, 0.1, 0.1}, Eigen::Quaternionf::Identity()),
            Pose3D({-0.1, -0.1, -0.1}, Eigen::Quaternionf::Identity()),
            Pose3D({0.1, -0.1, 0}, Eigen::Quaternionf::Identity()),
            Pose3D({0.0, 0.0, 0.0}, Eigen::Quaternionf(Eigen::AngleAxisf(-5.0*std::numbers::pi/180.0, Eigen::Vector3f(0,0,1)))),
            Pose3D({-0.2, 0.0, 0.0}, Eigen::Quaternionf(Eigen::AngleAxisf(12.0*std::numbers::pi/180.0, Eigen::Vector3f(0,0,1)))),
    };

    for (const auto& guess_pose : guess_poses) {
        std::cerr<<"guess transform: t: "<<guess_pose.translation.transpose() <<" q: "<<guess_pose.rotation<<std::endl;

        auto guess_cloud = CloudTransformer::transform(*subsampled_cloud, guess_pose.inverse());

        auto final_transform = matcher.align(keyframe, *guess_cloud, Pose3D());

        std::cout<<final_transform.translation.transpose()<<std::endl;

        auto error = final_transform.relativeTo(guess_pose);

        std::cout<<final_transform.rotation<<std::endl;
        //std::cout<<guess_pose.rotation<<std::endl;

        auto rotation_error = 1.0 - fabs(final_transform.rotation.dot(guess_pose.rotation));

        ASSERT_LT(error.translation.norm(), 0.05);
        ASSERT_LT(rotation_error, 0.01);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}