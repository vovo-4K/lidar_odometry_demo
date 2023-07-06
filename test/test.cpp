#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <numbers>

#include "../src/cloud_matcher.h"
#include "../src/voxel_grid.h"
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

    VoxelGrid voxel_grid(0.5);
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

    VoxelGrid voxel_grid(0.5);
    voxel_grid.addCloud(input_cloud);

    ASSERT_EQ(voxel_grid.size(), size_t(2));

    auto output_cloud = voxel_grid.getCloud();

    ASSERT_EQ(output_cloud->size(), size_t(2));

    ASSERT_FALSE(exactPointCmp(output_cloud->points.at(0), output_cloud->points.at(1)));
}

TEST(CloudMatcher, MatchingTest)
{
    auto full_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>("lidar_odometry_test_data/intersection00056.pcd", *full_cloud);

    VoxelGrid keyframe(0.2);
    keyframe.addCloud(*full_cloud);

    VoxelGrid voxel_filter(1.0);
    voxel_filter.addCloud(*full_cloud);

    auto subsampled_cloud = voxel_filter.getCloud();
    CloudMatcher matcher;

    std::vector<Pose3D> guess_poses{
        Pose3D({0.0, 0.0, 0.2}, Eigen::Quaternionf::Identity()),
        Pose3D({0.1, 0.1, 0.1}, Eigen::Quaternionf::Identity()),
        Pose3D({-0.1, -0.1, -0.1}, Eigen::Quaternionf::Identity()),
        Pose3D({0.1, -0.05, 0}, Eigen::Quaternionf::Identity()),
        Pose3D({0.0, 0.0, 0.0}, Eigen::Quaternionf(Eigen::AngleAxisf(-10.0*std::numbers::pi/180.0, Eigen::Vector3f(0,0,1))))
    };

    for (const auto& guess_pose : guess_poses) {
        std::cerr<<"guess transform: t: "<<guess_pose.translation.transpose() <<" q: "<<guess_pose.rotation<<std::endl;
        auto guess_cloud = CloudTransformer::transform(*subsampled_cloud, guess_pose.inverse());
        auto final_transform = matcher.align(keyframe, *guess_cloud, Pose3D());

        std::cout<<final_transform.translation.transpose()<<std::endl;

        auto error = final_transform.relativeTo(guess_pose);

        std::cout<<final_transform.rotation<<std::endl;
        std::cout<<guess_pose.rotation<<std::endl;

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