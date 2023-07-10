//
// Created by vl on 29.06.23.
//

#ifndef BUILD_LIDARODOMETRY_H
#define BUILD_LIDARODOMETRY_H

#include <map>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>

#include "lidar_point_type.h"
#include "pose_3d.h"
#include "voxel_grid.h"

class LidarOdometry {
public:

    using CloudType = pcl::PointCloud<lidar_point::PointXYZIRT>;

    class Params {
    public:
        float lidar_min_range;
        float lidar_max_range;
        float keyframe_voxel_size;
        size_t keyframe_max_points_cnt;
        float keyframe_matching_voxel_size;
        float keyframe_update_voxel_size;
        float keyframe_cleanup_range;
        float angular_divergence_threshold;

        using ROS2ParametersMap = std::map<std::string, rclcpp::ParameterValue>;

        static ROS2ParametersMap GetROSDeclaration()
        {
            return {
                {"lidar_min_range", rclcpp::ParameterValue(4.0)},
                {"lidar_max_range", rclcpp::ParameterValue(80.0)},
                {"keyframe_voxel_size", rclcpp::ParameterValue(0.2)},
                {"keyframe_max_points_cnt", rclcpp::ParameterValue(20)},
                {"keyframe_matching_voxel_size", rclcpp::ParameterValue(0.3)},
                {"keyframe_update_voxel_size", rclcpp::ParameterValue(0.1)},
                {"keyframe_cleanup_range", rclcpp::ParameterValue(80.0)},
                {"angular_divergence_threshold", rclcpp::ParameterValue(5.0)},
            };
        }

        static Params FromROS2Params(const ROS2ParametersMap& params)
        {
            Params output;
            output.lidar_min_range = params.at("lidar_min_range").get<float>();
            output.lidar_max_range = params.at("lidar_max_range").get<float>();
            output.keyframe_voxel_size = params.at("keyframe_voxel_size").get<float>();
            output.keyframe_max_points_cnt = params.at("keyframe_max_points_cnt").get<size_t>();
            output.keyframe_matching_voxel_size = params.at("keyframe_matching_voxel_size").get<float>();
            output.keyframe_update_voxel_size = params.at("keyframe_update_voxel_size").get<float>();
            output.keyframe_cleanup_range = params.at("keyframe_cleanup_range").get<float>();
            output.angular_divergence_threshold = params.at("angular_divergence_threshold").get<float>();
            return output;
        }
    };

    explicit LidarOdometry(const Params& config);

    void processCloud(const CloudType& input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getKeyFrameCloud() const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFullKeyFrameCloud() const;

    Pose3D getCurrentPose() const;

    auto getTempCloud() const {
        return temp_cloud_;
    }
private:
    pcl::PointCloud<lidar_point::PointXYZIRT>::Ptr temp_cloud_;

    const Params config_;

    VoxelGrid keyframe_;

    Pose3D previous_transform_;
    Pose3D current_transform_;
};


#endif //BUILD_LIDARODOMETRY_H
