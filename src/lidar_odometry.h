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

        using ROS2ParametersMap = std::map<std::string, rclcpp::ParameterValue>;

        static ROS2ParametersMap GetROSDeclaration()
        {
            return {
                {"lidar_min_range", rclcpp::ParameterValue(2.0)},
                {"lidar_max_range", rclcpp::ParameterValue(80.0)},
                {"keyframe_voxel_size", rclcpp::ParameterValue(0.05)}
            };
        }

        static Params FromROS2Params(const ROS2ParametersMap& params)
        {
            Params output;
            output.lidar_min_range = params.at("lidar_min_range").get<float>();
            output.lidar_max_range = params.at("lidar_max_range").get<float>();
            output.keyframe_voxel_size = params.at("keyframe_voxel_size").get<float>();
            return output;
        }
    };

    explicit LidarOdometry(const Params& config);

    void processCloud(const CloudType& input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getKeyFrameCloud() const;

    Pose3D getCurrentPose() const;
private:
    const Params config_;

    VoxelGrid keyframe_grid_;

    Pose3D previous_transform_;
    Pose3D current_transform_;
};


#endif //BUILD_LIDARODOMETRY_H
