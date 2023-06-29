//
// Created by vl on 29.06.23.
//

#ifndef BUILD_LIDARODOMETRY_H
#define BUILD_LIDARODOMETRY_H

#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>

class LidarOdometry {
public:

    class Params {
    public:
        double lidar_max_range = 80.0;
        double keyframe_voxel_size = 0.04;


        using ROS2ParametersMap = std::map<std::string, rclcpp::ParameterValue>;

        static ROS2ParametersMap GetROSDeclaration()
        {
            return {{"lidar_max_range", rclcpp::ParameterValue(80.0)},
                    {"keyframe_voxel_size", rclcpp::ParameterValue(0.05)}
            };
        }

        static Params FromROSParams(const ROS2ParametersMap& params)
        {
            Params output;
            output.lidar_max_range = params.at("lidar_max_range").get<double>();
            output.keyframe_voxel_size = params.at("keyframe_voxel_size").get<double>();
            return output;
        }
    };

    LidarOdometry(const Params& config)
    : config_(config) {}

protected:
    const Params config_;
};


#endif //BUILD_LIDARODOMETRY_H
