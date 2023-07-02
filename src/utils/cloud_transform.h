//
// Created by vl on 02.07.23.
//

#ifndef BUILD_CLOUD_TRANSFORM_H
#define BUILD_CLOUD_TRANSFORM_H

#include <execution>

#include "../pose_3d.h"

namespace utils {

    template<typename PointType>
    typename pcl::PointCloud<PointType>::Ptr transformNonRigid(const pcl::PointCloud<PointType>& input, const Pose3D& start_pose, const Pose3D& end_pose)
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<PointType>>();
        output_cloud->points.resize(input.points.size());

        std::transform(std::execution::par, input.points.cbegin(), input.points.cend(), output_cloud->points.begin(),
                       [start_pose = &start_pose, end_pose = &end_pose](const auto& input_point) {
                           auto output_point = input_point;

                           Eigen::Vector3f input_coord(input_point.x, input_point.y, input_point.z);
                           Eigen::Vector3f transformed_coord = (start_pose->rotation.slerp(input_point.time, end_pose->rotation) * input_coord) +
                                                               start_pose->translation * input_point.time + end_pose->translation * (1.0 - input_point.time);

                           output_point.x = transformed_coord.x();
                           output_point.y = transformed_coord.y();
                           output_point.z = transformed_coord.z();
                           return output_point;
                       });

        return output_cloud;
    }

    template<typename PointType>
    typename pcl::PointCloud<PointType>::Ptr transform(const pcl::PointCloud<PointType>& input, const Pose3D& pose)
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<PointType>>();
        output_cloud->points.resize(input.points.size());

        std::transform(std::execution::par, input.points.cbegin(), input.points.cend(), output_cloud->points.begin(),
                       [pose](const auto& input_point) {
                           auto output_point = input_point;

                           Eigen::Vector3f input_coord(input_point.x, input_point.y, input_point.z);
                           Eigen::Vector3f transformed_coord = pose.rotation * input_coord + pose.translation;

                           output_point.x = transformed_coord.x();
                           output_point.y = transformed_coord.y();
                           output_point.z = transformed_coord.z();
                           return output_point;
                       });

        return output_cloud;
    }

}

#endif //BUILD_CLOUD_TRANSFORM_H