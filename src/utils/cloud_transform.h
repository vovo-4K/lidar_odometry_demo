//
// Created by vl on 02.07.23.
//

#ifndef BUILD_CLOUD_TRANSFORM_H
#define BUILD_CLOUD_TRANSFORM_H

#include <execution>

#include "../pose_3d.h"
#include "../lidar_point_type.h"

class CloudTransformer {
public:
    static typename pcl::PointCloud<lidar_point::PointXYZIRT>::Ptr
    transformNonRigid(const pcl::PointCloud<lidar_point::PointXYZIRT> &input, const Pose3D &start_pose,
                      const Pose3D &end_pose) {
        auto output_cloud = std::make_shared<pcl::PointCloud<lidar_point::PointXYZIRT>>();
        output_cloud->points.resize(input.points.size());

        std::transform(std::execution::par, input.points.cbegin(), input.points.cend(),
                       output_cloud->points.begin(),
                       [start_pose = &start_pose, end_pose = &end_pose](const auto &input_point) {
                           Eigen::Vector3f input_coord(input_point.x, input_point.y, input_point.z);

                           Eigen::Vector3f transformed_coord =
                                   (start_pose->rotation.slerp(input_point.time, end_pose->rotation) *
                                    input_coord) +
                                   start_pose->translation * input_point.time +
                                   end_pose->translation * (1.0 - input_point.time);

                           auto output_point = input_point;
                           output_point.x = transformed_coord.x();
                           output_point.y = transformed_coord.y();
                           output_point.z = transformed_coord.z();
                           return output_point;
                       });

        return output_cloud;
    }

    template<typename PointType>
    static typename pcl::PointCloud<PointType>::Ptr
    transform(const pcl::PointCloud<PointType> &input, const Pose3D &pose) {
        auto output_cloud = std::make_shared<pcl::PointCloud<PointType>>();
        output_cloud->points.resize(input.points.size());

        const Eigen::Matrix3f R = pose.rotationMatrix();
        const Eigen::Vector3f t = pose.translation;

        std::transform(std::execution::par, input.points.cbegin(), input.points.cend(),
                       output_cloud->points.begin(),
                       [&R, &t](const auto &input_point) {
                           auto output_point = input_point;

                           Eigen::Vector3f input_coord(input_point.x, input_point.y, input_point.z);
                           Eigen::Vector3f transformed_coord = R * input_coord + t;

                           output_point.x = transformed_coord.x();
                           output_point.y = transformed_coord.y();
                           output_point.z = transformed_coord.z();
                           return output_point;
                       });

        return output_cloud;
    }

    static pcl::PointCloud<pcl::PointNormal>::Ptr
    transformWithNormals(const pcl::PointCloud<pcl::PointNormal> &input, const Pose3D &pose) {
        auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        output_cloud->points.resize(input.points.size());

        const Eigen::Matrix3f R = pose.rotationMatrix();
        const Eigen::Vector3f t = pose.translation;

        std::transform(std::execution::par, input.points.cbegin(), input.points.cend(),
                       output_cloud->points.begin(),
                       [&R, &t](const auto &input_point) {
                           auto output_point = input_point;

                           Eigen::Vector3f input_coord(input_point.x, input_point.y, input_point.z);
                           Eigen::Vector3f transformed_coord = R * input_coord + t;
                           Eigen::Vector3f transformed_normal = R * Eigen::Vector3f(input_point.normal_x, input_point.normal_y, input_point.normal_z);

                           output_point.x = transformed_coord.x();
                           output_point.y = transformed_coord.y();
                           output_point.z = transformed_coord.z();

                           output_point.normal_x = transformed_normal.x();
                           output_point.normal_y = transformed_normal.y();
                           output_point.normal_z = transformed_normal.z();

                           return output_point;
                       });

        return output_cloud;
    }
};

#endif //BUILD_CLOUD_TRANSFORM_H
