//
// Created by vl on 03.07.23.
//

#include "cloud_matcher.h"
#include "utils/cloud_transform.h"

Pose3D CloudMatcher::align(const VoxelGrid &keyframe, pcl::PointCloud<lidar_point::PointXYZIRT>::ConstPtr cloud,
                           const Pose3D &position_guess) {

    const int max_iter = 1;
    const float max_correspondence_distance = 0.2;

    Pose3D current_pose = position_guess;

    for (size_t i = 0; i < max_iter; i++) {
        //get correspondences
        auto transformed_cloud = utils::transform(*cloud, current_pose);

        auto matching_pairs = keyframe.fingMatchingPairs(*transformed_cloud, max_correspondence_distance);

        //solve
        Eigen::MatrixXd Jacobian(matching_pairs.size()*3, 6);
        Eigen::MatrixXd error_vector(matching_pairs.size(), 3);
        Eigen::VectorXd weights(matching_pairs.size());

        for (size_t row = 0; row<matching_pairs.size(); row++) {
            const auto& transformed_point = transformed_cloud->at(matching_pairs.at(row).source_point_idx);
            Eigen::Vector3f source_point(transformed_point.x, transformed_point.y, transformed_point.z);
            error_vector.block<1, 3>(row, 0) = (source_point - matching_pairs.at(row).target_point).cast<double>();

            weights(row) = 1.0;

            //e = |Rsource_orig + t - target|
            // translation part
            Jacobian.block<3, 3>(row, 0) = Eigen::Matrix3d::Identity();
            //rotation part
            Jacobian.block<3, 3>(row, 3) = Eigen::Matrix3d::Zero(); // TODO: replace with proper derivatives
        }

        auto A = Jacobian.transpose() * Jacobian; // TODO: weights
        auto b = Jacobian.transpose() * error_vector;

        Eigen::Matrix<double, 6, 1> delta = A.ldlt().solve(-b);
        std::cout<<delta<<std::endl;
        //check for convergence
    }

    return Pose3D();
}
