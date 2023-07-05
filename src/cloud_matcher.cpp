//
// Created by vl on 03.07.23.
//

#include "cloud_matcher.h"
#include "utils/cloud_transform.h"

Pose3D CloudMatcher::align(const VoxelGrid &keyframe, const pcl::PointCloud<pcl::PointXYZ> &cloud,
                           const Pose3D &position_guess) {

    const int max_iter = 20;
    const float max_correspondence_distance = 0.2;

    Pose3D current_pose = position_guess;

    for (size_t i = 0; i < max_iter; i++) {
        //get correspondences
        auto transformed_cloud = CloudTransformer::transform(cloud, current_pose);

        auto matching_pairs = keyframe.findMatchingPairs(*transformed_cloud, max_correspondence_distance);

        //solve
        Eigen::MatrixXd Jacobian(matching_pairs.size()*3, 6);
        Eigen::MatrixXd error_vector(matching_pairs.size()*3, 1);
        Eigen::VectorXd weights(matching_pairs.size()*3);

        double q0 = current_pose.rotation.w();
        double q1 = current_pose.rotation.x();
        double q2 = current_pose.rotation.y();
        double q3 = current_pose.rotation.z();

        // rotation matrix derivative wrt quaternion components
        Eigen::Matrix3d dRdqw;
        dRdqw << q0, -q3,  q2,
                q3,  q0, -q1,
                -q2,  q1,  q0;
        dRdqw = 2.0 * dRdqw;

        Eigen::Matrix3d dRdqx;
        dRdqx << q1,  q2,  q3,
                q2, -q1, -q0,
                q3,  q0, -q1;
        dRdqx = 2.0 * dRdqx;

        Eigen::Matrix3d dRdqy;
        dRdqy << -q2, q1, q0,
                q1, q2, q3,
                -q0, q3, -q2;
        dRdqy = 2.0 * dRdqy;

        Eigen::Matrix3d dRdqz;
        dRdqz << -q3, -q0, q1,
                q0, -q3, q2,
                q1, q2, q3;
        dRdqz = 2.0 * dRdqz;

        //quaternion derivative wrt quaternion parametrization
        double a = (1.0 - q3) / (q3 + 1.0); a = a*a + 1;
        double a_mult = -4.0/a*a;

        double x = q0 / (1.0 + q3);
        double y = q1 / (1.0 + q3);
        double z = q2 / (1.0 + q3);

        Eigen::Vector4d dqdx(x*x - 0.5*a, x*y, x*z, x);
        dqdx *= a_mult;

        Eigen::Vector4d dqdy(y*x, y*y - 0.5*a, y*z, y);
        dqdy *= a_mult;

        Eigen::Vector4d dqdz(z*x, z*y, z*z - 0.5*a, z);
        dqdz *= a_mult;

        // chain rule
        Eigen::Matrix3d dRdx = dRdqw*dqdx(0) + dRdqx*dqdx(1) + dRdqy*dqdx(2) + dRdqz*dqdx(3);
        Eigen::Matrix3d dRdy = dRdqw*dqdy(0) + dRdqx*dqdy(1) + dRdqy*dqdy(2) + dRdqz*dqdy(3);
        Eigen::Matrix3d dRdz = dRdqw*dqdz(0) + dRdqx*dqdz(1) + dRdqy*dqdz(2) + dRdqz*dqdz(3);

        for (size_t row = 0; row<matching_pairs.size(); row++) {
            const auto& original_point = cloud.points.at(matching_pairs.at(row).source_point_idx); // TODO: move to correspondence
            Eigen::Vector3d orig_point(original_point.x, original_point.y, original_point.z);

            const auto& transformed_point = transformed_cloud->at(matching_pairs.at(row).source_point_idx);
            Eigen::Vector3d source_point(transformed_point.x, transformed_point.y, transformed_point.z);
            error_vector.block<3, 1>(row*3, 0) = (source_point - matching_pairs.at(row).target_point.cast<double>());

            const double delta = 0.1;
            if (matching_pairs.at(row).range_sq < delta * delta) {
                weights(row*3) = matching_pairs.at(row).range_sq;
            } else {
                weights(row*3) = delta * (sqrt(matching_pairs.at(row).range_sq) - 0.5 * delta);
            }
            weights(row*3 + 1) = weights(row*3);
            weights(row*3 + 2) = weights(row*3);

            //e = |R * source_orig + t - target|
            // translational part
            Jacobian.block<3, 3>(row*3, 0) = Eigen::Matrix3d::Identity();
            //rotational part
          //  Jacobian.block<3, 1>(row*3, 3) =
          //  Jacobian.block<3, 1>(row*3, 4) =
          //  Jacobian.block<3, 1>(row*3, 5) =

            Eigen::Matrix3d J;
            J <<
            dRdx.block<1, 3>(0, 0) * source_point, dRdy.block<1, 3>(0, 0) * source_point, dRdz.block<1, 3>(0, 0) * source_point,
            dRdx.block<1, 3>(1, 0) * source_point, dRdy.block<1, 3>(1, 0) * source_point, dRdz.block<1, 3>(1, 0) * source_point,
            dRdx.block<1, 3>(2, 0) * source_point, dRdy.block<1, 3>(2, 0) * source_point, dRdz.block<1, 3>(2, 0) * source_point;

            Jacobian.block<3, 3>(row*3, 3) = J;

        }

        auto diag_weights = weights.asDiagonal();
        auto A = Jacobian.transpose() * diag_weights * Jacobian;
        auto b = Jacobian.transpose() * diag_weights * error_vector;

        Eigen::Matrix<double, 6, 1> delta = A.ldlt().solve(-b);

        std::cout<<delta.norm()<<std::endl;

        current_pose.translation += delta.block<3, 1>(0,0).cast<float>().transpose();
        // recover quaternion
        x += delta(3,0);
        y += delta(4,0);
        z += delta(5,0);
        double a_sq = x*x + y*y + z*z;

        current_pose.rotation = Eigen::Quaternionf(2.0 * x / (a_sq + 1.0),
                2.0 * y / (a_sq + 1.0),
                2.0 * z / (a_sq + 1.0),
                (1.0 - a_sq) / (a_sq + 1.0));

        //check for convergence
    }
    return current_pose;
}
