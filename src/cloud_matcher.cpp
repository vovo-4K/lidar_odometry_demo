//
// Created by vl on 03.07.23.
//

#include "cloud_matcher.h"
#include "utils/cloud_transform.h"

Pose3D CloudMatcher::align(const Keyframe& keyframe, const pcl::PointCloud<pcl::PointXYZ> &planar_cloud,
                           const pcl::PointCloud<pcl::PointXYZ> &unclassified_cloud,
                           const Pose3D& position_guess) {

    const int max_iter = 30;
    const float max_correspondence_distance = 0.3;

    Pose3D current_pose = position_guess;

    for (size_t i = 0; i < max_iter; i++) {
        //get correspondences
        auto [planar_matching_pairs, unclassified_matching_pairs] =
                keyframe.findMatchingPairs(planar_cloud, unclassified_cloud,  current_pose, max_correspondence_distance);

        // Transformation Jacobian precompute
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
        double a_square = (1.0 - q3) / (q3 + 1.0);
        double a_square_plus_one = a_square + 1;
        double a_mult = -4.0/(a_square_plus_one * a_square_plus_one);

        double x = q0 / (1.0 + q3);
        double y = q1 / (1.0 + q3);
        double z = q2 / (1.0 + q3);

        Eigen::Vector4d dqdx(x*x - 0.5*a_square_plus_one, x*y, x*z, x);
        dqdx *= a_mult;

        Eigen::Vector4d dqdy(y*x, y*y - 0.5*a_square_plus_one, y*z, y);
        dqdy *= a_mult;

        Eigen::Vector4d dqdz(z*x, z*y, z*z - 0.5*a_square_plus_one, z);
        dqdz *= a_mult;

        // chain rule
        Eigen::Matrix3d dRdx = dRdqw*dqdx(0) + dRdqx*dqdx(1) + dRdqy*dqdx(2) + dRdqz*dqdx(3);
        Eigen::Matrix3d dRdy = dRdqw*dqdy(0) + dRdqx*dqdy(1) + dRdqy*dqdy(2) + dRdqz*dqdy(3);
        Eigen::Matrix3d dRdz = dRdqw*dqdz(0) + dRdqx*dqdz(1) + dRdqy*dqdz(2) + dRdqz*dqdz(3);


        //solve
        auto total_errors_count = planar_matching_pairs.size() + unclassified_matching_pairs.size();

        Eigen::MatrixXd Jacobian(total_errors_count, 6);
        Eigen::MatrixXd error_vector(total_errors_count, 1);
        Eigen::VectorXd weights(total_errors_count);

        // Point2Plane
        for (size_t row = 0; row < planar_matching_pairs.size(); row++) {
            auto row_id = row;

            const auto& corr = planar_matching_pairs.at(row);

            error_vector(row_id) = (corr.source_point - corr.plane_origin).dot(corr.plane_normal);

            Jacobian.block<1, 3>(row_id, 0) = corr.plane_normal;

            Jacobian(row_id, 3) = (dRdx*corr.source_point_local).dot(corr.plane_normal);
            Jacobian(row_id, 4) = (dRdy*corr.source_point_local).dot(corr.plane_normal);
            Jacobian(row_id, 5) = (dRdz*corr.source_point_local).dot(corr.plane_normal);

            const double delta = 0.05;
            if (corr.range_sq < delta * delta) {
                weights(row_id) = corr.range_sq;
            } else {
                weights(row_id) = delta * (sqrt(corr.range_sq) - 0.5 * delta);
            }
        }

        // Point2Point
        for (size_t row = 0; row < unclassified_matching_pairs.size(); row++) {
            auto row_id = row + planar_matching_pairs.size();

            const auto& corr = unclassified_matching_pairs.at(row);
            Eigen::Vector3d e = corr.source_point - corr.target_point;

            error_vector(row_id) = e.squaredNorm();

            Jacobian.block<1, 3>(row_id, 0) = 2.0 * e;
            Jacobian(row_id, 3) = 2.0 * (dRdx*corr.source_point_local).dot(e);
            Jacobian(row_id, 4) = 2.0 * (dRdy*corr.source_point_local).dot(e);
            Jacobian(row_id, 5) = 2.0 * (dRdz*corr.source_point_local).dot(e);

            const double delta = 0.1;
            if (corr.range_sq < delta * delta) {
                weights(row_id) = corr.range_sq;
            } else {
                weights(row_id) = delta * (sqrt(corr.range_sq) - 0.5 * delta);
            }
        }

        auto diag_weights = weights.asDiagonal();
        auto A = Jacobian.transpose() * diag_weights * Jacobian;
        auto b = Jacobian.transpose() * diag_weights * error_vector;

        Eigen::Matrix<double, 6, 1> delta = A.ldlt().solve(-b);

        std::cout<<"error: "<<error_vector.sum()<<" delta: "<<delta.norm()<<std::endl;

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

        //current_pose.rotation.normalize();
        //check for convergence
    }
    return current_pose;
}
