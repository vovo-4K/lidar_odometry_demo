//
// Created by vl on 03.07.23.
//

#include "cloud_matcher.h"

#include <utility>
#include <ceres/loss_function.h>
#include <ceres/manifold.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include "utils/cloud_transform.h"
#include <pcl/io/pcd_io.h>


struct PointToPlaneError
{
    Eigen::Vector3d local_point;
    Eigen::Vector3d plane_origin;
    Eigen::Vector3d plane_normal;

    PointToPlaneError(Eigen::Vector3d local_point, Eigen::Vector3d plane_origin, Eigen::Vector3d plane_normal)
            : local_point(std::move(local_point)), plane_origin(std::move(plane_origin)), plane_normal(std::move(plane_normal))
    {}

    template <typename T>
    bool operator()(const T* const rotation, const T* const translation, T* residual) const {

        Eigen::Quaternion<T> rot(rotation[0], rotation[1], rotation[2], rotation[3]);
        Eigen::Vector<T, 3> t(translation[0], translation[1], translation[2]);

        residual[0] = (rot*local_point.cast<T>() + t - plane_origin.cast<T>()).dot(plane_normal.cast<T>());

        return true;
    }
};

class PointToPlaneErrorAnalytic : public ceres::SizedCostFunction<1, 4, 3> {
public:
    Eigen::Vector3d local_point;
    Eigen::Vector3d plane_origin;
    Eigen::Vector3d plane_normal;

    PointToPlaneErrorAnalytic(Eigen::Vector3d local_point, Eigen::Vector3d plane_origin, Eigen::Vector3d plane_normal)
    : local_point(std::move(local_point)), plane_origin(std::move(plane_origin)), plane_normal(std::move(plane_normal))
    {}

    bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const final
    {
        Eigen::Quaternion<double> rot(parameters[0][0], parameters[0][1], parameters[0][2], parameters[0][3]);
        Eigen::Vector<double, 3> t(parameters[1][0], parameters[1][1], parameters[1][2]);

        residuals[0] = (rot*local_point + t - plane_origin).dot(plane_normal);

        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                double q0 = parameters[0][0];
                double q1 = parameters[0][1];
                double q2 = parameters[0][2];
                double q3 = parameters[0][3];
                // de/dq
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

                jacobians[0][0] = (dRdqw*local_point).dot(plane_normal);
                jacobians[0][1] = (dRdqx*local_point).dot(plane_normal);
                jacobians[0][2] = (dRdqy*local_point).dot(plane_normal);
                jacobians[0][3] = (dRdqz*local_point).dot(plane_normal);
            }
            if (jacobians[1] != nullptr) {
                // de/qt
                jacobians[1][0] = Eigen::Vector3d(1,0,0).dot(plane_normal);
                jacobians[1][1] = Eigen::Vector3d(0,1,0).dot(plane_normal);
                jacobians[1][2] = Eigen::Vector3d(0,0,1).dot(plane_normal);
            }
        }

        return true;
    }
};

Pose3D CloudMatcher::align(const VoxelGrid& keyframe, const pcl::PointCloud<pcl::PointXYZ> &planar_cloud,
                           const Pose3D& position_guess) {
    Pose3D current_pose = position_guess;

    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.max_num_iterations = 3;
    solver_options.function_tolerance = 1e-4;
    //solver_options.minimizer_progress_to_stdout = true;

    ceres::Problem::Options problem_options;


    //auto keyframe_cloud = keyframe.getCloud();
    //pcl::io::savePCDFileBinary("/home/vl/temp/target_.pcd", *keyframe_cloud);
    //auto current_transformed = CloudTransformer::transform(planar_cloud, current_pose);
    //pcl::io::savePCDFileBinary("/home/vl/temp/iter_init.pcd", *current_transformed);

    for (int i=0; i<20; i++) {
        // prepare ceres solver
        ceres::Problem problem(problem_options);

        ceres::Manifold *quaternion_manifold = new ceres::QuaternionManifold();
        double quat[4] =
                {current_pose.rotation.w(),
                 current_pose.rotation.x(),
                 current_pose.rotation.y(),
                 current_pose.rotation.z()};
        problem.AddParameterBlock(quat, 4, quaternion_manifold);

        double translation[3] = {current_pose.translation(0),
                                 current_pose.translation(1),
                                 current_pose.translation(2)};
        problem.AddParameterBlock(translation, 3);

        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

        // find correspondences
        auto planar_matching_pairs =
                keyframe.findMatchingPairs(planar_cloud,  current_pose, 0.3);

        // build optimization problem
        for (const auto& matching_pair : planar_matching_pairs) {

            //ceres::CostFunction *cost_function =
            //        new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 4, 3>(
            //                new PointToPlaneError(matching_pair.source_point_local, matching_pair.plane_origin, matching_pair.plane_normal));
            //problem.AddResidualBlock(cost_function, loss_function, quat, translation);

            problem.AddResidualBlock(new PointToPlaneErrorAnalytic(matching_pair.source_point_local, matching_pair.plane_origin, matching_pair.plane_normal),
                                     loss_function, quat, translation);
        }

        // optimize
        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);
        //std::cout<<summary.FullReport()<<std::endl;

        current_pose.rotation = {static_cast<float>(quat[0]),
                                 static_cast<float>(quat[1]),
                                 static_cast<float>(quat[2]),
                                 static_cast<float>(quat[3])};
        current_pose.translation = {static_cast<float>(translation[0]),
                                    static_cast<float>(translation[1]),
                                    static_cast<float>(translation[2])};


        //auto current_transformed = CloudTransformer::transform(planar_cloud, current_pose);
        //pcl::io::savePCDFileBinary("/home/vl/temp/iter_"+std::to_string(i)+".pcd", *current_transformed);

        if (summary.iterations.back().step_norm < 1e-4 && (i>3)) {
            //minimization convergence
            break;
        }
    }

    return current_pose;
}
