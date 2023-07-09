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

Pose3D CloudMatcher::align(const VoxelGrid& keyframe, const pcl::PointCloud<pcl::PointXYZ> &planar_cloud,
                           const Pose3D& position_guess) {
    Pose3D current_pose = position_guess;

    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.max_num_iterations = 3;
    solver_options.function_tolerance = 1e-4;
    //solver_options.minimizer_progress_to_stdout = true;

    ceres::Problem::Options problem_options;

    for (int i=0; i<10; i++) {
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

            ceres::CostFunction *cost_function =
                    new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 4, 3>(
                            new PointToPlaneError(matching_pair.source_point_local, matching_pair.plane_origin, matching_pair.plane_normal));

            problem.AddResidualBlock(cost_function, loss_function, quat, translation);
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

        if ((summary.final_cost>0)
            && (summary.final_cost<1e-4)
            && (i>3)) {
            std::cout<<"Criteria reached"<<std::endl;
            break;
        }
    }

    return current_pose;
}
