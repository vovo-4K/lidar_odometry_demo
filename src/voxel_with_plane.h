//
// Created by vl on 08.07.23.
//

#ifndef BUILD_VOXEL_WITH_PLANE_H
#define BUILD_VOXEL_WITH_PLANE_H

#include <eigen3/Eigen/Eigen>


template<unsigned int MaxPoints>
struct VoxelWithPlanes
{
    Eigen::MatrixXf points_mat;

    Eigen::Vector3f plane_normal;
    Eigen::Vector3f plane_origin;

    size_t points_count = 0;
    //const size_t MinPointsToCalcNormal = 4;

    void addPoint(float x, float y, float z)
    {
        if (points_count < MaxPoints) {
            if (points_count == 0) {
                plane_origin = Eigen::Vector3f(x, y, z);
                points_mat = Eigen::MatrixXf(MaxPoints, 3);
            }

            points_mat.row(points_count) = Eigen::Vector3f(x, y, z);
            points_count++;

            if (points_count>=3) {
                // update plane
                //const auto actual_points_mat = points_mat.block(0, 0, points_count, 3);

                plane_origin = points_mat.block(0, 0, points_count, 3).colwise().sum() / points_count;

                Eigen::JacobiSVD<Eigen::MatrixXf> svd;
                svd.compute(points_mat.block(0, 0, points_count, 3), Eigen::ComputeThinV);

                plane_normal = svd.matrixV().block<1, 3>(2,0);
            }
        }
    }

    struct Correspondence
    {
        Eigen::Vector3d source_point;
        Eigen::Vector3d plane_origin;
        Eigen::Vector3d plane_normal;

        Eigen::Vector3d source_point_local;
        double range_sq;

        Correspondence() = default;
        Correspondence(Eigen::Vector3d source_point, Eigen::Vector3d plane_origin, Eigen::Vector3d plane_normal,
                       double range_sq)
                : source_point(std::move(source_point)),
                  plane_origin(std::move(plane_origin)),
                  plane_normal(std::move(plane_normal)),
                  range_sq(range_sq)
        {}
    };

    Correspondence getCorrespondence(const Eigen::Vector3d &query) const
    {
        double min_range_sq = std::numeric_limits<double>::max();

        if (points_count >= 3) {
            min_range_sq = (query.cast<float>() - plane_origin).dot(plane_normal);
            min_range_sq *= min_range_sq;
        }

        return Correspondence(query, plane_origin.cast<double>(), plane_normal.cast<double>(), min_range_sq);
    }

    Eigen::Vector3f getOrigin() const
    {
        return plane_origin;
    }
};

#endif //BUILD_VOXEL_WITH_PLANE_H
