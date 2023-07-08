//
// Created by vl on 08.07.23.
//

#ifndef BUILD_VOXEL_WITH_POINTS_H
#define BUILD_VOXEL_WITH_POINTS_H

#include <eigen3/Eigen/Eigen>


template<unsigned int MaxPoints>
struct VoxelWithPoints
{
    std::vector<Eigen::Vector3f> points;

    void addPoint(float x, float y, float z)
    {
        if (points.size() < MaxPoints) {
            points.emplace_back(x, y, z);
        }
    }

    struct Correspondence
    {
        Eigen::Vector3d source_point;
        Eigen::Vector3d target_point;
        Eigen::Vector3d source_point_local;
        double range_sq;

        Correspondence() = default;
        Correspondence(Eigen::Vector3d source_point, Eigen::Vector3d target_point, double range_sq)
                : source_point(std::move(source_point)),
                  target_point(std::move(target_point)),
                  range_sq(range_sq)
        {}
    };

    Correspondence getCorrespondence(const Eigen::Vector3d &query) const
    {
        double min_range_sq = std::numeric_limits<double>::max();
        Eigen::Vector3d target(query);

        for (const Eigen::Vector3f& point : points) {
            double range_sq = (point.cast<double>() - query).squaredNorm();
            if (range_sq < min_range_sq) {
                target = point.cast<double>();
                min_range_sq = range_sq;
            }
        }

        return Correspondence(query, target, min_range_sq);
    }

    Eigen::Vector3f getOrigin() const{
        return points.front();
    }
};

#endif //BUILD_VOXEL_WITH_POINTS_H
