//
// Created by vl on 08.07.23.
//

#ifndef BUILD_VOXEL_WITH_PLANES_H
#define BUILD_VOXEL_WITH_PLANES_H

#include <eigen3/Eigen/Eigen>

struct VoxelWithPlanes
{
    struct PointWithNormal
    {
        Eigen::Vector3f point;
        Eigen::Vector3f normal;

        PointWithNormal() = default;

        PointWithNormal(Eigen::Vector3f point, Eigen::Vector3f normal) :
                point(std::move(point)), normal(std::move(normal)) {}
    };
    std::vector<PointWithNormal> points_with_normals;

    VoxelWithPlanes() = default;

    void addPoint(float x, float y, float z, float normal_x, float normal_y, float normal_z)
    {
        points_with_normals.emplace_back(Eigen::Vector3f{x, y, z},
                                         Eigen::Vector3f{normal_x, normal_y, normal_z});
    }

    Eigen::Vector3f getOrigin() const
    {
        return points_with_normals.front().point;
    }
};

#endif //BUILD_VOXEL_WITH_PLANES_H
