//
// Created by vl on 07.07.23.
//

#ifndef BUILD_KEYFRAME_H
#define BUILD_KEYFRAME_H

#include "voxel_grid.h"

class Keyframe
{
public:
    using PlanarVoxelGrid = VoxelGrid<VoxelWithPoints<20>>;
    using UnclassifiedVoxelGrid = VoxelGrid<VoxelWithPoints<20>>;

    Keyframe() = default;

    Keyframe(float planar_voxel_size, float unclassified_voxel_size)
    {
        setVoxelsSize(planar_voxel_size, unclassified_voxel_size);
    }

    void setVoxelsSize(double planar_voxel_size, double unclassified_voxel_size)
    {
        planar_grid_.setVoxelSize(planar_voxel_size);
        unclassified_grid_.setVoxelSize(unclassified_voxel_size);
    }

    void addClouds(const pcl::PointCloud<pcl::PointXYZ>& planar, const pcl::PointCloud<pcl::PointXYZ>& unclassified)
    {
        planar_grid_.addCloud(planar);
        unclassified_grid_.addCloud(unclassified);
    }

    size_t size() const
    {
        return planar_grid_.size() + unclassified_grid_.size();
    }

    void radiusCleanup(const Eigen::Vector3f &origin, float range)
    {
        planar_grid_.radiusCleanup(origin, range);
        unclassified_grid_.radiusCleanup(origin, range);
    }

    [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() const
    {
        return unclassified_grid_.getCloud();
    }

    [[nodiscard]] std::pair<std::vector<PlanarVoxelGrid::Correspondence>,
            std::vector<UnclassifiedVoxelGrid::Correspondence>>
    findMatchingPairs(const pcl::PointCloud<pcl::PointXYZ>& planar, const pcl::PointCloud<pcl::PointXYZ>& unclassified,
                     const Pose3D& transform, float max_correspondence_distance) const
    {
        return {planar_grid_.findMatchingPairs(planar, transform, max_correspondence_distance),
                unclassified_grid_.findMatchingPairs(unclassified, transform, max_correspondence_distance)};
    }
private:

    PlanarVoxelGrid planar_grid_;
    UnclassifiedVoxelGrid unclassified_grid_;
};

#endif //BUILD_KEYFRAME_H
