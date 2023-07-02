//
// Created by vl on 03.07.23.
//

#ifndef BUILD_VOXELGRID_H
#define BUILD_VOXELGRID_H

#define PCL_NO_PRECOMPILE

#include <vector>
#include "lidar_point_type.h"
#include <tsl/robin_map.h>

class VoxelGrid {
public:
    struct Voxel
    {
        lidar_point::PointXYZIRT point;
    };

    void setVoxelSize(float voxel_size)
    {
        voxels.clear();
        voxel_size = voxel_size;
    }

    uint64_t getHash(float x, float y, float z) const
    {
        auto ix = static_cast<int64_t>(x / voxel_size);
        auto iy = static_cast<int64_t>(y / voxel_size);
        auto iz = static_cast<int64_t>(z / voxel_size);

        uint64_t index = ((ix & 0b11111'11111111'11111111) << 42) +
                         ((iy & 0b11111'11111111'11111111) << 21) +
                         (iz & 0b11111'11111111'11111111);

        return index;
    }

    void addCloud(const pcl::PointCloud<lidar_point::PointXYZIRT>& cloud)
    {
        for (const auto& point : cloud.points) {
            auto hash = getHash(point.x, point.y, point.z);
            if (!voxels.contains(hash)) {
                Voxel voxel{};
                voxel.point = point;
                voxels.insert({hash, voxel});
            }
        }
    }

    pcl::PointCloud<lidar_point::PointXYZIRT>::Ptr getCloud() const
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<lidar_point::PointXYZIRT>>();
        output_cloud->points.reserve(voxels.size());
        for (const auto& voxel : voxels) {
            output_cloud->points.push_back(voxel.second.point);
        }
        return output_cloud;
    }

    float voxel_size;
    tsl::robin_map<uint64_t, Voxel> voxels;
};


#endif //BUILD_VOXELGRID_H
