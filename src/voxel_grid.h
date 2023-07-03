//
// Created by vl on 03.07.23.
//

#ifndef BUILD_VOXELGRID_H
#define BUILD_VOXELGRID_H

//#define PCL_NO_PRECOMPILE

#include <vector>
#include "lidar_point_type.h"
#include <tsl/robin_map.h>

class VoxelGrid {
public:
    struct Voxel
    {
        Eigen::Vector3f point;
    };

    VoxelGrid() = default;
    explicit VoxelGrid(float voxel_size)
    {
        setVoxelSize(voxel_size);
    }

    void setVoxelSize(float voxel_size)
    {
        voxels_.clear();
        voxel_size_ = voxel_size;
        max_range_  = (pow(2, 21) * voxel_size_) * 0.5;
    }

    uint64_t getHash(float x, float y, float z) const
    {
        auto idx = static_cast<int64_t>(x / voxel_size_);
        auto idy = static_cast<int64_t>(y / voxel_size_);
        auto idz = static_cast<int64_t>(z / voxel_size_);

        return getHash(idx, idy, idz);
    }

    uint64_t getHash(int64_t idx, int64_t idy, int64_t idz) const
    {
        uint64_t index = ((idx & 0b11111'11111111'11111111) << 42) +
                         ((idy & 0b11111'11111111'11111111) << 21) +
                         (idz & 0b11111'11111111'11111111);

        return index;
    }

    void addCloud(const pcl::PointCloud<lidar_point::PointXYZIRT>& cloud)
    {
        for (const auto& point : cloud.points) {
            if (point.x < -max_range_ || point.x > max_range_ ||
                point.y < -max_range_ || point.y > max_range_ ||
                point.z < -max_range_ || point.z > max_range_) {
                continue;
            }
            auto hash = getHash(point.x, point.y, point.z);
            if (!voxels_.contains(hash)) {
                Voxel voxel{};
                voxel.point = {point.x, point.y, point.z};
                voxels_.insert({hash, voxel});
            }
        }
    }

    pcl::PointCloud<lidar_point::PointXYZIRT>::Ptr getCloud() const
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<lidar_point::PointXYZIRT>>();
        output_cloud->points.reserve(voxels_.size());
        for (const auto& voxel : voxels_) {
            lidar_point::PointXYZIRT output_point;
            output_point.x = voxel.second.point.x();
            output_point.y = voxel.second.point.y();
            output_point.z = voxel.second.point.z();
            output_cloud->points.push_back(output_point);
        }
        return output_cloud;
    }

    struct Correspondence
    {
        size_t source_point_idx;
        Eigen::Vector3f target_point;
        float range_sq;
    };

    float getNearestPoint(const lidar_point::PointXYZIRT& point, float max_correspondence_distance, Eigen::Vector3f& best_match) const
    {
        auto origin_idx = static_cast<int64_t>(point.x / voxel_size_);
        auto origin_idy = static_cast<int64_t>(point.y / voxel_size_);
        auto origin_idz = static_cast<int64_t>(point.z / voxel_size_);

        float min_dist = std::numeric_limits<float>::max();

        for (auto ix = origin_idx - 1; ix <= origin_idx + 1; ix++) {
            for (auto iy = origin_idy - 1;  iy <= origin_idy + 1; iy++) {
                for (auto iz = origin_idz - 1; iz <= origin_idz + 1; iz++) {
                    auto hash = getHash(ix, iy, iz);
                    if (voxels_.contains(hash)) {
                        const auto& voxel = voxels_.at(hash);
                        auto dist_sq = pow(voxel.point.x() - point.x, 2) +
                                       pow(voxel.point.y() - point.y, 2) +
                                       pow(voxel.point.z() - point.z, 2);

                        if (dist_sq < min_dist) {
                            min_dist = dist_sq;
                            best_match = voxel.point;
                        }
                    }
                }
            }
        }

        return min_dist;
    }

    std::vector<Correspondence> fingMatchingPairs(const pcl::PointCloud<lidar_point::PointXYZIRT>& cloud, float max_correspondence_distance) const
    {
        const auto max_distance_sq = max_correspondence_distance * max_correspondence_distance;
        std::vector<Correspondence> output;
        for (size_t source_id = 0; source_id<cloud.points.size(); source_id++) {
            const auto& source_point = cloud.points.at(source_id);
            Eigen::Vector3f target_point;
            if (auto range = getNearestPoint(source_point, max_correspondence_distance, target_point); range<max_distance_sq) {
                output.emplace_back(source_id, target_point, range);
            }
        }
        return output;
    }

    void radiusCleanup(const lidar_point::PointXYZIRT& point, float radius);

    size_t size() const
    {
        return voxels_.size();
    }
private:
    float voxel_size_;
    float max_range_;
    tsl::robin_map<uint64_t, Voxel> voxels_;
};


#endif //BUILD_VOXELGRID_H
