//
// Created by vl on 03.07.23.
//

#ifndef BUILD_VOXELGRID_H
#define BUILD_VOXELGRID_H

//#define PCL_NO_PRECOMPILE

#include <utility>
#include <vector>
#include <execution>
#include <mutex>
#include "lidar_point_type.h"
#include <tsl/robin_map.h>

class VoxelGrid {
public:
    struct Voxel
    {
        Eigen::Vector3f point;
    };

    struct Indices {
        int64_t ix;
        int64_t iy;
        int64_t iz;

        bool operator==(const Indices& another) const
        {
            return ix == another.ix && iy == another.iy && iz == another.iz;
        }
    };

    struct IndicesHash
    {
        std::size_t operator()(Indices const& indices) const noexcept
        {
            // https://niessnerlab.org/papers/2013/4hashing/niessner2013hashing.pdf
            return indices.ix * 73856093 ^ indices.iy * 19349669 ^ indices.iz * 83492791;
        }
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
        max_range_  = 100.0;//(pow(2, 11) * voxel_size_) * 0.5;
    }

    Indices getIndices(float x, float y, float z) const
    {
        auto idx = static_cast<int64_t>(x / voxel_size_);
        auto idy = static_cast<int64_t>(y / voxel_size_);
        auto idz = static_cast<int64_t>(z / voxel_size_);

        return {idx, idy, idz};
    }

    void addCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        for (const auto& point : cloud.points) {
           /* if (point.x < -max_range_ || point.x > max_range_ ||
                point.y < -max_range_ || point.y > max_range_ ||
                point.z < -max_range_ || point.z > max_range_) {
                continue;
            }*/
            auto indices = getIndices(point.x, point.y, point.z);

            if (!voxels_.contains(indices)) {
                Voxel voxel{};
                voxel.point = {point.x, point.y, point.z};
                voxels_.insert({indices, voxel});
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() const
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output_cloud->points.reserve(voxels_.size());
        for (const auto& voxel : voxels_) {
            pcl::PointXYZ output_point;
            output_point.x = voxel.second.point.x();
            output_point.y = voxel.second.point.y();
            output_point.z = voxel.second.point.z();
            output_cloud->points.push_back(output_point);
        }
        return output_cloud;
    }

    struct Correspondence
    {
        Eigen::Vector3d source_point_local;
        Eigen::Vector3d source_point;
        Eigen::Vector3d target_point;
        double range_sq;

        Correspondence() = default;
        Correspondence(Eigen::Vector3d source_point_local, Eigen::Vector3d source_point, Eigen::Vector3d target_point, double range_sq)
                : source_point_local(std::move(source_point_local)),
                  source_point(std::move(source_point)),
                  target_point(std::move(target_point)),
                  range_sq(range_sq)
        {}
    };

    double getNearestPoint(const Eigen::Vector3d& point, double max_correspondence_distance_sq, Eigen::Vector3d& best_match) const
    {
        auto origin_idx = static_cast<int64_t>(point.x() / voxel_size_);
        auto origin_idy = static_cast<int64_t>(point.y() / voxel_size_);
        auto origin_idz = static_cast<int64_t>(point.z() / voxel_size_);

        double min_dist = max_correspondence_distance_sq;

        for (auto ix = origin_idx - 1; ix <= origin_idx + 1; ix++) {
            for (auto iy = origin_idy - 1;  iy <= origin_idy + 1; iy++) {
                for (auto iz = origin_idz - 1; iz <= origin_idz + 1; iz++) {
                    auto it = voxels_.find(Indices{ix, iy, iz});
                    if (it!=voxels_.end()) {
                        const auto& voxel = it->second;
                        auto dist_sq = (voxel.point.cast<double>() - point).squaredNorm();
                        if (dist_sq < min_dist) {
                            min_dist = dist_sq;
                            best_match = voxel.point.cast<double>();
                        }
                    }
                }
            }
        }

        return min_dist;
    }

    std::vector<Correspondence> findMatchingPairs(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Pose3D& transform, float max_correspondence_distance) const
    {
        const auto max_distance_sq = max_correspondence_distance * max_correspondence_distance;
        std::vector<Correspondence> output;
        output.reserve(cloud.size());
        std::mutex output_mutex;

        auto R = transform.rotation.cast<double>().matrix();
        auto t = transform.translation.cast<double>();

        std::for_each(std::execution::par, cloud.points.cbegin(), cloud.points.cend(),
                      [&output, &output_mutex, max_distance_sq, &R, &t, this](const auto& point) {

                          Eigen::Vector3d source_point_local(point.x, point.y, point.z);
                          Eigen::Vector3d source_point_transformed = R * source_point_local + t;

                          Eigen::Vector3d nearest_point;
                          auto range_sq = getNearestPoint(source_point_transformed, max_distance_sq, nearest_point);

                          if (range_sq < max_distance_sq) {
                              std::lock_guard lock(output_mutex);
                              output.emplace_back(source_point_local, source_point_transformed, nearest_point, range_sq);
                          }
                      });

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
    tsl::robin_map<Indices, Voxel, IndicesHash> voxels_;
};


#endif //BUILD_VOXELGRID_H
