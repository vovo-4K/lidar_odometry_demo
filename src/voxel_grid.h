//
// Created by vl on 03.07.23.
//

#ifndef BUILD_VOXELGRID_H
#define BUILD_VOXELGRID_H

#include <utility>
#include <vector>
#include <execution>
#include <mutex>
#include <tsl/robin_map.h>
#include <pcl/io/pcd_io.h>

#include "voxel_with_planes.h"

class VoxelGrid {
public:

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
            return ((1<<33)-1) & (indices.ix * 73856093 ^ indices.iy * 19349669 ^ indices.iz * 83492791);
        }
    };

    struct Correspondence
    {
        Eigen::Vector3d source_point_local;
        Eigen::Vector3d plane_origin;
        Eigen::Vector3d plane_normal;
        bool valid = false;
    };

    VoxelGrid() = default;

    VoxelGrid(float voxel_size, size_t max_points)
    {
        setMaxPoints(max_points);
        setVoxelSize(voxel_size);
    }

    void setMaxPoints(size_t max_points)
    {
        max_points_ = max_points;
    }

    void setVoxelSize(float voxel_size)
    {
        voxels_.clear();

        voxel_size_ = voxel_size;
    }

    Indices getIndices(float x, float y, float z) const
    {
        auto idx = static_cast<int64_t>(x / voxel_size_);
        auto idy = static_cast<int64_t>(y / voxel_size_);
        auto idz = static_cast<int64_t>(z / voxel_size_);

        return {idx, idy, idz};
    }

    void addCloud(const pcl::PointCloud<pcl::PointNormal>& cloud)
    {
        for (const auto& point : cloud.points) {
            auto indices = getIndices(point.x, point.y, point.z);

            auto it = voxels_.find(indices);
            if (it==voxels_.end()) {
                VoxelWithPlanes voxel;
                voxel.points_with_normals.reserve(max_points_);
                voxel.addPoint(point.x, point.y, point.z, point.normal_x, point.normal_y, point.normal_z);
                voxels_.insert({indices, voxel});
            } else {
                if (it.value().points_with_normals.size()<max_points_)
                    it.value().addPoint(point.x, point.y, point.z, point.normal_x, point.normal_y, point.normal_z);
            }
        }
    }

    void addCloudWithoutNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        for (const auto& point : cloud.points) {
            auto indices = getIndices(point.x, point.y, point.z);

            auto it = voxels_.find(indices);
            if (it==voxels_.end()) {
                VoxelWithPlanes voxel;
                voxel.addPoint(point.x, point.y, point.z, 0, 0, 0);
                voxels_.insert({indices, voxel});
            } else {
                if (it.value().points_with_normals.size()<max_points_)
                    it.value().addPoint(point.x, point.y, point.z, 0, 0, 0);
            }
        }
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr getCloud() const
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        output_cloud->points.reserve(voxels_.size() * max_points_);
        for (const auto& voxel : voxels_) {
            for (const auto &p : voxel.second.points_with_normals) {
                pcl::PointNormal output_point;
                output_point.x = p.point.x();
                output_point.y = p.point.y();
                output_point.z = p.point.z();
                output_point.normal_x = p.normal.x();
                output_point.normal_y = p.normal.y();
                output_point.normal_z = p.normal.z();
                output_cloud->points.push_back(output_point);
            }
        }
        return output_cloud;
    }

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudWithoutNormals() const
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output_cloud->points.reserve(voxels_.size() * max_points_);
        for (const auto& voxel : voxels_) {
            for (const auto &p : voxel.second.points_with_normals) {
                pcl::PointXYZ output_point;
                output_point.x = p.point.x();
                output_point.y = p.point.y();
                output_point.z = p.point.z();
                output_cloud->points.push_back(output_point);
            }
        }
        return output_cloud;
    }

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr getSparseCloudWithoutNormals() const
    {
        auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output_cloud->points.reserve(voxels_.size() * max_points_);
        for (const auto& voxel : voxels_) {
            const auto &p = voxel.second.points_with_normals.front();
            pcl::PointXYZ output_point;
            output_point.x = p.point.x();
            output_point.y = p.point.y();
            output_point.z = p.point.z();
            output_cloud->points.push_back(output_point);
        }
        return output_cloud;
    }

    Correspondence getCorrespondence(const Eigen::Vector3f& query, double max_correspondence_distance_sq) const
    {
        auto origin_idx = static_cast<int64_t>(query.x() / voxel_size_);
        auto origin_idy = static_cast<int64_t>(query.y() / voxel_size_);
        auto origin_idz = static_cast<int64_t>(query.z() / voxel_size_);

        Correspondence correspondence;

        VoxelWithPlanes::PointWithNormal closest_point;
        double min_dist = std::numeric_limits<double>::max();

        for (auto ix = origin_idx - 1; ix <= origin_idx + 1; ix++) {
            for (auto iy = origin_idy - 1;  iy <= origin_idy + 1; iy++) {
                for (auto iz = origin_idz - 1; iz <= origin_idz + 1; iz++) {
                    auto it = voxels_.find(Indices{ix, iy, iz});
                    if (it != voxels_.end()) {
                        const auto& voxel = it->second;

                        // get all points within search radius
                        for (const auto& point_with_normal : voxel.points_with_normals) {
                            double sq_dist = (query - point_with_normal.point).squaredNorm();

                            if (sq_dist < max_correspondence_distance_sq
                                && sq_dist < min_dist) {
                                closest_point = point_with_normal;
                                min_dist = sq_dist;
                            }
                        }
                    }
                }
            }
        }

        if (min_dist<max_correspondence_distance_sq) {
            correspondence.valid = true;
            correspondence.plane_normal = closest_point.normal.cast<double>();
            correspondence.plane_origin = closest_point.point.cast<double>();
        }

        return correspondence;
    }

    std::vector<Correspondence> findMatchingPairs(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Pose3D& transform, float max_correspondence_distance) const
    {
        std::vector<Correspondence> output;
        output.reserve(cloud.size());
        std::mutex output_mutex;

        Eigen::Matrix3d R = transform.rotationMatrix().cast<double>();//transform.rotation.cast<double>().toRotationMatrix();
        Eigen::Vector3d t = transform.translation.cast<double>();

        float max_correspondence_distance_sq = max_correspondence_distance * max_correspondence_distance;

        std::for_each(std::execution::par, cloud.points.cbegin(), cloud.points.cend(),
                      [&output, &output_mutex, max_correspondence_distance_sq, &R, &t, this](const auto& point) {

                          Eigen::Vector3d source_point_local(point.x, point.y, point.z);
                          Eigen::Vector3d source_point_transformed = R * source_point_local + t;

                          auto correspondence = getCorrespondence(source_point_transformed.cast<float>(), max_correspondence_distance_sq);
                          if (correspondence.valid) {
                              //inject source points
                              correspondence.source_point_local = source_point_local;

                              std::lock_guard lock(output_mutex);
                              output.emplace_back(correspondence);
                          }
                      });

        return output;
    }

    void radiusCleanup(const Eigen::Vector3f& point, float radius)
    {
        auto radius_sq = radius * radius;
        for (auto it = voxels_.begin(); it!=voxels_.end(); ) {
            if ((it->second.getOrigin() - point).squaredNorm()>radius_sq) {
                it = voxels_.erase(it);
            } else {
                ++it;
            }
        }
    }

    size_t size() const
    {
        return voxels_.size();
    }
private:
    size_t max_points_ = 10;
    float voxel_size_ = 0.5;

    tsl::robin_map<Indices, VoxelWithPlanes, IndicesHash> voxels_;
};


#endif //BUILD_VOXELGRID_H
