//
// Created by vl on 29.06.23.
//
#define PCL_NO_PRECOMPILE

#include "lidar_odometry.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

LidarOdometry::LidarOdometry(const LidarOdometry::Params& config) : config_(config) {
    previous_transform_.translation.setZero();
    previous_transform_.rotation.setIdentity();
}

void LidarOdometry::processCloud(const pcl::PointCloud<lidar_point::PointXYZIRT> &input_cloud) {

    // pre filter cloud
    auto filtered_cloud = rangeFilter(input_cloud, config_.lidar_min_range, config_.lidar_max_range);

    // deskew cloud
    //auto deskewed_cloud = nonRigidTransform(filtered_cloud, previous_transform_);

    // match with keyframe
    if (keyframe_cloud_ == nullptr) {
        // initialize keyframe cloud
        keyframe_cloud_ = std::make_shared<CloudType>();
        *keyframe_cloud_ += *filtered_cloud;

        pcl::VoxelGrid<CloudType::PointType> voxel_filter_;
        voxel_filter_.setLeafSize(config_.keyframe_voxel_size, config_.keyframe_voxel_size, config_.keyframe_voxel_size);
        voxel_filter_.setInputCloud(keyframe_cloud_);

        auto downsampled_cloud = std::make_shared<CloudType>();
        voxel_filter_.filter(*downsampled_cloud);

        keyframe_cloud_ = downsampled_cloud;
        return;
    }

    pcl::IterativeClosestPoint<CloudType::PointType, CloudType::PointType> icp;

    icp.setInputSource(filtered_cloud);
    icp.setInputTarget(keyframe_cloud_);
    auto aligned = std::make_shared<CloudType>();
    icp.align(*aligned);

    *keyframe_cloud_ += *aligned;

    pcl::VoxelGrid<CloudType::PointType> voxel_filter_;
    voxel_filter_.setLeafSize(config_.keyframe_voxel_size, config_.keyframe_voxel_size, config_.keyframe_voxel_size);
    voxel_filter_.setInputCloud(keyframe_cloud_);

    auto downsampled_cloud = std::make_shared<CloudType>();
    voxel_filter_.filter(*downsampled_cloud);

    keyframe_cloud_ = downsampled_cloud;
}

LidarOdometry::CloudType::Ptr
LidarOdometry::transformNonRigid(const LidarOdometry::CloudType &input, const Pose3D &start_pose,
                                 const Pose3D &end_pose) const {
    return nullptr;
}

LidarOdometry::CloudType::Ptr LidarOdometry::transform(const LidarOdometry::CloudType &input, const Pose3D &pose) const {
    return nullptr;
}

LidarOdometry::CloudType::Ptr
LidarOdometry::rangeFilter(const LidarOdometry::CloudType &input, float min_range, float max_range) const {
    const auto min_range_sq = min_range*min_range;
    const auto max_range_sq = max_range*max_range;

    auto output_cloud_ptr = std::make_shared<CloudType>();
    output_cloud_ptr->points.reserve(input.points.size());

    for (const auto& point : input.points) {
        const auto range_sq = point.x*point.x + point.y*point.y + point.z*point.z;
        if (range_sq>=min_range_sq && range_sq<=max_range_sq) {
            output_cloud_ptr->points.push_back(point);
        }
    }

    return output_cloud_ptr;
}

LidarOdometry::CloudType::ConstPtr LidarOdometry::getKeyFrameCloud() const {
    return keyframe_cloud_;
}
