//
// Created by vl on 29.06.23.
//
#define PCL_NO_PRECOMPILE

#include "lidar_odometry.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include "utils/cloud_transform.h"
#include "utils/point_time_normalize.h"

LidarOdometry::LidarOdometry(const LidarOdometry::Params& config) : config_(config) {
    current_transform_.translation.setZero();
    current_transform_.rotation.setIdentity();
    previous_transform_ = current_transform_;
}

void LidarOdometry::processCloud(const pcl::PointCloud<lidar_point::PointXYZIRT> &input_cloud) {

    // pre filter cloud
    auto filtered_cloud = rangeFilter(input_cloud, config_.lidar_min_range, config_.lidar_max_range);

    auto time_normalized = utils::pointTimeNormalize(*filtered_cloud);

    // init keyframe
    if (keyframe_cloud_ == nullptr) {
        // initialize keyframe cloud
        keyframe_cloud_ = std::make_shared<CloudType>();
        *keyframe_cloud_ += *time_normalized;

        pcl::VoxelGrid<CloudType::PointType> voxel_filter_;
        voxel_filter_.setLeafSize(config_.keyframe_voxel_size, config_.keyframe_voxel_size, config_.keyframe_voxel_size);
        voxel_filter_.setInputCloud(keyframe_cloud_);

        auto downsampled_cloud = std::make_shared<CloudType>();
        voxel_filter_.filter(*downsampled_cloud);

        keyframe_cloud_ = downsampled_cloud;
        return;
    }

    // deskew cloud

        //TODO: move implementation in Pose3D
        Eigen::Isometry3f previous_transform_matrix;
        previous_transform_matrix.fromPositionOrientationScale(previous_transform_.translation, previous_transform_.rotation, Eigen::Vector3f(1.0, 1.0, 1.0));

        Eigen::Isometry3f current_transform_matrix;
        current_transform_matrix.fromPositionOrientationScale(current_transform_.translation, current_transform_.rotation, Eigen::Vector3f(1.0, 1.0, 1.0));

        auto relative_transform = previous_transform_matrix.inverse() * current_transform_matrix;

        Pose3D end_point_local;
        end_point_local.translation = relative_transform.translation();
        end_point_local.rotation = relative_transform.rotation();

    auto deskewed_cloud = utils::transformNonRigid(*time_normalized, Pose3D(), end_point_local);
    filtered_cloud = deskewed_cloud;

    // match with keyframe
    {
        pcl::VoxelGrid<CloudType::PointType> voxel_filter_;
        voxel_filter_.setLeafSize(0.5, 0.5, 0.5);
        voxel_filter_.setInputCloud(filtered_cloud);

        auto downsampled_cloud = std::make_shared<CloudType>();
        voxel_filter_.filter(*downsampled_cloud);

        filtered_cloud = downsampled_cloud;
    }

    Eigen::Isometry3f guess;
    guess.fromPositionOrientationScale(current_transform_.translation, current_transform_.rotation, Eigen::Vector3f(1.0, 1.0, 1.0));

    pcl::IterativeClosestPoint<CloudType::PointType, CloudType::PointType> icp;

    icp.setInputSource(filtered_cloud);
    icp.setInputTarget(keyframe_cloud_);
    auto aligned = std::make_shared<CloudType>();
    icp.align(*aligned, guess.matrix());

    auto final_transform = icp.getFinalTransformation();
    Eigen::Isometry3f transform(final_transform);

    previous_transform_ = current_transform_;
    current_transform_.translation = transform.translation();
    current_transform_.rotation = transform.rotation();

    {
        auto deskewed_cloud_transformed = utils::transform(*deskewed_cloud, current_transform_);
        *keyframe_cloud_ += *deskewed_cloud_transformed;
    }
    pcl::VoxelGrid<CloudType::PointType> voxel_filter_;
    voxel_filter_.setLeafSize(config_.keyframe_voxel_size, config_.keyframe_voxel_size, config_.keyframe_voxel_size);
    voxel_filter_.setInputCloud(keyframe_cloud_);

    auto downsampled_cloud = std::make_shared<CloudType>();
    voxel_filter_.filter(*downsampled_cloud);

    keyframe_cloud_ = downsampled_cloud;
}

LidarOdometry::CloudType::Ptr
LidarOdometry::rangeFilter(const CloudType &input, float min_range, float max_range) const {
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

Pose3D LidarOdometry::getCurrentPose() const {
    return current_transform_;
}
