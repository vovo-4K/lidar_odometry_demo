//
// Created by vl on 29.06.23.
//
//#define PCL_NO_PRECOMPILE

#include "lidar_odometry.h"
#include "utils/cloud_transform.h"
#include "utils/point_time_normalize.h"
#include "utils/range_filter.h"
#include "voxel_grid.h"
#include "cloud_matcher.h"

LidarOdometry::LidarOdometry(const LidarOdometry::Params& config) : config_(config) {
    current_transform_.translation.setZero();
    current_transform_.rotation.setIdentity();
    previous_transform_ = current_transform_;
    keyframe_grid_.setVoxelSize(0.2);//config_.keyframe_voxel_size);
}

void LidarOdometry::processCloud(const pcl::PointCloud<lidar_point::PointXYZIRT> &input_cloud) {
std::cout<<"cloud"<<std::endl;
    // pre filter cloud
    auto filtered_cloud = utils::rangeFilter(input_cloud, config_.lidar_min_range, config_.lidar_max_range);

    // init keyframe
    if (keyframe_grid_.size() == 0) {
        keyframe_grid_.addCloud(*filtered_cloud);
        return;
    }

    // normalize time
    auto time_normalized = utils::pointTimeNormalize(*filtered_cloud);

    // deskew cloud
    //TODO: move implementation in Pose3D
    Eigen::Isometry3f previous_transform_matrix;
    previous_transform_matrix.fromPositionOrientationScale(previous_transform_.translation, previous_transform_.rotation, Eigen::Vector3f(1.0, 1.0, 1.0));

    Eigen::Isometry3f current_transform_matrix;
    current_transform_matrix.fromPositionOrientationScale(current_transform_.translation, current_transform_.rotation, Eigen::Vector3f(1.0, 1.0, 1.0));

    auto relative_transform = previous_transform_matrix.inverse() * current_transform_matrix; //TODO: check
    previous_transform_ = current_transform_;

    Pose3D end_point_local;
    end_point_local.translation = relative_transform.translation();
    end_point_local.rotation = relative_transform.rotation();

    auto deskewed_cloud = utils::transformNonRigid(*time_normalized, Pose3D(), end_point_local);

    // match with keyframe
    VoxelGrid scan_downsampler(0.5);
    scan_downsampler.addCloud(*deskewed_cloud);
    auto deskewed_voxelized = scan_downsampler.getCloud();

    CloudMatcher matcher;
    current_transform_ = matcher.align(keyframe_grid_, deskewed_voxelized, current_transform_);

    // update keyframe
    auto deskewed_full_cloud_transformed = utils::transform(*deskewed_cloud, current_transform_);
    keyframe_grid_.addCloud(*deskewed_full_cloud_transformed);
}

LidarOdometry::CloudType::ConstPtr LidarOdometry::getKeyFrameCloud() const {
    return keyframe_grid_.getCloud();
}

Pose3D LidarOdometry::getCurrentPose() const {
    return current_transform_;
}
