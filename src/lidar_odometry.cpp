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
    keyframe_grid_.setVoxelSize(0.25);//config_.keyframe_voxel_size);
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
    auto relative_transform = previous_transform_.getRelativeTo(current_transform_);
    previous_transform_ = current_transform_;

    auto deskewed_cloud = utils::transformNonRigid(*time_normalized, Pose3D(), relative_transform);

    // match with keyframe
    VoxelGrid scan_downsampler(1.0);
    scan_downsampler.addCloud(*deskewed_cloud);
    auto deskewed_voxelized = scan_downsampler.getCloud();

    CloudMatcher matcher;
    current_transform_ = matcher.align(keyframe_grid_, deskewed_voxelized, current_transform_.compose(relative_transform));

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
