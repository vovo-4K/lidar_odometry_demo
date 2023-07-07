//
// Created by vl on 29.06.23.
//
//#define PCL_NO_PRECOMPILE

#include "lidar_odometry.h"
#include "utils/cloud_classifier.h"
#include "utils/cloud_transform.h"
#include "utils/point_time_normalize.h"
#include "utils/range_filter.h"
#include "voxel_grid.h"
#include "cloud_matcher.h"

LidarOdometry::LidarOdometry(const LidarOdometry::Params& config) : config_(config) {
    current_transform_.translation.setZero();
    current_transform_.rotation.setIdentity();
    previous_transform_ = current_transform_;
    keyframe_grid_.setVoxelSize(config_.keyframe_voxel_size);
}

void LidarOdometry::processCloud(const pcl::PointCloud<lidar_point::PointXYZIRT> &input_cloud) {

    //TODO: normalize time and deskew at first

    // pre filter cloud
    auto filtered_cloud = utils::rangeFilter(input_cloud, config_.lidar_min_range, config_.lidar_max_range);

    auto [planar, unclassified] = CloudClassifier::classify(*filtered_cloud);
    temp_cloud_ = planar;

    // init keyframe
    if (keyframe_grid_.size() == 0) {
        auto initial_cloud = CloudTransformer::transformNonRigid(*filtered_cloud, Pose3D(), Pose3D());
        // TODO: voxelize
        keyframe_grid_.addCloud(*initial_cloud);
        return;
    }

    // normalize time
    auto time_normalized = utils::pointTimeNormalize(*filtered_cloud);

    // deskew cloud
    auto relative_transform = previous_transform_.relativeTo(current_transform_);
    previous_transform_ = current_transform_;

    auto deskewed_cloud = CloudTransformer::transformNonRigid(*time_normalized, relative_transform.inverse(), Pose3D());

    // match with keyframe
    VoxelGrid scan_downsampler(0.3, 1);
    scan_downsampler.addCloud(*deskewed_cloud);
    auto deskewed_voxelized = scan_downsampler.getCloud();

    CloudMatcher matcher;
    current_transform_ = matcher.align(keyframe_grid_, *deskewed_voxelized, current_transform_.compose(relative_transform));

    keyframe_grid_.radiusCleanup(current_transform_.translation, 50.0);

    // update keyframe
    auto deskewed_voxelized_transformed = CloudTransformer::transform(*deskewed_voxelized, current_transform_);
    keyframe_grid_.addCloud(*deskewed_voxelized_transformed);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::getKeyFrameCloud() const {
    return keyframe_grid_.getCloud();
}

Pose3D LidarOdometry::getCurrentPose() const {
    return current_transform_;
}
