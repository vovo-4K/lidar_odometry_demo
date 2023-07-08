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
    keyframe_.setVoxelsSize(0.5, config_.keyframe_voxel_size);
}

void LidarOdometry::processCloud(const pcl::PointCloud<lidar_point::PointXYZIRT> &input_cloud) {
    // normalize time
    auto time_normalized = utils::pointTimeNormalize(input_cloud);

    // range filter cloud
    auto filtered_cloud = utils::rangeFilter(*time_normalized, config_.lidar_min_range, config_.lidar_max_range);

    auto [planar, unclassified] = CloudClassifier::classify(*filtered_cloud);
    //temp_cloud_ = planar;

    // deskew cloud
    auto relative_transform = previous_transform_.relativeTo(current_transform_);
    previous_transform_ = current_transform_;

    auto deskewed_planar_cloud = CloudTransformer::transformNonRigid(*planar, relative_transform.inverse(), Pose3D());
    auto deskewed_unclassified_cloud = CloudTransformer::transformNonRigid(*unclassified, relative_transform.inverse(), Pose3D());

    // match with keyframe
    VoxelGrid<VoxelWithPoints<1>> scan_downsampler_planar(0.2, 1);
    scan_downsampler_planar.addCloud(*deskewed_planar_cloud);
    auto planar_voxelized = scan_downsampler_planar.getCloud();

    VoxelGrid<VoxelWithPoints<1>> scan_downsampler_unclassified(0.2, 1);
    scan_downsampler_unclassified.addCloud(*deskewed_planar_cloud);
    auto unclassified_voxelized = scan_downsampler_unclassified.getCloud();

    // init keyframe
    if (keyframe_.size() == 0) {
        keyframe_.addClouds(*planar_voxelized, *unclassified_voxelized);
        return;
    }

    CloudMatcher matcher;
    current_transform_ = matcher.align(keyframe_, *planar_voxelized, *unclassified_voxelized, current_transform_.compose(relative_transform));

    keyframe_.radiusCleanup(current_transform_.translation, 70.0);

    // update keyframe
    auto planar_transformed = CloudTransformer::transform(*planar_voxelized, current_transform_);
    auto unclassified_transformed = CloudTransformer::transform(*unclassified_voxelized, current_transform_);
    keyframe_.addClouds(*planar_transformed, *unclassified_transformed);

    temp_cloud_ = planar_transformed;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::getKeyFrameCloud() const {
    return keyframe_.getCloud();
}

Pose3D LidarOdometry::getCurrentPose() const {
    return current_transform_;
}
