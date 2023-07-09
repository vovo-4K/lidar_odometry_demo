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
    keyframe_.setVoxelSize(config_.keyframe_voxel_size);
    keyframe_.setMaxPoints(20);
}

void LidarOdometry::processCloud(const pcl::PointCloud<lidar_point::PointXYZIRT> &input_cloud) {
    auto start_time = std::chrono::high_resolution_clock::now();
    // normalize time
    auto time_normalized = utils::pointTimeNormalize(input_cloud);
    // deskew cloud
    auto relative_transform = previous_transform_.relativeTo(current_transform_);
    previous_transform_ = current_transform_;
    auto deskewed_input_cloud = CloudTransformer::transformNonRigid(*time_normalized, relative_transform.inverse(), Pose3D());
    temp_cloud_ = deskewed_input_cloud;

    auto [planar, unclassified] = CloudClassifier::classify(*deskewed_input_cloud);

    //pcl::io::savePCDFileBinary("/home/vl/temp/normals.pcd", *planar);

    auto filtered_cloud = utils::rangeFilter(*planar, config_.lidar_min_range, config_.lidar_max_range);

    VoxelGrid keyframe_downsampler(0.1, 1);
    keyframe_downsampler.addCloud(*filtered_cloud);

    if (keyframe_.size() == 0) {
        // init keyframe
        keyframe_.addCloud(*keyframe_downsampler.getCloud());
        return;
    }

    VoxelGrid matching_downsampler(0.5, 1);
    matching_downsampler.addCloud(*filtered_cloud);

    CloudMatcher matcher;
    current_transform_ = matcher.align(keyframe_, *matching_downsampler.getCloudWithoutNormals(),
                                       current_transform_.compose(relative_transform));

    keyframe_.radiusCleanup(current_transform_.translation, 80.0);

    auto keyframe_update = CloudTransformer::transformWithNormals(*keyframe_downsampler.getCloud(), current_transform_);
    keyframe_.addCloud(*keyframe_update);


    auto end_time = std::chrono::high_resolution_clock::now();
    auto time = end_time - start_time;
    std::cout<<"processing time: "<< time/std::chrono::milliseconds(1) <<"ms"<<std::endl;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::getKeyFrameCloud() const {
    //return keyframe_.getCloudWithoutNormals();
    return keyframe_.getSparseCloudWithoutNormals();
}

Pose3D LidarOdometry::getCurrentPose() const {
    return current_transform_;
}
