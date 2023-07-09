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

    auto filtered_cloud = utils::rangeFilter(*planar, config_.lidar_min_range, config_.lidar_max_range);

    VoxelGrid keyframe_downsampler(0.1, 1);
    keyframe_downsampler.addCloud(*filtered_cloud);

    if (keyframe_.size() == 0) {
        // init keyframe
        keyframe_.addCloud(*keyframe_downsampler.getCloud());
        return;
    }

    VoxelGrid matching_downsampler(0.25, 1);
    matching_downsampler.addCloud(*filtered_cloud);

    CloudMatcher matcher;
    Pose3D new_transform_ = matcher.align(keyframe_, *matching_downsampler.getCloudWithoutNormals(),
                                       current_transform_.compose(relative_transform));

    {
        Eigen::Vector3f angles =
                (new_transform_.rotation * current_transform_.rotation.inverse()).toRotationMatrix().eulerAngles(0, 1,2) *  180.0 / std::numbers::pi;
        if (!((std::abs(angles(0)) < 5.0 || std::abs(angles(0)) > 175.0) &&
              (std::abs(angles(1)) < 5.0 || std::abs(angles(1)) > 175.0) &&
              (std::abs(angles(2)) < 5.0 || std::abs(angles(2)) > 175.0))) {
            std::cout << "unstable rotation " << angles.transpose() << std::endl;
            //new_transform_.rotation = current_transform_.compose(relative_transform).rotation;
            new_transform_ = current_transform_.compose(relative_transform);
        }
    }

    current_transform_ = new_transform_;

    keyframe_.radiusCleanup(current_transform_.translation, 80.0);

    auto keyframe_update = CloudTransformer::transformWithNormals(*keyframe_downsampler.getCloud(), current_transform_);
    keyframe_.addCloud(*keyframe_update);


    auto end_time = std::chrono::high_resolution_clock::now();
    auto time = end_time - start_time;
    std::cout<<"processing time: "<< time/std::chrono::milliseconds(1) <<"ms"<<std::endl;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::getKeyFrameCloud() const {
    return keyframe_.getSparseCloudWithoutNormals();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::getFullKeyFrameCloud() const {
    return keyframe_.getCloudWithoutNormals();
}

Pose3D LidarOdometry::getCurrentPose() const {
    return current_transform_;
}
