//
// Created by vl on 03.07.23.
//

#ifndef BUILD_CLOUDMATCHER_H
#define BUILD_CLOUDMATCHER_H

#include <pcl/point_cloud.h>
#include "lidar_point_type.h"
#include "pose_3d.h"
#include "keyframe.h"

class CloudMatcher {
public:
    Pose3D align(const Keyframe& keyframe, const pcl::PointCloud<pcl::PointXYZ> &planar_cloud,
                 const pcl::PointCloud<pcl::PointXYZ> &unclassified_cloud,
                 const Pose3D& position_guess);
};


#endif //BUILD_CLOUDMATCHER_H
