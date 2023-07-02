//
// Created by vl on 30.06.23.
//

#ifndef BUILD_POSE3D_H
#define BUILD_POSE3D_H

#include <eigen3/Eigen/Eigen>

class Pose3D {
public:
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
};

#endif //BUILD_POSE3D_H
