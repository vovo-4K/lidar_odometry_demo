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

    Pose3D() {
        translation.setZero();
        rotation.setIdentity();
    }

    Pose3D(const Eigen::Vector3f& translation, const Eigen::Quaternionf& rotation)
    : translation(translation), rotation(rotation) {}

    Pose3D getRelativeTo(const Pose3D& target) const
    {
        Eigen::Quaternionf inverse_rotation = rotation.inverse();
        Eigen::Vector3f inverse_translation = inverse_rotation * (-translation);
        return {inverse_translation, inverse_rotation};
    }

    Pose3D compose(const Pose3D& another) const
    {
        return {translation + rotation * another.translation, rotation * another.rotation};
    }
};

#endif //BUILD_POSE3D_H
