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

    Pose3D relativeTo(const Pose3D& target) const
    {
        auto inv = inverse();
        return inv.compose(target);
    }

    Pose3D compose(const Pose3D& another) const
    {
        return {translation + rotation * another.translation, rotation * another.rotation};
    }

    Pose3D inverse() const
    {
        Eigen::Quaternionf inverse_rotation = rotation.inverse();
        Eigen::Vector3f inverse_translation = inverse_rotation * (-translation);
        return {inverse_translation, inverse_rotation};
    }

    Eigen::Matrix3f rotationMatrix() const
    {
        return rotation.toRotationMatrix();
        /*
        const float q0 = rotation.w();
        const float q1 = rotation.x();
        const float q2 = rotation.y();
        const float q3 = rotation.z();

        Eigen::Matrix3f R;

        R << q0*q0 + q1*q1 - q2*q2 - q3*q3, 2.0f*(q1*q2 - q0*q3), 2.0f*(q1*q3 + q0*q2),
                2.0f*(q1*q2 + q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2.0f*(q2*q3 - q0*q1),
                2.0f*(q1*q3 - q0*q2), 2.0f*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3;

        return R;*/
    }

};

#endif //BUILD_POSE3D_H
