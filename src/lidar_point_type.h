//
// Created by vl on 30.06.23.
//

#ifndef BUILD_LIDAR_POINT_TYPE_H
#define BUILD_LIDAR_POINT_TYPE_H

#include <pcl/point_types.h>
#include <pcl/PCLHeader.h>

namespace lidar_point
        {
                struct PointXYZIRT
                {
                    PCL_ADD_POINT4D;                    // quad-word XYZ
                    float    intensity;                 ///< laser intensity reading
                    uint16_t ring;                      ///< laser ring number
                    float time;
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
                } EIGEN_ALIGN16;

        };

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_point::PointXYZIRT,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(uint16_t, ring, ring)
(float, time, time)
)

#endif //BUILD_LIDAR_POINT_TYPE_H
