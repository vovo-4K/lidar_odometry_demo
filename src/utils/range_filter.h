//
// Created by vl on 03.07.23.
//

#ifndef BUILD_RANGEFILTER_H
#define BUILD_RANGEFILTER_H

#include "../pose_3d.h"
#include <pcl/point_cloud.h>

namespace utils {
    template<typename PointType>
    typename pcl::PointCloud<PointType>::Ptr rangeFilter(const pcl::PointCloud<PointType>& input, float min_range, float max_range) {
        const auto min_range_sq = min_range * min_range;
        const auto max_range_sq = max_range * max_range;

        auto output_cloud_ptr = std::make_shared<pcl::PointCloud<PointType>>();
        output_cloud_ptr->points.reserve(input.points.size());

        for (const auto& point : input.points) {
            const auto range_sq = point.x*point.x + point.y*point.y + point.z*point.z;
            if (range_sq>=min_range_sq && range_sq<=max_range_sq) {
                output_cloud_ptr->points.push_back(point);
            }
        }

        return output_cloud_ptr;
    }
}


#endif //BUILD_RANGEFILTER_H
