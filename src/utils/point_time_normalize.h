//
// Created by vl on 02.07.23.
//

#ifndef BUILD_POINT_TIME_NORMALIZE_H
#define BUILD_POINT_TIME_NORMALIZE_H

#include <algorithm>
#include <limits>
#include <pcl/point_cloud.h>

namespace utils {

    template<typename PointType>
    typename pcl::PointCloud<PointType>::Ptr pointTimeNormalize(const pcl::PointCloud<PointType>& input)
    {
        float min_time = std::numeric_limits<float>::max();
        float max_time = std::numeric_limits<float>::lowest();

        std::for_each(std::execution::seq, input.points.cbegin(), input.points.cend(),
                      [&min_time, &max_time](const auto& point){
                          min_time = std::min(min_time, point.time);
                          max_time = std::max(max_time, point.time);
                      });

        float time_range = max_time - min_time;

        auto output = std::make_shared<pcl::PointCloud<PointType>>();
        output->points.resize(input.points.size());

        std::transform(std::execution::par, input.points.cbegin(), input.points.cend(), output->points.begin(),
                       [min_time, time_range](const auto& input_point){
                           PointType output_point = input_point;
                           output_point.time = (input_point.time - min_time) / time_range;
                           return output_point;
                       });

        return output;
    }
}


#endif //BUILD_POINT_TIME_NORMALIZE_H
