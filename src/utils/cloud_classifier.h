//
// Created by vl on 07.07.23.
//

#ifndef BUILD_CLOUD_CLASSIFIER_H
#define BUILD_CLOUD_CLASSIFIER_H

#include <pcl/point_cloud.h>

class CloudClassifier {
public:
    //!
    //! \tparam PointType pcl point type
    //! \param input input cloud
    //! \return std::pair where first element is planar points cloud, the second one is unclassified points
    template<typename PointType>
    static std::pair<typename pcl::PointCloud<PointType>::Ptr, typename pcl::PointCloud<PointType>::Ptr>
        classify(const pcl::PointCloud<PointType> &input) {

            auto organize_cloud = [](const pcl::PointCloud<PointType> &input){
                auto output = std::make_shared<pcl::PointCloud<PointType>>();
                output->points.reserve(input.size());

                std::unordered_map<uint8_t, std::vector<PointType>> organized;

                for (const auto& point : input) {
                    auto ring_id = point.ring; //TODO: what if there is no ring field?
                    if (organized.contains(ring_id)) {
                        organized.at(ring_id).push_back(point);
                    } else {
                        organized.emplace(ring_id, std::vector<PointType>{point});
                    }
                }

                for (const auto& ray : organized) {
                    //TODO: sort by azimuth
                    output->points.insert(output->points.end(), ray.second.cbegin(), ray.second.cend());
                }

                return output;
            };

            auto organized_cloud = organize_cloud(input);

            auto planar_points_cloud = std::make_shared<pcl::PointCloud<PointType>>();
            auto unclassified_points_cloud = std::make_shared<pcl::PointCloud<PointType>>();

            planar_points_cloud->points.reserve(organized_cloud->size());
            unclassified_points_cloud->points.reserve(organized_cloud->size());

            const auto &cloud = organized_cloud->points;
            const int window_size = 5;
            for (int i=window_size; i<cloud.size()-window_size; i++) {
                float range = sqrt(powf(cloud.at(i).x, 2) + powf(cloud.at(i).y, 2) + powf(cloud.at(i).z, 2));
                if (range<0.1) continue;

                float dx = -cloud.at(i).x*(window_size*2.0 + 1.0);
                float dy = -cloud.at(i).y*(window_size*2.0 + 1.0);
                float dz = -cloud.at(i).z*(window_size*2.0 + 1.0);

                for (int w = -window_size; w <= window_size; w++) {
                    dx += cloud.at(i + w).x;
                    dy += cloud.at(i + w).y;
                    dz += cloud.at(i + w).z;
                }

                float curvature = sqrt(dx*dx + dy*dy + dz*dz) / range;

                auto output_point = cloud.at(i);
                output_point.intensity = curvature;
                planar_points_cloud->points.push_back(output_point);
            }

            return {planar_points_cloud, unclassified_points_cloud};
    }
};

#endif //BUILD_CLOUD_CLASSIFIER_H
