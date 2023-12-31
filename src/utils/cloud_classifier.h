//
// Created by vl on 07.07.23.
//

#ifndef BUILD_CLOUD_CLASSIFIER_H
#define BUILD_CLOUD_CLASSIFIER_H

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

class CloudClassifier {
public:
    //!
    //! \tparam PointType pcl point type
    //! \param input input cloud
    //! \return std::pair where first element is planar points cloud, the second one is unclassified points
    template<typename PointType>
    static std::pair<typename pcl::PointCloud<pcl::PointNormal>::Ptr, typename pcl::PointCloud<PointType>::Ptr>
    classify(const pcl::PointCloud<PointType> &input) {

        auto organize_cloud = [](const pcl::PointCloud<PointType> &input){

            std::map<uint8_t, std::vector<PointType>> organized_rays;

            for (const auto& point : input) {
                auto ring_id = point.ring; //TODO: what if there is no ring field?
                if (organized_rays.contains(ring_id)) {
                    organized_rays.at(ring_id).push_back(point);
                } else {
                    organized_rays.emplace(ring_id, std::vector<PointType>{point});
                }
            }

            // find max row width
            size_t max_row_width = 0;
            for (const auto& ray : organized_rays) {
                if (ray.second.size() > max_row_width) {
                    max_row_width = ray.second.size();
                }
            }

            // sort by azimuth
            std::map<uint8_t, std::vector<PointType>> sorted_rays;
            for (const auto& ray : organized_rays) {
                std::vector<PointType> indexed_row;
                indexed_row.resize(max_row_width, PointType());

                for (const auto &point : ray.second) {
                    float azimuth = atan2(-point.y, point.x) + std::numbers::pi;
                    size_t approx_point_index = std::abs(azimuth * max_row_width / (2.0 * std::numbers::pi));

                    if (approx_point_index<max_row_width) {
                        indexed_row[approx_point_index] = point;
                    }
                }
                sorted_rays.insert({ray.first, indexed_row});
            }

            // form output cloud
            auto output = std::make_shared<pcl::PointCloud<PointType>>();
            output->points.reserve(input.size());

            for (const auto& ray : sorted_rays) {
                output->points.insert(output->points.end(), ray.second.cbegin(), ray.second.cend());
            }

            output->height = sorted_rays.size();
            output->width = max_row_width;

            return output;
        };

        auto organized_cloud = organize_cloud(input);

        auto planar_points_cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        auto unclassified_points_cloud = std::make_shared<pcl::PointCloud<PointType>>();

        planar_points_cloud->points.reserve(organized_cloud->size());
        unclassified_points_cloud->points.reserve(organized_cloud->size());

        // mark points curvature
        auto &cloud = organized_cloud->points;
        const int curvature_window_size = 4;
        const float intensity_max = 1000.0;
        for (int i=curvature_window_size; i < cloud.size() - curvature_window_size; i++) {
            auto& output_point = cloud.at(i);
            float range = powf(output_point.x, 2) + powf(output_point.y, 2) + powf(output_point.z, 2);
            if (range<0.1) {
                output_point.intensity = intensity_max;
                continue;
            }

            float dx = -cloud.at(i).x*(curvature_window_size * 2.0 + 1.0);
            float dy = -cloud.at(i).y*(curvature_window_size * 2.0 + 1.0);
            float dz = -cloud.at(i).z*(curvature_window_size * 2.0 + 1.0);

            for (int w = -curvature_window_size; w <= curvature_window_size; w++) {
                dx += cloud.at(i + w).x;
                dy += cloud.at(i + w).y;
                dz += cloud.at(i + w).z;
            }

            float curvature = sqrt(dx*dx + dy*dy + dz*dz) / range;

            output_point.intensity = curvature;
        }

        // find normals
        const int normals_window_size = 4;
        const int cloud_height = organized_cloud->height;
        const int cloud_width = organized_cloud->width;
        const float flatness_threshold = 0.05;

        for (int ray_id = 1; ray_id<cloud_height; ray_id++) {
            for (int point_index = normals_window_size; point_index<cloud_width - normals_window_size; point_index++) {
                const auto &point = organized_cloud->points.at( ray_id*cloud_width + point_index);
                if (point.intensity < flatness_threshold) {
                    // find neighbour flat points
                    auto prev_ray_id = ray_id - 1;

                    int found_points = 0;
                    Eigen::Vector3f leftmost_point;
                    for (int prev_ray_point = point_index - normals_window_size; prev_ray_point<point_index; prev_ray_point++) {
                        const auto &neighbour_point = organized_cloud->points.at( prev_ray_id*cloud_width + prev_ray_point);
                        if (neighbour_point.intensity < flatness_threshold*10.0) {
                            leftmost_point<<neighbour_point.x, neighbour_point.y, neighbour_point.z;
                            found_points++;
                            break;
                        }
                    }
                    Eigen::Vector3f rightmost_point;
                    for (int prev_ray_point = point_index + normals_window_size; prev_ray_point>point_index; prev_ray_point--) {
                        const auto &neighbour_point = organized_cloud->points.at( prev_ray_id*cloud_width + prev_ray_point);
                        if (neighbour_point.intensity < flatness_threshold*10.0) {
                            rightmost_point<<neighbour_point.x, neighbour_point.y, neighbour_point.z;
                            found_points++;
                            break;
                        }
                    }

                    if (found_points == 2) {
                        Eigen::Vector3f origin(point.x, point.y, point.z);
                        Eigen::Vector3f normal = ((leftmost_point - origin).cross(rightmost_point - origin)).normalized();

                        pcl::PointNormal planar_point;
                        planar_point.x = point.x;
                        planar_point.y = point.y;
                        planar_point.z = point.z;

                        planar_point.normal_x = normal.x();
                        planar_point.normal_y = normal.y();
                        planar_point.normal_z = normal.z();

                        planar_points_cloud->points.push_back(planar_point);
                    } else {
                        unclassified_points_cloud->points.push_back(point);
                    }
                } else {
                    if (point.intensity < intensity_max) {
                        unclassified_points_cloud->points.push_back(point);
                    }
                }

            }
        }

        return {planar_points_cloud, unclassified_points_cloud};
    }
};

#endif //BUILD_CLOUD_CLASSIFIER_H
