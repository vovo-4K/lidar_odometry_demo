cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(third-party/robin-map)

include_directories(
        ${PCL_INCLUDE_DIRS}
)

add_executable(lidar_odometry_node
        src/cloud_matcher.cpp
        src/keyframe.h
        src/lidar_odometry_node.cpp
        src/lidar_odometry.cpp
        src/utils/cloud_classifier.h
        src/utils/point_time_normalize.h
        src/utils/range_filter.h
        src/voxel_grid.h
        )
target_include_directories(lidar_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar_odometry_node PUBLIC c_std_99 cxx_std_23)

target_link_libraries(lidar_odometry_node ${PCL_LIBRARIES} tsl::robin_map)
ament_target_dependencies(lidar_odometry_node rclcpp sensor_msgs tf2 tf2_ros pcl_conversions Eigen3)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
          test/test.cpp
          src/cloud_matcher.cpp
          src/voxel_grid.h
          src/keyframe.h
          )

  target_compile_features(${PROJECT_NAME}_test PUBLIC cxx_std_23)

  install(DIRECTORY test/test_data/
          DESTINATION ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_test_data
          )

  target_include_directories(${PROJECT_NAME}_test PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          )
  ament_target_dependencies(${PROJECT_NAME}_test
          Eigen3
          )
  target_link_libraries(${PROJECT_NAME}_test ${PCL_LIBRARIES} tsl::robin_map)
endif()

ament_package()
