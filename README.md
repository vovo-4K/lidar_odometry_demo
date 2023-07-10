# Lidar odometry demo
![image_overall](/images/img_overall.png)

Lidar-only odometry demo package for ROS2.

This implementation uses Point2Plane error function to match input point cloud with accumulated keyframe cloud.
For fast neighbor search implemented VoxelGrid structure which is mainly inspired by [KISS-ICP](https://github.com/PRBonn/kiss-icp) implementation.
The Point2Plane errors are minimized using [Ceres Solver](http://ceres-solver.org/) and the output transform used to deskew the next input cloud.  

Input topics:

* **/lidar_points** sensor_msgs/msg/PointCloud2 - expected not organized VLP16 pointcloud with XYZIRT point format

Output topics:

* **/odometry** nav_msgs/msg/Odometry - actual odometry output
* **/deskewed_cloud** sensor_msgs/msg/PointCloud2 - deskewed input cloud
* **/keyframe_cloud** sensor_msgs/msg/PointCloud2 - current keyframe cloud

Output tf:

* **/odom** -> **/base_scan**

### Install

There are two methods to run this package:

#### Into existing ROS2 workspace

Clone into workspace/src folder with:

```
cd workspace/src
git clone https://github.com/vovo-4K/lidar_odometry_demo.git
```

Build workspace:

```
cd workspace
colcon build
```

Run:

```
source workspace/install/setup.sh
ros2 run lidar_odometry lidar_odometry_node 
```

or

```
source workspace/install/setup.sh
ros2 run lidar_odometry lidar_odometry_node --ros-args --params-file <path_to_config>
```

where **<path_to_config>** is path to the configuration file. Template for the configuration file can be found in config/params.yaml

#### Using Docker

There is Dockerfile provided in the root of the source dir.

Clone the repo:

```
git clone https://github.com/vovo-4K/lidar_odometry_demo.git
```

Build the image:

```
cd lidar_odometry_demo
docker build . -t odometry_demo
```

Run the container:

```
docker run -it --rm odometry_demo ros2 run lidar_odometry lidar_odometry_node
```

### Limitations

- Only VLP16 lidar is supported due to input point format
- May be unreliable in high dynamic conditions or in case of sensor occlusion
  
### Used libraries

- [ROS2](https://docs.ros.org/)
- [Eigen3](https://eigen.tuxfamily.org/)
- [PCL](https://pointclouds.org/)
- [Ceres Solver](http://ceres-solver.org/)
- [tsl::robin_map](https://github.com/Tessil/robin-map)

