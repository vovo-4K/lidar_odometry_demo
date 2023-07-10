# syntax=docker/dockerfile:1

FROM osrf/ros:humble-desktop

# build ceres2.2
RUN apt-get update && apt-get install -y cmake libgoogle-glog-dev libgflags-dev

RUN git clone https://ceres-solver.googlesource.com/ceres-solver --depth 1 && \
    mkdir ceres-solver/build && cd ceres-solver/build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF && make -j4 && make install && rm -rf /ceres-solver
  
# build lidar odometry package  
RUN mkdir -p /ws/src && cd /ws/src && \
    git clone https://github.com/vovo-4K/lidar_odometry_demo.git --depth 1 && \
    cd lidar_odometry_demo && \
    git submodule init && git submodule update

WORKDIR /ws

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    
RUN sed -i '$ d' /ros_entrypoint.sh && \
    echo "source /ws/install/setup.sh" >> /ros_entrypoint.sh && \
    echo 'exec "$@"' >> /ros_entrypoint.sh
