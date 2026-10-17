FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-thread-dev \
    libceres-dev \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    python3-numpy \
    ros-noetic-cmake-modules \
    ros-noetic-cv-bridge \
    ros-noetic-eigen-conversions \
    ros-noetic-image-transport \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-rosbag \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-tf2-eigen \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/src/Fast_Livo2

# The repository carries the legacy Sophus ABI used by the copied upstream
# implementation. Install that exact library in the image so the executable
# also resolves it at runtime, not only at link time.
RUN cp /ws/src/Fast_Livo2/vendor/sophus_legacy/lib/libSophus.so /usr/local/lib/ \
    && ldconfig

# Do not overcommit 12 compiler jobs on an 11 GiB NUC/laptop. Parallel
# translation-unit compilation otherwise pages heavily into swap and can
# starve ROS bag replays on the same machine.
ARG CATKIN_JOBS=6
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make -j${CATKIN_JOBS} -DCMAKE_BUILD_TYPE=Release'

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
