# Dockerfile: Ubuntu 20.04 + ROS 2 Foxy + GTSAM + LIO-SAM

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    git \
 && rm -rf /var/lib/apt/lists/*

#
# 1) Base locale and essential packages
#
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
 && rm -rf /var/lib/apt/lists/*

# Configure UTF-8 locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8

#
# 2) Add the official ROS 2 apt repository and install Foxy + colcon
#
RUN curl -sSL http://repo.ros2.org/repos.key | apt-key add -
RUN echo "deb [arch=amd64] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2-latest.list

RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

#
# 3) Install GTSAM 4.x from the BorgLab PPA (available on Ubuntu 20.04)
#
RUN apt-get update && apt-get install -y software-properties-common \
 && rm -rf /var/lib/apt/lists/*
RUN add-apt-repository ppa:borglab/gtsam-release-4.0
RUN apt-get update && apt-get install -y \
    libgtsam-dev \
    libgtsam-unstable-dev \
 && rm -rf /var/lib/apt/lists/*

#
# 4) Additional dependencies needed by LIO-SAM
#    (including python3-empy which is crucial for Foxy’s msg generator)
#
RUN apt-get update && apt-get install -y \
    libatlas-base-dev \
    libeigen3-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    libglew-dev \
    libgeographic-dev \
    ros-foxy-pcl-conversions \
    ros-foxy-pcl-ros \
    ros-foxy-tf2 \
    ros-foxy-tf2-ros \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-image-transport-plugins \
    ros-foxy-xacro \
    ros-foxy-robot-state-publisher \
    ros-foxy-rviz2 \
    python3-empy \
 && rm -rf /var/lib/apt/lists/*

#
# 5) Clone LIO-SAM (liorf) for ROS 2 and build
#
# RUN mkdir -p /root/ros2_ws/src

WORKDIR /root/ros2_ws

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

#
# Final default command
#
CMD ["/bin/bash"]
