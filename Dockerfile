FROM ros:humble-ros-core

RUN apt update && apt upgrade -y && \
    apt install -y ros-${ROS_DISTRO}-slam-toolbox && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./slam_params.yaml /slam_params.yaml
