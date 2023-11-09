FROM ros:humble-ros-core

RUN rm -f /etc/apt/apt.conf.d/docker-clean
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt update && apt upgrade -y && \
    apt install -y python3 python3-pip \
    python3-rosdep ros-${ROS_DISTRO}-slam-toolbox \
    libudev-dev python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-cartographer \
    && apt-get autoremove -y && \
    apt-get clean \ 
    rm -f /etc/apt/apt.conf.d/docker-clean

RUN rosdep init && rosdep update

COPY ./scripts/start.sh /bb/start.sh
COPY ./ros_ws/src/ldrobot-lidar-ros2/ /bb/ros_ws/src/ldrobot-lidar-ros2/

RUN cd /bb/ros_ws && bash -c ". /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    --packages-select ldlidar_node ldlidar_component" \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros_ws/src/basketbuddy /bb/ros_ws/src/basketbuddy
RUN cd /bb/ros_ws && bash -c ". /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    --packages-select basketbuddy"

CMD ["bash"]
