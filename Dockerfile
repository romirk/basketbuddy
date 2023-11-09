FROM ros:humble-ros-core

RUN apt update && apt upgrade -y && \
    apt install -y python3 python3-pip \
    python3-rosdep libudev-dev git \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-diagnostic-updater

RUN rosdep init && rosdep update %

RUN mkdir -p /bb/ros_ws/src && \
    cd /bb/ros_ws/src && \
    git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git

RUN cd /bb/ros_ws && bash -c ". /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    --packages-select ldlidar_node ldlidar_component"

# everything above this line is cached

COPY ./ros_ws/src/basketbuddy /bb/ros_ws/src/basketbuddy
RUN cd /bb/ros_ws && bash -c ". /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    --packages-select basketbuddy"

RUN apt install -y ros-${ROS_DISTRO}-cartographer \
    && apt-get autoremove -y && apt-get clean \
    && rm -rf /var/lib/apt/lists/* 

COPY ./scripts/start.sh /bb/start.sh

CMD ["bash"]
