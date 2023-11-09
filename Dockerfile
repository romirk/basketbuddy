FROM ros:humble-ros-core

RUN apt update && apt upgrade -y && \
    apt install -y python3 python3-pip \
    python3-rosdep libudev-dev git \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-diagnostic-updater

RUN rosdep init && rosdep update

RUN mkdir -p /bb/ros_ws/src && \
    cd /bb/ros_ws/src && \
    git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git

RUN cd /bb/ros_ws && bash -c ". /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release"

# everything above this line is cached

RUN apt install -y ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer

COPY ./ros_ws/src/basketbuddy /bb/ros_ws/src/basketbuddy
RUN cd /bb/ros_ws && bash -c ". /opt/ros/humble/setup.bash && \
    rosdep install --from-path src/basketbuddy --ignore-src -r -y && \
    colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    --packages-select basketbuddy"

RUN apt remove -y python3-pip python3-rosdep \
    python3-colcon-common-extensions git && \
    SUDO_FORCE_REMOVE=yes apt autoremove -y && apt clean && \
    rm -rf /var/lib/apt/lists/* /bb/ros_ws/log /etc/ros/rosdep \
    /bb/ros_ws/src /bb/ros_ws/build

COPY ./scripts/start.sh /bb/start.sh

CMD ["bash"]
