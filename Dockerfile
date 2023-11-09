FROM ros:humble-ros-core

RUN apt update && apt upgrade -y && \
    apt install -y git python3 python3-pip \
    libudev-dev ros-${ROS_DISTRO}-slam-toolbox \
    python3-rosdep python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-diagnostic-updater \
    && apt-get autoremove -y && \
    apt-get clean

RUN rosdep init && rosdep update

COPY ./ros_ws/src /bb/ros_ws/src
COPY ./scripts/start.sh /bb/start.sh

RUN cd /bb/ros_ws && bash -c ". /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release"

CMD ["bash"]
