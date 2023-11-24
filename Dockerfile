FROM ros:humble-ros-perception

WORKDIR /bb

RUN bash <<EOF
source /opt/ros/humble/setup.bash
apt update
apt install -y python3 python3-pip python3-rosdep libudev-dev git \
    ros-${ROS_DISTRO}-slam-toolbox python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs \
    libopencv-dev ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins 
apt upgrade -y

rosdep init && rosdep update --as-root apt:false --rosdistro=${ROS_DISTRO}

mkdir src && cd src
git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git
cd ..


rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --install-base \
    /bb/install --symlink-install
EOF

# everything above this line is cached

COPY ./ros_ws/src/basketbuddy src/basketbuddy/

RUN bash <<EOF
source /opt/ros/humble/setup.bash
rosdep install --from-path src/basketbuddy --ignore-src -r -y
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --install-base \
    /bb/install --packages-select basketbuddy --symlink-install
EOF

COPY ./scripts/start.sh .

RUN <<EOF
apt remove -y python3-pip python3-rosdep \
    python3-colcon-common-extensions git
SUDO_FORCE_REMOVE=yes apt autoremove -y
apt clean -y
rm -rf /var/lib/apt/lists/* /bb/ros_ws/log /etc/ros/rosdep \
    /bb/ros_ws/src /bb/ros_ws/build
EOF

CMD ["bash", "/bb/start.sh"]
