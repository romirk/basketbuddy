#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash

apt update
apt install -y libopencv-dev libudev-dev git pkg-config python3 python3-colcon-common-extensions \
  python3-pip python3-rosdep ros-${ROS_DISTRO}-cartographer ros-${ROS_DISTRO}-cartographer-ros \
  ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-nav2-amcl \
  ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-tf2-geometry-msgs ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-tf2-sensor-msgs

apt upgrade -y

mkdir src

if [ "$(uname -m)" = "aarch64" ]; then
  # check if src/nav2_bringup does not exist
  if [ ! -d "src/nav2_bringup" ]; then
    curl -L https://api.github.com/repos/ros-planning/navigation2/tarball/1.2.2 |
      tar xz -C src/ --wildcards "*/nav2_bringup" --strip-components=1
  fi
fi

rosdep update --as-root apt:false --rosdistro=${ROS_DISTRO}

cd src
git clone https://github.com/romirk/ldrobot-lidar-ros2.git
cd ..

rosdep install --from-paths src --ignore-src -r -y
colcon build --install-base /bb/install --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
