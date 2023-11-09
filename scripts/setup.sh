#!/bin/bash

apt update && apt upgrade -y

BB_ROOT=/bb
BRANCH=docker

# check for packages
apt install -y python3 python3-pip libudev-dev ros-${ROS_DISTRO}-slam-toolbox

apt-get autoremove -y
apt-get clean
rm -rf /var/lib/apt/lists/*

pushd $BB_ROOT/ros_ws || exit 1

git pull
git checkout $BRANCH

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

popd || exit 1 # $BB_ROOT/ros_ws
