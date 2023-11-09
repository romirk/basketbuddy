#!/bin/bash

sudo apt update

BB_ROOT=/home/jetson/basketbuddy
BRANCH=docker

function require_package(package_name) {
    if ! dpkg -s $package_name >/dev/null 2>&1; then
        sudo apt-get install $package_name
    fi
}

# check for packages
require_package git
require_package libudev-dev

pushd $BB_ROOT || exit 1

git pull
git checkout $BRANCH
git submodule update --init --recursive

pushd ./ros_ws/src/ldrobot-lidar-ros2/scripts || exit 1
./create_udev_rules.sh
popd || exit 1 # ./ros_ws/src/ldrobot-lidar-ros2/scripts

pushd ./ros_ws || exit 1
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
popd || exit 1 # ./ros_ws

popd || exit 1 # $BB_ROOT

