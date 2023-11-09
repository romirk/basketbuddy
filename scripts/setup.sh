#!/bin/bash

BB_ROOT=/bb
BRANCH=docker

pushd $BB_ROOT/ros_ws || exit 1

git pull
git checkout $BRANCH

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

popd || exit 1 # $BB_ROOT/ros_ws
