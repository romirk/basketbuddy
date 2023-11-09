#!/bin/bash

BB_ROOT=/bb

pushd $BB_ROOT/ros_ws || exit 1

source /opt/ros/humble/setup.bash
rosdep init && rosdep update || exit 1
rosdep install --from-paths src --ignore-src -r -y || exit 1
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release || exit 1

popd || exit 1 # $BB_ROOT/ros_ws
