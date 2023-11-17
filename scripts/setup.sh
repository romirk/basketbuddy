#!/bin/bash

BB_ROOT=/bb

pushd $BB_ROOT/ros_ws || exit 1

source /opt/ros/humble/setup.bash

popd || exit 1 # $BB_ROOT/ros_ws
