BB_ROOT=/bb
BRANCH=docker

pushd $BB_ROOT/ros_ws || exit 1

ls -la

source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

ros2 launch ldlidar_node ldlidar.launch.py

popd || exit 1 # $BB_ROOT/ros_ws