BB_ROOT=/bb

pushd $BB_ROOT || exit 1

source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

ros2 launch ldlidar_node ldlidar.launch.py

popd || exit 1 # $BB_ROOT/ros_ws
