BB_ROOT=/bb

pushd $BB_ROOT >/dev/null || exit 1

source /opt/ros/${ROS_DISTRO}/setup.bash
source ./install/local_setup.bash

ros2 launch ldlidar_node ldlidar.launch.py

popd >/dev/null || exit 1 # $BB_ROOT/ros_ws
