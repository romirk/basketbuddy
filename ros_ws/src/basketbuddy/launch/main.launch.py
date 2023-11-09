import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

BB_SHARE = "/bb/ros_ws/install/basketbuddy/share"

def generate_launch_description():

    og_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        arguments="--resolution 0.05 --publish_period_sec 1.0".split(),
        # remappings=[("/ldlidar_node/scan", "/scan")]
    )
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        arguments=f"--configuration_directory {BB_SHARE}/config --configuration_basename lds_2d.lua".split(),
        # remappings=[("/ldlidar_node/scan", "/scan")]
    )

    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
               os.path.join(get_package_share_directory("ldlidar_node"), "launch", "ldlidar_with_mgr.launch.py")
            ])
        )
    
    ld = LaunchDescription()

    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="--frame-id odom --child-frame-id ldlidar_base".split(),
    ))
    # ld.add_action(Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments="0 0 0 0 0 0 map odom".split(),
    # ))
    ld.add_action(lidar_launch)
    ld.add_action(og_node)
    ld.add_action(cartographer_node)

    return ld
