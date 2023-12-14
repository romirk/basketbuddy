import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

BB_SHARE = get_package_share_directory("basketbuddy")

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
        arguments=f"--configuration_directory {BB_SHARE}/config --configuration_basename lidar_2d.lua".split(),
        # remappings=[("/ldlidar_node/scan", "/scan")]
    )

    # slam toolbox
    # slam_toolbox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.launch.py")
    #     )
    # )

    
    ld = LaunchDescription()

    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="--frame-id odom --child-frame-id base_link".split(),
    ))
    ld.add_action(slam_toolbox_launch)

    return ld
