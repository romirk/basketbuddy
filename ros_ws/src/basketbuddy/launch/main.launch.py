import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # slam_node = Node(
    #     package="slam_toolbox",
    #     executable="map_and_localization_slam_toolbox_node",
    #     arguments=[{"params_file": "/bb/slam.yaml"}],
    #     # remappings=[("/ldlidar_node/scan", "/scan")]
    # )

    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
               os.path.join(get_package_share_directory("ldlidar_node"), "launch", "ldlidar_with_mgr.launch.py")
            ])
        )
    
    ld = LaunchDescription()

    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 map ldlidar_link".split(),
    ))
    # ld.add_action(Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments="0 0 0 0 0 0 map odom".split(),
    # ))
    ld.add_action(lidar_launch)
    # ld.add_action(slam_node)

    return ld
