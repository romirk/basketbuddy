import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitution import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            get_package_share_directory("basketbuddy"), "config", "slam.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false", description="Launch RViz when true"
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ldlidar_node'),
                    'launch',
                    'ldlidar.launch.py'
                ])
            ]),
        )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(use_rviz_arg)

    ld.add_action(slam_node)
    ld.add_action(lidar_launch)

    return ld
