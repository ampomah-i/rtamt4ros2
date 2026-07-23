"""Launch a configurable RTAMT monitor."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [get_package_share_directory("rtamt4ros2"), "config", "monitor.yaml"]
    )
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the monitor parameter YAML file",
            ),
            Node(
                package="rtamt4ros2",
                executable="stl_monitor_node",
                name="stl_monitor",
                output="screen",
                parameters=[config],
            ),
        ]
    )
