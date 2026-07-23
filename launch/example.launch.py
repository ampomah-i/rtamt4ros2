"""Launch the monitor together with a small sine-wave signal source."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = PathJoinSubstitution(
        [get_package_share_directory("rtamt4ros2"), "config", "monitor.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="rtamt4ros2",
                executable="stl_monitor_node",
                name="stl_monitor",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rtamt4ros2",
                executable="sine_wave_publisher",
                name="sine_wave_source",
                output="screen",
            ),
        ]
    )
