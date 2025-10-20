from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtamt4ros2',
            executable='stl_monitor_node',
            name='stl_monitor',
            output='screen',
            parameters=[
                {'formula': 'out = always[0,5](x > 0.5)'},
                {'vars': ['x']},
                {'sampling_period': 0.1},
            ]
        )
    ])
