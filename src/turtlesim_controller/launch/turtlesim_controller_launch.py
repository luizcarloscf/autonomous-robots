from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='',
            executable='turtlesim_node',
            name='turtlesim_node',
        ),
        Node(
            package='turtlesim_controller',
            namespace='',
            executable='controller',
            name='turtlesim_controller',
            parameters=[
                {"kp_linear": 2.0},
                {"kp_angular": 7.0},
                {"tolerance": 0.1},
            ]
        ),
    ])
