import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    turtlebot3_empty_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'empty_world.launch.py'),
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    turtlebot3_controller_node = Node(
        package='turtlebot3_controller',
        namespace='',
        executable='controller',
        name='turtlebot_controller',
        parameters=[
            {
                "kp_linear": 0.5
            },
            {
                "kp_angular": 1.0
            },
            {
                "tolerance": 0.1
            },
        ],
    )
    ld = LaunchDescription()
    ld.add_action(turtlebot3_empty_world_cmd)
    ld.add_action(turtlebot3_controller_node)
    return ld