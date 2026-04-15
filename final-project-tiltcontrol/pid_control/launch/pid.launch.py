"""
pid.launch.py. Launches everything needed for PID control in Gazebo...
- Includes spawn.launch.py (starts Gazebo, spawns robot, activates controllers,)...
- Launches the PID controller node after a delay...
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Including the spawn launch file (starts Gazebo + robot + controllers)...
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('description'),
                'launch', 'spawn.launch.py'
            )
        )
    )

    # the PID controller params file...
    pid_params = os.path.join(
        get_package_share_directory('pid_control'),
        'config', 'pid_params.yaml'
    )

    # PID controller node...
    pid_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='pid_control',
                executable='pid_controller_node',
                name='pid_controller',
                parameters=[
                    pid_params,
                    {'use_sim_time': True},
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        spawn_launch,
        pid_node,
    ])
