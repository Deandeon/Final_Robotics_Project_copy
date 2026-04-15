import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription , TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Include the full simulation launch
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('description'),
                'launch',
                'spawn.launch.py'
            )
        )
    )

    # MPC controller node
    mpc_node = Node(
        package='mpc_control',
        executable='mpc_controller',
        name='mpc_controller',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('mpc_control'),
                'config',
                'mpc_params.yaml'
            ),
            {'use_sim_time': True}
        ]
    )

    timed_mpc = TimerAction(
        period=15.0,
        actions=[mpc_node]
    )

    return LaunchDescription([
        spawn,
        timed_mpc,
    ])
