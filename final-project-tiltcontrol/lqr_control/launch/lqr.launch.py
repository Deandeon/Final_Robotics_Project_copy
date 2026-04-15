import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for LQR controller."""
    pkg_name = 'lqr_control'
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    lqr_node = Node(
        package='lqr_control',
        executable='lqr_controller',
        name='lqr_controller',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'lqr_params.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        lqr_node,
    ])