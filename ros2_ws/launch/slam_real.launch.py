import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    slam_share = get_package_share_directory('slam_toolbox')
    slam_params = os.path.join(
        package_share, 'config', 'slam_toolbox_real.yaml')

    scan_self_filter = Node(
        package='tai_robot_one',
        executable='scan_self_filter',
        name='scan_self_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_filtered',
            'rviz_output_topic': '/scan_filtered_rviz',
            'min_x': 0.42,
            'max_x': 0.51,
            'min_y': -0.155,
            'max_y': 0.155,
        }],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_params,
        }.items(),
    )

    return LaunchDescription([scan_self_filter, slam])
