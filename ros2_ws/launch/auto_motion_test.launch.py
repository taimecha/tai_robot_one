import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            # The tester is the only /joint_states publisher in this launch.
            'use_joint_state_publisher': 'false',
        }.items(),
    )

    motion_tester = Node(
        package='tai_robot_one',
        executable='test_robot_motion',
        name='robot_motion_tester',
        output='screen',
    )

    return LaunchDescription([
        robot_description_launch,
        motion_tester,
    ])
