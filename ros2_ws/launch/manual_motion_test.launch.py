import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    xacro_file = os.path.join(
        package_share, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    parameters = [{
        'robot_description': robot_description,
        'use_sim_time': False,
    }]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=parameters,
    )

    # GUI sliders publish the only /joint_states stream in this launch.
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=parameters,
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_gui,
    ])
