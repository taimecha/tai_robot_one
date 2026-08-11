import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')

    # Process the URDF file
    pkg_path = get_package_share_directory('tai_robot_one')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    # Publish zero positions for movable joints while viewing the model in RViz.
    # Disable this node when the real motor driver publishes /joint_states.
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params],
        condition=IfCondition(use_joint_state_publisher)
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='true',
            description='Publish zero wheel positions for RViz when no hardware driver is active'),

        node_joint_state_publisher,
        node_robot_state_publisher
    ])
