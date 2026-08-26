import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    slam_share = get_package_share_directory('slam_toolbox')

    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    use_phone_teleop = LaunchConfiguration('use_phone_teleop')
    phone_teleop_port = LaunchConfiguration('phone_teleop_port')
    headless = LaunchConfiguration('headless')
    slam_start_delay = LaunchConfiguration('slam_start_delay')
    teleop_start_delay = LaunchConfiguration('teleop_start_delay')
    slam_params = os.path.join(package_share, 'config', 'slam_toolbox.yaml')
    slam_rviz = os.path.join(package_share, 'config', 'slam_robot.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            # This launch owns a map-focused RViz instance below.
            'use_rviz': 'false',
            'headless': headless,
            'use_phone_teleop': use_phone_teleop,
            'phone_teleop_port': phone_teleop_port,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params,
        }.items(),
    )

    teleop = Node(
        package='tai_robot_one',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_teleop),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='slam_rviz',
        output='screen',
        arguments=['-d', slam_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    imu_visualizer = Node(
        package='tai_robot_one',
        executable='imu_visualizer',
        name='imu_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open the map-focused SLAM RViz configuration'),
        DeclareLaunchArgument(
            'use_teleop', default_value='false',
            description=(
                'Open the optional keyboard teleop window; phone teleop is '
                'already started by gazebo_sim.launch.py')),
        DeclareLaunchArgument(
            'use_phone_teleop', default_value='true',
            description='Serve the phone teleop UI while mapping'),
        DeclareLaunchArgument(
            'phone_teleop_port', default_value='8080',
            description='TCP port used by the phone teleop web UI'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without its 3D client'),
        DeclareLaunchArgument(
            'slam_start_delay', default_value='5.0',
            description='Wall-clock seconds before starting slam_toolbox'),
        DeclareLaunchArgument(
            'teleop_start_delay', default_value='6.0',
            description='Wall-clock seconds before optional keyboard teleop'),
        gazebo,
        TimerAction(
            period=slam_start_delay,
            actions=[slam, imu_visualizer, rviz]),
        TimerAction(period=teleop_start_delay, actions=[teleop]),
    ])
