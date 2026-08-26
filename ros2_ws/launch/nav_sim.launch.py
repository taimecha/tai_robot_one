import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_map(context):
    """Reject ambiguous map paths before starting the simulator."""
    map_file = LaunchConfiguration('map').perform(context)
    if not os.path.isabs(map_file):
        raise RuntimeError(
            f'The map argument must be an absolute path, got: {map_file}')
    if not os.path.isfile(map_file):
        raise RuntimeError(f'Map YAML file does not exist: {map_file}')
    if not map_file.lower().endswith(('.yaml', '.yml')):
        raise RuntimeError(f'The map argument must name a YAML file: {map_file}')
    return []


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_phone_teleop = LaunchConfiguration('use_phone_teleop')
    phone_teleop_port = LaunchConfiguration('phone_teleop_port')
    headless = LaunchConfiguration('headless')
    startup_delay = LaunchConfiguration('startup_delay')
    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    nav2_rviz = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            'use_rviz': 'false',
            'headless': headless,
            'use_phone_teleop': use_phone_teleop,
            'phone_teleop_port': phone_teleop_port,
        }.items(),
    )

    # Bringup starts map_server + AMCL as well as the complete navigation
    # stack. SLAM stays disabled because this launch consumes a saved map.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'slam': 'false',
            'use_localization': 'true',
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'false',
            'params_file': nav2_params,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='nav2_rviz',
        output='screen',
        arguments=['-d', nav2_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    # Convert the cloud once for all Nav2 consumers. The level 0.8 m camera
    # covers rack posts and loaded pallets, not every near-field empty pallet.
    camera_obstacle_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='camera_obstacle_scan',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/points'),
            ('scan', '/camera/obstacle_scan'),
        ],
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            'min_height': 0.05,
            'max_height': 0.75,
            'angle_min': -0.5235987756,
            'angle_max': 0.5235987756,
            'angle_increment': 0.01745329252,
            'queue_size': 1,
            'scan_time': 0.10,
            'range_min': 0.60,
            'range_max': 6.00,
            'use_inf': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Absolute path to a map YAML saved by slam_toolbox'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open the Nav2 RViz configuration'),
        DeclareLaunchArgument(
            'use_phone_teleop', default_value='true',
            description='Serve the phone teleop UI during localization/Nav2'),
        DeclareLaunchArgument(
            'phone_teleop_port', default_value='8080',
            description='TCP port used by the phone teleop web UI'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without its 3D client'),
        DeclareLaunchArgument(
            'startup_delay', default_value='7.0',
            description='Wall-clock seconds before starting AMCL and Nav2'),
        OpaqueFunction(function=_validate_map),
        gazebo,
        camera_obstacle_scan,
        TimerAction(period=startup_delay, actions=[navigation, rviz]),
    ])
