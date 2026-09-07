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
from nav2_common.launch import RewrittenYaml


def _validate_map(context):
    """Reject ambiguous map paths before starting the simulator."""
    map_file = LaunchConfiguration('map_file').perform(context)
    if not os.path.isabs(map_file):
        raise RuntimeError(
            f'The map argument must be an absolute path, got: {map_file}')
    if not os.path.isfile(map_file):
        raise RuntimeError(f'Map YAML file does not exist: {map_file}')
    if not map_file.lower().endswith(('.yaml', '.yml')):
        raise RuntimeError(f'The map argument must name a YAML file: {map_file}')
    return []


def _launch_nav2(context, nav2_share, nav2_params, nav_to_pose_bt):
    """Start localization and navigation with a concrete map path."""
    map_file = LaunchConfiguration('map_file').perform(context)
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav_to_pose_bt,
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('spawn_x'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('spawn_y'),
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('spawn_yaw'),
        },
        convert_types=True,
    )
    common_arguments = {
        'use_sim_time': 'true',
        'autostart': 'true',
        'use_composition': 'False',
        'params_file': configured_params,
    }
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={
            **common_arguments,
            'map': map_file,
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments=common_arguments.items(),
    )
    return [localization, navigation]


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    nav2_share = get_package_share_directory('nav2_bringup')

    # Keep the parent argument name distinct from Nav2's nested `map`
    # argument. Forwarding `map` to another `map` through a delayed include
    # can resolve against the child default and become an empty string.
    map_file = LaunchConfiguration('map_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_phone_teleop = LaunchConfiguration('use_phone_teleop')
    phone_teleop_port = LaunchConfiguration('phone_teleop_port')
    headless = LaunchConfiguration('headless')
    startup_delay = LaunchConfiguration('startup_delay')
    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    nav_to_pose_bt = os.path.join(
        package_share, 'behavior_trees', 'navigate_to_pose_no_reverse.xml')
    nav2_rviz = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            'use_rviz': 'false',
            'headless': headless,
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'use_phone_teleop': use_phone_teleop,
            'phone_teleop_port': phone_teleop_port,
        }.items(),
    )

    # Bringup starts map_server + AMCL as well as the complete navigation
    # stack. SLAM stays disabled because this launch consumes a saved map.
    navigation = OpaqueFunction(
        function=_launch_nav2,
        args=[nav2_share, nav2_params, nav_to_pose_bt],
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

    # Relay Nav2 paths to visualization-only topics and publish empty paths
    # when NavigateToPose finishes. Nav2's control topics remain untouched.
    nav_path_display_filter = Node(
        package='tai_robot_one',
        executable='nav_path_display_filter',
        name='nav_path_display_filter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map_file',
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
        nav_path_display_filter,
        TimerAction(period=startup_delay, actions=[navigation, rviz]),
    ])
