import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    nav2_share = get_package_share_directory('nav2_bringup')
    slam_share = get_package_share_directory('slam_toolbox')

    use_rviz = LaunchConfiguration('use_rviz')
    use_phone_teleop = LaunchConfiguration('use_phone_teleop')
    phone_teleop_port = LaunchConfiguration('phone_teleop_port')
    headless = LaunchConfiguration('headless')
    slam_start_delay = LaunchConfiguration('slam_start_delay')
    navigation_start_delay = LaunchConfiguration('navigation_start_delay')
    slam_params = os.path.join(package_share, 'config', 'slam_toolbox.yaml')
    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    nav_to_pose_bt = os.path.join(
        package_share, 'behavior_trees', 'navigate_real_clearance_escape.xml')
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav_to_pose_bt,
            'collision_monitor.ros__parameters.VelocityStop.enabled': 'false',
        },
        convert_types=True,
    )
    nav2_rviz = os.path.join(package_share, 'config', 'nav_robot.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            # Use Nav2's RViz layout in this combined launch.
            'use_rviz': 'false',
            'cmd_vel_input_topic': '/cmd_vel_safe',
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

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'false',
            'params_file': configured_nav2_params,
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

    # Reduce the 320x240 RGB-D cloud once, then let both costmaps and the
    # collision monitor consume the lightweight 60-ray virtual scan. With the
    # level 0.8 m camera this helps for rack posts and loaded pallets, but an
    # empty ~35 mm pallet can remain in the near-field blind zone.
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
            'min_height': 0.12,
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

    nav_path_display_filter = Node(
        package='tai_robot_one', executable='nav_path_display_filter',
        name='nav_path_display_filter', output='screen',
        parameters=[{'use_sim_time': True, 'global_path_topic': '/plan_selected'}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_rviz', default_value='false'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open the Nav2 RViz configuration'),
        DeclareLaunchArgument(
            'use_phone_teleop', default_value='true',
            description='Serve the phone teleop UI during SLAM and Nav2'),
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
            'navigation_start_delay', default_value='20.0',
            description='Wall-clock seconds before starting Nav2'),
        GroupAction(actions=[gazebo]),
        camera_obstacle_scan,
        nav_path_display_filter,
        TimerAction(period=slam_start_delay, actions=[slam]),
        TimerAction(
            period=navigation_start_delay,
            actions=[GroupAction([
                SetRemap(src='/scan', dst='/scan_filtered'),
                navigation,
            ])]),
        TimerAction(
            period=PythonExpression([navigation_start_delay, ' + 12.0']),
            actions=[rviz]),
    ])
