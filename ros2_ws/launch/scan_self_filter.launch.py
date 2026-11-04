from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('max_x', default_value='0.51'),
        DeclareLaunchArgument('min_y', default_value='-0.155'),
        DeclareLaunchArgument('max_y', default_value='0.155'),
        DeclareLaunchArgument('speckle_max_difference', default_value='0.0'),
        DeclareLaunchArgument('max_scan_age_s', default_value='0.30'),
        Node(
            package='tai_robot_one',
            executable='scan_self_filter',
            name='scan_self_filter',
            output='screen',
            # This sensor relay owns no actuator. Restore its data source if
            # it exits unexpectedly; control guards still stop on stale scans.
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/scan',
                'output_topic': '/scan_filtered',
                'rviz_output_topic': '/scan_filtered_rviz',
                'min_x': 0.42,
                'max_x': ParameterValue(
                    LaunchConfiguration('max_x'), value_type=float),
                'min_y': ParameterValue(
                    LaunchConfiguration('min_y'), value_type=float),
                'max_y': ParameterValue(
                    LaunchConfiguration('max_y'), value_type=float),
                'speckle_max_difference': ParameterValue(
                    LaunchConfiguration('speckle_max_difference'), value_type=float),
                'max_scan_age_s': ParameterValue(
                    LaunchConfiguration('max_scan_age_s'), value_type=float),
            }],
        ),
    ])
