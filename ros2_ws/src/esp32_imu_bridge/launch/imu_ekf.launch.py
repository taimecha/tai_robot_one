"""Run the ESP32 IMU bridge and planar EKF for a standalone hardware test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Connect the BNO055, publish its mounting TF, and start the EKF."""
    package_share = get_package_share_directory('esp32_imu_bridge')
    bridge_config = os.path.join(package_share, 'config', 'imu_bridge.yaml')
    ekf_config = os.path.join(package_share, 'config', 'imu_ekf.yaml')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/tai_imu',
            description='Stable USB serial path for the BNO055 ESP32',
        ),
        Node(
            package='esp32_imu_bridge',
            executable='imu_node',
            name='esp32_imu_bridge',
            output='screen',
            parameters=[bridge_config, {'port': port}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_mount_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.55',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'imu_link',
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odom')],
        ),
    ])
