"""Launch the optional standalone ESP32 BNO055 serial bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create a parameterized standalone IMU bridge."""
    package_share = get_package_share_directory('esp32_imu_bridge')
    default_config = os.path.join(
        package_share, 'config', 'imu_bridge.yaml')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/tai_imu',
            description='Stable udev symlink for the standalone IMU ESP32',
        ),
        Node(
            package='esp32_imu_bridge',
            executable='imu_node',
            name='esp32_imu_bridge',
            output='screen',
            parameters=[default_config, {'port': port}],
        ),
    ])
