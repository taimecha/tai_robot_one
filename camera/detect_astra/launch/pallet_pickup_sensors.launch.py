"""Sensors and optional map/AMCL for pallet pickup; no motion owner and no HOME."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot=Path(get_package_share_directory('tai_robot_one'))
    lidar=Path(get_package_share_directory('sllidar_ros2'))
    def include(path,args=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=(args or {}).items())
    localization=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(__file__).resolve().parent/'pallet_pickup_localization.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_localization')))
    return LaunchDescription([
        DeclareLaunchArgument('start_localization',default_value='true'),
        include(robot/'launch/real_hardware.launch.py',dict(
            cmd_vel_input_topic='/cmd_vel_safe',use_cm029_teleop='false',
            use_lift='false',home_lift_on_activate='false',use_rviz='false',use_imu_visualizer='false')),
        include(lidar/'launch/sllidar_a1_launch.py',dict(serial_port='/dev/tai_lidar',frame_id='lidar_link')),
        include(robot/'launch/scan_self_filter.launch.py',dict(max_scan_age_s='0.40')),
        include(robot/'launch/astra_obstacles.launch.py'),
        localization,
    ])
