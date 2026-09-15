from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    teleop = Node(
        package='tai_robot_one',
        executable='cm029_teleop',
        name='cm029_teleop',
        output='screen',
        parameters=[{
            'joy_topic': LaunchConfiguration('joy_topic'),
            'linear_speed': ParameterValue(
                LaunchConfiguration('linear_speed'), value_type=float),
            'angular_speed': ParameterValue(
                LaunchConfiguration('angular_speed'), value_type=float),
            'deadzone': 0.10,
            'joy_timeout': 0.30,
            'lift_speed': 0.05,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_topic', default_value='/joy',
            description='Joy topic received from the Ubuntu laptop over DDS'),
        DeclareLaunchArgument(
            'linear_speed', default_value='0.50',
            description='Maximum commanded linear speed in m/s'),
        DeclareLaunchArgument(
            'angular_speed', default_value='1.00',
            description='Maximum commanded angular speed in rad/s'),
        teleop,
    ])
