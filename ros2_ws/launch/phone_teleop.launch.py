from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enabled = LaunchConfiguration('enabled')
    port = LaunchConfiguration('port')

    phone_teleop = Node(
        package='tai_robot_one',
        executable='phone_teleop_server',
        name='phone_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': ParameterValue(port, value_type=int),
        }],
        condition=IfCondition(enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo clock for ROS messages'),
        DeclareLaunchArgument(
            'enabled', default_value='true',
            description='Start the phone web teleop server'),
        DeclareLaunchArgument(
            'port', default_value='8080',
            description='TCP port exposed by the phone web interface'),
        phone_teleop,
    ])
