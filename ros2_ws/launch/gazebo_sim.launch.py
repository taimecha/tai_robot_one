import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    gz_args = LaunchConfiguration('gz_args')
    world_file = os.path.join(
        package_share, 'worlds', 'warehouse_empty.sdf')
    xacro_file = os.path.join(
        package_share, 'description', 'robot.urdf.xacro')
    rviz_config = os.path.join(
        package_share, 'config', 'gazebo_robot.rviz')

    robot_description = xacro.process_file(xacro_file).toxml()
    robot_parameters = {
        'robot_description': robot_description,
        'use_sim_time': use_sim_time,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Gazebo is the authoritative clock source. The '[' creates a one-way
    # Gazebo-to-ROS bridge and prevents ROS from writing back to /clock.
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        arguments=[
            # Gazebo -> ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # ROS -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/lift_position@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_parameters],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_tai_robot_one',
        output='screen',
        arguments=[
            '--world', 'warehouse',
            '--topic', 'robot_description',
            '--name', 'tai_robot_one',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
        ],
    )

    # Give the Gazebo server time to advertise the world creation service.
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo /clock topic for all ROS nodes'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz together with Gazebo'),
        DeclareLaunchArgument(
            'gz_args',
            default_value=['-r -v 4 ', world_file],
            description='Arguments passed to gz sim'),
        gazebo,
        gazebo_bridge,
        robot_state_publisher,
        delayed_spawn,
        rviz,
    ])
