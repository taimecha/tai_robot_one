import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start_after_success(action, process_name):
    """Prevent later controllers from starting after an earlier failure."""
    def _on_exit(event, _context):
        if event.returncode == 0:
            return [action]
        return [EmitEvent(event=Shutdown(
            reason=f'{process_name} failed with exit code {event.returncode}'))]

    return _on_exit


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    xacro_file = os.path.join(
        package_share, 'description', 'robot.urdf.xacro')
    controllers_file = os.path.join(
        package_share, 'config', 'ros2_controllers_real.yaml')
    ekf_file = os.path.join(package_share, 'config', 'ekf.yaml')
    rviz_config = os.path.join(
        package_share, 'config', 'gazebo_robot.rviz')

    drive_port = LaunchConfiguration('drive_serial_port')
    lift_port = LaunchConfiguration('lift_serial_port')
    home_lift = LaunchConfiguration('home_lift_on_activate')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' use_sim:=false',
            ' hardware_plugin:=tai_robot_one/TaiRobotSerialSystem',
            ' controllers_file:=', controllers_file,
            ' drive_serial_port:=', drive_port,
            ' lift_serial_port:=', lift_port,
            ' home_lift_on_activate:=', home_lift,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False},
            controllers_file,
        ],
    )

    cmd_vel_stamper = Node(
        package='tai_robot_one',
        executable='cmd_vel_stamper',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file, {'use_sim_time': False}],
        remappings=[('odometry/filtered', '/odom')],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '90',
            '--switch-timeout', '90',
            '--service-call-timeout', '90',
        ],
    )
    base_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='base_controller_spawner',
        output='screen',
        arguments=[
            'base_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '90',
            '--switch-timeout', '90',
            '--service-call-timeout', '90',
            '--param-file', controllers_file,
        ],
    )
    lift_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='lift_controller_spawner',
        output='screen',
        arguments=[
            'lift_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '90',
            '--switch-timeout', '90',
            '--service-call-timeout', '90',
            '--param-file', controllers_file,
        ],
    )

    start_joint_state_broadcaster = TimerAction(
        period=2.0, actions=[joint_state_broadcaster])
    start_base_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=_start_after_success(
                base_controller, 'joint_state_broadcaster spawner'),
        )
    )
    start_lift_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=base_controller,
            on_exit=_start_after_success(
                lift_controller, 'base_controller spawner'),
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_serial_port',
            default_value='/dev/tai_drive',
            description='Stable udev symlink for the base ESP32'),
        DeclareLaunchArgument(
            'lift_serial_port',
            default_value='/dev/tai_lift',
            description='Stable udev symlink for the lift ESP32'),
        DeclareLaunchArgument(
            'home_lift_on_activate',
            default_value='true',
            description='Home the lift automatically before arming it'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Open RViz on the Raspberry Pi'),
        robot_state_publisher,
        controller_manager,
        cmd_vel_stamper,
        ekf,
        start_joint_state_broadcaster,
        start_base_controller,
        start_lift_controller,
        rviz,
    ])
