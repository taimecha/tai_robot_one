import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def _start_after_success(action, process_name):
    """Start the next stage only when the previous process exited cleanly."""
    def _on_exit(event, _context):
        if event.returncode == 0:
            return [action]
        return [EmitEvent(event=Shutdown(
            reason=f'{process_name} failed with exit code {event.returncode}'))]

    return _on_exit


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_phone_teleop = LaunchConfiguration('use_phone_teleop')
    phone_teleop_port = LaunchConfiguration('phone_teleop_port')
    headless = LaunchConfiguration('headless')
    gz_args = LaunchConfiguration('gz_args')
    world_file = os.path.join(
        package_share, 'worlds', 'warehouse_empty.sdf')
    model_path = os.path.join(package_share, 'worlds', 'models')
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gazebo_resource_path = os.pathsep.join(
        path for path in (model_path, existing_resource_path) if path)
    xacro_file = os.path.join(
        package_share, 'description', 'robot.urdf.xacro')
    rviz_config = os.path.join(
        package_share, 'config', 'gazebo_robot.rviz')
    controllers_file = os.path.join(
        package_share, 'config', 'ros2_controllers.yaml')
    ekf_file = os.path.join(package_share, 'config', 'ekf.yaml')

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'use_sim': 'true',
            'hardware_plugin': 'gz_ros2_control/GazeboSimSystem',
            'controllers_file': controllers_file,
        },
    ).toxml()
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
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/fork_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/rear_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        # Keep the Gazebo render products private.  The filter below publishes
        # the hardware-compatible public depth/cloud topics after applying the
        # Astra Pro 0.6-8.0 m measurement range.  RGB stays direct because a
        # close object must remain visible even when depth cannot measure it.
        remappings=[
            ('/camera/depth_image', '/camera/sim_raw/depth_image'),
            ('/camera/points', '/camera/sim_raw/points'),
        ],
        output='screen',
    )

    astra_sim_range_filter = Node(
        package='tai_robot_one',
        executable='astra_sim_range_filter',
        name='astra_sim_range_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_depth': 0.60,
            'max_depth': 8.00,
        }],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_parameters],
    )

    # Nav2 and the existing teleop publish geometry_msgs/Twist. Jazzy's
    # diff_drive_controller requires TwistStamped on its private input.
    cmd_vel_stamper = Node(
        package='tai_robot_one',
        executable='cmd_vel_stamper',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    phone_teleop = Node(
        package='tai_robot_one',
        executable='phone_teleop_server',
        name='phone_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': ParameterValue(phone_teleop_port, value_type=int),
        }],
        condition=IfCondition(use_phone_teleop),
    )

    # Keep the odometry contract identical to the real launch: the controller
    # publishes raw wheel odometry privately and robot_localization owns the
    # public /odom topic plus odom -> base_footprint TF. Gazebo supplies the
    # IMU directly; real hardware may initially run this EKF wheel-only.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu/data', '/imu'),
            ('odometry/filtered', '/odom'),
        ],
    )

    imu_visualizer = Node(
        package='tai_robot_one',
        executable='imu_visualizer',
        name='imu_visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_tai_robot_one',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        arguments=[
            '--world', 'warehouse',
            # Read the URDF from this process's own parameter. This avoids a
            # DDS discovery race where `create` can wait forever for the
            # transient `/robot_description` topic, especially under WSL.
            '--param', 'robot_description',
            '--name', 'tai_robot_one',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'), '-z', '0.01',
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
    )

    # Give the Gazebo server time to advertise the world creation service.
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    # gz_ros2_control creates controller_manager inside the spawned model.
    # Load controllers in order so no spawner races the manager or another
    # spawner for hardware resources during startup.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='base_controller_spawner',
        output='screen',
        arguments=[
            'base_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', controllers_file,
        ],
    )

    lift_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='lift_effort_controller_spawner',
        output='screen',
        arguments=[
            'lift_effort_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', controllers_file,
        ],
    )

    # Preserve the same target-position API used by keyboard teleop and the
    # real ESP32 backend. Only the simulated actuator beneath it uses effort.
    lift_position_controller = Node(
        package='tai_robot_one',
        executable='lift_position_controller',
        name='lift_position_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=_start_after_success(
                joint_state_broadcaster_spawner, 'Gazebo robot spawn'),
        )
    )
    start_base_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=_start_after_success(
                base_controller_spawner, 'joint_state_broadcaster spawner'),
        )
    )
    start_lift_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=base_controller_spawner,
            on_exit=_start_after_success(
                lift_effort_controller_spawner, 'base_controller spawner'),
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Wait until ros2_control supplies joint states and raw wheel odometry; the
    # EKF then publishes odom -> base_footprint for visualization consumers.
    delayed_visualization = RegisterEventHandler(
        OnProcessExit(
            target_action=lift_effort_controller_spawner,
            on_exit=_start_after_success(
                TimerAction(
                    period=1.0,
                    actions=[lift_position_controller, imu_visualizer, rviz],
                ),
                'lift_effort_controller spawner'),
        )
    )

    # The forward effort controller retains its last value if its publisher
    # disappears. Stop the simulation if the position loop exits so stale
    # lift force cannot remain applied.
    stop_if_lift_position_controller_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=lift_position_controller,
            on_exit=[EmitEvent(event=Shutdown(
                reason='lift_position_controller exited'))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        # WSLg exposes Windows GPUs through Mesa's D3D12 Gallium driver.
        # The RGB-D sensor uses Ogre1, which is stable on this path.  LiDAR is
        # now physics-based and no longer shares the rendering backend.
        SetEnvironmentVariable(
            name='MESA_D3D12_DEFAULT_ADAPTER_NAME',
            value='NVIDIA'),
        SetEnvironmentVariable(
            name='GALLIUM_DRIVER',
            value='d3d12'),
        SetEnvironmentVariable(
            name='LIBGL_ALWAYS_SOFTWARE',
            value='0'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gazebo_resource_path),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo /clock topic for all ROS nodes'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz together with Gazebo'),
        DeclareLaunchArgument(
            'use_phone_teleop',
            default_value='true',
            description='Serve the phone teleop UI on the WSL laptop'),
        DeclareLaunchArgument(
            'phone_teleop_port',
            default_value='8080',
            description='TCP port used by the phone teleop web UI'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run the Gazebo server without its 3D GUI'),
        DeclareLaunchArgument(
            'gz_args',
            default_value=[
                PythonExpression([
                    "'-r -s -v 2 ' if '", headless,
                    "'.lower() == 'true' else '-r -v 2 '",
                ]),
                world_file,
            ],
            description='Arguments passed to gz sim'),
        gazebo,
        gazebo_bridge,
        astra_sim_range_filter,
        robot_state_publisher,
        cmd_vel_stamper,
        phone_teleop,
        ekf,
        delayed_spawn,
        start_joint_state_broadcaster,
        start_base_controller,
        start_lift_controller,
        delayed_visualization,
        stop_if_lift_position_controller_exits,
    ])
