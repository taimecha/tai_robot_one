from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tai_robot_one',
            executable='scan_self_filter',
            name='scan_self_filter',
            output='screen',
            parameters=[{
                'input_topic': '/scan',
                'output_topic': '/scan_filtered',
                'rviz_output_topic': '/scan_filtered_rviz',
                'min_x': 0.42,
                'max_x': 0.51,
                'min_y': -0.155,
                'max_y': 0.155,
            }],
        ),
    ])
