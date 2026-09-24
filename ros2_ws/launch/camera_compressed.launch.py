"""Republish Astra Pro raw images using network-friendly transports."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    color = Node(
        package='image_transport',
        executable='republish',
        name='astra_color_compressor',
        output='screen',
        parameters=[{
            'in_transport': 'raw',
            'out_transport': 'compressed',
            'out.compressed.jpeg_quality': 85,
        }],
        remappings=[
            ('in', '/camera/color/image_raw'),
            ('out/compressed', '/camera/color/image_raw/compressed'),
        ],
    )

    depth = Node(
        package='image_transport',
        executable='republish',
        name='astra_depth_compressor',
        output='screen',
        parameters=[{
            'in_transport': 'raw',
            'out_transport': 'compressedDepth',
            # Fast PNG compression keeps CPU use low on the Pi 5.
            'out.compressedDepth.png_level': 1,
        }],
        remappings=[
            ('in', '/camera/depth/image_raw_10fps'),
            ('out/compressedDepth',
             '/camera/depth/image_raw/compressedDepth_10fps'),
        ],
    )

    return LaunchDescription([color, depth])
