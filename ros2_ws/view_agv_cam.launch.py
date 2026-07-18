import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Đường dẫn tới file launch XML gốc của hãng Orbbec
    camera_launch_dir = os.path.join(
        get_package_share_directory('astra_camera'), 'launch'
    )
    camera_driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(camera_launch_dir, 'astra_pro.launch.xml'))
    )

    # 2. Định vị và hiệu chuẩn toàn diện ma trận tọa độ Camera Astra
    # Quy ước thứ tự điền số: ['X', 'Y', 'Z', 'Yaw', 'Pitch', 'Roll', 'Parent', 'Child']
    # ----------------------------------------------------------------------------------
    # - '0', '0': Tọa độ tịnh tiến X, Y nằm ngay tâm robot
    # - '0.8': Chiều cao thấu kính cách mặt đất là 80cm (Giúp nâng mây điểm lên khỏi mặt lưới Grid)
    # - '0.12': Góc Yaw - Giữ nguyên thông số nắn tường thẳng song song với Grid của bạn
    # - '0.26': Góc Pitch (Mới) - Khai báo góc camera NGẨNG LÊN 15 độ để sửa lỗi cửa sổ bị xéo đứng và âm nền
    # - '0.08': Góc Roll - Giữ nguyên thông số nắn sườn nền nhà nằm bẹp lên Grid của bạn
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=['0', '0', '0.8', '0.1', '-0.05', '0.06', 'base_link', 'camera_link']
    )

    return LaunchDescription([
        camera_driver,
        tf_base_to_camera
    ])
