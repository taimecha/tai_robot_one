# ESP32 IMU bridge

Package gốc được đối chiếu với
`taimecha/agv_robot_pi5@4b2d9d38b8c378fe1addbd5eccaaad2b1889dd05`.
Phiên bản tích hợp bỏ hard-code `/dev/ttyUSB0`, tự reconnect, kiểm tra
quaternion/NaN/sequence và không còn bỏ qua exception im lặng.

```bash
ros2 launch tai_robot_one imu_bridge.launch.py port:=/dev/tai_imu
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

Node hiểu protocol `IMU,1,...` của project `bno055_imu_monitor` và vẫn đọc
được format cũ gồm bảy số. Format cũ không có gyro nên bridge đánh dấu
`angular_velocity_covariance[0] = -1`, tránh báo sai rằng tốc độ góc bằng zero
có độ tin cậy tuyệt đối.

Không chạy node này trên `/dev/tai_drive` hoặc `/dev/tai_lift`: backend
`ros2_control` giữ độc quyền hai cổng đó. Trong kiến trúc robot hai ESP32 hiện
tại, package này là công cụ bench/legacy và chưa được bật trong
`real_hardware.launch.py`.

Do repository hiện tại đã là package `tai_robot_one`, `colcon` không khám phá
package Python nằm lồng bên dưới. `CMakeLists.txt` của `tai_robot_one` vì vậy
cài node này với executable `esp32_imu_bridge`; thư mục package upstream vẫn
được giữ để theo dõi nguồn gốc và chạy unit test.
