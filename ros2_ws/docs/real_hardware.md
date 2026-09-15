# Raspberry Pi 5: drive + IMU qua hai ESP32

Phần cứng hiện tại dùng **hai cổng serial độc lập**:

- `/dev/tai_drive`, 460800 baud: bốn bánh theo đúng thứ tự `FL, FR, RL, RR`;
- `/dev/tai_imu`, 115200 baud: BNO055 đã đổi trục về REP-103, tự zero sau
  khoảng hai giây đứng yên, đồng thời điều khiển càng TB6600.
- `/dev/tai_lidar`: SLAMTEC/RPLIDAR trên CP2102. Vì lidar và ESP32 drive có
  cùng USB serial `0001`, hai tên này được cố định theo cổng USB vật lý.

Backend `tai_robot_one/TaiRobotSerialSystem` điều khiển ESP32 drive. IMU bridge
giữ độc quyền `/dev/tai_imu`, đọc IMU và gửi lệnh càng trên cùng một đường
Serial. Vì vậy launch thật vẫn đặt `use_lift:=false`: tùy chọn này chỉ tắt lift
backend cũ dùng cổng `/dev/tai_lift`, không tắt càng trong IMU bridge.

Các interface ROS giữ nguyên giữa Gazebo và xe thật: bốn bánh nhận vận tốc
rad/s, `lift_joint` nhận vị trí mét. Giới hạn cuối cùng ở cả ROS và firmware là
`5.7142857 rad/s`, tương ứng `0.5 m/s` với bán kính bánh `0.0875 m`; hành trình
càng là `0.000..0.520 m`.

## Tên cổng USB ổn định

Không dùng trực tiếp `/dev/ttyUSB0` và `/dev/ttyUSB1`: thứ tự này có thể đổi sau
mỗi lần khởi động hoặc khi cắm lidar. Lấy serial của từng thiết bị:

```bash
udevadm info --query=property --name=/dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'
udevadm info --query=property --name=/dev/ttyUSB1 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'
```

Sửa các placeholder trong
`udev/99-tai-robot-usb.rules.example`, sau đó người quản trị có thể sao chép nó
vào `/etc/udev/rules.d/99-tai-robot-usb.rules` và nạp lại rules. File mẫu không
được tự cài vào hệ thống và cố ý không khớp thiết bị nào khi còn placeholder.

Nếu hai USB-UART không có serial riêng, dùng `ID_PATH` (vị trí cổng USB vật lý)
trong rules, hoặc thay USB-UART bằng loại có serial duy nhất. Kiểm tra trước khi
chạy:

```bash
readlink -f /dev/tai_lidar
readlink -f /dev/tai_drive
readlink -f /dev/tai_imu
test "$(readlink -f /dev/tai_lidar)" != "$(readlink -f /dev/tai_drive)"
```

Drive backend và IMU bridge yêu cầu quyền truy cập độc quyền trên tty. Trước khi
launch, dừng PlatformIO serial monitor hoặc node cũ đang mở cùng cổng; kiểm tra:

```bash
fuser /dev/tai_drive /dev/tai_imu
```

## Trình tự an toàn do backend thực hiện

Khi configure, backend mở cổng drive và đưa base về `DISABLE`. Khi activate:

1. Base nhận một `CMD` bốn số 0; chỉ lỗi `COMMAND_TIMEOUT` được tự xóa. Lỗi
   software E-stop không được tự xóa.
2. Base được `ENABLE`; IMU bridge đồng thời đọc dữ liệu BNO055 ở 50 Hz.
3. Chu kỳ điều khiển gửi frame mới liên tục. Không có lệnh cũ nào được lưu để
   phát lại sau khi rớt cáp.

Trong trạng thái active, CRC/frame sai, telemetry quá 250 ms, ESP32 reset,
serial disconnect, NACK/ERR hoặc fault đều làm hardware trả `ERROR` và gửi
zero/`DISABLE` cho base. Watchdog 300 ms trong ESP32 drive vẫn là lớp bảo vệ
cuối khi Raspberry Pi treo hẳn.

## Build và chạy thật (chỉ sau khi mô phỏng đạt yêu cầu)

Các gói phát triển cần có ít nhất `ros-jazzy-ros2-control`,
`ros-jazzy-ros2-controllers` và `ros-jazzy-robot-localization`. Build
workspace, nâng cả bốn bánh khỏi sàn, tháo tải khỏi càng và giữ tay ở E-stop vật
lý trước lần chạy đầu tiên.

```bash
cd ~/tai_robot_one/tai_robot_one_backup/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tai_robot_one real_hardware.launch.py \
  use_lift:=false use_rviz:=false \
  drive_serial_port:=/dev/tai_drive imu_serial_port:=/dev/tai_imu
```

Với `use_lift:=false`, backend không tìm `/dev/tai_lift`; càng thật vẫn nhận
`/lift/home` và `/lift_controller/commands` qua bridge `/dev/tai_imu`.

Kiểm tra controller và feedback:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /base_controller/odom
ros2 topic echo /odom
```

`/base_controller/odom` là odometry bánh xe thô. EKF kết hợp vận tốc encoder,
yaw và yaw-rate BNO055, phát `/odom` và là node duy nhất phát TF
`odom -> base_footprint`. RViz đặt Fixed Frame là `odom` sẽ bám theo kết quả EKF.

`cmd_vel_stamper` đổi `/cmd_vel` kiểu `Twist` từ teleop/Nav2 thành
`/base_controller/cmd_vel` kiểu `TwistStamped`. Node này chỉ đóng dấu khi nhận
lệnh mới và **không lặp lại lệnh**, vì lặp lại sẽ vô hiệu hóa ý nghĩa watchdog.

CM029 cắm ở laptop và truyền `/joy` qua Wi-Fi. Chỉ sau khi kê bốn bánh và kiểm
tra E-stop vật lý mới bật teleop trên Pi với giới hạn thấp:

```bash
ros2 launch tai_robot_one real_hardware.launch.py \
  use_lift:=false use_rviz:=false use_cm029_teleop:=true \
  cm029_linear_speed:=0.10 cm029_angular_speed:=0.25
```

Trên laptop Ubuntu cùng mạng ROS, source ROS/workspace rồi chạy RViz và teleop:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
rviz2 -d ~/tai_robot_one_backup/ros2_ws/config/gazebo_robot.rviz

# Terminal laptop khác; bắt đầu bằng tốc độ thấp.
python3 ~/tai_robot_one_backup/ros2_ws/scripts/keyboard_teleop \
  --ros-args -p linear_speed:=0.10 -p angular_speed:=0.35
```

Software watchdog không thay thế E-stop/contactor cắt nguồn driver, mạch giới
hạn an toàn độc lập, kiểm tra tải cơ khí và quy trình khóa nguồn khi bảo trì.
