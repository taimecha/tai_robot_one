# Raspberry Pi 5: ROS 2 control qua hai ESP32

Backend `tai_robot_one/TaiRobotSerialSystem` là một `SystemInterface` duy nhất
nhưng mở **hai cổng serial độc lập**:

- `/dev/tai_drive`, 460800 baud: bốn bánh theo đúng thứ tự `FL, FR, RL, RR`;
- `/dev/tai_lift`, 115200 baud: càng nâng với session/sequence riêng.

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
readlink -f /dev/tai_drive
readlink -f /dev/tai_lift
test "$(readlink -f /dev/tai_drive)" != "$(readlink -f /dev/tai_lift)"
```

Backend yêu cầu quyền truy cập độc quyền `TIOCEXCL` trên cả hai tty. Trước khi
launch, dừng mọi chương trình cũ như `esp32_imu_bridge`, PlatformIO serial
monitor hoặc terminal đang mở cùng cổng; có thể kiểm tra bằng:

```bash
fuser /dev/tai_drive /dev/tai_lift
```

## Trình tự an toàn do backend thực hiện

Khi configure, backend mở cả hai cổng, đưa base về `DISABLE` và tạo một session
lift mới. Khi activate:

1. Base nhận một `CMD` bốn số 0; chỉ lỗi `COMMAND_TIMEOUT` được tự xóa. Lỗi
   software E-stop không được tự xóa.
2. Lift chỉ tự xóa `COMM_TIMEOUT`. Mọi lỗi limit/homing khác buộc người vận hành
   kiểm tra cơ khí và dây điện.
3. Nếu lift chưa home, nó home về công tắc dưới. Nếu một endstop đang mở, càng
   chỉ đi ra khỏi endstop tối đa 25 mm/5 giây để xác minh cạnh switch rồi mới
   tiếp tục; switch không nhả hoặc switch trên xuất hiện khi đang tìm đáy sẽ
   fault. Mọi lỗi hình học/endstop hủy trạng thái `homed` và buộc HOME lại.
4. Base mới được `ENABLE`, sau đó lift mới được `ARM`.
5. Chu kỳ điều khiển gửi frame mới liên tục. Không có lệnh cũ nào được lưu để
   phát lại sau khi rớt cáp.

Trong trạng thái active, CRC/frame sai, telemetry quá 250 ms, session cũ, ESP32
reset, serial disconnect, NACK/ERR hoặc fault đều làm hardware trả `ERROR`, gửi
zero/`DISABLE` cho base và `STOP` cho lift. Watchdog 300 ms trong từng ESP32 vẫn
là lớp bảo vệ cuối khi Raspberry Pi treo hẳn.

## Build và chạy thật (chỉ sau khi mô phỏng đạt yêu cầu)

Các gói phát triển cần có ít nhất `ros-jazzy-ros2-control`,
`ros-jazzy-ros2-controllers` và `ros-jazzy-robot-localization`. Build
workspace, nâng cả bốn bánh khỏi sàn, tháo tải khỏi càng và giữ tay ở E-stop vật
lý trước lần chạy đầu tiên.

```bash
ros2 launch tai_robot_one real_hardware.launch.py
```

Lệnh này có thể làm càng chuyển động do bước homing. Muốn chẩn đoán mà không cho
home tự động, dùng `home_lift_on_activate:=false`; nếu lift chưa home thì
activation sẽ thất bại an toàn, không ARM.

Kiểm tra controller và feedback:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /base_controller/odom
ros2 topic echo /odom
```

`/base_controller/odom` là odometry bánh xe thô. EKF trong
`robot_localization` phát `/odom` và là node duy nhất phát TF
`odom -> base_footprint`, tránh hai nguồn TF cạnh tranh nhau. Cấu hình EKF vẫn
hoạt động chỉ với odometry bánh xe khi chưa có IMU; sau này có thể tự kết hợp
`/imu/data` đã hiệu chuẩn và có covariance hợp lệ.

`cmd_vel_stamper` đổi `/cmd_vel` kiểu `Twist` từ teleop/Nav2 thành
`/base_controller/cmd_vel` kiểu `TwistStamped`. Node này chỉ đóng dấu khi nhận
lệnh mới và **không lặp lại lệnh**, vì lặp lại sẽ vô hiệu hóa ý nghĩa watchdog.
Càng nhận lệnh vị trí từ controller:

```bash
ros2 topic pub --once /lift_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.10]}"
```

Software watchdog không thay thế E-stop/contactor cắt nguồn driver, mạch giới
hạn an toàn độc lập, kiểm tra tải cơ khí và quy trình khóa nguồn khi bảo trì.
