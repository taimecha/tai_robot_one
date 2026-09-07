# Lộ trình mô phỏng đến robot thật

Tài liệu này là checklist nghiệm thu cho xe nâng bốn bánh dẫn động độc lập. Quy
ước quan trọng: bốn động cơ và bốn encoder vẫn là bốn kênh riêng; động học của
xe là skid-steer/differential, nên `front_left + rear_left` tạo phía trái và
`front_right + rear_right` tạo phía phải. Robot không nhận vận tốc ngang
`linear.y` như xe mecanum.

## 1. Trạng thái hiện tại

| Hạng mục | Đã có trong source | Còn phải nghiệm thu / bổ sung |
| --- | --- | --- |
| Mô hình | URDF/Xacro có bốn bánh, càng nâng, lidar, IMU và Astra Pro | Đo lại kích thước, khối lượng, tâm khối lượng, độ ma sát và tải hàng thật |
| `ros2_control` mô phỏng | `ros2_control.xacro`, plugin `gz_ros2_control`, `ros2_controllers.yaml`; các controller `joint_state_broadcaster`, `base_controller`, `lift_controller` | Chưa được coi là đạt cho đến khi build và chạy đủ tiêu chí ở mục 4 |
| Chuỗi lệnh xe | `/cmd_vel` (`Twist`) -> `cmd_vel_stamper` -> `/base_controller/cmd_vel` (`TwistStamped`) | Trước khi chạy thật cần bộ phân xử nguồn lệnh; không chạy teleop và Nav2 cùng xuất `/cmd_vel` |
| SLAM/Nav2 | Có `slam_toolbox.yaml`, `nav2_params.yaml`; `slam_sim.launch.py` để lập bản đồ, `slam_nav_sim.launch.py` để vừa mapping vừa điều hướng và `nav_sim.launch.py` để chạy map lưu + AMCL | Cả ba luồng vẫn phải qua gate runtime; initial pose `(0,0,0)` hiện chỉ phù hợp cách spawn mô phỏng |
| Vật cản thấp | Hai launch Nav2 chuyển `/camera/points` một lần thành `/camera/obstacle_scan` 10 Hz cho hai costmap và Collision Monitor | SLAM map chỉ dùng lidar cao; phải nghiệm thu điểm mù camera 0.6 m, nhiễu và vật nằm ngoài FOV trước |
| Camera mô phỏng | RGB, depth, camera info và point cloud ở 320 x 240, 10 Hz | Chưa có thuật toán xử lý ảnh cụ thể hoặc bộ dữ liệu/ground truth để chấm điểm |
| ESP32 truyền động | Firmware USB production, PID 50 Hz, telemetry 25 Hz, watchdog hai tầng; Wi-Fi/HTTP đã tách sang project tuner | Phải kiểm tra chiều quay/encoder, PPR, PID và đo watchdog trên phần cứng |
| ESP32 càng nâng | Firmware USB production, homing, hai công tắc NC, giới hạn cục bộ và watchdog | Phải kiểm tra chiều STEP/DIR, hành trình, bước/mm, endstop và thử có tải |
| PID thử nghiệm | PID dùng chung nằm ở `Common/amr_drive_pid/src/pid.cpp`; web/Wi-Fi nằm riêng trong `amr_pid_tuner` | Chỉ dùng tuner trên giá thử, không đưa web server vào firmware production |
| Pi 5 / USB | Có backend `TaiRobotSerialSystem`, launch robot thật và mẫu udev cho hai cổng ESP32 độc lập | Source chưa thay thế cho HIL: vẫn phải build trên Pi, điền udev thật và nghiệm thu mọi tình huống mất kết nối trước khi chạy sàn |
| EKF / IMU | `robot_localization` được launch trong mô phỏng và robot thật; nhận `/base_controller/odom`, phát `/odom` và là nguồn TF `odom -> base_footprint` duy nhất. Gazebo còn đưa `/imu` vào EKF. Project bench `bno055_imu_monitor` và package GitHub `esp32_imu_bridge` đã được tích hợp, có gyro/covariance, port parameter và reconnect | Bridge standalone vẫn cần một cổng/ESP32 riêng và không thể mở chung tty mà backend đã khóa. Với đúng hai ESP32, phải chọn nối BNO055 vào ESP32 càng rồi mở rộng cùng protocol, hoặc nối cảm biến trực tiếp vào Pi; hiện launch thật chạy wheel-only nếu `/imu/data` chưa có |

Các mục “đã có trong source” không đồng nghĩa đã chạy thử thành công. Không ghi
nhận gate nào là đạt nếu chưa lưu log/rosbag và kết quả đo tương ứng.

## 2. Giới hạn và ánh xạ cố định hiện tại

### Truyền động

- Bán kính bánh: `0.0875 m`; khoảng cách tâm bánh trái-phải: `0.428 m`.
- Giới hạn vận tốc xe: `|linear.x| <= 0.50 m/s`.
- Giới hạn từng bánh tương ứng: `5.71429 rad/s = 54.57 RPM`.
- `base_controller`: timeout lệnh `0.25 s`, gia tốc thẳng tối đa
  `0.50 m/s²`, vận tốc góc tối đa `1.20 rad/s`.
- Nav2 đặt vận tốc hành trình `0.45 m/s`; velocity smoother vẫn chặn ở
  `0.50 m/s`.
- Encoder firmware hiện đặt `1200 count/vòng`. Con số này phải được đo lại tại
  trục bánh, kể cả chế độ đếm cạnh và tỉ số truyền.

Thứ tự duy nhất trên ROS, USB và telemetry là `FL, FR, RL, RR`:

| Bánh / joint | Vị trí | RPWM/LPWM | Encoder A/B | Tên web cũ được giữ |
| --- | --- | --- | --- | --- |
| `front_left_wheel_joint` (`FL`) | trước, trái (`+Y`) | GPIO 27/14 | GPIO 18/19 | `fl` |
| `front_right_wheel_joint` (`FR`) | trước, phải (`-Y`) | GPIO 32/33 | GPIO 23/4 | `fr` |
| `rear_left_wheel_joint` (`RL`) | sau, trái (`+Y`) | GPIO 22/13 | GPIO 16/17 | `bl` |
| `rear_right_wheel_joint` (`RR`) | sau, phải (`-Y`) | GPIO 25/26 | GPIO 34/35 | `br` |

Các giá trị `command_sign` và `encoder_sign` hiện đều là `+1`. Chỉ xác nhận
chúng sau khi kích từng bánh một khi xe đang kê khỏi mặt đất. Không đổi thứ tự
field để sửa một bánh quay ngược; chỉ đổi sign đúng hàng.

### Càng nâng — ESP32 thứ hai

| Tín hiệu | GPIO | Ghi chú |
| --- | ---: | --- |
| TB6600 DIR | 18 | Chân đã dùng trước đây |
| TB6600 STEP/PUL | 19 | Chân đã dùng trước đây |
| Endstop dưới | 25 | COM-NC xuống GND, `INPUT_PULLUP`, active HIGH |
| Endstop trên | 26 | COM-NC xuống GND, `INPUT_PULLUP`, active HIGH |

GPIO 25/26 không xung đột với GPIO cùng số ở bộ truyền động vì đây là **ESP32
riêng**. Tiếp điểm bình thường đóng kéo input xuống LOW; khi chạm công tắc hoặc
đứt/rút dây, input lên HIGH và được coi là active. Không đưa tín hiệu 5 V/24 V
trực tiếp vào ESP32.

- Hành trình ROS: `0.000..0.520 m`.
- Tốc độ và gia tốc cục bộ tối đa: `0.050 m/s`, `0.050 m/s²`.
- Homing đi xuống ở `0.010 m/s`; firmware luôn chặn chuyển động đi xuyên qua
  endstop, độc lập với ROS. Nếu bắt đầu ngay tại một endstop, firmware chỉ cho
  phép pha nhả có xác minh tối đa `0.025 m`/`5 s`; switch trên xuất hiện trong
  lúc tìm đáy sẽ fault. Lỗi hình học/endstop luôn hủy `homed` và buộc HOME lại.
- Khởi động mới luôn ở trạng thái chưa home. Trình tự chuẩn là
  `HELLO -> CLEAR (nếu đã xử lý nguyên nhân) -> HOME -> ARM -> SET`.

## 3. Watchdog và đường an toàn

Ba lớp dừng hiện tại phục vụ ba lỗi khác nhau:

| Lớp | Timeout / tác dụng | Điều kiện phục hồi |
| --- | --- | --- |
| `diff_drive_controller` | Không có `/cmd_vel` mới trong `0.25 s` thì command về 0 | Lệnh ROS mới |
| ESP32 truyền động | Không có `CMD` mới trong `300 ms`: hãm có kiểm soát và latch `COMMAND_TIMEOUT`; đến `1000 ms`: PWM = 0, disable | Gửi `CMD` toàn 0 mới, `CLEAR`, rồi `ENABLE` |
| ESP32 càng nâng | Khi armed/homing, không có `SET` hoặc `HB` hợp lệ trong `300 ms`: ngừng STEP ngay, disarm, latch `COMM_TIMEOUT`, hủy session cũ | Session mới, xử lý lỗi, `CLEAR`, home nếu cần, `ARM`, target mới |

`PING` của bộ truyền động, `STATUS` của càng nâng và việc chỉ đọc telemetry
không gia hạn quyền chuyển động. `cmd_vel_stamper` chỉ đóng dấu và chuyển từng
lệnh nhận được, không lặp lại lệnh cũ. Vì vậy lệnh `/cmd_vel` gửi đúng **một
lần** phải làm xe mô phỏng dừng sau khoảng `0.25 s`; đây là hành vi mong muốn.

Chi phí CPU của watchdog gần như không đáng kể: mỗi vòng chỉ so sánh timestamp
và cập nhật vài biến. Trên Pi, `ros2_control`/serial cũng nhẹ hơn nhiều so với
Gazebo, point cloud và xử lý ảnh. Watchdog phần mềm, lidar thường và Nav2
Collision Monitor **không phải thiết bị an toàn chức năng**. Robot thật vẫn phải
có E-stop/contactor đấu cứng để ngắt enable/nguồn động lực, cùng giới hạn cơ khí
và quy trình đánh giá rủi ro.

## 4. Các gate mô phỏng bắt buộc

Chỉ chuyển gate khi tất cả ô của gate trước đã đạt. Nên lưu terminal log,
rosbag, cấu hình và commit tương ứng với mỗi lần nghiệm thu.

### Gate S0 — build, controller và TF

Chạy từ workspace đã source ROS 2 Jazzy:

```bash
colcon build --base-paths src/tai_robot_one_community/ros2_ws --symlink-install
source install/setup.bash
ros2 launch tai_robot_one gazebo_sim.launch.py
```

- [ ] Build không lỗi và launch không báo thiếu plugin/dependency.
- [ ] `ros2 control list_controllers` cho thấy cả ba controller ở trạng thái
  `active`.
- [ ] Đủ bốn wheel joint và `lift_joint` trong `/joint_states`; không có NaN.
- [ ] Cây TF `odom -> base_footprint -> base_link -> sensor` liên tục, chỉ có
  một publisher cho mỗi transform động; `/base_controller/odom` là dữ liệu
  bánh thô còn EKF là publisher duy nhất của `/odom` và TF động này.
- [ ] `/scan`, `/imu`, `/camera/image`, `/camera/depth_image`,
  `/camera/camera_info`, `/camera/points` có timestamp theo `/clock`.
- [ ] Khi chạy 10 phút, không có controller crash, physics explosion hoặc tăng
  bộ nhớ liên tục.

### Gate S1 — chuyển động cơ bản, odometry và càng nâng

Kiểm tra lệnh một lần:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}"
```

- [ ] Xe bắt đầu chạy rồi command về 0 không muộn hơn `0.35 s`; node stamper
  không tự lặp lại lệnh.
- [ ] Với lệnh liên tục 20 Hz, tốc độ đo không vượt `0.50 m/s`, kể cả khi vừa
  tiến vừa quay.
- [ ] Đi thẳng 5 m trong mô phỏng: sai số odometry dọc không quá 5%, lệch ngang
  không quá `0.10 m`.
- [ ] Quay tại chỗ 10 vòng: sai số yaw tích lũy không quá 5%; nếu không đạt,
  tune `wheel_separation_multiplier`, không sửa bừa kích thước URDF.
- [ ] Bốn encoder/joint tăng đúng dấu khi đi tiến; cặp trái và phải ngược dấu
  đúng theo convention khi quay tại chỗ.
- [ ] Gửi `/lift_controller/commands` lần lượt `0.0`, `0.2`, `0.52 m`; vị trí
  dừng trong `+/-0.005 m`, không vượt joint limit và không xuyên collision.
- [ ] Dừng command, restart controller và restart Gazebo đều đưa actuator về
  trạng thái không tự chuyển động.

### Gate S2 — SLAM Toolbox và lưu bản đồ

```bash
ros2 launch tai_robot_one slam_sim.launch.py
```

Sau khi đi đủ các lối và quay lại điểm đầu:

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/warehouse
```

- [ ] `/scan` chồng đúng lên tường/kệ ở RViz khi xe chạy và quay.
- [ ] Đi một vòng kín tối thiểu 20 m; loop closure không làm pose nhảy quá
  `0.20 m` hoặc `5 deg`.
- [ ] Các mép tường quan sát lại không tạo hai vệt cách nhau quá `0.10 m`.
- [ ] Tạo được cả `warehouse.yaml` và ảnh bản đồ, sau khi mở lại vẫn đúng
  scale `0.05 m/pixel` và origin.
- [ ] Lặp lại ba lượt từ pose đầu khác nhau; không mất tracking.

Lidar đang ở cao độ thực đo `0.835 m`, còn kệ mô phỏng chỉ cao khoảng
`0.680 m`; vì vậy bản đồ SLAM 2D có thể không chứa thân kệ/pallet thấp. Đây là
giới hạn có chủ ý cần nhìn thấy trong bài test, không phải lý do để coi camera
scan là một phần của map tĩnh. Hai launch Nav2 dùng camera scan làm obstacle
động để bù cho việc điều hướng; nếu robot thật cần kệ thấp xuất hiện trong bản
đồ tĩnh thì phải đổi vị trí lidar hoặc bổ sung pipeline hợp nhất scan đã được
hiệu chuẩn.

### Gate S3 — định vị, bám quỹ đạo, né vật cản và Nav2

`slam_nav_sim.launch.py` dùng để thử điều hướng trong lúc đang mapping:

```bash
ros2 launch tai_robot_one slam_nav_sim.launch.py
```

Nghiệm thu định vị từ map đã lưu bằng một đường dẫn YAML tuyệt đối:

```bash
ros2 launch tai_robot_one nav_sim.launch.py \
  map:=/home/<user>/ros2_ws/maps/warehouse.yaml
```

`nav_sim.launch.py` kiểm tra map tồn tại rồi chạy map server + AMCL + Nav2.
Cấu hình mô phỏng tự đặt initial pose `(0,0,0)` vì Gazebo và lượt mapping đều
spawn tại đó. Robot thật phải dùng pose được đo/chọn trong RViz hoặc một file
parameter production riêng; không được mặc định robot thật đang ở origin. Tiêu
chí cho cả hai chế độ:

- [ ] Sau initial pose mô phỏng tự động hoặc `2D Pose Estimate`, sai số so với
  ground truth không quá `0.10 m` và `5 deg` sau khi robot đã di chuyển để hội
  tụ.
- [ ] Hoàn thành 10/10 goal gồm đường thẳng, cua 90 độ, quay đầu và lối hẹp;
  sai số goal không quá `0.10 m` và `0.10 rad`.
- [ ] Sai số bám đường RMS không quá `0.10 m` trên đoạn thẳng và `0.15 m` ở
  đoạn cua; không dao động trái-phải kéo dài.
- [ ] Với vật cản tĩnh mới xuất hiện, costmap đánh dấu trong `0.5 s`, Nav2
  replan hoặc dừng, và không có collision.
- [ ] Với vật cản đi vào đường chạy, Collision Monitor dừng trước khi footprint
  bảo thủ chạm vật cản, còn tối thiểu `0.10 m` trong kịch bản thử.
- [ ] `/camera/obstacle_scan` giữ khoảng 10 Hz; cột kệ hẹp vẫn tạo ít nhất hai
  điểm ở khoảng cách thử. Ngừng camera quá `0.45 s` phải đưa Collision Monitor
  về hành vi an toàn đã định nghĩa, không tiếp tục dùng dữ liệu cũ.
- [ ] Rút `/scan` hoặc làm TF quá hạn: xe dừng, action báo lỗi rõ ràng, không
  tiếp tục bằng dữ liệu cũ.
- [ ] Footprint hiện dùng biên `x=-0.33..0.78 m`, `y=+/-0.25 m`, đã bao cả
  càng. Thêm pallet/tải nhô ra thì footprint/collision polygon phải đổi theo.

### Gate S4 — camera và thuật toán ảnh

- [ ] Bốn topic camera duy trì tối thiểu 9 Hz ở cấu hình 320 x 240, 10 Hz;
  timestamp, `camera_info` và TF thống nhất.
- [ ] Ghi một rosbag cố định làm tập regression; mỗi thay đổi thuật toán chạy
  lại đúng bag này.
- [ ] Mỗi thuật toán phải có metric trước khi nghiệm thu. Ví dụ: detection có
  precision và recall tối thiểu 90% trên tập mô phỏng đã gán nhãn; ước lượng
  khoảng cách 1--4 m sai số không quá `0.10 m`.
- [ ] Kiểm tra ánh sáng yếu/chói, vật ít texture, che khuất, pallet và vật cao
  ngoài mặt phẳng lidar; lỗi nhận thức phải dẫn tới giảm tốc/dừng, không tạo
  lệnh chuyển động trực tiếp bỏ qua Collision Monitor.
- [ ] Chạy đồng thời Nav2 + SLAM/camera trong 30 phút: không swap, không có
  watchdog giả do nghẽn CPU và không tụt chu kỳ điều khiển kéo dài.

## 5. Những thay đổi mô phỏng còn nên làm

- [ ] Đo và cập nhật mass/inertia, tâm khối lượng, wheel radius/track,
  ma sát dọc-ngang và hình học càng/pallet theo robot thật.
- [ ] Tạo ma trận kịch bản warehouse: lối hẹp, góc mù ở kệ, pallet lệch, người
  cắt ngang, vật thấp/vật treo cao, mất scan, camera trễ và nhiễu encoder.
- [ ] Mô phỏng nhiều mức tải và độ cao càng. Gazebo hiện không thay thế phép
  tính ổn định/chống lật hay giới hạn nhiệt-dòng của động cơ.
- [ ] Thêm profile footprint và speed theo tình trạng tải/càng. Với tải nâng
  cao, robot thật cần giảm tốc/giảm gia tốc hoặc khóa di chuyển theo kết quả
  đánh giá ổn định; không mặc định `0.5 m/s` là an toàn ở mọi độ cao.
- [ ] Thêm bộ phân xử lệnh (`twist_mux` hoặc safety arbiter) cho teleop,
  autonomy và stop. Collision Monitor phải nằm ở cuối chuỗi trước
  `cmd_vel_stamper`/`base_controller`.
- [ ] Bổ sung diagnostics chuẩn hóa cho controller, serial, encoder, endstop,
  pin nguồn và nhiệt độ; launch robot thật cơ bản đã có trong source.
- [ ] Dùng rosbag/seed cố định để so sánh regression thay vì chỉ quan sát RViz.

## 6. Bench và hardware-in-the-loop

### B0 — chuẩn bị USB ổn định

Trước khi cho xe chạm sàn, còn ba quyết định tăng cứng protocol phải được chốt
và thử lỗi có chủ đích:

- [ ] Thêm CRC ở tầng ứng dụng cho protocol càng nâng. USB đã có CRC ở tầng
  truyền dẫn nhưng chưa thay thế việc phát hiện frame ASCII hợp lệ về cú pháp
  mà bị lật bit thành một giá trị khác.
- [ ] Thêm session và kiểm tra sequence chống replay cho ESP32 truyền động;
  sequence hiện chỉ được echo. Việc backend mở độc quyền cổng và xóa buffer khi
  kết nối giúp giảm rủi ro nhưng không phải cơ chế chống lệnh cũ hoàn chỉnh.
- [ ] Chọn semantics deadman cho lệnh nâng từ người vận hành. Hiện một target
  ROS được controller giữ và backend tiếp tục gửi `SET` ở 50 Hz; watchdog 300 ms
  bắt lỗi controller/Pi/USB, nhưng không nhận biết riêng publisher cấp cao đã
  chết. Nếu yêu cầu “thả nút là dừng”, phải thêm supervisor/deadman ở cấp ROS.

Không dùng trực tiếp `/dev/ttyUSB0`, `/dev/ttyUSB1`: thứ tự này thay đổi sau
mỗi lần cắm/rút. Gán serial number khác nhau cho hai ESP32 nếu board hỗ trợ, rồi
tạo udev symlink:

```udev
SUBSYSTEM=="tty", ATTRS{serial}=="<SERIAL_DRIVE>", SYMLINK+="tai_drive", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{serial}=="<SERIAL_LIFT>",  SYMLINK+="tai_lift",  GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{serial}=="<SERIAL_LIDAR>", SYMLINK+="tai_lidar", GROUP="dialout", MODE="0660"
```

Không copy nguyên placeholder. Lấy thuộc tính thật bằng `udevadm info` và dùng
`ID_PATH`/cổng vật lý nếu hai ESP32 không có serial duy nhất. Checklist:

- [ ] `/dev/tai_drive` luôn là ESP32 bánh xe ở 460800 baud.
- [ ] `/dev/tai_lift` luôn là ESP32 càng ở 115200 baud.
- [ ] `/dev/tai_lidar` luôn là lidar; camera dùng `/dev/v4l/by-id/...` khi có.
- [ ] User chạy ROS thuộc group `dialout`; rút/cắm từng thiết bị không đổi tên
  hai thiết bị còn lại.
- [ ] Cáp ngắn, có chống tuột; hub có nguồn riêng nếu tổng dòng USB cao. Motor,
  driver và TB6600 không lấy nguồn công suất từ Raspberry Pi.

### B1 — thử riêng ESP32 truyền động, xe kê bánh

- [ ] Nạp firmware production; không bật project Wi-Fi/HTTP tuner khi vận hành.
- [ ] Ra lệnh từng field `FL`, `FR`, `RL`, `RR` ở tốc độ thấp. Đúng bánh quay,
  chiều tiến và encoder dương khớp bảng mục 2.
- [ ] Đánh dấu bánh, quay chính xác 10 vòng và xác nhận count/vòng trong 1%; cập
  nhật `1200` nếu phép đo khác.
- [ ] Tune PID từng bánh với tải đại diện; ghi Kp/Ki/Kd/feed-forward, overshoot,
  settling time và dòng motor. Không tune bánh chạm sàn khi robot chưa được giữ.
- [ ] Ngừng `CMD`: log cho thấy soft stop bắt đầu không quá 300 ms và PWM bằng
  0/disable không quá 1000 ms; reconnect không khôi phục lệnh cũ.
- [ ] `ESTOP`, CRC sai, frame thiếu, NaN và lệnh vượt `5.71429 rad/s` đều bị từ
  chối hoặc đưa hệ về trạng thái an toàn đúng protocol.

### B2 — thử riêng ESP32 càng, không tải

- [ ] Dùng đồng hồ đo xác nhận endstop NC: bình thường GPIO LOW; nhấn, đứt dây
  hoặc rút giắc đều cho active HIGH sau debounce.
- [ ] Kích từng endstop và cả hai cùng lúc; chuyển động vào giới hạn phải dừng,
  trạng thái/fault đúng và không thể `CLEAR` khi nguyên nhân vật lý còn tồn tại.
- [ ] Xác nhận hướng STEP/DIR, `100 step/mm` theo cấu hình hiện tại và đo lại
  toàn hành trình `0.520 m`; cập nhật thông số vít me/microstep nếu khác.
- [ ] Chạy HOME từ giữa hành trình và từ sát hai đầu; thử lỗi switch không nhả,
  timeout homing và mất USB.
- [ ] Ngừng `SET/HB`: STEP dừng không quá 300 ms, session cũ không thể làm
  chuyển động; sau reconnect phải qua trình tự session/home/arm mới.
- [ ] Thử tải tăng dần trong giới hạn cơ khí; theo dõi mất bước, dòng, nhiệt,
  độ võng và phanh/giữ tải khi mất điện.

### B3 — hardware-in-the-loop với Pi, bánh vẫn kê

- [ ] Backend `ros2_control` mở đúng hai symlink, parse được telemetry và đưa
  đủ bốn vị trí/vận tốc bánh cùng vị trí càng vào joint state.
- [ ] Mất riêng `/dev/tai_drive` không làm càng chạy; mất riêng `/dev/tai_lift`
  không phát lệnh bánh cũ. Cả hai lỗi được báo diagnostics.
- [ ] Kill Nav2, kill controller manager, treo process serial và rút USB: mỗi
  trường hợp đều dừng theo bảng watchdog, không tự resume khi process trở lại.
- [ ] CPU tải giả + camera + lidar không tạo timeout ngẫu nhiên. Mục tiêu ban
  đầu: còn ít nhất 30% headroom CPU tổng, không swap, không có controller-loop
  overrun kéo dài; điều chỉnh sau khi có số đo Pi thật.
- [ ] Odometry encoder được so với số vòng bánh đo độc lập trước khi cho xe
  chạm sàn.

## 7. Bố trí tài nguyên và triển khai Raspberry Pi 5

| Tài nguyên | Thiết bị | Gợi ý |
| --- | --- | --- |
| USB serial 1 | ESP32 bốn bánh | `/dev/tai_drive`, 460800 baud, cáp khóa/chống rung |
| USB serial 2 | ESP32 càng nâng | `/dev/tai_lift`, 115200 baud, tách lỗi khỏi drive |
| USB serial 3 | RPLidar | `/dev/tai_lidar`; không dùng chung symlink với ESP32 |
| USB video | Astra Pro | `/dev/v4l/by-id/...`; cân đối băng thông và nguồn của hub |
| Ethernet/Wi-Fi | Laptop camera tùy chọn | Đồng bộ giờ bằng chrony/NTP, cùng ROS domain và kiểm tra DDS QoS/firewall |

Nếu camera chạy trên laptop, chỉ gửi topic cần thiết. Raw RGB-D/point cloud tốn
băng thông và CPU hơn nhiều so với gửi detection, pose hoặc vùng depth đã lọc.
Node quyết định an toàn và watchdog vẫn phải ở trên robot/Pi, không phụ thuộc
Wi-Fi.

Checklist triển khai:

- [ ] Ubuntu 24.04 + ROS 2 Jazzy trên Pi 5; pin nguồn đủ công suất, không báo
  undervoltage/throttling.
- [ ] Build cùng commit đã qua mô phỏng; lưu toàn bộ YAML hiệu chỉnh theo serial
  của robot, không sửa trực tiếp trong install space.
- [ ] udev, `dialout`, service khởi động và log rotation đã thử sau cold boot.
- [ ] Launch thật dùng `use_sim_time:=false`; chỉ một nguồn phát `odom ->
  base_footprint`; timestamp lidar/camera/IMU cùng clock.
- [ ] EKF nhận `/base_controller/odom` và, khi phần cứng đã chốt, `/imu/data`
  có covariance hiệu chỉnh; output `/odom` công khai cho Nav2 và chỉ EKF phát
  TF `odom -> base_footprint`.
- [ ] Chốt đường phần cứng IMU không làm phát sinh ESP32 thứ ba: hoặc BNO055
  dùng I2C trên ESP32 càng và đi chung protocol `/dev/tai_lift`, hoặc cảm biến
  nối trực tiếp Pi. `esp32_imu_bridge` đã được sửa để dùng `/dev/tai_imu`, nhưng
  chỉ là đường standalone/bench; không trỏ nó vào `/dev/tai_lift` hay
  `/dev/tai_drive` vì backend đang giữ độc quyền hai tty này.
- [ ] Theo dõi `htop`, nhiệt độ/throttling, topic rate/latency và controller
  diagnostics trong ít nhất 60 phút. Gazebo không chạy trên Pi khi vận hành
  thật.

Mức tải dự kiến: watchdog/PID/serial thấp; `ros2_control` và Nav2 mức thấp đến
trung bình; SLAM Toolbox mức trung bình; RGB-D point cloud và xử lý ảnh thường
là phần nặng nhất. Trong mô phỏng, Gazebo rendering/physics còn nặng hơn và nên
chạy trên laptop. Có thể dùng `headless:=true`, giữ camera 320 x 240 @ 10 Hz và
tắt display/point cloud không cần thiết để dành tài nguyên.

## 8. Thử sàn an toàn và hiệu chỉnh cuối

- [ ] Khu vực rào kín, có người cầm E-stop, bánh xe/tải được cố định đúng cách;
  thử trước ở `0.10 m/s`, không tải và càng hạ thấp.
- [ ] Đo lại wheel radius hữu hiệu, wheel separation multiplier và encoder
  scale bằng đường thẳng/quay chuẩn; sau đó chạy lại tiêu chí odometry.
- [ ] Tune giới hạn gia tốc, phanh và PID theo độ bám sàn/tải; đo quãng dừng cho
  từng tốc độ và đưa kết quả vào đánh giá rủi ro.
- [ ] Xác nhận E-stop cứng ngắt năng lượng nguy hiểm dù Pi, ROS và cả hai ESP32
  đều treo. Ghi rõ cách khôi phục, tuyệt đối không tự resume.
- [ ] Chạy waypoint và vật cản ở 0.10, 0.20, rồi tăng dần đến tối đa 0.50 m/s
  chỉ khi mỗi mức đạt; kiểm tra tải nhô ra bằng footprint đúng.
- [ ] Thử mất lidar, camera, IMU, từng USB, mạng laptop và nguồn cảm biến. Mỗi
  lỗi phải đưa robot về trạng thái đã định nghĩa và có log dễ chẩn đoán.
- [ ] Với càng nâng cao hoặc có pallet, áp dụng interlock tốc độ/di chuyển đã
  được xác nhận bằng thử ổn định. Lidar 2D không phát hiện đầy đủ vật treo cao,
  càng hoặc pallet nên camera/thiết bị bảo vệ bổ sung vẫn cần thiết.
- [ ] Chỉ sau khi chạy endurance có tải, không watchdog giả, không quá nhiệt,
  không mất localization và đạt toàn bộ tiêu chí trên mới cho robot hoạt động
  ngoài khu thử nghiệm.

Tài liệu giao thức chi tiết nằm tại
`esp32_firmware/Projects/amr_firmware/README.md` và
`esp32_firmware/Projects/cang_nang_ha/PROTOCOL.md`.
