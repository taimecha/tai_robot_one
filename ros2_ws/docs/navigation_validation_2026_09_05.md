# Kiểm thử điều hướng ngày 05/09/2026

> Cập nhật 06/09: người dùng đã tìm thêm tình huống kẹt cạnh trụ phía dưới.
> Kết quả 16 chặng bên dưới chỉ là kết quả của bản ngày 05/09, không chứng minh
> mọi tình huống đều đi được. Xem `navigation_validation_2026_09_06.md` cho sửa
> vùng dừng theo hướng chuyển động và nhánh tiến thoát độc lập.

## Phạm vi và kết quả

Gazebo Harmonic, ROS 2 Jazzy, bản đồ `/home/tai/ros2_ws/maps/tai_warehouse.yaml`.
Hai mốc trong frame `map`:

- A: `(0.0, 0.0, yaw=0.0)`.
- B: `(1.532, -1.412, yaw=-0.012 rad)`.

Sau khi đưa nhánh **lập và thực thi đường SE2** lên trước các động tác recovery,
đã chạy 16 chặng liên tiếp A→B→A, tất cả trả về `SUCCEEDED`, bộ đếm recovery bằng 0.
Trong đó 4 chặng cuối chạy sau khi build lại phần khóa tham số và tách hàm kiểm
tra cung quay để kiểm thử đơn vị. Những thử nghiệm trước bản sửa này vẫn có
abort; không tính chúng vào chuỗi kiểm thử thành công.

| Chặng cuối | Thời gian thực | Sai số XY theo TF map | Sai số yaw | Recovery |
| --- | ---: | ---: | ---: | ---: |
| A→B | 27.90 s | 0.0446 m | -0.0348 rad | 0 |
| B→A | 45.56 s | 0.0378 m | 0.0367 rad | 0 |
| A→B | 37.10 s | 0.0498 m | -0.0392 rad | 0 |
| B→A | 46.96 s | 0.0298 m | 0.0337 rad | 0 |

Recovery=0 không có nghĩa là không lập lại đường: controller có thể từ chối cung
quay và chuyển sang thực thi đường SE2 mà không gọi Spin/DriveOnHeading.
Sai số được lấy từ TF ngay khi nhận kết quả, có độ trễ cập nhật; đây không phải
sai số vị trí thật tuyệt đối của Gazebo.

Log của lượt cuối:
`/home/tai/.ros/log/2026-09-05-16-31-33-889723-DESKTOP-B3BME4V-31018`.

## Kiểm tra bổ sung

- Build package thành công.
- 5/5 bài kiểm thử `test_rotation_sweep` thành công: chọn cung ngắn khi an toàn,
  đổi dấu đúng ở cả hai phía, từ chối khi cả hai phía bị chặn, kiểm tra cung giữa
  và điểm cuối, giữ dấu khi góc đi qua biên ±pi.
- Hủy goal đang chạy: `cancel_return_code=0`, action `CANCELED` (5), lệnh cuối
  `linear.x=0`, `angular.z=0`.
- Kiểm tra hình học độc lập bằng vị trí thật từ Gazebo: một đợt đủ 4 chặng có
  độ hở nhỏ nhất 0.1176 m; đợt quan sát tiếp theo có độ hở nhỏ nhất 0.08695 m.
  Không ghi nhận giao nhau giữa polygon bao xe và các vật cản được kiểm tra.
  Kệ được coi là hộp đặc bao ngoài, gồm thêm ba trụ và bốn tường. Đây **không phải
  phép đo contact vật lý**, không chứng nhận an toàn cho mọi tình huống hay xe thật.
- Kết quả lint toàn kho có lỗi cũ, bao gồm cây build/install và mã bên thứ ba;
  không tuyên bố toàn bộ test suite sạch. Chỉ test đích danh ở trên đã được xác minh.

## Thay đổi chính

1. `SafeRotationRPP` kiểm tra toàn bộ footprint quét qua cả hai cung quay, ưu
   tiên cung ngắn an toàn và giữ hướng đã chọn. Nếu dữ liệu mới làm cung không
   còn an toàn, dừng và trả lỗi để lập lại đường.
2. Behavior tree thử **ComputePathToPose + FollowPath** với SE2 trước các động
   tác quay/tiến recovery. Không coi chỉ lập được đường là đã thoát kẹt.
3. Footprint thu hẹp đúng theo phần càng, vẫn bao thân, bánh, trụ nâng và đầu càng;
   không bỏ kiểm tra va chạm. Collision Monitor có polygon riêng cho quay/dừng
   để không giữ nhầm polygon tiến của lần trước.
4. Giảm tốc độ theo đường, giữ cấm lùi; goal checker kiểm tra lại XY sau khi
   xoay cuối, tránh báo thành công khi xe còn dừng sớm gần trụ.

Không dùng vị trí thật Gazebo làm đầu vào điều hướng, không sửa hai goal để né lỗi,
không xóa vật cản khỏi bản đồ và không tắt Collision Monitor.

## Lệnh dùng lại

Build (đã thực hiện; chỉ cần chạy lại sau khi sửa mã):

```bash
cd /home/tai/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths /home/tai/ros2_ws/src/tai_robot_one_community/ros2_ws \
  --packages-select tai_robot_one --symlink-install \
  --build-base /home/tai/ros2_ws/build --install-base /home/tai/ros2_ws/install
```

Mở Gazebo + Nav2, giữ nguyên launch và bản đồ đã dùng:

```bash
cd /home/tai/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch tai_robot_one nav_sim.launch.py \
  map_file:=/home/tai/ros2_ws/maps/tai_warehouse.yaml \
  use_phone_teleop:=false use_rviz:=false headless:=false
```

Mở RViz ở terminal khác:

```bash
cd /home/tai/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 run rviz2 rviz2 \
  -d /home/tai/ros2_ws/install/tai_robot_one/share/tai_robot_one/config/nav_robot.rviz \
  --ros-args -p use_sim_time:=true
```

Tự gửi 4 chặng kiểm thử, **chỉ chạy trong mô phỏng**, khi Nav2 đã active và không
có goal/teleop khác:

```bash
python3 /home/tai/ros2_ws/src/tai_robot_one_community/ros2_ws/scripts/test_nav_round_trip
```

Theo dõi độ hở hình học của world kho chưa thay đổi (terminal đã source ROS):

```bash
python3 /home/tai/ros2_ws/src/tai_robot_one_community/ros2_ws/scripts/monitor_sim_clearance
```

Các tiến trình mô phỏng/kiểm thử do agent mở đã được dừng sau khi kiểm tra xong.
