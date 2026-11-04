# detect_astra trong Nav2 trên Pi 5

Mã nguồn, cấu hình và model từ `taimecha/detect_astra` commit
`fbb2d113753a76ae2c8afbbd788be2a493527b00` nằm ở
`camera/detect_astra/` (bỏ các ảnh chụp thử). Node tạo cloud cho Nav2
nằm tại `ros2_ws/scripts/publish_rgb_calibrated_cloud.py` để package
`tai_robot_one` cài đặt được. File launch dùng trên xe là
`ros2_ws/launch/astra_obstacles.launch.py`.

Repo nguồn không trực tiếp phát `/camera/obstacle_scan`. Pipeline mới:

```text
Astra: Depth đăng ký theo RGB, 640×480
  -> /camera/depth/image_raw + /camera/color/camera_info
  -> sửa nghiêng depth theo cột + bù cao độ nền + xác nhận sàn khi đứng yên
  -> /camera/depth/points_nav_corrected (base_footprint, tối đa 15 Hz)
  -> pointcloud_to_laserscan (cao độ 0,06–0,75 m)
  -> /camera/obstacle_scan
  -> Nav2 costmap và collision monitor
```

Calibration RGB và hệ số sửa lệch Depth theo cột ảnh nằm trong
`ros2_ws/config/calibration/`. `camera_scan_hz:=15.0` đổi tốc độ tối đa của
cloud và scan; `camera_cloud_pixel_stride:=3` đổi khoảng lấy mẫu pixel.
Stride 3 giảm cloud từ 76.800 xuống 34.240 điểm mỗi frame trên Pi, giữ nguyên
trường nhìn, khoảng cách và cửa sổ độ cao; tại 2 m vẫn khoảng 1 cm giữa hai cột.
Launch xe thật dùng `astra_pro_nav_floor_2026_11_02_restart.yaml` làm hiệu chỉnh
nền và bật `floor_surface_tracking_enabled`. `floor_surface_model.py` xác nhận
phần sai số sàn còn lại từ vùng rộng, có hỗ trợ hai bên/gần xa, chỉ khi odom
mới và xe đứng yên. Mặt nâng cao không qua mốc cao độ ±3 cm, vùng nhỏ, thiếu
75% hỗ trợ hoặc mô hình biến động không được chấp nhận. Cần >=8 phép đo trong
>=3,5 giây; khi xe chạy hoặc sàn bị che, giữ phép bù đã xác nhận.
Phần bù cao độ giữ nguyên X/Y sau hiệu chỉnh depth, không thay URDF/TF hoặc
các goal đã lưu. Bộ lọc vẫn giữ cửa sổ 0,06–0,75 m; không hạ ngưỡng để thu cả sàn.
Nếu camera đổi vị trí/góc lắp hoặc hiệu chỉnh depth thay đổi, phải đo lại profile.
Chưa có mô hình đủ điều kiện thì chưa phát cloud Nav; timeout camera giữ xe
dừng. `/camera/floor_height/status` báo qualified/reason. Chu trình kiểm tra
trạng thái này trước khi bắt đầu. Không học lại sàn khi xe đang chuyển động.
Kiểm chứng và giới hạn của các phiên nằm trong `camera/detect_astra/CYCLE.md`.

## Giữ luồng ảnh liên tục khi Nav2 chạy

`camera_fastdds.xml` cấp segment SHM 32 MiB cho tiến trình camera và cloud;
UDP vẫn hoạt động để laptop khám phá và nhận các topic cần hiển thị.
Profile được giới hạn trong nhóm camera, không thay middleware/tham số động cơ,
AMCL hay các server Nav2 khác. Giữ nguyên QoS ảnh và calibration của driver.
Một ảnh RGB 640×480×3 lớn hơn segment mặc định 512 KiB; phép đo trước sửa
cho thấy CameraInfo vẫn 15 Hz nhưng RGB nhận đồng thời depth chỉ khoảng 7 Hz.
Thử nghiệm ảnh giả lập ở domain riêng cho thấy profile lớn truyền đủ hai luồng
30 Hz; luôn kiểm tra lại trên camera thật khi đổi tải hoặc cấu hình.

Kiểm tra 30 giây sau khi camera khởi động ổn định (không phát lệnh xe):

```bash
ros2 run tai_robot_one check_camera_streams --seconds 30 \
  --output /tmp/camera_streams.json
```

Lệnh kiểm tra tốc độ, tuổi ảnh, khoảng ngắt tối đa 250 ms và khả năng ghép RGB,
depth, CameraInfo trong 50 ms. Đây là kiểm tra thời gian luồng camera, chưa thay
thế xác nhận YOLO và kiểm tra độ ổn định hình học trước khi căn pallet.
Point cloud gốc của driver bị tắt để giảm tải CPU. Node YOLO/pallet của
`detect_astra` được giữ trong thư mục dự án nguồn nhưng không được khởi động
cùng Nav2.

Build trong đúng workspace đang mở:

```bash
cd /home/admin/tai_robot_one/tai_robot_one/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select tai_robot_one --symlink-install
source install/setup.bash
```

Không chạy đồng thời camera launch cũ hoặc script mở camera của `detect_astra`.
Kiểm tra `ros2 topic hz /camera/obstacle_scan` và hình vật cản trong RViz trước
khi cho xe chạy. Hệ số sửa Depth của repo nguồn là thử nghiệm từ mặt sàn; chưa
được chứng minh là hiệu chuẩn khoảng cách tuyệt đối cho toàn ảnh.

## Loại sàn giả khi chở pallet, 2026-11-02

Trong lượt giao pallet 1, một phần sàn trống ở bên phải ảnh được hồ sơ ngày 01-11 tính cao 6,0–6,6 cm, vượt cửa sổ vật cản 6 cm. Các điểm này tạo ô lethal trong global costmap dù bản đồ PGM tại các tọa độ đó là free; Nav2 từ chối đường/góc kết thúc với NO_VALID_PATH 208. GridBased trả đường riêng chưa đủ để kết luận toàn bộ quỹ đạo và xoay cuối lọt footprint.

Hồ sơ ngày 02-11 dùng vùng sàn trống xác định trên RGB, rộng theo XY 1,50 × 1,35 m; 99,6% điểm fit trong 12 mm. Trên 15 frame kiểm chứng độc lập, percentile 99 cao độ sàn là 0,94 cm, không có điểm sàn vượt 6 cm. Trên frame pallet xanh đã lưu, số điểm pallet vượt 6 cm không giảm (14.097 → 14.115). Chỉ cập nhật mặt phẳng tham chiếu cao độ; giữ inverse-depth slope, toàn bộ X/Y, TF, cửa sổ 6–75 cm và footprint chở pallet. Mặt phẳng vẫn cố định khi chạy, không tự học từ vật cản.

Sau khi nạp lại node cloud cần làm mới costmap động khi xe đứng yên, không có goal đang chạy, để bỏ các dấu sàn giả đã lưu. Không sửa bản đồ PGM hoặc tọa độ goal đã lưu. Bộ kiểm thử cao độ có hồi quy sàn giả và pallet thấp. Dữ liệu kiểm chứng nằm trong `camera/detect_astra/data/diagnostics/delivery_floor_fit_report_20261102.json`.
