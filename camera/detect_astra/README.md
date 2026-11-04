# Astra Pallets

Nhận diện và phân đoạn pallet cùng hàng hóa từ camera RGB Astra Pro bằng YOLO. Model mặc định là `models/best_v3.pt`, gồm ba lớp `Box`, `emty pallet` và `pallets`. Node `/pallet/target` chỉ chọn các lớp pallet để đo, bỏ qua lớp `Box`.

**Giai đoạn hiện tại: đã triển khai bộ căn Nav2 theo yêu cầu thử xe thật; người vận hành mở và bấm bắt đầu.** Luồng nhận diện:
goal nhận diện thành công → YOLO xác nhận pallet → tắt YOLO → theo dõi RGB + Depth
và đánh giá ổn định. Xem [PERCEPTION_HANDOFF.md](PERCEPTION_HANDOFF.md) để chạy.
Bộ căn chọn goal trung gian trên pháp tuyến pallet, giữ gốc càng cách mặt pallet ít nhất 1,20 m rồi sửa góc bằng RGB-D. Xem [ALIGNMENT.md](ALIGNMENT.md).

**Lấy hàng bản mới (03-11):** dừng ngoài pallet để đo lại RGB-D, căn góc rồi chốt
đường thẳng trong odometry; không dùng goal pickup bản đồ. Chu trình chờ kiểm tra
thực tế trước khi nâng. Người vận hành đã xác nhận khoảng trống trong lỗ
**rộng 25,5 cm × dài 35,5 cm × cao 10 cm**, giữa bốn trụ ở bốn góc hình chữ nhật;
phía sau hở, các cặp trụ trước/sau thẳng hàng. Thêm `--auto-localize` để AMCL tự
tìm lại pose trên bản đồ khi xe đứng yên, kiểm tra laser và hội tụ trước khi lấy hàng. Lệnh khởi động/chạy và phạm vi kiểm
chứng tại [PICKUP.md](PICKUP.md), [CYCLE.md](CYCLE.md).

Để làm với **pallet xanh = pallet 1** khi có cả pallet thường trong ảnh, thêm `--target-color green` vào lệnh nhận diện. Mặc định `any` xét tất cả pallet và sẽ từ chối chốt mục tiêu khi có hai pallet.

**Góc giữa mặt trước xe và mặt pallet:** tùy chọn `--heading-floor-reference`
ước lượng mặt phẳng pallet với tham chiếu sàn, giữ nguyên khoảng cách/X/Y.
Hồ sơ thử bù góc hằng số 2026-11-01 đã bị loại. Cách chạy và điều kiện:
[HEADING_REFERENCE.md](HEADING_REFERENCE.md).

## Cài đặt

Dự án chạy trên Linux hoặc WSL2 có giao diện đồ họa, sử dụng camera V4L2 tại `/dev/video0`. Môi trường đã chạy dùng Python 3.12.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Chương trình tự dùng CUDA nếu PyTorch nhận GPU, nếu không sẽ dùng CPU.

## Kết nối camera trên WSL2

Cắm camera vào máy. Với usbipd-win đã cài trên Windows, kiểm tra BUSID của `Astra Pro HD Camera` bằng PowerShell:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

Lệnh `bind` cần PowerShell chạy với quyền quản trị và chỉ cần khi thiết bị chưa được chia sẻ. BUSID có thể thay đổi khi đổi cổng USB.

Kiểm tra `/dev/video0` đã xuất hiện trong WSL và người dùng có quyền truy cập camera. Nếu camera dùng đường dẫn khác, sửa biến `DEVICE` trong script tương ứng.

## Chạy

Chạy từng script riêng để tránh tranh chấp camera:

```bash
# Kiểm tra camera: 640x480, 15 FPS
python src/test_camera.py

# Nhận diện pallet và hàng (Box): 640x480, ngưỡng confidence 0.70
python src/test_pallets.py

# Chụp ảnh dữ liệu: 1280x720, 30 FPS
python src/capture_rgb.py
```

Trên Raspberry Pi 5 không có phiên đồ họa, `test_pallets.py` tự in số lượng từng lớp ra terminal. Có thể chạy rõ chế độ này và lưu khung hình cuối:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
.venv/bin/python src/test_pallets.py --headless --save-last data/last_detection.jpg
```

Nếu kết nối qua VS Code Remote SSH và muốn xem hình trực tiếp, chuyển tiếp cổng `8765` trong tab **Ports** của VS Code rồi mở `http://localhost:8765` trên máy tính:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
.venv/bin/python src/test_pallets.py --model models/best_v3.pt --web
```

Để web hiển thị vị trí **tâm lỗ pallet so với tâm xe**, web đọc topic `/pallet/target` từ node đo độc lập `publish_pallet_target.py`: theo dõi RGB, Depth đăng ký, CameraInfo và công thức hiệu chỉnh. YOLO chạy riêng trong `publish_pallet_detection.py`, chỉ cung cấp vùng pallet ban đầu qua `/pallet/detection`. Web không đọc Depth hoặc chạy thêm YOLO trong chế độ này. Ảnh theo dõi và số đo cùng timestamp được node đo phát qua `/pallet/preview/compressed`. Khi YOLO bật và có ảnh mới, khung chính hiển thị đủ nhãn **Box / Pallet / Empty pallet** từ `/pallet/detection/preview/compressed`; ảnh đo nằm trong phần **Xem thêm ảnh đo RGB + Depth**. Khi tắt YOLO, khung chính chuyển sang ảnh theo dõi RGB-D. Các ô số vẫn lấy từ cặp ảnh/phép đo RGB-D cùng timestamp, độc lập với thời điểm của ảnh YOLO. Trạng thái được đặt ngoài ảnh, không ghi khoảng cách/góc lên góc trái khung hình.

Nếu Astra và node `/pallet/target` đã chạy, chỉ cần mở web:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_web.sh
```

Nếu chưa chạy, mở Astra trong terminal thứ nhất:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
ASTRA_POINT_CLOUD=false bash scripts/open_registered_astra.sh
```

Trên Pi 5 này đã tạo bản NCNN từ đúng trọng số `models/best_v3.pt` tại `data/runtime_models/best_v3_ncnn_model`. Node nạp bản đó ở FP32; phép ghép RGB/Depth, hiệu chỉnh, tìm chân và các điều kiện hợp lệ giữ nguyên. Nếu cần tạo lại (Ultralytics sẽ cài thêm NCNN/PNNX vào venv khi thiếu):

```bash
.venv/bin/python scripts/export_pallet_ncnn.py
```

Trong terminal thứ hai, chạy bộ nhận diện + đo (script tự mở hai tiến trình riêng):

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_target.sh --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2
```

Sau đó chạy `bash scripts/open_pallet_web.sh` trong terminal thứ ba và mở `http://localhost:8765` (chuyển tiếp cổng 8765 nếu dùng VS Code Remote SSH).

### Bộ căn xe: Nav2 tới trục tiếp cận rồi sửa góc

Giữ camera, nhận diện và web đang chạy. Trong terminal thứ tư, dùng:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_alignment_real.sh
```

Lệnh này mở bánh xe, IMU/EKF, lidar, lớp tránh vật cản và bộ căn CHẠY THẬT; xe chờ bấm **Bắt đầu căn xe** trên web. Goal gần nhất trên pháp tuyến qua tâm lỗ, khoảng cách từ mặt pallet tới gốc càng **tối thiểu 1,20 m** (goal chừa thêm 10 cm), lệch ngang dưới **2 cm**, góc dưới **2°**. Dừng bộ căn `DRY RUN` cũ bằng Ctrl+C trước khi mở. Hướng dẫn đầy đủ: [ALIGNMENT.md](ALIGNMENT.md).

### Bật nhận diện theo giai đoạn để dành CPU cho điều hướng

Để node giữ model trong RAM nhưng khởi động ở trạng thái tạm ngừng suy luận:

```bash
bash scripts/open_pallet_target.sh --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2 --start-paused --max-fps 2
```

Mặc định sau khi model sẵn sàng, YOLO chạy **2 FPS trong 3–5 giây**:

1. Xác nhận ít nhất ba kết quả YOLO liên tiếp, confidence ≥0,75, cùng lớp và vùng pallet nhất quán.
2. Yêu cầu phép đo RGB + Depth mới, hợp lệ và ổn định trong ít nhất 0,2 giây. RGB-D chạy ngay từ đầu để kiểm chứng, không suy ra khoảng cách chỉ từ YOLO.
3. Từ giây thứ 3, nếu đủ các điều kiện thì **tự tắt YOLO**. Hết giây thứ 5 mà chưa đủ điều kiện cũng tắt YOLO và báo nhận diện lại.
4. Khi đã xác nhận pallet, RGB + Depth tiếp tục đo độc lập. Xe **chờ người dùng bấm căn chỉnh**; không tự chạy sau nhận diện.

Web hiển thị thời gian nhận diện và trạng thái chuyển sang đo. Nút **Nhận diện lại (3–5 giây)** mở lượt mới, xóa xác nhận cũ và yêu cầu seed mới. Nút **Dừng nhận diện** hủy lượt đang nhận diện. Mất dấu RGB không tự bật YOLO lại; phép đo không hợp lệ bị xóa như trước.

Nút căn riêng yêu cầu detector tắt YOLO, đợi khung hình suy luận đang chạy kết thúc và lấy ít nhất ba phép đo RGB-D **chụp sau xác nhận tắt YOLO**, rồi mới cho bộ căn bắt đầu. Tắt/nhận diện lại không tự gửi lệnh xe. Trong khi đang chuẩn bị hoặc thực hiện căn, không cho bật YOLO lại. Xem [ALIGNMENT.md](ALIGNMENT.md).

`--max-fps 2` chỉ giới hạn YOLO; mặc định hiện là 2 FPS, đặt `0` để không giới hạn. Node RGB + Depth độc lập chạy không giới hạn (`--track-max-fps 0`), lấy cặp ảnh mới nhất, không tạo hàng đợi xử lý ảnh cũ. JPEG web mặc định giới hạn 8 FPS (`--preview-fps 8`) để dành CPU cho đo; đây không phải tốc độ topic đo. Model được giữ trong RAM khi tạm ngừng. Supervisor giới hạn BLAS/OpenMP của mỗi tiến trình; NCNN dùng `--cpu-threads` đã chọn.

Chế độ tùy chọn `--recognize-after-goal` quan sát goal Nav2 mới rồi gọi cùng luồng
nhận diện, xem [PERCEPTION_HANDOFF.md](PERCEPTION_HANDOFF.md). Không bật tùy chọn
thì hành vi khởi động cũ được giữ nguyên. Hai nút chỉ đổi YOLO, không phát lệnh
vận tốc hay nâng càng. Trong terminal đã source ROS 2 và workspace:

```bash
# Tắt YOLO, giữ node đo RGB + Depth
ros2 service call /pallet/set_enabled std_srvs/srv/SetBool '{data: false}'
# Nhận diện ban đầu hoặc tìm lại sau khi mất dấu
ros2 service call /pallet/set_enabled std_srvs/srv/SetBool '{data: true}'
```

Trạng thái `/pallet/enabled` là trạng thái YOLO, phát mỗi 0,2 giây và lưu mẫu cuối. `/pallet/recognition/status` cung cấp lượt nhận diện, thời gian, kết quả xác nhận và trạng thái xử lý dở. Web khóa nút nếu không có heartbeat hoặc dịch vụ chưa sẵn sàng; dữ liệu đo vẫn được kiểm tra độc lập. Node detector không xuất `/pallet/target`; chỉ node RGB-D xuất topic đo. Hướng dẫn kiến trúc, build interface mới và giới hạn theo dõi: [TRACKING.md](TRACKING.md).

Web cập nhật CPU mỗi giây: CPU toàn Pi, tổng CPU hai tiến trình YOLO + RGB-D và từng lõi. Hai ô CPU tổng dùng thang 0–100% trên **toàn bộ các lõi**; trên Pi 5 bốn lõi, 25% tương đương một lõi chạy đầy tải. Mỗi thanh lõi riêng dùng thang 0–100% của lõi đó. Khi chạy RGB trực tiếp, ô nhận diện đo chính tiến trình YOLO/web. CPU toàn máy vẫn cập nhật khi các node chưa chạy.

Web hiển thị X tiến, Y ngang (dương về trái xe), khoảng cách phẳng `sqrt(X²+Y²)`, góc hướng từ trục tiến của xe tới tâm lỗ `atan2(Y,X)`, và riêng góc xoay mặt pallet từ node. Gốc tọa độ là `base_link` tại tâm hình học khung xe khi nhìn từ trên xuống, giữa hai trục bánh trước/sau. Gốc `base_footprint` là hình chiếu của nó xuống sàn, có cùng X/Y. Theo `robot_core.xacro`, khung dài 0,601 m nên web lấy `X = chassis_front_forward_m + 0,3005`; `Y = opening_lateral_m`. Cách này giữ nguyên offset camera do node đo cấu hình. Với offset mặc định camera trước mép xe 0,0665 m, X tương đương `camera_forward_m + 0,367` (phù hợp chuỗi URDF `forklift.xacro` và `sensor_mount.xacro`). Camera nằm trên trục giữa Y=0 và không lệch yaw theo mô hình hiện tại.

Web chỉ hiện phép đo khi node báo `valid=true`, đúng frame camera, có ảnh cùng timestamp và kết quả mới trong 1 giây. Khi node báo lỗi, mất dữ liệu, số đo không hữu hạn hoặc kết quả cũ, các ô số được xóa. Node đo vẫn giữ ngưỡng tuổi kết quả 500 ms và yêu cầu ba khung hình ổn định. Vị trí hiển thị là tâm lỗ tại mặt trước pallet chiếu trên mặt phẳng sàn; không phải khoảng cách 3D có tính độ cao lỗ. Hiệu chỉnh Depth/góc lắp hiện chỉ là bản đo thử; cần đối chiếu thước trước khi dùng để điều khiển xe.

Ảnh và phép đo được ghép trong bộ đệm ngắn rồi cập nhật cùng lúc. Trong lúc chờ ghép khung tiếp theo, web giữ cặp ảnh/số đo trước nếu vẫn còn hạn 1 giây; kết quả không hợp lệ hoặc quá hạn vẫn xóa số ngay. Cách này tránh chớp ô số chỉ vì ảnh JPEG và topic phép đo đến khác thời điểm. Vùng lấy mẫu chân dùng băng hàng chung ban đầu, chỉ mở rộng tối đa 13 hàng khi thiếu điểm; vẫn cần ít nhất 60 mẫu Depth hợp lệ, tỷ lệ hợp lệ 65% và kiểm tra độ ổn định bề mặt như trước.

Góc chính trên web là **Góc lệch hướng càng ↔ trục lỗ**: lấy `entry_heading_rad` của node (hướng đưa càng vào, vuông góc mặt pallet, bằng `-face_yaw_rad`). **Hiển thị góc sau lọc thích nghi, không quy góc nhỏ về 0°**; giao diện làm tròn đến 0,1°. Khi trị tuyệt đối góc sau lọc dưới 2°, web báo **Không cần chỉnh hướng**; từ 2° trở lên báo cần chỉnh hướng. Đánh giá này chỉ xét hướng, không xác nhận đã đúng vị trí ngang hoặc có thể đưa càng vào. API dùng `measurement.filtered_fork_alignment_deg` cho ô góc chính và `heading_correction_required` cho trạng thái. Góc trước lọc giữ ở `fork_alignment_deg` / `measured_fork_alignment_deg`, có ô riêng trong phần thông tin phụ và tooltip đến 0,01°. Ví dụ mặt pallet +8,6° thì lệch hướng đưa càng là −8,6°, kể cả khi tâm lỗ nằm gần thẳng phía trước xe. Cần căn cả hướng và vị trí lệch ngang Y khi đưa càng vào. Góc hướng tới điểm tâm lỗ từ tâm xe (`bearing_deg`) và góc mặt pallet nằm trong phần “Thông tin góc phụ” mặc định thu gọn. API cũng giữ hướng tới tâm lỗ từ gốc càng ở `measured_fork_bearing_deg`, nhưng không dùng góc vị trí này để kết luận đã thẳng trục lỗ. Vị trí X/Y và góc trên topic ROS giữ nguyên; bộ căn riêng sử dụng các scalar này cùng odometry như ALIGNMENT.md.

Bộ lọc góc web dùng EMA theo timestamp chụp ảnh: hằng số thời gian 0,8 s khi thay đổi nhỏ, 0,15 s khi chênh ít nhất 2° so với góc đang lọc. Ở 2 FPS, thay đổi lớn được cập nhật khoảng 96% ngay mẫu tiếp theo; lọc thay đổi nhỏ có độ trễ. Mỗi timestamp chỉ cập nhật một lần, polling HTTP không làm trôi số. Khi phép đo RGB-D lỗi hoặc hết hạn, bộ lọc được đặt lại; bật/tắt riêng YOLO không đặt lại bộ lọc; khoảng trống mẫu trên 1 s cũng đặt lại. Bộ lọc chạy trong web subscriber, không thêm YOLO, không sửa topic ROS `/pallet/target` và không gửi lệnh chuyển động. Bộ căn riêng lọc trong hệ odom và dùng cùng ngưỡng 2° để đánh giá hướng.

Ô **Gốc càng → mặt lỗ (theo X)** dùng mốc nơi càng bắt đầu nhô ra. Theo `fork_mount_offset` trong `forklift.xacro`, mốc này ở trước mép thân 0,065 m, nên `fork_root_forward_m = chassis_front_forward_m - 0,065`. Đây là khoảng tiến giữa hai mặt phẳng theo trục X của xe, dùng để đối chiếu phép đo bằng thước từ gốc càng.

Node cũng nhận trực tiếp `--model models/best_v3.pt`. Khi đo trên Pi này, PyTorch xử lý khoảng 465 ms/ảnh còn NCNN FP32 hai luồng khoảng 315 ms/ảnh (640×480, một ảnh RGB thử, có driver camera đang chạy). Hai mask trong ảnh thử có IoU 1,0 giữa hai runtime; đây chưa phải đánh giá toàn bộ tập validation. Một số khung hình thực vẫn có thể bị node bỏ vì quá 500 ms; web sẽ báo lý do và xóa số cũ.

Hướng dẫn đo và bù sai số khoảng cách/góc: [CALIBRATION.md](CALIBRATION.md). Công cụ `scripts/record_pallet_calibration.py` ghi 30 kết quả mới từ chính node RGB + Depth, đối chiếu số đo thước và lưu JSONL; không tự áp dụng hệ số bù hoặc chạy thêm YOLO.
Hồ sơ bù thực nghiệm ngày 2026-10-31 được bật bằng `--measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml`. Xem phạm vi đã đo và sai số kiểm tra trong hướng dẫn trước khi dùng hồ sơ.

Driver Pi cần bộ Orbbec OpenNI2 ARM64 trong `ros2_ws/src/ros2_astra_camera/astra_camera/openni2_redist/arm64` trước khi build (thư mục SDK không đưa vào Git). Trên máy này đã khôi phục bộ SDK từ workspace cũ và build lại driver cùng `detected_pallets_interfaces`.

Kiểm tra luồng ROS → HTTP (test dùng ROS domain riêng, bao gồm đổi tọa độ, ảnh cùng timestamp, dữ liệu lỗi và hết hạn):

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
.venv/bin/python -m unittest discover -s tests -p test_target_web.py -v
```

Chỉ chạy một chương trình dùng camera mỗi lúc. Dừng lệnh `--headless` bằng **Ctrl+C** trước khi chạy lệnh `--web`.

Nhấn **Ctrl+C** để dừng ở terminal và lưu khung hình cuối, hoặc **Q** trong cửa sổ camera nếu có giao diện. Thêm `--max-frames N` để tự dừng sau N khung hình, `--model /đường/dẫn/model.pt` để chọn model khác. Với `capture_rgb.py`, nhấn **S** để lưu ảnh vào `data/rgb/` (thư mục tự tạo). Ảnh chụp và môi trường Python không được đưa vào Git.

## Xem khoảng cách bằng RViz trên máy ROS 2 Jazzy

Để xem point cloud mà `yolo_depth.py` dùng, chạy Astra với `depth_registration=true`. Lệnh bên dưới dùng ROS và driver đã cài trên máy này. Nếu script báo thiếu quyền USB, chạy đúng lệnh `sudo chgrp ... && sudo chmod ...` mà nó in ra, rồi chạy lại. Giữ terminal camera mở.

```bash
cd /home/tai/detected_pallets
bash scripts/open_registered_astra.sh
```

Mở terminal thứ hai:

```bash
cd /home/tai/detected_pallets
bash scripts/open_pallet_distance_rviz.sh
```

RViz mở `config/pallet_distance.rviz`: point cloud `/camera/depth/points`, công cụ **Select** và panel **Selection**. Trong khung 3D, chọn một điểm trên mặt pallet rồi mở `Astra depth point cloud` trong Selection để đọc `Position`. Với fixed frame `camera_color_optical_frame`, `Z` là khoảng cách theo trục nhìn của camera tính bằng mét; khoảng cách thẳng từ tâm quang học tới điểm là `sqrt(X² + Y² + Z²)`. Một điểm chọn được không tự xác nhận đó là mặt trước pallet hoặc điểm YOLO đã chọn. Kết quả này vẫn cần đối chiếu thước đo thực tế trước khi dùng điều khiển xe.

### Xem point cloud với nội tại RGB đã hiệu chuẩn

`open_registered_astra.sh` nạp `config/calibration/astra_pro_rgb_640x480_2026-10-11.yaml` vào CameraInfo RGB. Point cloud mặc định của driver vẫn dùng tiêu cự Depth khoảng 570 px. Để xem XYZ chiếu theo nội tại RGB (`fx≈613`, `fy≈611` và hệ số méo), giữ Astra chạy và mở terminal khác:

```bash
cd /home/tai/detected_pallets
bash scripts/open_rgb_calibrated_depth_rviz.sh
```

Lệnh tạo `/camera/depth/points_rgb_calibrated` ở tối đa 10 Hz, lấy một điểm mỗi 2 pixel, rồi mở RViz có **Select** và panel **Selection**. Cách tính này giả định ảnh Depth đã đăng ký cùng pixel với RGB gốc. Nó chưa hiệu chỉnh sai số đo khoảng cách của cảm biến Depth; khi thử trên bàn cờ ở khoảng 1,1 m, Depth và khoảng cách suy từ bàn cờ còn lệch khoảng 6 cm.

### Xem thử hiệu chỉnh Depth lệch trái–phải

Sau khi giữ camera với góc roll 0°, các hàng sàn phẳng ở ba khoảng cách cho thấy độ dốc nghịch đảo Depth theo cột ảnh khoảng `0.00015807 1/m/pixel`. Giữ Astra chạy, mở terminal khác:

```bash
cd /home/tai/detected_pallets
bash scripts/open_spatial_corrected_depth_rviz.sh
```

RViz hiển thị `/camera/depth/points_spatial_corrected_preview` với **Select** và panel **Selection**. Công thức xem thử là `1/Z_mới = 1/Z_gốc + 0.00015807 × (370-u)`, trong đó `u` là cột pixel RGB. Cột `u=370` giữ nguyên Depth gốc; X/Y tính lại từ nội tại RGB đã hiệu chuẩn. Topic Depth gốc và cloud RGB cũ vẫn giữ nguyên để đối chiếu. Hệ số này lấy từ vùng sàn, chưa kiểm chứng độ chính xác tuyệt đối bằng thước trên toàn khung hình; không dùng cloud xem thử để điều khiển xe.

Cloud này dùng Depth cảm biến đã đăng ký theo RGB, nội tại/méo ống kính từ
`/camera/color/camera_info`, rồi áp dụng hệ số sửa Depth theo cột ảnh. Nó không
suy ra Depth mới từ độ sáng RGB hay model Depth AI. Point cloud giảm mật độ
(`pixel_stride=3`) chỉ để xem trong RViz; `/pallet/target` dùng cùng công thức
sửa tại pixel Depth gốc đủ 640×480 và dùng ảnh RGB/YOLO để chọn chân pallet.
Vì vậy phép đo pallet không phụ thuộc mật độ point cloud. Hiện chưa có topic
ảnh Depth **đã sửa** riêng cho các chức năng khác của xe; nếu cần bản đồ Depth
đầy đủ, phải xuất topic đó từ cùng công thức và kiểm chứng bằng thước trước khi
điều khiển chuyển động.

### Phát mục tiêu pallet riêng cho việc đo thử

Giữ `open_registered_astra.sh` chạy. Node mới lấy ảnh RGB và depth gốc
640×480, ghép theo timestamp tối đa 50 ms, kiểm tra CameraInfo và cấu hình
đăng ký depth, rồi dùng YOLO11-seg để tìm hai chân trước. Công thức hiệu chỉnh
được áp dụng trên pixel depth gốc, độc lập với mật độ point cloud RViz.

```bash
cd /home/tai/detected_pallets/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select detected_pallets_interfaces

# Terminal mới, sau khi đã mở Astra:
cd /home/tai/detected_pallets
bash scripts/open_pallet_target.sh

# Terminal theo dõi kết quả:
source /opt/ros/jazzy/setup.bash
source /home/tai/detected_pallets/ros2_ws/install/setup.bash
ros2 topic echo /pallet/target
```

Script mặc định dùng Python có ROS và Ultralytics tại
`camera/detect_astra/.venv/bin/python`; có thể chọn interpreter khác bằng
`PALLET_PYTHON=/đường/dẫn/python bash scripts/open_pallet_target.sh`.
`valid=false` và các số đo là NaN khi ảnh không đồng bộ, thiếu chân pallet,
CameraInfo không hợp lệ hoặc kết quả quá cũ. Bộ nhận phải kiểm tra `valid` và
tuổi của `header.stamp`, không giữ tọa độ cũ khi mất pallet.

Trục `forward` song song mặt sàn theo hướng xe chạy; `lateral` dương về bên
**trái** xe. `left_forward_m` và `right_forward_m` là khoảng cách hai chân đến
camera theo phương tiến. Hai chân được lấy Depth trong cùng một dải hàng hẹp
ở giữa phần chân nhìn thấy được; từng điểm được chiếu về phương song song mặt
sàn theo góc camera 20°. `camera_forward_m` là trung bình hai chân;
`chassis_front_forward_m` cộng thêm **6,65 cm** từ mặt trước ống kính tới mép
trước khung xe theo số đo lại của người vận hành; giá trị này cũng trùng với
thiết kế trong URDF.
`opening_lateral_m` là độ lệch tâm lỗ so với trục camera (giả định camera gắn
đúng giữa xe). `face_yaw_rad` dương nghĩa chân **trái xa hơn chân phải**;
`entry_heading_rad = -face_yaw_rad` là hướng tiến vuông góc mặt pallet.

`opening_width_m` lấy hai mép trong của chân; `opening_height_estimate_m` lấy
mép dưới dầm và chân từ mask RGB, giả định chúng cùng mặt phẳng trước. Chiều
cao này kém chắc chắn: mẫu camera hiện tại cho khoảng 7,5 cm trong khi thước
đo của người vận hành là 10 cm. Chiều rộng ảnh khoảng 23,7 cm so với thước
25,2 cm. Hệ số sửa depth, góc lắp camera và vị trí ống kính chưa được xác nhận
độc lập. `valid=true` chỉ nói rằng kiểm tra hình học của khung hình hiện tại
đã qua; chưa chứng nhận đủ chính xác để điều khiển AGV hay đút càng. Node này
**không phát lệnh xe**.
