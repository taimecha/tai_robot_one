# Kiểm chứng nhận diện → theo dõi RGB-D

Cập nhật tác vụ 2026-11-02: bộ căn và lệnh đưa càng/nâng đã tách khỏi luồng đo; xem [ALIGNMENT.md](ALIGNMENT.md) và [PICKUP.md](PICKUP.md). Các hướng dẫn chỉ đo bên dưới không tự mở điều khiển chuyển động.

Lượt sửa mới nhất ngày 2026-11-01 dùng tìm kiếm theo đối tượng và model NCNN
288×384, giữ ảnh đo 480×640: 900/900 phép đo hợp lệ trong 60 giây cùng Nav2,
toàn bộ tiêu chí đạt. Xem phần đầu [TRACKING.md](TRACKING.md) để lấy cách
dựng/chạy và phạm vi kiểm chứng; các kết quả chưa đạt bên dưới là lịch sử.

Góc giữa mặt phẳng trước xe và mặt pallet được kiểm tra riêng bằng tùy chọn
`--heading-floor-reference`, giữ nguyên khoảng cách/X/Y. Kết quả đo mới nhất,
kiểm tra mở lại và lệnh giao diện: [HEADING_REFERENCE.md](HEADING_REFERENCE.md).
Các số liệu bên dưới là lịch sử các lượt dùng góc cũ.

Luồng trong tài liệu này chỉ chuẩn bị dữ liệu; khi chỉ kiểm chứng camera không mở bộ căn. Node nhận diện/đo và công cụ kiểm chứng
không phát vận tốc, gửi goal điều hướng hay nâng càng.

## Sau goal nhận diện

Giữ camera Astra đang chạy (`ASTRA_POINT_CLOUD=false bash scripts/open_registered_astra.sh`).
Script camera chỉ mở RGB và Depth, tắt stream ảnh IR không được bộ đo sử dụng.
Mở bộ nhận diện/đo từ thư mục `camera/detect_astra`:

```bash
bash scripts/open_pallet_target.sh \
  --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2 \
  --target-color green \
  --measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml \
  --recognize-after-goal
```

Tùy chọn này tự đặt YOLO ở trạng thái tắt. Model được nạp sẵn trong RAM;
RGB/Depth và kiểm tra đăng ký vẫn chạy. Bộ quan sát chờ **một goal mới** trên
`/navigate_to_pose/_action/status` được tạo sau khi bắt đầu chờ, đã thấy trạng
thái accepted/executing, rồi chuyển sang succeeded. Khi đó mới mở lượt nhận
diện 3–5 giây. Thành công cũ lưu trong ROS không kích hoạt. Hủy/thất bại không
kích hoạt; goal mới đang chạy sẽ tắt YOLO và xóa xác nhận mục tiêu cũ. Kết quả
suy luận dở từ thế hệ cũ bị bỏ, không làm seed cho lượt mới.

**Goal kế tiếp sau khi bật chế độ này được coi là goal nhận diện.** Chưa có
cơ sở dữ liệu phân loại goal theo nhiệm vụ. Vì vậy chỉ mở chế độ/chờ lại ngay
trước goal nhận diện mong muốn; không dùng nó để tự đoán loại goal trong một
tuyến nhiều điểm. Không gửi goal thật chỉ để kiểm tra bộ nhận diện khi xe phải
đứng yên. Có thể kiểm tra camera bằng chế độ thủ công bên dưới.

Mỗi lần chờ chỉ tự kích hoạt một lượt. Trước goal nhận diện tiếp theo, hoặc sau
goal thất bại, gọi dịch vụ chờ lại **trước khi gửi goal mới**:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 service call /pallet/arm_recognition_goal std_srvs/srv/Trigger '{}'
```

Đổi namespace Nav2 bằng `--nav-status-topic /namespace/navigate_to_pose/_action/status`.
Nếu không thấy trạng thái đang chạy, hệ thống giữ YOLO tắt; không suy đoán thành công.
Luồng này không ra lệnh điều hướng và không tự khởi động bộ căn xe.

## Xác nhận pallet và dữ liệu sau khi tắt YOLO

Giữ các điều kiện nhận diện đang có: ít nhất ba kết quả cùng lớp pallet,
confidence ≥0,75, bbox nhất quán và RGB-D hợp lệ/ổn định. Từ giây thứ 3 đủ
điều kiện thì tắt YOLO; đến giây 5 chưa đạt cũng tắt và báo nhận diện thất bại.
Chỉ lớp pallet được chọn làm mục tiêu đo. YOLO vẫn nhận diện/hiển thị hàng Box
trong lượt bật; chưa xác nhận danh sách hàng độc lập sau khi tắt YOLO.

YOLO có thể gán đồng thời `pallets` và `emty pallet` lên cùng một vật. Chỉ khi
bbox trùng ít nhất IoU 0,80, giữ nhãn/mask có confidence cao nhất trước khi
đếm pallet. Vùng pallet riêng hoặc chồng ít hơn vẫn bị coi là nhiều mục tiêu
và không xác nhận. Không áp dụng lọc trùng này cho lớp hàng `Box`, không hạ
ngưỡng confidence xác nhận 0,75. Nhãn được chọn vẫn phải nhất quán ba lần.

Sau khi suy luận dở kết thúc, chỉ nhận phép đo **chụp sau thời điểm YOLO dừng**
để đánh giá thêm cửa sổ trượt 5 giây khi xe/pallet đứng yên:

- Ít nhất 98% phép đo hợp lệ, tốc độ hợp lệ ≥10 Hz, không ngắt mẫu hợp lệ quá 250 ms.
- Phép đo hiện tại mới ≤250 ms; RGB/Depth lệch ≤50 ms, đủ mẫu hai chân, đúng frame.
- Trong cửa sổ: dao động X ≤3 cm, Y ≤2 cm, hướng ≤2°.

`/pallet/recognition/status` bổ sung `goal_gate` và `rgbd_health`, gồm `stable`,
FPS, tỷ lệ hợp lệ, khoảng ngắt và biên độ X/Y/góc. Web hiện thông báo đang kiểm
tra hoặc dữ liệu ổn định. Dữ liệu lỗi/hết hạn vẫn bị xóa, không giữ tọa độ cũ
thành số đo mới, không tự bật lại YOLO. Có thể bấm nhận diện lại sau goal thành
công; trong lúc đang chờ/đi tới goal thì nút đó bị khóa.

`rgbd_health.stable` là tiêu chí chất lượng **khi đứng yên**, không phải lệnh
hoặc quyền chạy xe. Nếu dịch chuyển camera/pallet, phải đánh giá lại. Chưa dùng
tiêu chí biên độ đứng yên để đánh giá khi xe đang chuyển động, vì tọa độ tương
đối khi đó cần thay đổi. Bộ đo hình học và hệ số hiệu chuẩn giữ nguyên.

## Kiểm chứng camera thật khi xe đứng yên

Không cần chạy Nav2. Mở camera, rồi mở bộ đo **không có** `--recognize-after-goal`
để nhận diện ngay; giữ các tùy chọn model/CPU/bù như lệnh trên. Web tùy chọn:

```bash
bash scripts/open_pallet_web.sh
```

Ở terminal đã source ROS và workspace, chạy công cụ chỉ đọc:

```bash
.venv/bin/python scripts/verify_pallet_handoff.py --seconds 60 \
  --output data/diagnostics/pallet_handoff_live.json
```

Công cụ đợi dữ liệu ổn định sau khi YOLO tự tắt, sau đó ghi 60 giây số đo RGB-D,
FPS, tỷ lệ hợp lệ, tuổi số đo, độ lệch timestamp, dao động X/Y/góc và lý do lỗi.
Nó kiểm tra YOLO không bật/suy luận/seed mới trong thời gian đo, heartbeat còn
mới, phép đo RGB-D vẫn mới/duy nhất. Mặc định chỉ đọc các topic kết quả nhỏ;
thêm `--inspect-camera-streams` nếu cần đếm từng ảnh RGB/Depth trực tiếp (có
thêm tải truyền ảnh DDS). Mã thoát 0 nghĩa đạt các tiêu
chí ghi trong báo cáo; 1 nghĩa chưa đạt, kể cả timeout chờ nhận diện. Công cụ
không bật YOLO lại để che lỗi, không thay ngưỡng dữ liệu, không phát lệnh xe.

Kiểm thử Nav2 dùng goal tổng hợp trong ROS domain riêng, không gửi goal lên
xe thật. Sau khi dữ liệu đạt, việc chạy căn thật vẫn là một yêu cầu riêng của
người vận hành; thử đứng yên không xác nhận độ tin cậy khi xe di chuyển hoặc
độ chính xác tuyệt đối so với thước.

## Kết quả kiểm chứng ngày 2026-11-01

**Chưa đạt toàn bộ tiêu chí 60 giây; chưa xác nhận đủ ổn định để căn xe thật.**
74 kiểm thử của bộ kiểm thử đầy đủ đạt, sau đó kiểm thử ROS goal mới bổ sung
đạt (tổng 75 trường hợp khác nhau). Các goal được tạo giả trong domain 231;
không gửi goal hoặc lệnh động cơ lên xe thật.

Thử camera Astra thực, YOLO tự tắt, không có suy luận/seed YOLO mới trong cửa
sổ 60,03 giây. Báo cáo local: `data/diagnostics/pallet_handoff_2026_11_01_light.json`.

| Chỉ số | Kết quả | Đánh giá |
| --- | --- | --- |
| Phép đo hợp lệ | 1497/1499, 99,87% | Đạt ≥98% |
| Tốc độ phép đo hợp lệ | 24,94 Hz | Đạt ≥10 Hz |
| Tuổi phép đo | trung bình 26,3 ms; tối đa 61,3 ms | Đạt ≤250 ms |
| Đồng bộ RGB/Depth | −33,6 … +42,3 ms | Đạt ±50 ms |
| Khoảng ngắt phép đo hợp lệ lớn nhất | 308 ms | **Chưa đạt ≤250 ms** |
| Biên độ X trong 60 giây | 5,58 mm | Đạt ≤30 mm |
| Biên độ Y trong 60 giây | 0,574 mm | Đạt ≤20 mm |
| Biên độ góc thô trong 60 giây | 2,127° | **Chưa đạt ≤2°** |

Hai thông điệp không hợp lệ báo đang chờ cặp RGB/Depth đồng bộ; các số đo lỗi
không được giữ thành kết quả mới. Cửa sổ 5 giây cuối đạt không thay thế kết
luận chưa đạt của cả 60 giây. Góc trung bình khoảng +4,44° là số đo hiện tại,
không tự gán về 0 hoặc thay hiệu chuẩn để làm kết quả đẹp hơn.

Các lượt trước ghi nhận nhãn pallet/pallet rỗng trùng và confidence/nhãn biến
động, một lần mất dấu do ngắt đầu vào >500 ms. Đã sửa lọc bbox trùng, giữ
ngưỡng confidence 0,75 và điều kiện xác nhận ba lần; chưa chứng minh mọi lượt
nhận diện đều thành công. Khi xác nhận thất bại, YOLO vẫn tắt và cần yêu cầu
nhận diện lại. Báo cáo lỗi trước được giữ trong `data/diagnostics/`.

Công cụ giám sát ban đầu còn đọc thêm hai topic ảnh lớn; đã đổi mặc định sang
đọc topic kết quả để giảm tải đo kiểm. Chưa có phép đối chứng đủ để quy mọi
khoảng ngắt đầu vào cho công cụ đó. Cần xử lý/kiểm chứng tiếp độ liên tục RGB-D
và nhiễu góc trước giai đoạn điều khiển; không nới các ngưỡng để báo đạt.

### Lượt sau khi tắt stream IR không sử dụng

`open_registered_astra.sh` đã thêm `enable_ir:=false`; RGB/Depth vẫn 640×480,
30 Hz theo cấu hình driver, giữ đăng ký và hiệu chuẩn cũ. Driver xác nhận chỉ
khởi động Depth và UVC RGB. Trước khi đổi, log camera có một lỗi giải mã JPEG;
chưa đủ bằng chứng quy nguyên nhân lỗi cho stream IR hay USB.

Báo cáo mới nhất: `data/diagnostics/pallet_handoff_2026_11_01_rgbd_only_60s.json`.
Trong 60,00 giây có **1530/1530 phép đo hợp lệ (100%), 25,50 Hz**; YOLO không
suy luận hoặc phát seed mới. Tuổi số đo trung bình 29,6 ms, tối đa 62,7 ms;
độ lệch RGB/Depth −34,9 … +42,8 ms. Biên độ X 6,30 mm, Y 0,410 mm.

**Kết luận vẫn chưa đạt toàn bộ tiêu chí:** khoảng ngắt hợp lệ lớn nhất
**263 ms >250 ms**, biên độ góc thô **2,826° >2°** (2,847 … 5,673°,
độ lệch chuẩn 0,385°). Không có thông điệp invalid trong cửa sổ này nhưng điều
đó không loại bỏ khoảng ngắt nhận kết quả. Một lượt tốt hơn về tỷ lệ hợp lệ
không chứng minh lỗi gián đoạn trước đã được loại bỏ. Giữ RGB-D chạy, YOLO tắt;
chưa khởi chạy bộ căn xe thật.
