# Điều tra kẹt cạnh trụ phía dưới — 06/09/2026

## Nguyên nhân có bằng chứng

Ảnh mới của người dùng là tình huống chưa được bao phủ bởi 16 chặng thử ngày
05/09. Log của `controller_server_84249` ghi cả hai cung quay bị chặn; thử đường
khác vẫn bị collision checking từ chối.

Tư thế đọc trước khi mô phỏng của người dùng dừng:

- TF map: x=-0.0218, y=-1.4967, yaw=2.5174 rad.
- Gazebo: x=0.015828, y=-1.478305, quaternion z=0.954928, w=0.296836.
- Goal XY mới trong log: x=-0.97, y=-1.53. Hướng goal trong thử nghiệm là pi;
  không có bằng chứng đây chính xác là hướng goal đã chọn trên ảnh.

Trong mô phỏng tái hiện, DriveOnHeading không báo collision mà hết thời gian.
Collision Monitor báo `Robot to stop due to VelocityStop polygon`. Đọc scan
trong frame base_footprint có 13 điểm nằm trong vùng dừng tiến, ví dụ
(-0.074, -0.329). Đây là điểm phía sau tâm xe, ngoài thân có đệm y=±0.29 m,
nhưng nằm trong vùng dừng tiến cũ có y=±0.35 m tới x=-0.10 m.

Đường đỏ bao quanh xe trong RViz là display `Emergency Stop Zone`, topic
`/collision_monitor/footprint_stop`. Nó đổi hình theo vận tốc, không phải thân
xe hoặc vật cản vật lý tự lớn lên. Các điểm lidar cũng có thể được vẽ đỏ.

## Sửa giữ lại

- Polygon dừng tiến bao **toàn bộ footprint có đệm**, gồm cả phía sau, nhưng
  phần đệm dừng lớn hơn được dành cho hướng tiến. Điểm sau-bên ngoài thân không
  còn chặn nhầm chuyển động đi ra xa nó. FootprintApproach, các nguồn scan và
  kiểm tra va chạm của controller/behavior vẫn bật.
- Nếu đường SE2 không thực thi được, thử tiến 0.25 m mà không bắt buộc quay
  trước. Sau mỗi bước lập và thử đường lại; tối đa ba bước có kiểm tra va chạm
  trong nhánh này. Một bước thất bại làm nhánh dừng, không tiếp tục tiến mù.
- Cung quay bị invalidated được giữ ở trạng thái lỗi cho tới khi nhận plan mới,
  tránh chọn qua lại hai hướng trên cùng plan và liên tục reset thời gian chờ.
- Tăng trọng số chi phí tránh vật cản của GridBased từ 2 lên 4; giảm tốc trong
  vùng gần vật cản từ khoảng 0.45 m, tốc độ điều tiết tối thiểu 0.05 m/s.

Đã thử SE2 làm planner chính cùng một controller nhìn trước ngắn hơn, nhưng
kết quả hồi quy xấu hơn. Những thay đổi thử đó **đã bỏ**, không nằm trong bản
bàn giao. Không tắt collision checking để làm bài thử thành công.

## Kiểm thử

### Tái hiện trạng thái kẹt

Khởi tạo Gazebo và AMCL ở tư thế đã ghi ở trên để thử thoát. Đây là khởi tạo
tình huống thử, không tính thao tác đặt trạng thái là xe tự điều hướng.
Không tái tạo được toàn bộ lịch sử các ô costmap của phiên người dùng.

Sau khi sửa polygon và các bước tiến liên tiếp:

| Chặng | Kết quả | Thời gian thực | Sai số XY theo TF |
| --- | --- | ---: | ---: |
| Tư thế kẹt → (-0.97, -1.53, pi) | SUCCEEDED | 32.50 s | 0.0388 m |
| Goal trên → (0, 0, 0) | SUCCEEDED | 22.60 s | 0.0483 m |

Log: `/home/tai/.ros/log/2026-09-06-00-11-57-346110-DESKTOP-B3BME4V-136893`.
Log behavior xác nhận hai lần tiến thoát thành công. Không dùng riêng số
`recoveries` trong feedback để kết luận xe có hoặc không thực hiện recovery.

Đã trả spawn Gazebo và AMCL về (0,0,0) sau thử nghiệm.

### Hồi quy hai mốc cũ trước khi thêm chốt lỗi cung quay

4/4 chặng SUCCEEDED, thời gian 29.45 / 39.52 / 40.60 / 90.76 s. Lượt cuối
vẫn đổi hướng quay nhiều lần, dẫn tới sửa chốt lỗi cung quay nói trên.
Phép kiểm tra hình học độc lập trong phần được theo dõi của lượt này có độ hở
nhỏ nhất khoảng 0.1025 m và không giao nhau. Đây không phải cảm biến contact.

### Kiểm thử tự động

- `test_rotation_sweep`: 5 bài đạt.
- `test_stop_envelope`: 3 bài đạt, gồm bao đủ footprint có đệm, giữ vùng dừng
  trước/sau thật và loại điểm gây chặn nhầm đã đo.
- Build thành công. Không tuyên bố toàn bộ lint của kho sạch.

### Bản cuối có chốt lỗi cung quay — Gazebo GUI

Đã build lại và chạy Gazebo không headless. RViz được mở riêng trong lượt thử
(sau chặng đầu, trong chặng về thứ nhất). Cả 4 chặng thành công:

| Chặng | Thời gian thực | Sai số XY theo TF | Recovery theo feedback |
| --- | ---: | ---: | ---: |
| A→B | 30.90 s | 0.0645 m | 0 |
| B→A | 81.23 s | 0.0379 m | 5 |
| A→B | 44.36 s | 0.0339 m | 0 |
| B→A | 66.57 s | 0.0293 m | 0 |

A=(0,0,0), B=(1.532,-1.412,-0.012). Log:
`/home/tai/.ros/log/2026-09-06-01-36-23-386046-DESKTOP-B3BME4V-154001`.

Chiều về vẫn chậm và có thể cần recovery: không coi đây là bảo đảm tìm được
đường tối ưu, hoặc mọi trường hợp kẹt đã được giải quyết. Bài tái hiện trạng
thái kẹt ở trên chạy trước khi thêm chốt lỗi cung quay; bản cuối được kiểm tra
hồi quy với GUI theo bảng này. Cần tiếp tục thử thêm trạng thái đầu/goal và
dữ liệu costmap tích lũy khác trước khi triển khai thực tế.

Đã dừng Gazebo/RViz và các tiến trình thử do agent mở sau khi kiểm tra xong.

Lệnh build, mở Gazebo và RViz vẫn như báo cáo ngày 05/09. Không cần đổi map
hoặc xóa cache. Chỉ thử với mô phỏng; chưa chứng minh mọi goal đều có đường
khả thi hoặc bảo đảm an toàn cho xe thật.
